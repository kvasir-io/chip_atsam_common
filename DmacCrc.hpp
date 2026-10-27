#pragma once
// The SAM DMAC's CRC engine, Kvasir::DmaCrc: the engine alone, without the DMA channels (DMAC.hpp includes it; the
// NVM stores' record CRC, NVMCTRL.hpp, needs nothing else of the DMAC).
#include "chip/Package.hpp"
#include "chip/atsam_common/DmacCrcHold.hpp"
#include "chip/atsam_common/DmacCrcModes.hpp"
#include "core/Barrier.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Runtime/CrcImageEngine.hpp"   // the hook Dmac is named in, and the software engines (Util/Crc.hpp)
#include "peripherals/DMAC.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <utility>

namespace Kvasir {
// The DMAC's CRC engine as Kvasir::DmaCrc (the software engines are Kvasir::Crc, kvasir/Util/Crc.hpp; what the engine
// computes and how its checksum reads: DmacCrcModes.hpp):
//
//   calcCrc<CRC_Type>(first, last)  one CRC-16/IBM-3740 or CRC-32/ISO-HDLC (seed all ones) over 8-, 16- or 32-bit data
//                                   (NVMCTRL's records)
//   DmacEngine<P>                   Kvasir::Crc::Engine's shape (update/finish/compute) for a CRC the engine computes
//   DmacCalc<P>                     the `type` + `calc(span)` shape (aglio's packager): DmacEngine<P>::compute
//   Dmac                            the image's engine set (Kvasir::Crc::ImageEngines): every run-time CRC a library
//                                   takes as Kvasir::Crc::ImageEngine<P> - a CellStore's cells - runs here when the
//                                   engine computes P, any other on the software
//
// Fed by the CPU through the engine's I/O interface (CRCCTRL.CRCSRC = IO; "CRC can be performed on any data by loading
// them into the CRC engine using the CPU and writing the data to the CRCDATAIN register", D21 DS40001882L 20.6.3.7 md
// l.13184, C21 DS60001479M 25.6.3.7 md l.16034, E5x DS60001507N 22.6.3.8 md l.16564): synchronous, no DMA channel, no
// DmaBase, nothing to list - the DMAC's bus clocks are on out of reset (chip/Blocks.hpp). One engine per DMAC, and
// every start resets it: one user at a time, in one context (thread code; an ISR computing a DMAC CRC while thread code
// has one going corrupts both - a debug check, DmacCrcHold.hpp: a DmacEngine alive or a calcCrc running holds the
// engine, a second start panics in the debug and sanitize variants). Per chip: the D21/C21 turn the CRC on with
// CTRL.CRCENABLE (D21 20.8.1 md l.13387, C21 25.8.1 md l.16279), the E5x with a source in CRCCTRL.CRCSRC (22.6.2.2 md
// l.15899); two CRCDATAIN writes in consecutive instructions can corrupt the CRC on the D21 (errata DS80000760M 1.7.1,
// revisions A-F) and the C21 (DS80000740T 1.10.1, E/G/J revision B), so a NOP follows each write there
// (DmacTraits::crcDataInWritesNeedAGap); the E5x errata (DS80000748T) have no such entry.
//
// No wait on CRCSTATUS.CRCBUSY before a CRCDATAIN write. The I/O interface's paragraph ends "New data can be written
// only when CRCBUSY flag is not set" (D21 md l.13184, C21 md l.16034, E5x md l.16564), but with CRCSRC = IO the flag
// clears only when software writes a one to it (D21 20.8.5 md l.13601, C21 25.8.5 md l.16499, E5x 22.8.5 md
// l.17594-17596), and it is set from the moment the source is selected (D21 md l.13603, C21 md l.16501) or by every
// CRCDATAIN write (E5x md l.17598): a wait for it to clear would never end - before the first byte on the D21/C21,
// before the second on the E5x - and writing the one is the generation's end (the CRC-32 then reads final, 20.8.4 /
// 25.8.4 / 22.8.4; on the E5x it frees CRCSRC, md l.17413). The same paragraph's "The CRC complete is signaled by a set
// CRCBUSY bit" contradicts the flag's name and its description. The pace the engine needs is in CRCDATAIN's own
// description: "A new CRC Checksum is ready (CRCBEAT+1) clock cycles after the CRCDATAIN register is written" (D21
// 20.8.3 md l.13534, C21 25.8.3 md l.16432, E5x 22.8.3 md l.17525) - one cycle for the byte beats every engine user
// writes (CRCBEATSIZE BYTE = 0, E5x md l.17484), which two separate bus writes cannot undercut on a clock at least the
// core's (E5x: the AHB/APB clocks "always synchronous to the CPU clock", md l.6846; fHS >= fCPU, md l.7316) - except
// where the errata say they can (the NOP). calcCrc's 16- and 32-bit beats keep the engine busy longer - 2 cycles, and
// 4 for a word by the I/O paragraph's "a 32-bit data ... takes four cycles" (CRCDATAIN's rule says 3: the larger is
// taken) -, so each such write is followed by a DSB and that many NOPs (DmacCrcModes.hpp crcDataInPace, host-tested).
namespace DmaCrc {

    namespace detail {
        // Off. CRCBUSY first, written 1 (the I/O interface's end): with it set the D21/C21 do not turn the CRC off
        // ("Writing a '0' to this bit will disable the CRC calculation when the CRC Status Busy flag is cleared", D21
        // 20.8.1 md l.13389, C21 25.8.1 md l.16281), and the E5x keeps CRCSRC locked ("the CRCSRC cannot be modified
        // when the CRC operation is ongoing. The lock is signaled by the CRCBUSY status bit", 22.8.2 md l.17413) - a
        // DmacEngine leaves it set.
        template<typename CRC>
        void resetCrc() {
            if constexpr(Sam::Package::dmacHasCrcEnable) {
                if(1 == apply(read(CRC::CTRL::crcenable))) { apply(set(CRC::CRCSTATUS::crcbusy)); }
                apply(clear(CRC::CTRL::crcenable));
            } else {
                apply(set(CRC::CRCSTATUS::crcbusy));
                apply(CRC::CRCCTRL::overrideDefaults());
            }
        }

        template<typename CRC>
        void enableCrc() {
            if constexpr(Sam::Package::dmacHasCrcEnable) { apply(set(CRC::CTRL::crcenable)); }
        }

        // Off, the seed into CRCCHKSUM, polynomial + beat size + the I/O source, on: CRCCTRL and CRCCHKSUM are
        // enable-protected (D21 20.6.2.1 md l.12540-12544, C21 25.6.2.1 md l.15322, E5x 22.6.2.1 md l.15885).
        template<CrcType       Type,
                 std::size_t   Bytes,
                 std::uint32_t Seed>
        inline void start() {
            using CRC = Peripheral::DMAC::Registers<>;
            static_assert(Bytes == 1 || Bytes == 2 || Bytes == 4, "wrong data size");

            resetCrc<CRC>();
            apply(write(CRC::CRCCHKSUM::crcchksum32, Kvasir::Register::value<Seed>()));

            auto const poly = [] {
                if constexpr(Type == CrcType::crc16) {
                    return write(CRC::CRCCTRL::CRCPOLYValC::crc16);
                } else {
                    return write(CRC::CRCCTRL::CRCPOLYValC::crc32);
                }
            }();
            auto const beatsize = [] {
                if constexpr(Bytes == 1) {
                    return write(CRC::CRCCTRL::CRCBEATSIZEValC::byte);
                } else if constexpr(Bytes == 2) {
                    return write(CRC::CRCCTRL::CRCBEATSIZEValC::hword);
                } else {
                    return write(CRC::CRCCTRL::CRCBEATSIZEValC::word);
                }
            }();
            apply(poly, beatsize, write(CRC::CRCCTRL::CRCSRCValC::io));

            enableCrc<CRC>();
        }

        // N NOPs in a row, each kept (volatile) and in order with the memory accesses around it
        [[gnu::always_inline]] inline void nop() { asm volatile("nop" ::: "memory"); }

        template<std::size_t... I>
        [[gnu::always_inline]] inline void nops(std::index_sequence<I...>) {
            ((static_cast<void>(I), nop()), ...);
        }

        // One CRCDATAIN write as wide as the data - the 8-, 16- or 32-bit bus transfer CRCCTRL.CRCBEATSIZE names (D21
        // 20.8.2 md l.13481), a store the DMAC takes: "the 8-bit quarters and 16-bit halves of a 32-bit register ... can
        // be accessed directly" (D21 20.8 md l.13345, C21 25.8 md l.16239, E5x 22.8 md l.17315) - then its pace before
        // the next one (crcDataInPace, DmacCrcModes.hpp): nothing after a byte, or the errata's NOP where the chip needs
        // one; after a halfword or a word a DSB (the write has reached the DMAC) and a NOP for each cycle it keeps the
        // engine busy.
        template<typename Data>
        [[gnu::always_inline]] inline void put(Data v) {
            using CRC = Peripheral::DMAC::Registers<>;
            if constexpr(sizeof(Data) == 4) {
                Kvasir::Register::writeWhole(CRC::CRCDATAIN::FULLREGISTER, v);
            } else {
                Kvasir::Register::writeLane<Data, 0>(CRC::CRCDATAIN::FULLREGISTER, v);
            }
            constexpr CrcDataInPace pace
              = crcDataInPace(sizeof(Data), Sam::Package::Dmac::crcDataInWritesNeedAGap);
            if constexpr(pace.barrier) { Kvasir::Core::dsb(); }
            nops(std::make_index_sequence<pace.nops>{});
        }

        // The CRC of what the engine has taken so far, the engine left running (checksumOf, DmacCrcModes.hpp).
        template<CrcType Type>
        inline std::uint32_t current() {
            using CRC       = Peripheral::DMAC::Registers<>;
            bool const busy = 1 == apply(read(CRC::CRCSTATUS::crcbusy));
            auto const sum  = static_cast<std::uint32_t>(
              Kvasir::Register::get<0>(apply(read(CRC::CRCCHKSUM::crcchksum32))));
            return checksumOf<Type>(sum, busy);
        }
    }   // namespace detail

    template<CrcType Crc,
             typename It>
    auto calcCrc(It start,
                 It end) {
        using CRC = Peripheral::DMAC::Registers<>;

        static constexpr std::size_t dataSize = sizeof(decltype(*start));
        static_assert(dataSize == 1 || dataSize == 2 || dataSize == 4, "wrong data size");
        using Data
          = std::conditional_t<dataSize == 1,
                               std::uint8_t,
                               std::conditional_t<dataSize == 2, std::uint16_t, std::uint32_t>>;

        detail::Held const hold{};
        detail::start<Crc, dataSize, 0xFFFF'FFFFU>();
        for(; start != end; ++start) { detail::put(static_cast<Data>(*start)); }
        // the generation complete (CRCBUSY written 1, the I/O interface's end), then the checksum as the engine gives it
        apply(set(CRC::CRCSTATUS::crcbusy));
        return static_cast<std::conditional_t<Crc == CrcType::crc16, std::uint16_t, std::uint32_t>>(
          Kvasir::Register::get<0>(apply(read(CRC::CRCCHKSUM::crcchksum32))));
    }

    template<CrcType Crc,
             typename T>
    auto calcCrc(T const& v) {
        return calcCrc<Crc>(reinterpret_cast<std::uint8_t const*>(&v),
                            reinterpret_cast<std::uint8_t const*>(&v + 1));
    }

    /// Kvasir::Crc::Engine's shape over the DMAC's CRC engine: the same result as the software for P (dmacComputes<P>:
    /// CRC-32/ISO-HDLC, CRC-16/XMODEM, CRC-16/IBM-3740), byte by byte through CRCDATAIN. The constructor starts the
    /// engine and the object holds it until it goes: one engine at a time, in one context (the CRC lives in CRCCHKSUM,
    /// not in the object; a second user meanwhile is a debug check, DmacCrcHold.hpp). finish() reads it and leaves it
    /// running, so update() may go on; compute() is one engine over one span. Run time only: a constant expression
    /// takes Kvasir::Crc::Engine<P> (Kvasir::Crc::ImageEngine does).
    template<auto P>
    class DmacEngine {
        static constexpr CrcType type = dmacTypeFor<P>();

    public:
        using value_type             = std::remove_cvref_t<decltype(P.poly)>;
        static constexpr auto params = P;

        DmacEngine() { detail::start<type, 1, dmacSeedFor<P>()>(); }

        DmacEngine(DmacEngine const&)            = delete;
        DmacEngine& operator=(DmacEngine const&) = delete;

        DmacEngine& update(std::span<std::byte const> data) {
            for(auto const b : data) { detail::put(std::to_integer<std::uint8_t>(b)); }
            return *this;
        }

        /// The CRC of everything so far; update() may go on.
        [[nodiscard]] value_type finish() const {
            return static_cast<value_type>(detail::current<type>());
        }

        [[nodiscard]] static value_type compute(std::span<std::byte const> data) {
            return DmacEngine{}.update(data).finish();
        }

    private:
        // the only member: taken before the constructor starts the engine, given when the engine goes
        [[no_unique_address]] detail::Held hold_{};
    };

    /// The `type` + `calc(span)` shape (Kvasir::Crc::Calc's) over the DMAC's CRC engine: DmacEngine<P>::compute.
    template<auto P>
    struct DmacCalc {
        using type = std::remove_cvref_t<decltype(P.poly)>;

        static type calc(std::span<std::byte const> data) { return DmacEngine<P>::compute(data); }
    };

    /// The image's CRC engines on the DMAC (the hook Kvasir::Crc::ImageEngines, kvasir/Runtime/CrcImageEngine.hpp):
    /// every CRC the engine computes (dmacComputes<P>) on DmacEngine<P>, any other on the software engine at the
    /// caller's table size.
    /// No channel and no Startup entry: the image names it in the hook, before the first library header that
    /// instantiates a user, and that is all -
    ///
    ///     template<>
    ///     struct Kvasir::Crc::ImageEngines<Kvasir::Tag::User> { using type = Kvasir::DmaCrc::Dmac; };
    ///
    /// The libraries that take Kvasir::Crc::ImageEngine compute their CRCs in the calls they are given (a CellStore's
    /// load and store): thread code, one at a time - as every other DMAC CRC user of the image must be.
    struct Dmac {
        template<auto P, std::size_t TableSize>
        using For
          = std::conditional_t<dmacComputes<P>(), DmacEngine<P>, Kvasir::Crc::Engine<P, TableSize>>;
    };

    static_assert(std::is_same_v<Dmac::For<Kvasir::Crc::Presets::crc32IsoHdlc,
                                           16>,
                                 DmacEngine<Kvasir::Crc::Presets::crc32IsoHdlc>>);
    static_assert(std::is_same_v<Dmac::For<Kvasir::Crc::Presets::crc16Kermit,
                                           16>,
                                 Kvasir::Crc::Engine<Kvasir::Crc::Presets::crc16Kermit,
                                                     16>>);

}   // namespace DmaCrc

}   // namespace Kvasir
