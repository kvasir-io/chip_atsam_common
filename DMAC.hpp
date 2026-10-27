#pragma once
#include "chip/PM.hpp"
#include "chip/Package.hpp"
#include "chip/WaitBounds.hpp"
#include "chip/atsam_common/DmacCrc.hpp"   // Kvasir::DmaCrc, the CRC engine of the same block
#include "chip/atsam_common/DmacDescriptor.hpp"
#include "kvasir/Hal/Dma.hpp"
#include "kvasir/Hal/DmaHandoff.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/DMAC.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <utility>

namespace Kvasir { namespace DMAC {
    // A channel number; a channel past the chip's (D21/C21 12, E5x 32: DmacTraits::Channels) is refused where a
    // DmaBase names it (rd(), start(), Claims: it has no descriptor).
    enum class DMAChannel {
        ch0  = 0,
        ch1  = 1,
        ch2  = 2,
        ch3  = 3,
        ch4  = 4,
        ch5  = 5,
        ch6  = 6,
        ch7  = 7,
        ch8  = 8,
        ch9  = 9,
        ch10 = 10,
        ch11 = 11,
        ch12 = 12,
        ch13 = 13,
        ch14 = 14,
        ch15 = 15,
        ch16 = 16,
        ch17 = 17,
        ch18 = 18,
        ch19 = 19,
        ch20 = 20,
        ch21 = 21,
        ch22 = 22,
        ch23 = 23,
        ch24 = 24,
        ch25 = 25,
        ch26 = 26,
        ch27 = 27,
        ch28 = 28,
        ch29 = 29,
        ch30 = 30,
        ch31 = 31,
    };

    enum class DMAPriority {
        p0 = 0,
        p1,
        p2,
        p3,
    };

    using TriggerSource = Sam::Package::Dmac::TriggerSource;

    namespace Detail {
        template<unsigned Channel>
        using ChannelResource = Hal::DmaChannel<Peripheral::DMAC::Registers<>, Channel>;

        template<typename Seq>
        struct ChannelResources;

        template<std::size_t... Is>
        struct ChannelResources<std::index_sequence<Is...>> {
            using type = brigand::list<ChannelResource<static_cast<unsigned>(Is)>...>;
        };

        // The DMAC's registers as a name that depends on the channel: start()/stop()/takeEvent() name the D2x/C2x
        // window (CHID, CHCTRLB) in one if-constexpr arm and the E5x's CHANNEL<n> in the other, and only an arm whose
        // names are dependent is left unchecked when it is discarded.
        template<typename Regs, auto>
        struct DependentOn {
            using type = Regs;
        };

        template<std::size_t N>
        constexpr bool allDistinct(std::array<unsigned,
                                              N> const& v) {
            for(std::size_t i = 0; i < N; ++i) {
                for(std::size_t j = i + 1; j < N; ++j) {
                    if(v[i] == v[j]) { return false; }
                }
            }
            return true;
        }
    }   // namespace Detail

    // What a driver puts in its `Claims`: `Kvasir::DMAC::Claims<Dma, TxChannel, RxChannel>`. The
    // two checks a driver can make on its own are made here, with the driver in the error; the
    // cross-peripheral ones (two drivers on a channel, no DmaBase listed) are Startup's.
    template<typename Dma, DMAChannel... Cs>
    struct ClaimsFor {
        static_assert(
          ((static_cast<std::size_t>(Cs) < Dma::numberOfChannels) && ...),
          "a driver's DMA channel is not one of its DmaBase's channels (numberOfChannels): "
          "it has no descriptor");
        static_assert(Detail::allDistinct(std::array<unsigned,
                                                     sizeof...(Cs)>{static_cast<unsigned>(Cs)...}),
                      "a driver names the same DMA channel twice");
        using type = brigand::list<Detail::ChannelResource<static_cast<unsigned>(Cs)>...>;
    };

    template<typename Dma, DMAChannel... Cs>
    using Claims = typename ClaimsFor<Dma, Cs...>::type;

    template<typename DMAConfig>
    struct DmaBase {
        // needed config
        // numberOfChannels
        static constexpr auto numberOfChannels = DMAConfig::numberOfChannels;

        static_assert(Sam::Package::Dmac::channels >= numberOfChannels,
                      "DMAConfig::numberOfChannels: more DMA channels than this chip has "
                      "(Sam::Package::Dmac::Channels: D21/C21 12, E5x 32)");

        static inline std::array<DmacDescriptor, numberOfChannels> rdescriptors{};
        static inline std::array<DmacDescriptor, numberOfChannels> wdescriptors{};

        using Regs                   = Peripheral::DMAC::Registers<>;
        static constexpr unsigned ba = Regs::baseAddr;

        // Startup: the controller (one DmaBase per image) and its channels 0..numberOfChannels-1;
        // a driver claims the ones it runs on (DMAC::Claims below), so two drivers on one channel,
        // or a driver on a channel past numberOfChannels or without a listed DmaBase, do not build.
        using Provides = brigand::append<
          brigand::list<Hal::Owns<Regs>>,
          typename Detail::ChannelResources<std::make_index_sequence<numberOfChannels>>::type>;

        static constexpr auto powerClockEnable = list(typename PM::Enable<ba>::action{});

        template<DMAChannel Channel>
        static constexpr DmacDescriptor& rd() {
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));
            return rdescriptors[static_cast<std::size_t>(Channel)];
        }

        template<DMAChannel Channel>
        static constexpr DmacDescriptor& wd() {
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));
            return wdescriptors[static_cast<std::size_t>(Channel)];
        }

        [[gnu::always_inline]] static void runtimeInit() {
            apply(
              write(Regs::BASEADDR::baseaddr, reinterpret_cast<std::uint32_t>(rdescriptors.data())),
              write(Regs::WRBADDR::wrbaddr, reinterpret_cast<std::uint32_t>(wdescriptors.data())));
            apply(Regs::CTRL::overrideDefaults(set(Regs::CTRL::dmaenable),
                                               set(Regs::CTRL::lvlen0),
                                               set(Regs::CTRL::lvlen1),
                                               set(Regs::CTRL::lvlen2),
                                               set(Regs::CTRL::lvlen3)));
        }

        template<DMAChannel    Channel,
                 DMAPriority   Priority,
                 TriggerSource Trigger>
        static constexpr auto start() {
            using R = typename Detail::DependentOn<Regs, Channel>::type;
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));

            if constexpr(Sam::Package::Dmac::channelSelectedByChid) {
                return list(
                  write(
                    R::CHID::id,
                    Kvasir::Register::value<std::uint8_t, static_cast<std::uint8_t>(Channel)>()),
                  Kvasir::Register::sequencePoint,
                  write(R::CHCTRLB::CMDValC::noact),
                  write(R::CHCTRLB::TRIGACTValC::beat),
                  write(R::CHCTRLB::trigsrc,
                        Register::value<typename R::CHCTRLB::TRIGSRCVal, Trigger>()),
                  write(R::CHCTRLB::lvl,
                        Register::value<typename R::CHCTRLB::LVLVal,
                                        static_cast<typename R::CHCTRLB::LVLVal>(
                                          static_cast<int>(R::CHCTRLB::LVLVal::lvl0)
                                          + static_cast<int>(Priority))>()),
                  clear(R::CHCTRLB::evoe),
                  clear(R::CHCTRLB::evie),
                  write(R::CHCTRLB::EVACTValC::noact),
                  Kvasir::Register::sequencePoint,
                  set(R::CHCTRLA::enable),
                  clear(R::CHCTRLA::swrst));
            } else {
                using CHRegs = typename R::template CHANNEL<static_cast<std::size_t>(Channel)>;
                return list(
                  CHRegs::CHCTRLA::overrideDefaults(),

                  Kvasir::Register::sequencePoint,
                  CHRegs::CHCTRLA::overrideDefaults(set(CHRegs::CHCTRLA::swrst)),

                  Kvasir::Register::sequencePoint,
                  write(CHRegs::CHPRILVL::prilvl,
                        Register::value<typename CHRegs::CHPRILVL::PRILVLVal,
                                        static_cast<typename CHRegs::CHPRILVL::PRILVLVal>(
                                          static_cast<int>(CHRegs::CHPRILVL::PRILVLVal::lvl0)
                                          + static_cast<int>(Priority))>()),

                  Kvasir::Register::sequencePoint,
                  CHRegs::CHCTRLA::overrideDefaults(
                    write(CHRegs::CHCTRLA::TRIGACTValC::burst),
                    write(CHRegs::CHCTRLA::trigsrc,
                          Register::value<typename CHRegs::CHCTRLA::TRIGSRCVal,
                                          static_cast<typename CHRegs::CHCTRLA::TRIGSRCVal>(
                                            Trigger)>())),
                  Kvasir::Register::sequencePoint,
                  set(CHRegs::CHCTRLA::enable));
            }
        }

        /// Disable one channel: what an aborted transfer needs, so the channel stops moving
        /// bytes and the next start() finds it idle.
        template<DMAChannel Channel>
        static constexpr auto stop() {
            using R = typename Detail::DependentOn<Regs, Channel>::type;
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));

            if constexpr(Sam::Package::Dmac::channelSelectedByChid) {
                return list(
                  write(
                    R::CHID::id,
                    Kvasir::Register::value<std::uint8_t, static_cast<std::uint8_t>(Channel)>()),
                  Kvasir::Register::sequencePoint,
                  clear(R::CHCTRLA::enable));
            } else {
                using CHRegs = typename R::template CHANNEL<static_cast<std::size_t>(Channel)>;
                return list(clear(CHRegs::CHCTRLA::enable));
            }
        }

        /// stop(), then wait until the channel really is off, so a following start() or a
        /// peripheral reset does not race the last beat. A disabled channel finishes its beat and
        /// its write-back first: CHCTRLA.ENABLE reads 1 until then (Sam::DmaChannelStopBound,
        /// whatever policy the application injects: the caller recovers). Returns false if it
        /// never was off - the channel is wedged.
        template<DMAChannel Channel>
        static bool stopAndWait() {
            using R = typename Detail::DependentOn<Regs, Channel>::type;
            apply(stop<Channel>());
            // one CHID write per poll on the D21/C21, as before
            auto const off = [] {
                if constexpr(Sam::Package::Dmac::channelSelectedByChid) {
                    apply(write(
                      R::CHID::id,
                      Kvasir::Register::value<std::uint8_t, static_cast<std::uint8_t>(Channel)>()));
                    return apply(read(R::CHCTRLA::enable)) == 0;
                } else {
                    using CHRegs = typename R::template CHANNEL<static_cast<std::size_t>(Channel)>;
                    return apply(read(CHRegs::CHCTRLA::enable)) == 0;
                }
            };
            bool const stopped = static_cast<bool>(
              Kvasir::Register::waitUntil<Kvasir::Chip::Sam::DmaChannelStopBound,
                                          Kvasir::Register::OnTimeout::Report>(off));
            // the descriptors and buffers are the core's again (Kvasir/Hal/DmaHandoff.hpp)
            Hal::takeFromHardware();
            return stopped;
        }

        enum class ChannelEvent : std::uint8_t { none, complete, error };

        /// Reads and clears CHINTFLAG TERR/TCMPL; set even with the interrupt off (DS40001882L
        /// 20.6.5). Uses CHID on the D21/C21. A caller that goes on to read what the channel wrote
        /// calls Hal::takeFromHardware() after an event first.
        template<DMAChannel Channel>
        static ChannelEvent takeEvent() {
            using R = typename Detail::DependentOn<Regs, Channel>::type;
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));
            if constexpr(Sam::Package::Dmac::channelSelectedByChid) {
                apply(write(
                  R::CHID::id,
                  Kvasir::Register::value<std::uint8_t, static_cast<std::uint8_t>(Channel)>()));
                return takeFlags<typename R::CHINTFLAG>();
            } else {
                using CHRegs = typename R::template CHANNEL<static_cast<std::size_t>(Channel)>;
                return takeFlags<typename CHRegs::CHINTFLAG>();
            }
        }

    private:
        // TERR and TCMPL as read, written back whole: a runtime mask of W1C flags (SUSP stays)
        template<typename Flags>
        [[gnu::always_inline]] static ChannelEvent takeFlags() {
            auto const flags = static_cast<std::uint8_t>(get<0>(apply(read(Flags::FULLREGISTER)))
                                                         & (Flags::terr.mask | Flags::tcmpl.mask));
            if(flags != 0) { apply(write(Flags::FULLREGISTER, flags)); }
            if((flags & Flags::terr.mask) != 0) { return ChannelEvent::error; }
            if((flags & Flags::tcmpl.mask) != 0) { return ChannelEvent::complete; }
            return ChannelEvent::none;
        }
    };

}}   // namespace Kvasir::DMAC
