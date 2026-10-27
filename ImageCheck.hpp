#pragma once
// The SAM backend of Kvasir::ImageCheck (Kvasir_SDK kvasir/Runtime/ImageCheck.hpp): the Device Service Unit computes
// the CRC-32 over flash itself.
//
//     using ImageCheck = Kvasir::ImageCheck::Checker<Kvasir::ImageCheck::SamDsu<>,
//                                                    Kvasir::ImageCheck::Paced<HW::SystickClock, 50, 2048>>;
//
// The CRC32 command (SAM D21 13.11.3, SAM C20/C21 13.12.3; the D5x/E5x 12.11.3 is the same): ADDR and LENGTH
// word-aligned (AMOD 0), DATA the seed - 0xFFFFFFFF, or "the result of a previous CRC32 calculation if generating a
// common CRC32 of separate memory blocks" - CTRL.CRC starts it, STATUSA.DONE ends it, STATUSA.BERR says a bus error;
// DATA is then the raw value, "complemented to match standard CRC32 implementations or kept non-inverted if used as
// starting point" (polynomial 0xEDB88320 reflected: CRC-32/ISO-HDLC). From the CPU the internal range has no
// restriction (D21 13.11.2.3, C21 13.12.2.3). Bytes before the first aligned word and after the last whole one go
// through Kvasir::Crc::Crc32 on the same running value. SAM D21 errata 1.8.3 ("not functional on RAM") does not
// apply: the image is in flash.
//
// The DSU is write-protected at reset on both (D21: PAC1.WPSET reset 0x000002, bit 1 = DSU, md l.2360; C21:
// PAC.STATUSB reset 0x00000002, bit 1 = DSU, md l.3217); begin() lifts it once - only if set: a second unprotect
// is an error on both (D21: a CPU exception, md l.2110; C21: INTFLAGA.PAC, 11.5.2.6 md l.2632). C21: WRCTRL with
// PERID = 32 * bridge + index = 33 (bridge B, index 1; Table 11-2) and KEY CLR. D21: one PAC per bridge, each its
// own peripheral in the SVD (PAC0..PAC2, one field per protected peripheral, DS40001882L 11.6.2), the DSU on PAC1;
// WPCLR.DSU is write-one-to-clear, a 0 does nothing (11.6.2.2.1).
//
// Synchronous: chunk() waits for DONE (bounded, whatever policy the application injects; a chunk that does not
// finish in TimeoutUs - at the chip's ceiling clock - resets the DSU, CTRL.SWRST - "A running CRC32 operation can be
// canceled by resetting the module" - and fails: the checker drops the pass). The CPU waits meanwhile; how long a DSU
// read of a flash word takes is measured, not given (Sam::DsuCrcTimeoutUs in SamWaitBounds.hpp).
#include "chip/WaitBounds.hpp"
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Runtime/ImageCheck.hpp"
#include "kvasir/Util/Crc.hpp"
#include "peripherals/DSU.hpp"
#if defined(                                                                                \
  KVASIR_CHIP_ATSAMD21)   // the D21 has a PAC per bridge (PAC0..PAC2), the C21/E5x one PAC
    #include "peripherals/PAC1.hpp"
#else
    #include "peripherals/PAC.hpp"
#endif

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace Kvasir::ImageCheck {

template<std::uint32_t TimeoutUs = Kvasir::Chip::Sam::dsuCrcTimeoutUs>
struct SamDsu {
    using Dsu = Kvasir::Peripheral::DSU::Registers<>;

    static void begin() {
        unprotect();
        running_ = 0xFFFF'FFFFU;
    }

    [[nodiscard]] static bool chunk(std::uintptr_t address,
                                    std::size_t    length) {
        auto       a    = static_cast<std::uint32_t>(address);
        auto const end  = a + static_cast<std::uint32_t>(length);
        auto const head = std::min<std::uint32_t>((4U - (a & 3U)) & 3U, end - a);
        bytes(a, head);
        a += head;
        std::uint32_t const words = (end - a) & ~3U;
        if(words != 0) {
            // STATUSA.DONE and BERR are write-one-to-clear (D21 md l.3574, C21 md l.4429)
            apply(reset(Dsu::STATUSA::done, Dsu::STATUSA::berr));
            apply(write(Dsu::ADDR::FULLREGISTER, a));   // AMOD 0: the array
            apply(write(Dsu::LENGTH::FULLREGISTER, words));
            apply(write(Dsu::DATA::FULLREGISTER, running_));
            apply(set(Dsu::CTRL::crc));
            if(!finished()) { return false; }
            running_ = get<0>(apply(read(Dsu::DATA::FULLREGISTER)));
            a += words;
        }
        bytes(a, end - a);
        return true;
    }

    [[nodiscard]] static std::uint32_t result() { return ~running_; }

private:
    using Timeout = Kvasir::Register::MicrosecondsBound<TimeoutUs>;

    static bool finished() {
        if(Kvasir::Register::waitUntil<Timeout, Kvasir::Register::OnTimeout::Report>(
             Kvasir::Register::isSet(Dsu::STATUSA::done)))
        {
            return apply(read(Dsu::STATUSA::berr)) == 0;
        }
        apply(set(Dsu::CTRL::swrst));   // cancels the CRC32
        return false;
    }

    static void bytes(std::uint32_t a,
                      std::uint32_t n) {
        if(n == 0) { return; }
        auto e = Kvasir::Crc::Crc32::resume(~running_);
        for(std::uint32_t i = 0; i != n; ++i) {
            e.update(std::byte{Kvasir::Memory::load<std::uint8_t>(a + i)});
        }
        running_ = ~e.finish();
    }

    // the PAC differs: C21/E5x a WRCTRL key register, D21 a WPCLR/WPSET pair per bridge, each bridge's PAC its own
    // peripheral (PAC0..PAC2) - the other chip's names do not exist, hence #if, not if constexpr.
    static void unprotect() {
#ifdef KVASIR_CHIP_ATSAMD21
        using Pac1 = Kvasir::Peripheral::PAC1::Registers<>;
        if(apply(read(Pac1::WPSET::dsu))) { apply(set(Pac1::WPCLR::dsu)); }
#else
        using Pac0 = Kvasir::Peripheral::PAC::Registers<>;
        if(apply(read(Pac0::STATUSB::dsu_))) {
            // PERID 33: bridge B x 32 + index 1, the DSU
            apply(write(Pac0::WRCTRL::perid, Kvasir::Register::value<std::uint32_t, 33>()),
                  write(Pac0::WRCTRL::KEYValC::clr));
        }
#endif
    }

    static inline std::uint32_t running_{0xFFFF'FFFFU};
};
}   // namespace Kvasir::ImageCheck
