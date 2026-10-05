#pragma once
// The SAM backend of Kvasir::ImageCheck (Kvasir_SDK kvasir/Util/ImageCheck.hpp): the Device Service Unit computes
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
// PERID = 32 * bridge + index = 33 (bridge B, index 1; Table 11-2) and KEY CLR.
//
// Synchronous: chunk() waits for DONE (bounded; a chunk that does not finish in TimeoutPolls resets the DSU,
// CTRL.SWRST - "A running CRC32 operation can be canceled by resetting the module" - and fails: the checker drops
// the pass). The CPU waits meanwhile; how long a DSU read of a flash word takes is measured, not given.
#include "kvasir/Util/Crc.hpp"
#include "kvasir/Util/ImageCheck.hpp"
#include "peripherals/DSU.hpp"
#include "peripherals/PAC.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace Kvasir::ImageCheck {

template<std::uint32_t TimeoutPolls = 1'000'000>
struct SamDsu {
    using Dsu = Kvasir::Peripheral::DSU::Registers<>;

    static void begin() {
        unprotect<SamDsu>();
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
            // STATUSA.DONE (bit 0) and BERR (bit 2) are write-one-to-clear (D21 md l.3574, C21 md l.4429)
            apply(write(Dsu::STATUSA::FULLREGISTER, (1U << 0U) | (1U << 2U)));
            apply(write(Dsu::ADDR::FULLREGISTER, a));   // AMOD 0: the array
            apply(write(Dsu::LENGTH::FULLREGISTER, words));
            apply(write(Dsu::DATA::FULLREGISTER, running_));
            apply(write(Dsu::CTRL::FULLREGISTER, 1U << 2U));   // CTRL.CRC
            if(!finished()) { return false; }
            running_ = get<0>(apply(read(Dsu::DATA::FULLREGISTER)));
            a += words;
        }
        bytes(a, end - a);
        return true;
    }

    [[nodiscard]] static std::uint32_t result() { return ~running_; }

private:
    static bool finished() {
        for(std::uint32_t i = 0; i != TimeoutPolls; ++i) {
            if(apply(read(Dsu::STATUSA::done))) { return !apply(read(Dsu::STATUSA::berr)); }
        }
        apply(write(Dsu::CTRL::FULLREGISTER, 1U << 0U));   // CTRL.SWRST: cancels the CRC32
        return false;
    }

    static void bytes(std::uint32_t a,
                      std::uint32_t n) {
        if(n == 0) { return; }
        auto e = Kvasir::Crc::Crc32::resume(~running_);
        for(std::uint32_t i = 0; i != n; ++i) {
            e.update(std::byte{*reinterpret_cast<std::uint8_t const volatile*>(a + i)});
        }
        running_ = ~e.finish();
    }

    // the PAC differs: C21/E5x a WRCTRL key register, D21 a WPCLR/WPSET pair per bridge. Dep keeps the other
    // chip's branch uninstantiated.
    template<typename Dep>
    static void unprotect() {
        using Pac0 = Kvasir::Peripheral::PAC::Registers<sizeof(Dep) * 0>;
        if constexpr(requires { Pac0::WRCTRL::perid; }) {
            if(apply(read(Pac0::STATUSB::dsu_))) {
                apply(write(Pac0::WRCTRL::FULLREGISTER, (1U << 16U) | 33U));   // KEY CLR, PERID 33
            }
        } else {
            using Pac1 = Kvasir::Peripheral::PAC::Registers<1 + sizeof(Dep) * 0>;
            if((get<0>(apply(read(Pac1::WPSET::FULLREGISTER))) & (1U << 1U)) != 0) {
                apply(write(Pac1::WPCLR::FULLREGISTER, 1U << 1U));
            }
        }
    }

    static inline std::uint32_t running_{0xFFFF'FFFFU};
};
}   // namespace Kvasir::ImageCheck
