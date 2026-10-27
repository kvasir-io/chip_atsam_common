#pragma once
// The SAM DMAC's transfer descriptor (D21 DS40001882L 20.10, C21 DS60001479M 25.10, E5x DS60001507N 22.10: the same
// 16-byte layout and the same BTCTRL bits on every chip), register-free: a host test builds it (tests/
// dmac_descriptor_test.cpp). Plain memory, the core's until the channel is enabled and again once the channel says it
// is done - fill it, Hal::handToHardware(), start(); the done flag (takeEvent), Hal::takeFromHardware(), then read;
// stopAndWait() takes it back itself (DMAC.hpp).
//
//     Dma::rd<Channel>() = DMAC::DmacDescriptor{{.srcInc = DMAC::DmacDescriptor::srcinc::increment,
//                                                .count  = n,
//                                                .src    = reinterpret_cast<std::uint32_t>(buffer + n),
//                                                .dst    = Regs::DATA8::Addr::value}};
//
// An incrementing address points past the block's end (D21 20.10.3, 20.10.4).
#include "kvasir/Hal/DmaHandoff.hpp"

#include <cstdint>

namespace Kvasir::DMAC {
struct alignas(16) DmacDescriptor {
    enum class Stepsize : unsigned char { x1, x2, x4, x8, x16, x32, x64, x128 };
    enum class Stepsel : unsigned char { dst = 0x00, src = 0x01 };
    enum class Dstinc : unsigned char { noIncrement = 0x00, increment = 0x01 };
    enum class Srcinc : unsigned char { noIncrement = 0x00, increment = 0x01 };
    enum class Beatsize : unsigned char { byte = 0x00, hword = 0x01, word = 0x02 };
    enum class Blockact : unsigned char {
        noact     = 0x00,
        interrupt = 0x01,
        suspend   = 0x02,
        both      = 0x03
    };
    enum class Evosel : unsigned char { disabled = 0x00, block = 0x01, beat = 0x03 };

    // BTCTRL's fields (D21 20.10.1 md l.14588-14681, C21 25.10.1 md l.17542-17635, E5x 22.10.1 md l.18642-18738):
    // STEPSIZE 15:13, STEPSEL 12, DSTINC 11, SRCINC 10, BEATSIZE 9:8, BLOCKACT 4:3, EVOSEL 2:1, VALID 0.
    struct Field {
        unsigned shift;
        unsigned width;

        [[nodiscard,
          gnu::always_inline]] constexpr std::uint16_t
        mask() const {
            return static_cast<std::uint16_t>(((1U << width) - 1U) << shift);
        }

        [[nodiscard,
          gnu::always_inline]] constexpr std::uint16_t
        deposit(unsigned v) const {
            return static_cast<std::uint16_t>((v << shift) & mask());
        }
    };

    struct Btctrl {
        static constexpr Field valid{0, 1};
        static constexpr Field evosel{1, 2};
        static constexpr Field blockact{3, 2};
        static constexpr Field beatsize{8, 2};
        static constexpr Field srcinc{10, 1};
        static constexpr Field dstinc{11, 1};
        static constexpr Field stepsel{12, 1};
        static constexpr Field stepsize{13, 3};
    };

    // The builder is always_inline (Field, btctrlOf, the constructor, setValid): a descriptor of constants is then the
    // constant words an argument-list constructor gives - else -Oz keeps the constructor out of line (78 B) with a
    // Transfer built on the stack at each call, and gcc keeps deposit() and the Field objects as well.
    // One block transfer, by name: what is not named is the reset meaning of the field (no increment, a byte beat,
    // no block action, no event, the step on the destination at one beat).
    struct Transfer {
        bool          valid    = true;
        Stepsize      stepSize = Stepsize::x1;
        Stepsel       stepSel  = Stepsel::dst;
        Dstinc        dstInc   = Dstinc::noIncrement;
        Srcinc        srcInc   = Srcinc::noIncrement;
        Beatsize      beatSize = Beatsize::byte;
        Blockact      blockAct = Blockact::noact;
        Evosel        evOutput = Evosel::disabled;
        std::uint16_t count    = 0;   // BTCNT: beats
        std::uint32_t src      = 0;   // SRCADDR
        std::uint32_t dst      = 0;   // DSTADDR
        std::uint32_t next     = 0;   // DESCADDR: 0 = the last block
    };

    [[nodiscard,
      gnu::always_inline]] static constexpr std::uint16_t
    btctrlOf(Transfer const& t) {
        return static_cast<std::uint16_t>(
          Btctrl::valid.deposit(t.valid ? 1U : 0U) | Btctrl::evosel.deposit(unsigned(t.evOutput))
          | Btctrl::blockact.deposit(unsigned(t.blockAct))
          | Btctrl::beatsize.deposit(unsigned(t.beatSize))
          | Btctrl::srcinc.deposit(unsigned(t.srcInc)) | Btctrl::dstinc.deposit(unsigned(t.dstInc))
          | Btctrl::stepsel.deposit(unsigned(t.stepSel))
          | Btctrl::stepsize.deposit(unsigned(t.stepSize)));
    }

    constexpr DmacDescriptor() = default;

    [[gnu::always_inline]] constexpr explicit DmacDescriptor(Transfer const& t)
      : btctrl{btctrlOf(t)}
      , btcnt{t.count}
      , srcaddr{t.src}
      , dstaddr{t.dst}
      , descaddr{t.next} {}

    // BTCTRL.VALID of a write-back descriptor, which the DMAC writes while the channel runs: one load each call
    // (a caller polls it), Kvasir/Hal/DmaHandoff.hpp
    [[nodiscard]] bool isValid() const {
        return (Hal::loadHardwareField(btctrl) & Btctrl::valid.mask()) != 0;
    }

    [[gnu::always_inline]] constexpr void setValid(bool b) {
        btctrl = static_cast<std::uint16_t>((btctrl & ~Btctrl::valid.mask())
                                            | Btctrl::valid.deposit(b ? 1U : 0U));
    }

    std::uint16_t btctrl{};
    std::uint16_t btcnt{};
    std::uint32_t srcaddr{};
    std::uint32_t dstaddr{};
    std::uint32_t descaddr{};
};

static_assert(sizeof(DmacDescriptor) == 16);
}   // namespace Kvasir::DMAC
