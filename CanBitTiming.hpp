#pragma once
// The M_CAN's nominal bit timing from a clock, a bit rate and a sample point (CAN.hpp's
// CANBehavior asserts it). Standard headers and Kvasir/Util/Prescaler.hpp only: host-tested in
// tests/can_bit_timing_test.cpp.
#include "kvasir/Util/Prescaler.hpp"

#include <algorithm>
#include <cstdint>

namespace Kvasir { namespace CAN { namespace Detail {
    /// The M_CAN's nominal bit timing (NBTP; SAM C20/C21 DS60001479 and D5x/E5x data sheets, CAN
    /// NBTP): a time quantum is NBRP + 1 GCLK_CAN periods, a bit is 1 + (NTSEG1 + 1) + (NTSEG2
    /// + 1) quanta, 4 to 385; NBRP 0..511, NTSEG1 1..255, NTSEG2 0..127, NSJW 0..127, each field
    /// used as one more than written.
    struct BitTiming {
        std::uint32_t       nbrp{};
        std::uint32_t       ntseg1{};
        std::uint32_t       ntseg2{};
        std::uint32_t       nsjw{};
        Prescaler::Rational achieved{};
        bool                found{};
    };

    // Not constexpr: reaching it in bitTiming() is the compile error.
    inline void canPhaseSeg2UnderThreeQuantaRefusedMoveTheSamplePointEarlier() {}

    /// NBTP.NTSEG2 0 and 1 (phase segment 2 of one or two quanta) are refused. NTSEG2 = 0 is the
    /// erratum: "When NBTP.NTSEG2 is configured to zero (Phase_Seg2(N) = 1), and when there is a
    /// pending transmission request, a dominant third bit of Intermission may cause the CAN to
    /// wrongly transmit the first identifier bit dominant instead of recessive"; no workaround
    /// (SAM C20/C21 errata DS80000740T 1.6.14; SAM D5x/E5x errata DS80000748T 2.4.2, the same text,
    /// there "update configuration range of NBTP.NTSEG2 from 0..127 to 1..127"). NTSEG2 = 1 is
    /// refused too, one quantum of margin above it (ISO 11898-1's minimum range for Phase_Seg2 is
    /// 2..8 tq). The note "in call to 'refuseNtseg2BelowTwo(...)'" has the numbers.
    constexpr void refuseNtseg2BelowTwo(std::uint32_t clock,
                                        std::uint32_t baud,
                                        std::uint32_t samplePointPermille) {
        static_cast<void>(clock);
        static_cast<void>(baud);
        static_cast<void>(samplePointPermille);
        canPhaseSeg2UnderThreeQuantaRefusedMoveTheSamplePointEarlier();
    }

    /// The prescaler whose bit is closest to `baud` in rate, the smallest prescaler (most
    /// quanta) on a tie; the sample point at `samplePointPermille` of the bit (sync segment
    /// included), rounded to a quantum; the jump width as wide as phase segment 2. 8 MHz,
    /// 500 kbit/s, 750 give the reset value 0x06000A03 (16 quanta, 12 before the sample point).
    /// A timing with NTSEG2 below 2 does not compile (refuseNtseg2BelowTwo).
    consteval BitTiming bitTiming(std::uint32_t clock,
                                  std::uint32_t baud,
                                  std::uint32_t samplePointPermille) {
        BitTiming best{};
        for(std::uint64_t pre = 1; pre <= 512; ++pre) {
            std::uint64_t const quanta = (clock + pre * baud / 2) / (pre * baud);
            if(quanta < 4 || quanta > 385) { continue; }
            std::uint64_t const beforeSample = (quanta * samplePointPermille + 500) / 1000;
            std::uint64_t const seg2 = std::clamp<std::uint64_t>(quanta - beforeSample, 1, 128);
            std::uint64_t const seg1 = quanta - 1 - seg2;
            if(seg1 < 2 || seg1 > 256) { continue; }
            Prescaler::Rational const achieved{clock, pre * quanta};
            auto const                error = [&](Prescaler::Rational const& r) {
                // |r - baud| * den, compared across candidates as |num - baud den| / den
                return r.num > baud * r.den ? r.num - baud * r.den : baud * r.den - r.num;
            };
            if(best.found
               && error(achieved) * best.achieved.den >= error(best.achieved) * achieved.den)
            {
                continue;
            }
            best = BitTiming{.nbrp     = static_cast<std::uint32_t>(pre - 1),
                             .ntseg1   = static_cast<std::uint32_t>(seg1 - 1),
                             .ntseg2   = static_cast<std::uint32_t>(seg2 - 1),
                             .nsjw     = static_cast<std::uint32_t>(std::min(seg1, seg2) - 1),
                             .achieved = achieved,
                             .found    = true};
        }
        if(best.found && best.ntseg2 < 2) {
            refuseNtseg2BelowTwo(clock, baud, samplePointPermille);
        }
        return best;
    }
}}}   // namespace Kvasir::CAN::Detail
