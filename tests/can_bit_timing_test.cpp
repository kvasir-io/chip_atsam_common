// CanBitTiming.hpp: the M_CAN's nominal bit timing (NBTP) for a clock, a bit rate and a sample point. NBTP
// fields as the data sheet has them (SAM C20/C21 DS60001479M 34.8.8; SAM D5x/E5x the same register): NSJW
// bits 31:25, NBRP 24:16, NTSEG1 15:8, NTSEG2 6:0, each used as one more than written; a bit is
// 1 + (NTSEG1 + 1) + (NTSEG2 + 1) quanta of NBRP + 1 GCLK_CAN periods, 4 to 385. Every check is a
// static_assert (bitTiming is consteval). Needs Kvasir_SDK's Util/Prescaler.hpp only.
#include "../CanBitTiming.hpp"

#include <array>
#include <cstdint>
#include <cstdio>

namespace {
using Kvasir::CAN::Detail::BitTiming;
using Kvasir::CAN::Detail::bitTiming;

constexpr std::uint32_t nbtp(BitTiming const& t) {
    return (t.nsjw << 25U) | (t.nbrp << 16U) | (t.ntseg1 << 8U) | t.ntseg2;
}

constexpr std::uint64_t quanta(BitTiming const& t) { return 3U + t.ntseg1 + t.ntseg2; }

constexpr bool hasRate(BitTiming const& t,
                       std::uint64_t    clock,
                       std::uint64_t    baud) {
    return t.achieved.num == clock && t.achieved.den == (t.nbrp + 1U) * quanta(t)
        && clock == baud * t.achieved.den;
}

// 8 MHz, 500 kbit/s, 75 %: the register's reset value 0x06000A03 - 16 quanta, 12 before the sample
// point (NSJW 3, NBRP 0, NTSEG1 10, NTSEG2 3). From the bit table's reset row and the Note under it
// (DS60001479M 34.8.8, "With a CAN clock (GCLK_CAN) of 8 MHz, the reset value 0x06000A03 configures
// the CAN for a bit rate of 500 kBits/s"; D5x/E5x DS60001507 NBTP says the same). The register's
// header line in the converted C21 sheet says "Reset: 0x00000A33": that line contradicts its own bit
// table and is not used.
constexpr auto reset = bitTiming(8'000'000, 500'000, 750);
static_assert(reset.found && reset.nbrp == 0 && reset.ntseg1 == 10 && reset.ntseg2 == 3
              && reset.nsjw == 3);
static_assert(nbtp(reset) == 0x06000A03U);
static_assert(hasRate(reset,
                      8'000'000,
                      500'000));

// 48 MHz: 1 Mbit/s and 500 kbit/s fit with prescaler 1 (48 and 96 quanta); 125 kbit/s would need 384
// quanta and a phase segment 1 of 287 > 256 at prescaler 1, so it takes prescaler 2 (192 quanta)
constexpr auto m1 = bitTiming(48'000'000, 1'000'000, 750);
static_assert(m1.found && m1.nbrp == 0 && m1.ntseg1 == 34 && m1.ntseg2 == 11 && m1.nsjw == 11);
static_assert(hasRate(m1,
                      48'000'000,
                      1'000'000));
constexpr auto k500 = bitTiming(48'000'000, 500'000, 750);
static_assert(k500.found && k500.nbrp == 0 && k500.ntseg1 == 70 && k500.ntseg2 == 23
              && k500.nsjw == 23);
static_assert(hasRate(k500,
                      48'000'000,
                      500'000));
constexpr auto k125 = bitTiming(48'000'000, 125'000, 750);
static_assert(k125.found && k125.nbrp == 1 && k125.ntseg1 == 142 && k125.ntseg2 == 47
              && k125.nsjw == 47);
static_assert(hasRate(k125,
                      48'000'000,
                      125'000));

// a tie: 48 MHz / 1 Mbit/s is exact with prescalers 1, 2, 3, 4, 6, 8 and 12 - the smallest wins
// (the most quanta)
static_assert(bitTiming(48'000'000,
                        1'000'000,
                        750)
                .nbrp
              == 0);
// not exact anywhere: 8 MHz / 300 kbit/s, 27 quanta (296 296 bit/s) at prescaler 1 and prescaler 3
// (9 quanta) are equally close; prescaler 1 is kept
constexpr auto k300 = bitTiming(8'000'000, 300'000, 750);
static_assert(k300.found && k300.nbrp == 0 && quanta(k300) == 27);
static_assert(k300.achieved.num == 8'000'000 && k300.achieved.den == 27);

// no timing: fewer than 4 quanta even at prescaler 1, or more than 385 at prescaler 512
static_assert(!bitTiming(8'000'000,
                         3'000'000,
                         750)
                 .found);
static_assert(!bitTiming(8'000'000,
                         10,
                         750)
                 .found);

// NTSEG2 0 and 1 (phase segment 2 of one or two quanta) are refused at compile time. NTSEG2 = 0 is
// SAM C20/C21 errata DS80000740T 1.6.14, SAM D5x/E5x errata DS80000748T 2.4.2 ("When NBTP.NTSEG2 is
// configured to zero (Phase_Seg2(N) = 1), ... a dominant third bit of Intermission may cause the CAN
// to wrongly transmit the first identifier bit dominant instead of recessive"); NTSEG2 = 1 is
// refused as one quantum of margin. The refused settings are must_fail cases (CMakeLists.txt):
// 8 MHz, 1 Mbit/s, 87.5 % (8 quanta, 7 before the sample point: NTSEG2 0); a sample point at the
// very end of the bit (16 quanta, 15 before: NTSEG2 0); 8 MHz, 500 kbit/s, 87.5 % (16 quanta, 14
// before: NTSEG2 1); 8 MHz, 1 Mbit/s, 75 % (8 quanta, 6 before: NTSEG2 1).
// What is just accepted, phase segment 2 of three quanta (NTSEG2 2):
// - 8 MHz, 500 kbit/s at 81.2 %: 16 quanta, 13 before the sample point (81.25 %) - the latest
//   sample point that 16 quanta of 8 MHz allow at 500 kbit/s
constexpr auto k500at812 = bitTiming(8'000'000, 500'000, 812);
static_assert(k500at812.found && k500at812.nbrp == 0 && k500at812.ntseg1 == 11
              && k500at812.ntseg2 == 2 && k500at812.nsjw == 2 && quanta(k500at812) == 16);
static_assert(hasRate(k500at812,
                      8'000'000,
                      500'000));
// - 8 MHz, 1 Mbit/s at 62.5 %: 8 quanta, 5 before the sample point
constexpr auto m1at625 = bitTiming(8'000'000, 1'000'000, 625);
static_assert(m1at625.found && m1at625.ntseg1 == 3 && m1at625.ntseg2 == 2);
// - 8 MHz, 500 kbit/s at 84.3 %: 13.49 quanta before the sample point round to 13, 3 after
static_assert(bitTiming(8'000'000,
                        500'000,
                        843)
                .ntseg2
              == 2);

// Every found timing over a grid up to maxBaud: fields in their ranges, NTSEG2 at least 2, a bit of
// 4..385 quanta, phase segment 1 at least 2 quanta, the jump width the shorter phase segment, the
// achieved rate what the fields give.
consteval bool gridHolds(std::uint32_t clock,
                         std::uint32_t samplePermille,
                         std::uint32_t maxBaud = 1'000'000) {
    constexpr std::array<std::uint32_t, 10>
      bauds{10'000, 20'000, 50'000, 83'333, 100'000, 125'000, 250'000, 500'000, 800'000, 1'000'000};
    for(auto const baud : bauds) {
        if(baud > maxBaud) { continue; }
        auto const t = bitTiming(clock, baud, samplePermille);
        if(!t.found) { continue; }
        if(t.nbrp > 511 || t.ntseg1 < 1 || t.ntseg1 > 255 || t.ntseg2 < 2 || t.ntseg2 > 127
           || t.nsjw > 127)
        {
            return false;
        }
        if(quanta(t) < 4 || quanta(t) > 385) { return false; }
        if(t.nsjw != (t.ntseg1 < t.ntseg2 ? t.ntseg1 : t.ntseg2)) { return false; }
        if(t.achieved.num != clock || t.achieved.den != (t.nbrp + 1U) * quanta(t)) { return false; }
    }
    return true;
}

// 8 MHz leaves too few quanta after the sample point at the fast rates: at 75 % 800 kbit/s (10
// quanta, 8 before) and 1 Mbit/s (8, 6); at 87.5 % 500 kbit/s (16, 14) and above - refused
static_assert(gridHolds(8'000'000,
                        750,
                        500'000));
static_assert(gridHolds(8'000'000,
                        875,
                        250'000));
static_assert(gridHolds(16'000'000,
                        500));
static_assert(gridHolds(24'000'000,
                        625));
static_assert(gridHolds(48'000'000,
                        750));
static_assert(gridHolds(48'000'000,
                        900));
}   // namespace

int main() {
    std::puts("can bit timing: every check is a static_assert");
    return 0;
}
