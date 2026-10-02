// ClockSolver.hpp on the test limits (TestLimits.hpp): FDPLL settings, the tie-break rules one by
// one, the output prescaler, LDRFRAC, wait-state tables at their edges, the DFLL48M multiplier and
// the message texts. Most checks are static_asserts. The chips' real limits are tested in each chip
// package's tests/.
#include "TestLimits.hpp"

#include <cstdio>

namespace {
using namespace Kvasir;
namespace L = ClockLimits::Test;
using ClockLimits::Supply;

// 8 MHz crystal -> 48 MHz: DCO 48 MHz (the lowest), the reference 1 MHz: 2 MHz (DIV 1) sits on
// the f_IN limit, 1.333 MHz (DIV 2) is not a whole number of hertz
constexpr auto d8 = DPLL::fromXosc<L::Plain>(8'000'000, 48'000'000);
static_assert(d8.found && d8.div == 3 && d8.ldr == 47 && d8.ldrFrac == 0 && d8.presc == 0
              && d8.gclkDiv == 1);
// the value, not the representation: Rational is not reduced
static_assert(d8.achieved.num == 48'000'000 * d8.achieved.den);
static_assert(d8.dco.num == 48'000'000 * d8.dco.den && d8.ref.num == 1'000'000 * d8.ref.den);
// the same numbers checked the other way round
static_assert(DPLL::checkXosc<L::Plain>(8'000'000,
                                        3,
                                        47,
                                        0)
                .ok());

// DIV 128 gives 31 007 Hz, below f_IN
constexpr auto low = DPLL::checkXosc<L::Plain>(8'000'000, 128, 3095, 0, 0, 2);
static_assert(!low.refInRange && low.dcoInRange && low.fieldsFit && !low.ok());
static_assert(low.out.num == 48'000'000 * low.out.den);
// a field wider than its bits
static_assert(!DPLL::checkXosc<L::Plain>(8'000'000,
                                         3,
                                         1U << 12,
                                         0)
                 .fieldsFit);
static_assert(!DPLL::checkXosc<L::Plain>(8'000'000,
                                         3,
                                         47,
                                         16)
                 .fieldsFit);

// 12 MHz: 2 MHz (DIV 2) on the limit, 1.5 MHz (DIV 3) x 32; 16 MHz: 2 MHz (DIV 3) on the limit,
// 1.6 MHz (DIV 4) x 30
constexpr auto d12 = DPLL::fromXosc<L::Plain>(12'000'000, 48'000'000);
static_assert(d12.found && d12.div == 3 && d12.ldr == 31 && d12.gclkDiv == 1);
constexpr auto d16 = DPLL::fromXosc<L::Plain>(16'000'000, 48'000'000);
static_assert(d16.found && d16.div == 4 && d16.ldr == 29 && d16.gclkDiv == 1);
// 8 -> 96 MHz: the DCO at its maximum, 1 MHz x 96
constexpr auto d96 = DPLL::fromXosc<L::Plain>(8'000'000, 96'000'000);
static_assert(d96.found && d96.div == 3 && d96.ldr == 95 && d96.gclkDiv == 1);
// 24 MHz is below the DCO's 48 MHz: 48 MHz and the generator divides by 2
constexpr auto d24 = DPLL::fromXosc<L::Plain>(8'000'000, 24'000'000);
static_assert(d24.found && d24.div == 3 && d24.ldr == 47 && d24.gclkDiv == 2);

// 32 768 Hz straight in: 48 MHz / 32768 = 1464.84 - not exact in integer mode, and not within 1/16
// steps either (1464 + 14/16 = 48.0010 MHz)
static_assert(!DPLL::fromReference<L::Plain>(32'768,
                                             48'000'000)
                 .found);
constexpr auto x32
  = DPLL::fromReference<L::Plain>(32'768, 48'000'000, Prescaler::Tolerance::ppm(100), true);
static_assert(x32.found && x32.ldr == 1463 && x32.ldrFrac == 14);
// ... and in integer mode with a 1000 ppm tolerance: 1465 x 32768 = 48.005 MHz (+107 ppm)
constexpr auto x32i
  = DPLL::fromReference<L::Plain>(32'768, 48'000'000, Prescaler::Tolerance::ppm(1000));
static_assert(x32i.found && x32i.ldr == 1464 && x32i.ldrFrac == 0);
// 32.768 kHz -> 96 MHz: the nearest ratio, 2930, gives 96.010 MHz - above the DCO's maximum
static_assert(!DPLL::fromReference<L::Plain>(32'768,
                                             96'000'000,
                                             Prescaler::Tolerance::ppm(1000))
                 .found);
// a reference below f_IN is refused outright
static_assert(!DPLL::fromReference<L::Plain>(31'000,
                                             48'000'000,
                                             Prescaler::Tolerance::ppm(100'000),
                                             true)
                 .found);
// nothing reaches 200 MHz
static_assert(!DPLL::fromXosc<L::Plain>(8'000'000,
                                        200'000'000)
                 .found);

// the output prescaler: a full tie between it and the generator divider keeps the first
// candidate, PRESC /1
constexpr auto p24 = DPLL::fromXosc<L::WithPresc>(8'000'000, 24'000'000);
static_assert(p24.found && p24.ldr == 47 && p24.presc == 0 && p24.gclkDiv == 2);
// with the generator divider limited to 1 the prescaler does it
constexpr auto p12 = DPLL::fromXosc<L::WithPresc>(8'000'000, 12'000'000, {0, 1}, false, 1);
static_assert(p12.found && p12.ldr == 47 && p12.presc == 2 && p12.gclkDiv == 1);
// ... and without a prescaler nothing does
static_assert(!DPLL::fromXosc<L::Plain>(8'000'000,
                                        12'000'000,
                                        {0,
                                         1},
                                        false,
                                        1)
                 .found);
static_assert(!DPLL::checkXosc<L::WithPresc>(8'000'000,
                                             3,
                                             47,
                                             0,
                                             3)
                 .fieldsFit,
              "PRESC above prescMax");

// LDRFRAC in 1/32: 32 768 Hz -> 120 MHz, 3661 + 5 + 4/32
constexpr auto frac
  = DPLL::fromReference<L::Fine>(32'768, 120'000'000, Prescaler::Tolerance::ppm(10), true);
static_assert(frac.found && frac.ldr == 3661 && frac.ldrFrac == 4);
// 8 MHz -> 120 MHz: DIV 0 (4 MHz) is above f_IN, DIV 1 (2 MHz, whole hertz, not on a limit) x 60
constexpr auto f120 = DPLL::fromXosc<L::Fine>(8'000'000, 120'000'000);
static_assert(f120.found && f120.div == 1 && f120.ldr == 59 && f120.gclkDiv == 1);
// a hand-written setting the solver would not pick is still fine: 1.333 MHz x 90
constexpr auto hand = DPLL::checkXosc<L::Fine>(8'000'000, 2, 89, 0);
static_assert(hand.ok() && hand.out.num == 120'000'000 * hand.out.den);

// the tie-break rules one by one
namespace DD = DPLL::detail;

constexpr DPLL::Setting mk(std::uint64_t refNum,
                           std::uint64_t refDen,
                           std::uint64_t dco,
                           std::uint32_t frac = 0) {
    return {
      0,
      0,
      frac,
      0,
      1,
      {   dco,      1},
      {refNum, refDen},
      {   dco,      1},
      true
    };
}

// integer mode first, whatever the DCO
static_assert(DD::better<L::Plain>(mk(1'000'000,
                                      1,
                                      96'000'000),
                                   mk(1'000'000,
                                      1,
                                      48'000'000,
                                      3),
                                   48'000'000));
// the lower DCO next
static_assert(DD::better<L::Plain>(mk(1'000'000,
                                      1,
                                      48'000'000),
                                   mk(2'000'000,
                                      1,
                                      96'000'000),
                                   48'000'000));
// a reference inside the range beats one on its limit (2 MHz), and one on the lower limit too
static_assert(DD::better<L::Plain>(mk(500'000,
                                      1,
                                      48'000'000),
                                   mk(2'000'000,
                                      1,
                                      48'000'000),
                                   48'000'000));
static_assert(DD::better<L::Plain>(mk(40'000,
                                      1,
                                      48'000'000),
                                   mk(32'000,
                                      1,
                                      48'000'000),
                                   48'000'000));
// a whole number of hertz beats a higher fraction
static_assert(DD::better<L::Plain>(mk(1'000'000,
                                      1,
                                      48'000'000),
                                   mk(8'000'000,
                                      6,
                                      48'000'000),
                                   48'000'000));
// then the higher reference; equal values in other representations are a tie, not "better"
static_assert(DD::better<L::Plain>(mk(1'000'000,
                                      1,
                                      48'000'000),
                                   mk(500'000,
                                      1,
                                      48'000'000),
                                   48'000'000));
static_assert(!DD::better<L::Plain>(mk(2'000'000,
                                       2,
                                       48'000'000),
                                    mk(1'000'000,
                                       1,
                                       48'000'000),
                                    48'000'000));

// wait states at every row edge
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from2V7,
                              24'000'000)
              == 0);
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from2V7,
                              24'000'001)
              == 1);
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from2V7,
                              48'000'000)
              == 1);
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from2V7,
                              48'000'001)
              == Nvm::NoWaitStateCount);
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from1V62,
                              14'000'000)
              == 0);
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from1V62,
                              14'000'001)
              == 1);
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from1V62,
                              28'000'001)
              == 2);
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from1V62,
                              42'000'001)
              == 3);
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from1V62,
                              48'000'000)
              == 3);
// a supply the table does not have, and the empty second row of a one-row table
static_assert(Nvm::waitStates(L::WaitStates,
                              Supply::from4V5,
                              1'000'000)
              == Nvm::NoWaitStateCount);
static_assert(Nvm::waitStates(L::WaitStatesOneRow,
                              Supply::from1V62,
                              1)
              == Nvm::NoWaitStateCount);
static_assert(Nvm::waitStates(L::WaitStatesOneRow,
                              Supply::from2V7,
                              40'000'000)
              == 1);
// the asserted forms compile for good numbers
static_assert(Nvm::waitStatesFrom<48'000'000,
                                  Supply::from1V62,
                                  L::WaitStates>()
              == 3);

consteval bool enoughWaitStates() {
    Nvm::assertWaitStatesFrom<48'000'000, Supply::from1V62, 3, L::WaitStates>();
    return true;
}

static_assert(enoughWaitStates());

// DFLL48M closed loop on 32 768 Hz: MUL 1465, 48 005 120 Hz; a 40 kHz reference is out of range
constexpr auto dfll = DFLL::closedLoop<L::Dfll48>(32'768, 48'000'000);
static_assert(dfll.mul == 1465 && dfll.achieved.num == 48'005'120 * dfll.achieved.den
              && dfll.refInRange && dfll.mulFits);
static_assert(!DFLL::closedLoop<L::Dfll48>(40'000,
                                           48'000'000)
                 .refInRange);
static_assert(!DFLL::closedLoop<L::Dfll48>(732,
                                           48'000'000)
                 .mulFits,
              "65574 needs 17 bits");
// the step limits: 50 % of COARSE (6 bits) and FINE (10 bits)
static_assert(DFLL::MaxCoarseStep<L::Dfll48> == 31 && DFLL::MaxFineStep<L::Dfll48> == 511);

// the messages, as text (the must-fail tests check that the compiler prints them)
static_assert(Nvm::WaitStateMessage<48'000'000,
                                    Supply::from1V62,
                                    1,
                                    L::WaitStates>{}
                .view()
              == "flash wait states: 48000000 Hz at 1.62-2.7 V needs 3 (test table), RWS is 1");
static_assert(Nvm::WaitStateMessage<49'000'000,
                                    Supply::from2V7,
                                    0,
                                    L::WaitStates>{}
                .view()
              == "flash wait states: no count for 49000000 Hz at 2.7 V and above in test table");
static_assert(ClockLimits::RangeMessage<Prescaler::Rational{8'000'000,
                                                            258},
                                        32'000,
                                        2'000'000,
                                        "FDPLL reference",
                                        L::PlainWhere>{}
                .view()
              == "FDPLL reference: wanted 32000..2000000 Hz, got 31007.8 Hz (test DPLL)");
static_assert(DPLL::NotFoundMessage<L::PlainWhere,
                                    8'000'000,
                                    200'000'000,
                                    0,
                                    false>{}
                .view()
              == "test DPLL: no setting makes 200000000 Hz from 8000000 Hz (integer mode, 0 ppm)");

// the asserted forms compile for good numbers
static_assert(DPLL::solveXosc<L::Plain,
                              L::PlainWhere,
                              8'000'000,
                              48'000'000,
                              0,
                              false>()
                .ldr
              == 47);
static_assert(DPLL::solveReference<L::Plain,
                                   L::PlainWhere,
                                   32'768,
                                   48'000'000,
                                   100,
                                   true>()
                .ldrFrac
              == 14);
static_assert(DPLL::assertXoscSetting<L::Plain,
                                      L::PlainWhere,
                                      8'000'000,
                                      3,
                                      47,
                                      0>()
                .ok());
}   // namespace

int main() {
    std::puts("clock solver: every check is a static_assert");
    return 0;
}
