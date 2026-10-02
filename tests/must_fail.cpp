// One compile error per MUST_FAIL value; CMakeLists.txt checks that the compiler prints the message
// with the numbers. Without MUST_FAIL the file compiles (and is not built).
#include "TestLimits.hpp"

namespace L = Kvasir::ClockLimits::Test;
using Kvasir::ClockLimits::Supply;

#if MUST_FAIL == 1
// a hand-written RWS too small for the supply
consteval void f() {
    Kvasir::Nvm::assertWaitStatesFrom<48'000'000, Supply::from1V62, 1, L::WaitStates>();
}
#elif MUST_FAIL == 2
// a frequency the table has no row for
constexpr unsigned ws = Kvasir::Nvm::waitStatesFrom<49'000'000, Supply::from2V7, L::WaitStates>();
#elif MUST_FAIL == 3
// DIV 128: reference 31 007.75 Hz
constexpr auto c
  = Kvasir::DPLL::assertXoscSetting<L::Plain, L::PlainWhere, 8'000'000, 128, 3095, 0>();
#elif MUST_FAIL == 4
// nothing reaches 200 MHz
constexpr auto s
  = Kvasir::DPLL::solveXosc<L::Plain, L::PlainWhere, 8'000'000, 200'000'000, 0, false>();
#elif MUST_FAIL == 5
// a reference taken directly, above f_IN
constexpr auto s
  = Kvasir::DPLL::solveReference<L::Plain, L::PlainWhere, 4'000'000, 48'000'000, 0, false>();
#elif MUST_FAIL == 6
// a field wider than its bits
constexpr auto c = Kvasir::DPLL::assertXoscSetting<L::Plain, L::PlainWhere, 8'000'000, 3, 47, 16>();
#elif MUST_FAIL == 7
// a DFLL closed loop outside the tolerance asked for: 1465 x 32768 Hz
consteval void f() {
    constexpr auto loop = Kvasir::DFLL::closedLoop<L::Dfll48>(32'768, 48'000'000);
    Kvasir::Prescaler::assertInTolerance<loop.achieved,
                                         48'000'000,
                                         Kvasir::Prescaler::Tolerance::ppm(100),
                                         "DFLL48M closed loop">();
}
#endif

int main() { return 0; }
