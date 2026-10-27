// EicLines.hpp: the EXTINT line of every port A/B pad, both rules, against tables written out row by row. samE5x from
// SAM D5x/E5x Table 6-1 (DS60001507N md l.1278-1430, PDF pages 30 and 34). samD2xC2x keeps the values the D21/C21
// switch has had since it was written from SAM D21 Table 7-1 / SAM C21 Table 6-2 (the rows here are that switch, so a
// change to it shows; PA26 and PA29 are pin % 16 there). Needs nothing but the standard library.
#include "../EicLines.hpp"

#include <array>
#include <cstdio>

namespace {
namespace L          = Kvasir::EIC::Lines;
constexpr unsigned n = L::none;

// index = pin number 0..31
using Row = std::array<unsigned, 32>;

// D21/C21 port A: PA08 NMI, PA24/25 -> 12/13, PA27 -> 15, PA28 -> 8, PA30/31 -> 10/11
constexpr Row d2xPortA{0, 1, 2, 3, 4, 5, 6, 7, n,  9,  10, 11, 12, 13, 14, 15,
                       0, 1, 2, 3, 4, 5, 6, 7, 12, 13, 10, 15, 8,  13, 10, 11};
// D21/C21 port B: pin % 16 throughout
constexpr Row d2xPortB{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                       0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

// E5x port A: PA08 NMI; PA26, PA28, PA29 are not in the table (no such pad); PA24 -> 8, PA25 -> 9, PA27 -> 11,
// PA30 -> 14, PA31 -> 15
constexpr Row e5xPortA{0, 1, 2, 3, 4, 5, 6, 7, n, 9, 10, 11, 12, 13, 14, 15,
                       0, 1, 2, 3, 4, 5, 6, 7, 8, 9, n,  11, n,  n,  14, 15};
// E5x port B: PB22/23 -> 6/7, PB24/25 -> 8/9, PB26..29 -> 12..15, PB30/31 -> 14/15
constexpr Row e5xPortB{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                       0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 14, 15, 14, 15};

consteval unsigned d2x(int port,
                       int pin) {
    return L::samD2xC2x(port, pin);
}

consteval unsigned e5x(int port,
                       int pin) {
    return L::samE5x(port, pin);
}

enum class Rule { d2x, e5x };

consteval bool matches(Rule       rule,
                       int        port,
                       Row const& row) {
    for(int pin = 0; pin != 32; ++pin) {
        auto const line = rule == Rule::d2x ? d2x(port, pin) : e5x(port, pin);
        if(line != row[static_cast<unsigned>(pin)]) { return false; }
    }
    return true;
}

static_assert(matches(Rule::d2x,
                      0,
                      d2xPortA));
static_assert(matches(Rule::d2x,
                      1,
                      d2xPortB));
static_assert(matches(Rule::e5x,
                      0,
                      e5xPortA));
static_assert(matches(Rule::e5x,
                      1,
                      e5xPortB));

// the pads whose line the two families disagree on, spelled out
static_assert(e5x(0,
                  24)
                == 8
              && d2x(0,
                     24)
                   == 12);
static_assert(e5x(0,
                  25)
                == 9
              && d2x(0,
                     25)
                   == 13);
static_assert(e5x(0,
                  27)
                == 11
              && d2x(0,
                     27)
                   == 15);
static_assert(e5x(0,
                  30)
                == 14
              && d2x(0,
                     30)
                   == 10);
static_assert(e5x(0,
                  31)
                == 15
              && d2x(0,
                     31)
                   == 11);
static_assert(e5x(1,
                  22)
                == 6
              && e5x(1,
                     23)
                   == 7);
static_assert(e5x(0,
                  8)
                == n
              && d2x(0,
                     8)
                   == n);
// ports C and D of the E5x are not encoded
static_assert(e5x(2,
                  7)
                == L::notEncoded
              && e5x(3,
                     8)
                   == L::notEncoded);
}   // namespace

int main() {
    std::puts("EIC lines (SAM D21/C21, D5x/E5x): every check is a static_assert");
    return 0;
}
