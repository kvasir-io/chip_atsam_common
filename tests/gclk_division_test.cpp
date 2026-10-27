// GclkDivision.hpp: how a GCLK generator's divisor is written (GENCTRL.DIVSEL + DIV), for the DIV widths of the three
// families (D21: 8 bits, generator 1 16, generator 2 5; C2x/E5x: 8 bits, generator 1 16). All static_asserts:
// building is the test. The values are what the old per-chip lambdas (D21 GCLK.hpp, GclkIndexed.hpp) computed.
#include "../GclkDivision.hpp"

namespace {
using namespace Kvasir::Sam::Gclk;

static_assert(directMaximum(8) == 255 && directMaximum(5) == 31 && directMaximum(16) == 65'535);

// held by DIV: written as is, DIVSEL 0 (0 and 1 are both /1)
static_assert(!divisionOf(0,
                          8)
                 .divsel
              && divisionOf(0,
                            8)
                     .div
                   == 0);
static_assert(!divisionOf(1,
                          8)
                 .divsel
              && divisionOf(1,
                            8)
                     .div
                   == 1);
static_assert(!divisionOf(3,
                          8)
                 .divsel
              && divisionOf(3,
                            8)
                     .div
                   == 3);
static_assert(!divisionOf(255,
                          8)
                 .divsel
              && divisionOf(255,
                            8)
                     .div
                   == 255);
static_assert(!divisionOf(256,
                          16)
                 .divsel
              && divisionOf(256,
                            16)
                     .div
                   == 256);
static_assert(!divisionOf(31,
                          5)
                 .divsel
              && divisionOf(31,
                            5)
                     .div
                   == 31);

// past DIV: DIVSEL 1, 2^(DIV + 1) = the divisor
static_assert(divisionOf(256,
                         8)
                .divsel
              && divisionOf(256,
                            8)
                     .div
                   == 7);
static_assert(divisionOf(512,
                         8)
                .divsel
              && divisionOf(512,
                            8)
                     .div
                   == 8);
static_assert(divisionOf(64,
                         5)
                .divsel
              && divisionOf(64,
                            5)
                     .div
                   == 5);
static_assert(divisionOf(32,
                         5)
                .divsel
              && divisionOf(32,
                            5)
                     .div
                   == 4);
static_assert(divisionOf(131'072,
                         16)
                .divsel
              && divisionOf(131'072,
                            16)
                     .div
                   == 16);
static_assert(divisionOf(1ULL << 20,
                         8)
                .divsel
              && divisionOf(1ULL << 20,
                            8)
                     .div
                   == 19);

// what can be written at all
static_assert(divisionWritable(0,
                               8)
              && divisionWritable(255,
                                  8)
              && divisionWritable(256,
                                  8)
              && divisionWritable(4096,
                                  8));
static_assert(!divisionWritable(257,
                                8)
              && !divisionWritable(300,
                                   8)
              && !divisionWritable(33,
                                   5));
static_assert(divisionWritable(65'535,
                               16)
              && !divisionWritable(65'537,
                                   16));
}   // namespace

int main() { return 0; }
