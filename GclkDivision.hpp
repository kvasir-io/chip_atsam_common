#pragma once
// A GCLK generator's division as GENCTRL writes it, register-free (host test tests/gclk_division_test.cpp). The same
// rule on every SAM family: DIVSEL 0 divides by DIV (0 and 1 both mean /1), DIVSEL 1 by 2^(DIV + 1) - SAM D21
// DS40001882L 15.8.4 GENCTRL.DIVSEL (md l.5280-5284), SAM C21 DS60001479M 16.6.2.7 (md l.6028-6030), SAM D5x/E5x
// DS60001507N 14.6.2.7 (md l.6161). A divisor DIV can hold is written as is (0 too: /1), a larger one must be a power
// of two (DIVSEL). Whether the result is within the data sheet's "Maximum Division Factor" is the chip's check.

#include <bit>

namespace Kvasir { namespace Sam { namespace Gclk {
    struct Division {
        bool               divsel;   // GENCTRL.DIVSEL: true = 2^(div + 1)
        unsigned long long div;      // GENDIV.DIV (D21) / GENCTRLn.DIV (C2x, E5x)
    };

    // The largest divisor DIV holds directly, for a DIV field of `divBits` bits.
    consteval unsigned long long directMaximum(unsigned divBits) { return (1ULL << divBits) - 1; }

    // Whether `divisor` can be written at all: DIV holds it, or it is a power of two for DIVSEL.
    consteval bool divisionWritable(unsigned long long divisor,
                                    unsigned           divBits) {
        return divisor <= directMaximum(divBits) || std::has_single_bit(divisor);
    }

    consteval Division divisionOf(unsigned long long divisor,
                                  unsigned           divBits) {
        if(divisor > directMaximum(divBits)) {
            return {.divsel = true,
                    .div    = static_cast<unsigned long long>(std::countr_zero(divisor) - 1)};
        }
        return {.divsel = false, .div = divisor};
    }
}}}   // namespace Kvasir::Sam::Gclk
