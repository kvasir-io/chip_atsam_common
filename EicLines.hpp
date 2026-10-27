#pragma once
// The EXTINT line a pad drives on peripheral function A (EIC), per chip family. Each chip package picks its rule in
// its own chip/EIC_Traits.hpp (Kvasir::EIC::Traits::extIntOf), which EIC.hpp includes. Plain functions of a port
// (0 = A, 1 = B, ...) and a pin number, nothing else: host-testable alone (tests/eic_lines_test.cpp).

namespace Kvasir { namespace EIC { namespace Lines {
    // the pad drives no EXTINT line (the NMI pad, or a pad the family does not have)
    inline constexpr unsigned none = 16;
    // the pad's port is not encoded in this rule (see the rule's comment)
    inline constexpr unsigned notEncoded = 17;

    // SAM D21 and C21: pin % 16, except PA24/PA25 -> 12/13, PA27 -> 15, PA28 -> 8, PA30/PA31 -> 10/11, and PA08
    // is the NMI (SAM D21 Table 7-1, md 1032-1093; SAM C21 Table 6-2, md 1253-1316).
    consteval unsigned samD2xC2x(int port,
                                 int pin) {
        if(port == 0) {
            switch(pin) {
            case 8:  return none;
            case 24: return 12;
            case 25: return 13;
            case 27: return 15;
            case 28: return 8;
            case 30: return 10;
            case 31: return 11;
            default: break;
            }
        }
        return static_cast<unsigned>(pin % 16);
    }

    // SAM D5x/E5x (DS60001507N Table 6-1, md l.1278-1430 and PDF pages 30 and 34 for the rows the .md has as
    // picture text): port A is pin % 16 with PA08 the NMI (PA26, PA28 and PA29 do not exist in the family); port
    // B is pin % 16 except PB26..PB29 -> 12..15 (PB24/25 -> 8/9 and PB30/31 -> 14/15 are pin % 16). So PA24 -> 8,
    // PA25 -> 9, PA27 -> 11, PA30 -> 14, PA31 -> 15 - the D21/C21 table gives other lines for these. Ports C and D
    // (100/128-pin parts) have their own exceptions (PC07 -> 9, PD08 -> 3, ...) and are not encoded.
    consteval unsigned samE5x(int port,
                              int pin) {
        if(port == 0) {
            switch(pin) {
            case 8:
            case 26:
            case 28:
            case 29: return none;
            default: break;
            }
        } else if(port == 1) {
            if(pin >= 26 && pin <= 29) { return static_cast<unsigned>(pin - 14); }
        } else {
            return notEncoded;
        }
        return static_cast<unsigned>(pin % 16);
    }
}}}   // namespace Kvasir::EIC::Lines
