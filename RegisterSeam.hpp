#pragma once
// The SAM register seam: Kvasir_SDK Register/Exec.hpp includes the header a chip package names in
// KVASIR_CHIP_REGISTER_SEAM at its end (cmake/chip.cmake of chip_atsamd21, chip_atsamc21, chip_atsame51), so every TU
// that reaches apply() has it before its first instantiation. No access of its own (ExecuteSeam stays the plain store):
// the PORT's shared states (Register::SharedState).
//
// A pin group's DIR is written through DIR, DIRCLR, DIRSET and DIRTGL, its OUT through OUT, OUTCLR, OUTSET and OUTTGL.
// One step of an apply() runs by register address, highest first (Register/Apply.hpp), not in the order written:
// `makeOutput(pin)` (OUTCLR, Io.hpp) and `set(pin)` (OUTSET) in one list left the pin LOW - OUTSET at +0x18 ran before
// OUTCLR at +0x14 - and `makeOutputInitHigh(pin)` with `clear(pin)` left it high. With these states apply() refuses
// such a list and names the pin; a Startup phase (every entry's initStepPinConfig) is one apply(), so two entries that
// write one pin's level are refused too.
//
// Only the SDK's Tags and Exec, and the PORT's base as a literal per family: this header is read in the middle of the
// register layer (a generated PORT header would include Register.hpp half-way); Io.hpp checks the literals against the
// generated PORT registers. The D21's and C21's IOBUS window (0x60000000) has no generated registers: not covered.
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Register/Exec.hpp"

#include <string_view>

namespace Kvasir::Sam::PortSeam {
#ifdef KVASIR_CHIP_ATSAMD21
// DS40001882L Table 12-1 "Peripherals Configuration Summary": PORT 0x41004400
inline constexpr unsigned baseAddress = 0x41004400;
#elifdef KVASIR_CHIP_ATSAMC21
// DS60001479M Table 12-3 "Peripherals Configuration Summary SAM C21 E/G/J": PORT 0x41000000
inline constexpr unsigned baseAddress = 0x41000000;
#elifdef KVASIR_CHIP_ATSAME51
// DS60001507N Figure 8-1 "Product Mapping": PORT 0x41008000
inline constexpr unsigned baseAddress = 0x41008000;
#else
    #error "RegisterSeam.hpp: no SAM family define (KVASIR_CHIP_ATSAMD21, _ATSAMC21, _ATSAME51)"
#endif

// Groups 0x80 apart, PA = 0, PB = 1, ... (DS40001882L 23.7, DS60001479M 28.7, DS60001507N 32.7). Four reach the most
// pins of the three families (the E5x's 128-pin parts: PA..PD) and stay inside each family's PORT window (the next
// peripheral is 0x400 above the D21's PORT, 0x2000 above the C21's and the E5x's; the same tables).
inline constexpr unsigned groupStride = 0x80;
inline constexpr unsigned groups      = 4;
// in each group (the same register summaries): DIR 0x00, DIRCLR 0x04, DIRSET 0x08, DIRTGL 0x0C, OUT 0x10, OUTCLR 0x14,
// OUTSET 0x18, OUTTGL 0x1C - a state's own register, then its one-to-clear, -set and -toggle registers
inline constexpr unsigned outOffset = 0x10;
inline constexpr unsigned stateSpan = 0x10;

// the address of the DIR or OUT a PORT register writes (its own group's), 0 for any other address
constexpr unsigned stateOf(unsigned address) {
    if(address < baseAddress || address >= baseAddress + groups * groupStride || address % 4 != 0) {
        return 0;
    }
    unsigned const inGroup = (address - baseAddress) % groupStride;
    if(inGroup >= outOffset + stateSpan) { return 0; }
    return address - inGroup % stateSpan;
}

// "PA17, PA18": the pins of `mask` in the group of `state`
consteval void pinNames(Kvasir::Diagnostic::Text& t,
                        unsigned                  state,
                        unsigned                  mask) {
    char const group = static_cast<char>('A' + (state - baseAddress) / groupStride);
    bool       first = true;
    for(unsigned pin = 0; pin < 32; ++pin) {
        if(((mask >> pin) & 1U) == 0) { continue; }
        t << (first ? "P" : ", P") << std::string_view{&group, 1};
        t.dec(pin);
        first = false;
    }
}
}   // namespace Kvasir::Sam::PortSeam

namespace Kvasir::Register {
template<unsigned A, unsigned Z, unsigned O, typename T, typename M>
    requires(Sam::PortSeam::stateOf(A) != 0)
struct SharedState<::Kvasir::Tag::User, Address<A, Z, O, T, M>> {
    static constexpr unsigned value = Sam::PortSeam::stateOf(A);

    // the rest of apply()'s message (Register/Diagnostic.hpp OneStateTwoRegisters): the pins and the fix. Short: the
    // whole message is cut at Kvasir::Diagnostic::Text's 768 characters (about 250 are the SDK's, 300 these)
    static consteval void describe(::Kvasir::Diagnostic::Text& t,
                                   unsigned                    mask) {
        namespace P = Sam::PortSeam;
        t << "SAM PORT ";
        P::pinNames(t, value, mask);
        if((value - P::baseAddress) % P::groupStride == P::outOffset) {
            t << ", its level: makeOutput(pin) writes it low (OUTCLR), "
                 "makeOutputInitHigh(pin) high (OUTSET), a pulled input its pull, "
                 "set/clear/toggle(pin) OUTSET/OUTCLR/OUTTGL. Give it once: "
                 "makeOutputInitHigh(pin) for high, makeOutput(pin) for low, no set()/clear() "
                 "of the pin beside it - or drive the pin in an apply() of its own";
        } else {
            t << ", its direction: makeOutput(pin) sets it (DIRSET), makeInput(pin) and a "
                 "peripheral function without a pull clear it (DIRCLR). Configure the pin "
                 "once per list";
        }
    }
};
}   // namespace Kvasir::Register
