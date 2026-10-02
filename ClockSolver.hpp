#pragma once
// The SAM clock numbers from targets instead of by hand:
// FDPLL settings for a wanted core clock, the flash wait states for a frequency and supply, the
// DFLL48M closed-loop multiplier, and the compile-time messages that print the numbers when one of
// them does not fit. Every datasheet number comes from the chip's ClockLimits.hpp; this file knows
// none. Standard headers and Kvasir/Util/Prescaler.hpp only: host-tested in tests/.
//
//     constexpr auto dpll = Kvasir::DPLL::fromXosc<Limits::Fdpll96m>(8'000'000, 48'000'000);
//     // {div 3, ldr 47, ldrFrac 0, presc 0, gclkDiv 1}: 8 MHz / (2 x 4) = 1 MHz x 48 = 48 MHz
//     constexpr unsigned ws = Kvasir::Nvm::waitStates(Limits::WaitStates85C, Supply::from2V7,
//                                                     48'000'000);   // 1
//
// The chip packages wrap these in templates that assert (chip/ClockLimits.hpp, chip/Dpll.hpp).

#include "kvasir/Util/Prescaler.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir {

namespace ClockLimits {
    // The FDPLL of a chip: f_ref = f_xosc / (2 (DIV + 1)) on the crystal path;
    // f_dco = f_ref (LDR + 1 + LDRFRAC / 2^fracBits); f_out = f_dco / 2^PRESC (C21 only).
    struct Dpll {
        std::uint64_t refMin;   // f_IN
        std::uint64_t refMax;
        std::uint64_t outMin;   // f_OUT of the DCO
        std::uint64_t outMax;
        unsigned      ldrBits;
        unsigned      fracBits;
        unsigned      divBits;
        unsigned      prescMax;   // the output prescaler's largest exponent; 0: none
    };

    // The supply ranges the wait-state tables are split by (each chip uses the ones it has)
    enum class Supply : std::uint8_t {
        from1V62,   // D21: 1.62 V to 2.7 V
        from1V71,   // E5x: 1.71 V to 2.7 V (and the EFP table: above 1.71 V)
        from2V7,    // D21: 2.7 V to 3.63 V; C21 and E5x: above 2.7 V
        from4V5,    // C21: above 4.5 V
    };

    constexpr std::string_view supplyName(Supply s) {
        switch(s) {
        case Supply::from1V62: return "1.62-2.7 V";
        case Supply::from1V71: return "1.71-2.7 V";
        case Supply::from2V7:  return "2.7 V and above";
        case Supply::from4V5:  return "4.5 V and above";
        }
        return "?";
    }

    // the maximum frequency for 0, 1, 2, ... wait states at one supply range; 0 ends the list
    struct WaitStateRow {
        Supply                       supply;
        std::array<std::uint64_t, 8> maxHz;
    };

    // one datasheet table: its name for the messages, its rows
    struct WaitStateTable {
        std::string_view            name;
        std::array<WaitStateRow, 2> rows;
    };

    struct Dfll {
        std::uint64_t refMin;   // closed-loop reference
        std::uint64_t refMax;
        unsigned      mulBits;
        unsigned      coarseBits;   // DFLLVAL.COARSE
        unsigned      fineBits;     // DFLLVAL.FINE
    };
}   // namespace ClockLimits

namespace DPLL {
    struct Setting {
        std::uint32_t       div{};        // DPLLCTRLB.DIV (XOSC reference only)
        std::uint32_t       ldr{};        // DPLLRATIO.LDR
        std::uint32_t       ldrFrac{};    // DPLLRATIO.LDRFRAC
        std::uint32_t       presc{};      // DPLLPRESC.PRESC (C21): the DCO / 2^presc
        std::uint32_t       gclkDiv{};    // the generic clock generator after the DPLL
        Prescaler::Rational achieved{};   // the core clock
        Prescaler::Rational ref{};        // f_ref
        Prescaler::Rational dco{};        // the DCO output
        bool                found{};

        constexpr bool integerMode() const { return ldrFrac == 0; }
    };

    namespace detail {
        // a < b as values (a Rational is not reduced, so == on the members is not equality)
        constexpr bool less(Prescaler::Rational a,
                            Prescaler::Rational b) {
            return Prescaler::mulChecked(a.num, b.den) < Prescaler::mulChecked(b.num, a.den);
        }

        constexpr bool atLimit(Prescaler::Rational r,
                               std::uint64_t       lo,
                               std::uint64_t       hi) {
            return r.num == Prescaler::mulChecked(lo, r.den)
                || r.num == Prescaler::mulChecked(hi, r.den);
        }

        constexpr bool wholeHz(Prescaler::Rational r) { return r.num % r.den == 0; }

        // Candidate b is better than a. The rules, in order:
        //  1. integer mode (LDRFRAC = 0): "the fractional part has a negative impact on the jitter"
        //     (SAM D21 17.6.8.3, C21 20.6.5, E5x 28.6.5);
        //  2. the lowest DCO: less current, and on the D21 the FDPLL96M "operation above 64 MHz is
        //     not functional below 0 C" (errata DS80000760M 1.3.2);
        //  3. a reference strictly inside f_IN over one on a table limit (2 MHz is the D21's
        //     maximum, Table 37-58);
        //  4. a reference that is a whole number of hertz (1 MHz over 1.333... MHz): easier to
        //     check by hand;
        //  5. the highest reference: the shorter lock (Table 37-58: 1.3 ms at 32 kHz, 25 us at
        //     2 MHz);
        //  6. the smaller error. A full tie keeps the first candidate (smaller DIV, then smaller
        //     PRESC, then smaller generator divider).
        template<ClockLimits::Dpll L>
        constexpr bool better(Setting const& b,
                              Setting const& a,
                              std::uint64_t  want) {
            if(!a.found) { return true; }
            if(a.integerMode() != b.integerMode()) { return b.integerMode(); }
            if(less(b.dco, a.dco) || less(a.dco, b.dco)) { return less(b.dco, a.dco); }
            bool const aEdge = atLimit(a.ref, L.refMin, L.refMax);
            bool const bEdge = atLimit(b.ref, L.refMin, L.refMax);
            if(aEdge != bEdge) { return !bEdge; }
            if(wholeHz(a.ref) != wholeHz(b.ref)) { return wholeHz(b.ref); }
            if(less(b.ref, a.ref) || less(a.ref, b.ref)) { return less(a.ref, b.ref); }
            return Prescaler::closer(b.achieved, a.achieved, want);
        }

        // one reference, every output prescaler and generator divider: the loop ratio in
        // 1/2^fracBits steps, nearest
        template<ClockLimits::Dpll L>
        constexpr Setting forReference(Prescaler::Rational  ref,
                                       std::uint32_t        div,
                                       std::uint64_t        want,
                                       Prescaler::Tolerance tol,
                                       bool                 allowFractional,
                                       std::uint32_t        maxGclkDiv) {
            Setting    best{};
            auto const unit = std::uint64_t{1} << L.fracBits;
            for(std::uint32_t p = 0; p <= L.prescMax; ++p) {
                for(std::uint32_t g = 1; g <= maxGclkDiv; ++g) {
                    // the DCO the target needs: want * 2^p * g; ratio = dco / ref in 1/unit,
                    // rounded to nearest (whole steps of `unit` in integer mode)
                    auto const post = Prescaler::mulChecked(std::uint64_t{1} << p, g);
                    auto const num  = Prescaler::mulChecked(Prescaler::mulChecked(want, post),
                                                            Prescaler::mulChecked(ref.den, unit));
                    auto const step
                      = allowFractional ? ref.num : Prescaler::mulChecked(ref.num, unit);
                    auto const r = (num + step / 2) / step * (allowFractional ? 1 : unit);
                    if(r < unit) { continue; }
                    auto const ldr  = r / unit - 1;
                    auto const frac = r % unit;
                    if(ldr >= (std::uint64_t{1} << L.ldrBits) || (frac != 0 && !allowFractional)) {
                        continue;
                    }
                    Prescaler::Rational const dco{Prescaler::mulChecked(ref.num, r),
                                                  Prescaler::mulChecked(ref.den, unit)};
                    if(Prescaler::below(dco, L.outMin) || Prescaler::above(dco, L.outMax)) {
                        continue;
                    }
                    Prescaler::Rational const out{dco.num, Prescaler::mulChecked(dco.den, post)};
                    if(!Prescaler::inTolerance(out, want, tol)) { continue; }
                    Setting const s{div,
                                    static_cast<std::uint32_t>(ldr),
                                    static_cast<std::uint32_t>(frac),
                                    p,
                                    g,
                                    out,
                                    ref,
                                    dco,
                                    true};
                    if(better<L>(s, best, want)) { best = s; }
                }
            }
            return best;
        }
    }   // namespace detail

    // A crystal through DPLLCTRLB.DIV. Exact unless a tolerance is given; fractional only when
    // asked.
    template<ClockLimits::Dpll L>
    constexpr Setting fromXosc(std::uint64_t        xoscHz,
                               std::uint64_t        want,
                               Prescaler::Tolerance tol             = {0,
                                                                       1},
                               bool                 allowFractional = false,
                               std::uint32_t        maxGclkDiv      = 8) {
        Setting best{};
        for(std::uint32_t div = 0; div < (std::uint32_t{1} << L.divBits); ++div) {
            Prescaler::Rational const ref{xoscHz, 2 * (std::uint64_t{div} + 1)};
            if(Prescaler::below(ref, L.refMin)) { break; }   // falls with DIV
            if(Prescaler::above(ref, L.refMax)) { continue; }
            auto const s
              = detail::forReference<L>(ref, div, want, tol, allowFractional, maxGclkDiv);
            if(s.found && detail::better<L>(s, best, want)) { best = s; }
        }
        return best;
    }

    // A reference taken directly (XOSC32K, a GCLK): no DIV.
    template<ClockLimits::Dpll L>
    constexpr Setting fromReference(std::uint64_t        refHz,
                                    std::uint64_t        want,
                                    Prescaler::Tolerance tol             = {0,
                                                                            1},
                                    bool                 allowFractional = false,
                                    std::uint32_t        maxGclkDiv      = 8) {
        Prescaler::Rational const ref{refHz, 1};
        if(Prescaler::below(ref, L.refMin) || Prescaler::above(ref, L.refMax)) { return {}; }
        return detail::forReference<L>(ref, 0, want, tol, allowFractional, maxGclkDiv);
    }

    // A setting written by hand, measured against the limits: what it makes and where it breaks
    // them. For a firmware that keeps its own numbers (smart_hive) and the messages below.
    struct Check {
        Prescaler::Rational ref{};
        Prescaler::Rational dco{};
        Prescaler::Rational out{};
        bool                refInRange{};
        bool                dcoInRange{};
        bool                fieldsFit{};

        constexpr bool ok() const { return refInRange && dcoInRange && fieldsFit; }
    };

    template<ClockLimits::Dpll L>
    constexpr Check checkXosc(std::uint64_t xoscHz,
                              std::uint32_t div,
                              std::uint32_t ldr,
                              std::uint32_t ldrFrac,
                              std::uint32_t presc   = 0,
                              std::uint32_t gclkDiv = 1) {
        auto const                unit = std::uint64_t{1} << L.fracBits;
        Prescaler::Rational const ref{xoscHz, 2 * (std::uint64_t{div} + 1)};
        auto const                r = (std::uint64_t{ldr} + 1) * unit + ldrFrac;
        Prescaler::Rational const dco{Prescaler::mulChecked(ref.num, r),
                                      Prescaler::mulChecked(ref.den, unit)};
        Prescaler::Rational const out{
          dco.num,
          Prescaler::mulChecked(dco.den,
                                Prescaler::mulChecked(std::uint64_t{1} << presc, gclkDiv))};
        return {ref,
                dco,
                out,
                !Prescaler::below(ref, L.refMin) && !Prescaler::above(ref, L.refMax),
                !Prescaler::below(dco, L.outMin) && !Prescaler::above(dco, L.outMax),
                div < (std::uint32_t{1} << L.divBits) && ldr < (std::uint32_t{1} << L.ldrBits)
                  && ldrFrac < unit && presc <= L.prescMax};
    }
}   // namespace DPLL

namespace Nvm {
    inline constexpr unsigned NoWaitStateCount = 0xFF;

    // The fewest wait states that allow hz at this supply; NoWaitStateCount when the table has
    // none (a static_assert at the caller).
    constexpr unsigned waitStates(ClockLimits::WaitStateTable const& table,
                                  ClockLimits::Supply                supply,
                                  std::uint64_t                      hz) {
        for(auto const& row : table.rows) {
            if(row.supply != supply || row.maxHz[0] == 0) { continue; }
            for(unsigned ws = 0; ws < row.maxHz.size() && row.maxHz[ws] != 0; ++ws) {
                if(hz <= row.maxHz[ws]) { return ws; }
            }
        }
        return NoWaitStateCount;
    }

    // "flash wait states: 48000000 Hz at 1.62-2.7 V needs 3 (Table 37-42 ...), RWS is 1", or
    // "... has no wait-state count for 49000000 Hz at 2.7 V and above (Table 37-42 ...)"
    template<std::uint64_t Hz, ClockLimits::Supply S, unsigned Rws, auto const& Table>
    struct WaitStateMessage {
        static constexpr Prescaler::detail::Writer storage = [] {
            Prescaler::detail::Writer w{};
            auto const                need = waitStates(Table, S, Hz);
            w.text("flash wait states: ");
            if(need == NoWaitStateCount) {
                w.text("no count for ").integer(Hz).text(" Hz at ").text(supplyName(S));
                w.text(" in ").text(Table.name);
            } else {
                w.integer(Hz).text(" Hz at ").text(supplyName(S)).text(" needs ").integer(need);
                w.text(" (").text(Table.name).text("), RWS is ").integer(Rws);
            }
            return w;
        }();

        constexpr std::size_t size() const { return storage.n; }

        constexpr char const* data() const { return storage.out.data(); }

        constexpr std::string_view view() const { return {data(), size()}; }
    };

    // the count from the table, asserted to exist
    template<std::uint64_t       Hz,
             ClockLimits::Supply S,
             auto const&         Table>
    consteval unsigned waitStatesFrom() {
        constexpr unsigned ws = waitStates(Table, S, Hz);
        static_assert(ws != NoWaitStateCount, WaitStateMessage<Hz, S, 0, Table>{});
        return ws;
    }

    // a count written by hand, asserted to be enough
    template<std::uint64_t       Hz,
             ClockLimits::Supply S,
             unsigned            Rws,
             auto const&         Table>
    consteval void assertWaitStatesFrom() {
        constexpr unsigned ws = waitStates(Table, S, Hz);
        static_assert(ws != NoWaitStateCount && Rws >= ws, WaitStateMessage<Hz, S, Rws, Table>{});
    }
}   // namespace Nvm

namespace DFLL {
    struct ClosedLoop {
        std::uint32_t       mul{};
        Prescaler::Rational achieved{};
        bool                refInRange{};
        bool                mulFits{};
    };

    // DFLLMUL.MUL = round(target / ref); f = MUL x ref
    template<ClockLimits::Dfll L>
    constexpr ClosedLoop closedLoop(std::uint64_t refHz,
                                    std::uint64_t target) {
        auto const mul = (target + refHz / 2) / refHz;
        return {
          static_cast<std::uint32_t>(mul),
          {Prescaler::mulChecked(mul, refHz), 1},
          refHz >= L.refMin && refHz <= L.refMax,
          mul < (std::uint64_t{1}
          << L.mulBits)
        };
    }

    // CSTEP and FSTEP "should not be higher than 50% of the maximum value of DFLLVAL.COARSE and
    // DFLLVAL.FINE" (SAM D21 DS40001882L 17.6.7.1.2 step 2, md line 7253; E5x DS60001507N
    // 28.6.4.1 "Closed-Loop Operation" step 2, md line 31813)
    template<ClockLimits::Dfll L>
    inline constexpr std::uint32_t MaxCoarseStep = ((std::uint32_t{1} << L.coarseBits) - 1) / 2;
    template<ClockLimits::Dfll L>
    inline constexpr std::uint32_t MaxFineStep = ((std::uint32_t{1} << L.fineBits) - 1) / 2;
}   // namespace DFLL

namespace ClockLimits {
    // "<what>: wanted <lo>..<hi> Hz, got <x> Hz (<context>)" for a frequency out of a table range
    template<Prescaler::Rational    Got,
             std::uint64_t          Lo,
             std::uint64_t          Hi,
             Prescaler::FixedString What,
             Prescaler::FixedString Where>
    struct RangeMessage {
        static constexpr Prescaler::detail::Writer storage = [] {
            Prescaler::detail::Writer w{};
            w.text(What.view()).text(": wanted ").integer(Lo).text("..").integer(Hi);
            w.text(" Hz, got ").fixed1(Got).text(" Hz (").text(Where.view()).text(")");
            return w;
        }();

        constexpr std::size_t size() const { return storage.n; }

        constexpr char const* data() const { return storage.out.data(); }

        constexpr std::string_view view() const { return {data(), size()}; }
    };

    template<Prescaler::Rational    Got,
             std::uint64_t          Lo,
             std::uint64_t          Hi,
             Prescaler::FixedString What,
             Prescaler::FixedString Where>
    consteval void assertInRange() {
        static_assert(!Prescaler::below(Got, Lo) && !Prescaler::above(Got, Hi),
                      RangeMessage<Got, Lo, Hi, What, Where>{});
    }
}   // namespace ClockLimits

namespace DPLL {
    // "<Where>: no setting makes 48000000 Hz from 8000000 Hz (integer mode, 0 ppm)"
    template<Prescaler::FixedString Where,
             std::uint64_t          In,
             std::uint64_t          Want,
             std::uint64_t          TolerancePpm,
             bool                   Fractional>
    struct NotFoundMessage {
        static constexpr Prescaler::detail::Writer storage = [] {
            Prescaler::detail::Writer w{};
            w.text(Where.view()).text(": no setting makes ").integer(Want).text(" Hz from ");
            w.integer(In).text(" Hz (").text(Fractional ? "fractional allowed" : "integer mode");
            w.text(", ").integer(TolerancePpm).text(" ppm)");
            return w;
        }();

        constexpr std::size_t size() const { return storage.n; }

        constexpr char const* data() const { return storage.out.data(); }

        constexpr std::string_view view() const { return {data(), size()}; }
    };

    // the solver, asserted: a crystal through DIV
    template<ClockLimits::Dpll      L,
             Prescaler::FixedString Where,
             std::uint64_t          XoscHz,
             std::uint64_t          Hz,
             std::uint64_t          TolerancePpm,
             bool                   Fractional>
    consteval Setting solveXosc() {
        constexpr auto s
          = fromXosc<L>(XoscHz, Hz, Prescaler::Tolerance::ppm(TolerancePpm), Fractional);
        static_assert(s.found, NotFoundMessage<Where, XoscHz, Hz, TolerancePpm, Fractional>{});
        return s;
    }

    // ... a reference taken directly, which must be inside f_IN itself
    template<ClockLimits::Dpll      L,
             Prescaler::FixedString Where,
             std::uint64_t          RefHz,
             std::uint64_t          Hz,
             std::uint64_t          TolerancePpm,
             bool                   Fractional>
    consteval Setting solveReference() {
        ClockLimits::assertInRange<Prescaler::Rational{RefHz, 1},
                                   L.refMin,
                                   L.refMax,
                                   "FDPLL reference",
                                   Where>();
        constexpr auto s
          = fromReference<L>(RefHz, Hz, Prescaler::Tolerance::ppm(TolerancePpm), Fractional);
        static_assert(s.found, NotFoundMessage<Where, RefHz, Hz, TolerancePpm, Fractional>{});
        return s;
    }

    // A setting written by hand, asserted against the limits:
    // "FDPLL reference: wanted 32000..2000000 Hz, got 31007.8 Hz (SAM D21 Tables 37-58/59/60)"
    template<ClockLimits::Dpll      L,
             Prescaler::FixedString Where,
             std::uint64_t          XoscHz,
             std::uint32_t          Div,
             std::uint32_t          Ldr,
             std::uint32_t          LdrFrac,
             std::uint32_t          Presc   = 0,
             std::uint32_t          GclkDiv = 1>
    consteval Check assertXoscSetting() {
        constexpr auto c = checkXosc<L>(XoscHz, Div, Ldr, LdrFrac, Presc, GclkDiv);
        static_assert(c.fieldsFit, "FDPLL: DIV, LDR, LDRFRAC or PRESC wider than its field");
        ClockLimits::assertInRange<c.ref, L.refMin, L.refMax, "FDPLL reference", Where>();
        ClockLimits::assertInRange<c.dco, L.outMin, L.outMax, "FDPLL DCO", Where>();
        return c;
    }
}   // namespace DPLL

}   // namespace Kvasir
