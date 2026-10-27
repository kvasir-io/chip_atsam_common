#pragma once
// The SAM D2x/C2x FDPLL96M as a building block: the same fields on both families (D21 SYSCTRL, C21 OSCCTRL:
// DPLLRATIO.LDR/LDRFRAC, DPLLCTRLB.DIV/LBYPASS/REFCLK, DPLLCTRLA.ENABLE/RUNSTDBY/ONDEMAND, DPLLSTATUS.CLKRDY); the C21
// adds the output prescaler DPLLPRESC (HasPresc). The numbers come from the solver (ClockSolver.hpp); the chip's
// `Kvasir::DPLL` namespace keeps the solver calls, its Options (the errata defaults with their citations) and the
// names firmwares spell:
//
//     using Impl = Sam::Fdpll96m<Peripheral::SYSCTRL::Registers<>, /*HasPresc*/ false>;
//     using Reference = Impl::Reference;
//
// The D5x/E5x FDPLL200M is another block (two instances, other register names) and stays in its chip.

#include "chip/atsam_common/ClockSolver.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"

#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace Sam {
    template<typename Regs, bool HasPresc>
    struct Fdpll96m {
        // DPLLCTRLB.REFCLK: XOSC32K, XOSC (through DIV), GCLK_DPLL (D21 DS40001882L 17.8.19, md line 9498; C21
        // DS60001479M 20.8.14)
        enum class Reference : unsigned { xosc32k = 0, xosc = 1, gclk = 2 };

        // DPLLRATIO, DPLLCTRLB, the output prescaler when it divides, then DPLLCTRLA with ENABLE and ONDEMAND off; the
        // caller waits for ready() and only then switches a generator over (D21 17.6.8.4, md line 7433).
        // O: the chip's DPLL::Options (lockBypass, runStandby).
        template<DPLL::Setting S,
                 Reference     R,
                 auto          O>
        [[nodiscard]] static constexpr auto configure() {
            static_assert(S.found, "DPLL::configure: a setting the solver did not find");
            static_assert(HasPresc || S.presc == 0, "this FDPLL96M has no output prescaler");
            static_assert(R == Reference::xosc || S.div == 0,
                          "DIV only divides the XOSC reference");
            using Kvasir::Register::value;
            using RV         = typename Regs::DPLLCTRLB::REFCLKVal;
            auto const ratio = list(write(Regs::DPLLRATIO::ldr, value<S.ldr>()),
                                    write(Regs::DPLLRATIO::ldrfrac, value<S.ldrFrac>()));
            // an option bit is written 1 or 0; 0 is its reset value, so the word is the one a missing write gave
            using B = typename std::remove_cvref_t<decltype(Regs::DPLLCTRLB::lbypass)>::DataType;
            using A = typename std::remove_cvref_t<decltype(Regs::DPLLCTRLA::runstdby)>::DataType;
            auto const ctrlb = Regs::DPLLCTRLB::overrideDefaults(
              write(Regs::DPLLCTRLB::div, value<S.div>()),
              write(Regs::DPLLCTRLB::lbypass, value<B, O.lockBypass ? 1U : 0U>()),
              write(Regs::DPLLCTRLB::refclk, value<RV, static_cast<RV>(R)>()));
            auto const ctrla = Regs::DPLLCTRLA::overrideDefaults(
              set(Regs::DPLLCTRLA::enable),
              write(Regs::DPLLCTRLA::runstdby, value<A, O.runStandby ? 1U : 0U>()),
              clear(Regs::DPLLCTRLA::ondemand));
            if constexpr(S.presc == 0) {
                return list(ratio, ctrlb, Kvasir::Register::sequencePoint, ctrla);
            } else {
                using PV = typename Regs::DPLLPRESC::PRESCVal;
                return list(ratio,
                            ctrlb,
                            write(Regs::DPLLPRESC::presc, value<PV, static_cast<PV>(S.presc)>()),
                            Kvasir::Register::sequencePoint,
                            ctrla);
            }
        }

        // CLK_FDPLL96M runs (DPLLSTATUS.CLKRDY; with LBYPASS the lock bit no longer gates it)
        [[nodiscard]] static bool ready() { return apply(read(Regs::DPLLSTATUS::clkrdy)); }
    };
}}   // namespace Kvasir::Sam
