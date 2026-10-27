#pragma once
// The indexed generic clock controller of the SAM C2x and D5x/E5x: one GENCTRLn register per generator, one PCHCTRLm
// per peripheral channel (the D21's GCLK is the older indirect GENDIV/GENCTRL/CLKCTRL block and stays in its chip).
// The chip's GCLK.hpp names the generated GENCTRL<0>::SRCVal as its GeneratorSource, keeps its Peripheral table
// (the PCHCTRLm index: the SVD has no enumeration for it) and names these through two alias templates with the
// generator's DIV width and the data sheet's maximum division factor from its ClockLimits:
//
//     template<unsigned G, GeneratorSource S, unsigned long long D>
//     using GenericClockGenerator
//       = Sam::Gclk::Generator<ClockLimits::C21::gclkDivBits(G), ClockLimits::C21::gclkMaxDivision(G), G, S, D>;
//     template<unsigned G, Peripheral P> using PeripheralChannelController = Sam::Gclk::Channel<G, P>;

#include "GclkDivision.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/GCLK.hpp"

namespace Kvasir { namespace Sam { namespace Gclk {
    namespace detail {
        // A warning, not an error: what the silicon does with a division above the maximum factor is unmeasured (the
        // chip's ClockLimits cites its table). The note names Generator and Div.
        template<unsigned           Generator,
                 unsigned long long Div>
        [[deprecated(
          "GCLK generator division above the datasheet's Maximum Division Factor; "
          "unmeasured what the part does")]]
        constexpr bool divisionAboveTableMaximum() {
            return true;
        }
    }   // namespace detail

    template<unsigned           DivBits,
             unsigned long long MaxDivision,
             unsigned           G,
             auto               Source,
             unsigned long long Div>
    struct Generator {
        using GC = Kvasir::Peripheral::GCLK::Registers<>::GENCTRL<G>;

        static_assert(
          divisionWritable(Div,
                           DivBits),
          "GCLK division: GENCTRLn.DIV does not hold it, and it is no power of 2 (DIVSEL)");
        static constexpr Division division = divisionOf(Div, DivBits);

        static constexpr bool divisionInTable = Div <= MaxDivision;

        [[nodiscard]] static constexpr auto enable() {
            // Opt-in (-DKVASIR_WARN_GCLK_DIVISION) so firmwares stay warning-free; divisionInTable
            // stays for a check of your own.
#ifdef KVASIR_WARN_GCLK_DIVISION
            if constexpr(!divisionInTable) {
                [[maybe_unused]] constexpr bool warned
                  = detail::divisionAboveTableMaximum<G, Div>();
            }
#endif
            return list(GC::overrideDefaults(
              set(GC::genen),
              write(GC::divsel,
                    Register::value<typename GC::DIVSELVal,
                                    division.divsel ? GC::DIVSELVal::div2 : GC::DIVSELVal::div1>()),
              set(GC::runstdby),
              write(GC::div, Register::value<division.div>()),
              write(
                GC::src,
                Register::value<typename GC::SRCVal, static_cast<typename GC::SRCVal>(Source)>())));
        }
    };

    template<unsigned Generator, auto Peripheral>
    struct Channel {
        using PC
          = Kvasir::Peripheral::GCLK::Registers<>::PCHCTRL<static_cast<unsigned>(Peripheral)>;

        [[nodiscard]] static constexpr auto enable() {
            return list(PC::overrideDefaults(
              set(PC::chen),
              write(PC::gen,
                    Register::value<typename PC::GENVal,
                                    static_cast<typename PC::GENVal>(Generator)>())));
        }
    };
}}}   // namespace Kvasir::Sam::Gclk
