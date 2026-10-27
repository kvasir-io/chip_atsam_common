#pragma once
// The SAM D2x/C2x crystal oscillator (XOSC) as a building block: the same fields on both families (D21 SYSCTRL.XOSC,
// C21 OSCCTRL.XOSCCTRL), another register and ready bit. The chip names it through its `Kvasir::XOSC` namespace,
// which keeps the Options, the frequency check and the GAIN table with their data sheet citations:
//
//     using Impl = Sam::Xosc<SYSCTRL::XOSC, SYSCTRL::PCLKSR::xoscrdy, GainTable>;
//
// `Gain::gainFor(hz)` gives the GAIN value for a crystal or clock of `hz` (0..4 on both families).

#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"

#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace Sam {
    template<typename Reg, auto const& ReadyField, typename Gain>
    struct Xosc {
        // XOSC enabled, ONDEMAND off (it runs whether or not a generator asks for it). O: the chip's XOSC::Options.
        template<std::uint64_t Hz,
                 auto          O>
        [[nodiscard]] static constexpr auto configure() {
            using Kvasir::Register::value;
            using G = typename Reg::GAINVal;
            // STARTUP's own value type: an enum where the SVD names the values (C21), the register's word (D21)
            using S = typename std::remove_cvref_t<decltype(Reg::startup)>::DataType;
            return Reg::overrideDefaults(
              write(Reg::startup, value<S, static_cast<S>(O.startup)>()),
              write(Reg::ampgc, value<std::uint16_t, O.ampgc ? 1U : 0U>()),
              write(Reg::gain, value<G, static_cast<G>(Gain::gainFor(Hz))>()),
              clear(Reg::ondemand),
              write(Reg::runstdby, value<std::uint16_t, O.runStandby ? 1U : 0U>()),
              write(Reg::xtalen, value<std::uint16_t, O.crystal ? 1U : 0U>()),
              set(Reg::enable));
        }

        [[nodiscard]] static bool ready() { return apply(read(ReadyField)); }
    };
}}   // namespace Kvasir::Sam
