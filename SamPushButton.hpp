#pragma once

#include "EIC.hpp"
#include "kvasir/Devices/PushButton.hpp"

namespace Kvasir {
/// A PushButton on an EXTINT line, both edges, pull-up (Config_::pull to change it), the EIC's filter on. Listing it in
/// Startup is all it takes: its pin, its line and its share of the EIC interrupt come with it
/// (an EicBase has to be in the list too). Config_ is PushButton's UserConfig plus
/// `eicPriority`, the NVIC priority of the EIC interrupt (all lines on it must agree).
/// A wrapper that wants its own edge callback takes `EicLineWith<&itsCallback>` as its SubIsrs.
template<typename Clock, typename Pin, std::size_t EventQSize, typename Config_>
struct SamPushButton : Kvasir::PushButton<Clock, Pin, EventQSize, Config_> {
    using Base = Kvasir::PushButton<Clock, Pin, EventQSize, Config_>;
    using Base::handler;

    static_assert(
      requires { Config_::eicPriority; },
      "SamPushButton: the config needs eicPriority, the NVIC priority of the EIC "
      "interrupt");

    // PullUp unless the config says otherwise: PullNone or PullDown for a line with an external
    // pull resistor, which the 20-60 k internal pull-up (SAM C21 DS60001479M RPULL md l.50199)
    // would otherwise fight.
    static constexpr auto pull = [] {
        if constexpr(requires { Config_::pull; }) {
            return Config_::pull;
        } else {
            return Kvasir::Io::PullConfiguration::PullUp;
        }
    }();

    template<auto Callback>
    using EicLineWith = Kvasir::EIC::ExtInt<Pin,
                                            Kvasir::EIC::InterruptType::EdgeBoth,
                                            Callback,
                                            static_cast<int>(Config_::eicPriority),
                                            pull,
                                            true,
                                            false>;
    using EicLine     = EicLineWith<&Base::edgeCallback>;

    using Provides                                = typename EicLine::Provides;
    using Claims                                  = typename EicLine::Claims;
    using SubIsrs                                 = typename EicLine::SubIsrs;
    static constexpr auto initStepPinConfig       = EicLine::initStepPinConfig;
    static constexpr auto initStepPeripheryConfig = EicLine::initStepPeripheryConfig;
};
}   // namespace Kvasir
