#pragma once

#include "EIC.hpp"
#include "kvasir/Devices/PushButton.hpp"

#include <cstddef>

namespace Kvasir {
namespace detail {
    /// A button's Startup share. Edge mode: its EXTINT line (pin, sense, filter) and its share of the
    /// EIC interrupt.
    template<bool Debounced, typename Pin, auto EdgeCallback, typename Config>
    struct SamButtonEntry
      : EIC::LinesEntry<EIC::HelperLine<Pin, EIC::InterruptType::edgeBoth, EdgeCallback, Config>> {
    };

    /// Debounced mode: handler() polls the pin, so it is an input with the config's pull and nothing
    /// else - no EXTINT line, no interrupt per bounce edge, no `eicPriority` needed.
    template<typename Pin, auto EdgeCallback, typename Config>
    struct SamButtonEntry<true, Pin, EdgeCallback, Config> {
        static constexpr auto initStepPinConfig
          = list(action(Io::Action::Input<EIC::pullOf<Config>>{}, Pin{}));
    };
}   // namespace detail

/// A PushButton on a SAM pin, pull-up by default (Config_::pull to change it, EIC::pullOf). Listing it
/// in Startup is all it takes.
/// - Edge mode (no debounce times in the config): an EXTINT line, both edges, the EIC's filter on;
///   its pin, its line and its share of the EIC interrupt come with it (an EicBase has to be in the
///   list too). Config_ is PushButton's UserConfig plus `eicPriority`, the NVIC priority of the EIC
///   interrupt (all lines on it must agree). A wrapper that wants its own edge callback takes
///   `EicLineWith<&itsCallback>` as its SubIsrs.
/// - Debounced mode (`debouncePress` and `debounceRelease`): handler() polls the pin, so only the
///   pin is configured (input + pull).
template<typename Clock, typename Pin, std::size_t EventQSize, typename Config_>
struct SamPushButton
  : Kvasir::PushButton<Clock, Pin, EventQSize, Config_>
  , detail::SamButtonEntry<Kvasir::PushButton<Clock, Pin, EventQSize, Config_>::Config::debounce,
                           Pin,
                           &Kvasir::PushButton<Clock, Pin, EventQSize, Config_>::edgeCallback,
                           Config_> {
    using Base = Kvasir::PushButton<Clock, Pin, EventQSize, Config_>;
    using Base::handler;

    static constexpr auto pull = Kvasir::EIC::pullOf<Config_>;

    /// Edge mode only: the button's line with another callback, for a wrapper's SubIsrs. A class, not
    /// an alias: the line (and its eicPriority) is looked at only where a wrapper names it.
    template<auto Callback>
    struct EicLineWith
      : Kvasir::EIC::HelperLine<Pin, Kvasir::EIC::InterruptType::edgeBoth, Callback, Config_> {};
};
}   // namespace Kvasir
