#pragma once
#include "EIC.hpp"
#include "kvasir/Devices/RotaryEncoder.hpp"
#include "kvasir/Util/Config.hpp"

namespace Kvasir {
/// A RotaryEncoder whose A pin is an EXTINT line (pull-up unless `pull` says otherwise, the EIC's
/// filter on) and whose B pin is
/// read in the edge callback. Listing it in Startup is all it takes: both pins, the line and its
/// share of the EIC interrupt come with it (an EicBase has to be in the list too).
///
/// Config_ is RotaryEncoder's UserConfig plus `eicPriority`, the NVIC priority of the EIC
/// interrupt (all lines on it must agree), and may also have `edge`: which edges of PinA count,
/// an EIC::InterruptType (EdgeBoth). EdgeBoth is for an encoder with half a quadrature cycle
/// per detent. One that does a whole cycle per detent pulses A once per click; both edges then
/// count the click twice, and the second, a few ms after the first, is taken for fast turning
/// by the acceleration curve - give it EdgeFall or EdgeRise. B tells the direction on either.
/// `pull` (Io::PullConfiguration, PullUp) is PinA's pull; PinB is a plain input either way.
/// A wrapper that wants its own edge callback takes `EicLineWith<&itsCallback>` as its SubIsrs.
template<typename Clock, typename PinA, typename PinB, typename ValueType, typename Config_>
struct SamRotaryEncoder
  : Kvasir::RotaryEncoder<Clock, PinA, PinB, ValueType, Config_>
  , Kvasir::EIC::LinesEntry<Kvasir::EIC::HelperLine<
      PinA,
      KVASIR_CONFIG_OR(Config_, edge, Kvasir::EIC::InterruptType::edgeBoth),
      &Kvasir::RotaryEncoder<Clock, PinA, PinB, ValueType, Config_>::edgeCallback,
      Config_>> {
    using Base = Kvasir::RotaryEncoder<Clock, PinA, PinB, ValueType, Config_>;
    using Base::cnt;

    static constexpr auto edge
      = KVASIR_CONFIG_OR(Config_, edge, Kvasir::EIC::InterruptType::edgeBoth);
    static_assert(edge == Kvasir::EIC::InterruptType::edgeBoth
                    || edge == Kvasir::EIC::InterruptType::edgeRise
                    || edge == Kvasir::EIC::InterruptType::edgeFall,
                  "edge has to be one of the Edge* interrupt types");

    static constexpr auto pull = Kvasir::EIC::pullOf<Config_>;

    template<auto Callback>
    using EicLineWith = Kvasir::EIC::HelperLine<PinA, edge, Callback, Config_>;
    using EicLine     = EicLineWith<&Base::edgeCallback>;

    // B is only read: "If the Input Enable bit in the Pin Configuration registers
    // (PINCFGy.INEN) is '0', the input value will not be sampled" (SAM C21 data sheet, PORT,
    // SAMC20_C21_Family_Datasheet.md 20208; the D21's PINCFG.INEN, md 17597, says the same)
    static constexpr auto initStepPinConfig = list(makeInput(PinB{}), EicLine::initStepPinConfig);
};
}   // namespace Kvasir
