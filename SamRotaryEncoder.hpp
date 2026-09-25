#pragma once
#include "EIC.hpp"
#include "kvasir/Devices/RotaryEncoder.hpp"

namespace Kvasir {
/// Config_ is RotaryEncoder's UserConfig and may also have `edge`: which edges of PinA count,
/// an EIC::InterruptType (EdgeBoth). EdgeBoth is for an encoder with half a quadrature cycle
/// per detent. One that does a whole cycle per detent pulses A once per click; both edges then
/// count the click twice, and the second, a few ms after the first, is taken for fast turning
/// by the acceleration curve - give it EdgeFall or EdgeRise. B tells the direction on either.
template<typename Clock, typename PinA, typename PinB, typename ValueType, typename Config_>
struct SamRotaryEncoder : Kvasir::RotaryEncoder<Clock, PinA, PinB, ValueType, Config_> {
    using Base = Kvasir::RotaryEncoder<Clock, PinA, PinB, ValueType, Config_>;
    using Base::cnt;

    static constexpr auto edge = [] {
        if constexpr(requires { Config_::edge; }) {
            return Config_::edge;
        } else {
            return Kvasir::EIC::InterruptType::EdgeBoth;
        }
    }();
    static_assert(edge == Kvasir::EIC::InterruptType::EdgeBoth
                    || edge == Kvasir::EIC::InterruptType::EdgeRise
                    || edge == Kvasir::EIC::InterruptType::EdgeFall,
                  "edge has to be one of the Edge* interrupt types");

    static constexpr auto initStepPinConfig = list(makeInput(PinB{}));

    struct EicConfig {
        static constexpr auto pin         = PinA{};
        static constexpr auto pull        = Kvasir::Io::PullConfiguration::PullUp;
        static constexpr auto type        = edge;
        static constexpr auto filter      = true;
        static constexpr auto callback    = Base::edgeCallback;
        static constexpr auto enableEvent = false;
    };
};
}   // namespace Kvasir
