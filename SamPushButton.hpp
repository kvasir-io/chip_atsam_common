#pragma once

#include "EIC.hpp"
#include "kvasir/Devices/PushButton.hpp"

namespace Kvasir {
template<typename Clock, typename Pin, std::size_t EventQSize, typename Config_>
struct SamPushButton : Kvasir::PushButton<Clock, Pin, EventQSize, Config_> {
    using Base = Kvasir::PushButton<Clock, Pin, EventQSize, Config_>;
    using Base::handler;

    struct EicConfig {
        static constexpr auto pin         = Pin{};
        static constexpr auto pull        = Kvasir::Io::PullConfiguration::PullUp;
        static constexpr auto type        = Kvasir::EIC::InterruptType::EdgeBoth;
        static constexpr auto filter      = true;
        static constexpr auto callback    = Base::edgeCallback;
        static constexpr auto enableEvent = false;
    };
};
}   // namespace Kvasir
