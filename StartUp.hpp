#pragma once

#include "core/core.hpp"
#include "kvasir/Common/Core.hpp"

namespace Kvasir { namespace Startup {
    template<typename... Ts>
    struct FirstInitStep<Tag::User, Ts...> {
        void operator()() { Core::startup(); }
    };
}}   // namespace Kvasir::Startup

#include "kvasir/StartUp/StartUp.hpp"
