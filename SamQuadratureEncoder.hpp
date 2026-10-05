#pragma once
#include "EIC.hpp"
#include "kvasir/Devices/QuadratureDecoder.hpp"

#include <atomic>
#include <cstdint>

namespace Kvasir {
/// A rotary encoder decoded from BOTH of its lines: A and B are each an EXTINT line on both
/// edges, and every edge feeds the levels of the two into a QuadratureDecoder, which counts a
/// detent when the knob arrives in one. Contact bounce, a knob turned half way and let go, or
/// one rocked on its detent counts nothing - unlike SamRotaryEncoder, which counts one edge of
/// A and takes a bounce of A for a click. The price: two EXTINT lines per knob instead of one.
///
/// Config_ needs `eicPriority` (the NVIC priority of the EIC interrupt, which every line on it
/// must share) and may have `stepsPerDetent` (4: a whole quadrature cycle per click, as the
/// SparkFun COM-10982 does; 2: half a cycle; 1), `restState` ((A << 1) | B
/// in a detent, 0) and `pull` (Io::PullConfiguration of both pins, PullUp; PullNone for lines
/// with their own resistors). The count is `cnt`, wrapping, up for A leading B - the direction
/// SamRotaryEncoder counts the same knob. `skipped` counts changes of both lines at once (an
/// edge the interrupt came too late for).
///
/// Listing it in Startup is all it takes (an EicBase has to be in the list too). The decoder
/// starts from the lines' levels in preEnableRuntimeInit, before the EIC is switched on.
template<typename PinA, typename PinB, typename ValueType, typename Config_>
struct SamQuadratureEncoder {
    static_assert(
      requires { Config_::eicPriority; },
      "SamQuadratureEncoder: the config needs eicPriority, the NVIC priority of the EIC "
      "interrupt");

    static constexpr unsigned stepsPerDetent = [] {
        if constexpr(requires { Config_::stepsPerDetent; }) {
            return static_cast<unsigned>(Config_::stepsPerDetent);
        } else {
            return 4U;
        }
    }();
    static constexpr unsigned restState = [] {
        if constexpr(requires { Config_::restState; }) {
            return static_cast<unsigned>(Config_::restState);
        } else {
            return 0U;
        }
    }();
    static constexpr auto pull = [] {
        if constexpr(requires { Config_::pull; }) {
            return Config_::pull;
        } else {
            return Kvasir::Io::PullConfiguration::PullUp;
        }
    }();

    using Decoder = Kvasir::QuadratureDecoder<stepsPerDetent, restState>;

    using type = ValueType;
    static inline std::atomic<ValueType>     cnt{};
    static inline std::atomic<std::uint32_t> skipped{};

    // only the EIC interrupt touches it (all lines of the vector share one priority, so the
    // callbacks of A and B never interrupt each other); preEnableRuntimeInit runs before the
    // EIC is on
    static inline Decoder decoder{};

    static void edgeCallback() {
        auto const pins = apply(read(PinA{}, PinB{}));
        auto const detents
          = decoder.update(Kvasir::Register::get<0>(pins), Kvasir::Register::get<1>(pins));
        if(detents != 0) {
            cnt.store(static_cast<ValueType>(cnt.load(std::memory_order_relaxed)
                                             + static_cast<ValueType>(detents)),
                      std::memory_order_relaxed);
        }
        skipped.store(decoder.skipped(), std::memory_order_relaxed);
    }

    static void preEnableRuntimeInit() {
        auto const pins = apply(read(PinA{}, PinB{}));
        decoder         = Decoder{Kvasir::Register::get<0>(pins), Kvasir::Register::get<1>(pins)};
    }

    template<typename Pin>
    using Line  = Kvasir::EIC::ExtInt<Pin,
                                      Kvasir::EIC::InterruptType::EdgeBoth,
                                      &edgeCallback,
                                      static_cast<int>(Config_::eicPriority),
                                      pull,
                                      true,
                                      false>;
    using LineA = Line<PinA>;
    using LineB = Line<PinB>;

    using Provides = brigand::append<typename LineA::Provides, typename LineB::Provides>;
    using Claims   = brigand::append<typename LineA::Claims, typename LineB::Claims>;
    using SubIsrs  = brigand::append<typename LineA::SubIsrs, typename LineB::SubIsrs>;

    static constexpr auto initStepPinConfig
      = list(LineA::initStepPinConfig, LineB::initStepPinConfig);
    static constexpr auto initStepPeripheryConfig
      = list(LineA::initStepPeripheryConfig, LineB::initStepPeripheryConfig);
};
}   // namespace Kvasir
