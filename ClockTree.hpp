#pragma once
// The SAM core clock tree from a target, one skeleton for the family: the chip says which clock sources it can put
// under generator 0 and how (its "builders", chip/ClockTree.hpp), the skeleton checks f_CPU against the part,
// takes the flash wait states from the chip's table and provides the clocks it makes as Startup resources:
//
//     using Tree = Kvasir::Clocks::Sam::Tree<{.cpu = 48'000'000, .source = {.hz = 8'000'000}}>;   // the chip's alias
//     using Provides = Tree::Provides;   // Hal::Clk<ClkMain, cpu>, Startup::ProcessorClock<cpu> [, the kind's own]
//     static void coreClockInit() { Tree::coreClockInit(); }
//
// The builders B: `Config` (the chip's TreeConfig, with `cpu` and `kind`), `has(Source)`, `cpuMax(Config)`,
// `waitStates<Config>()` and `Kind<Config>` - one struct per source kind with `ExtraProvides` (a brigand::list) and
// `start<WaitStates>()`, the register writes and bounded waits that bring the source up and switch generator 0 to it.
// `start` runs in ClockSettings::coreClockInit(), before the memory init: registers only, no variables.

#include "chip/atsam_common/Clocks.hpp"
#include "kvasir/Mpl/brigand.hpp"
#include "kvasir/StartUp/Resources.hpp"

#include <concepts>
#include <cstdint>

namespace Kvasir { namespace Clocks { namespace Sam {
    enum class Source : std::uint8_t {
        xoscFdpll,   // a crystal (or a clock on XIN) on XOSC -> FDPLL96M (E5x: XOSCn -> FDPLL0) -> generator 0
        dfllOpenLoop,   // DFLL48M in open loop: on the factory's coarse value (D21), as a reset leaves it (E5x)
        dfllUsbRecovery,   // DFLL48M locked to the USB start-of-frame (D21 with USB)
        dfllClosedLoop,    // DFLL48M locked to a reference generator (D21)
        xoscFdpllElseDfll   // xoscFdpll, and when the crystal or the FDPLL does not come up in a bounded wait, XOSC and
        // FDPLL off again and DFLL48M open loop instead; Tree::onCrystal() says which (D21)
    };

    struct Xosc {
        std::uint64_t hz{};
        bool          crystal = true;
    };

    template<typename B>
    concept Builders = requires(typename B::Config const& c) {
        { B::has(Source::xoscFdpll) } -> std::same_as<bool>;
        { B::cpuMax(c) } -> std::convertible_to<std::uint64_t>;
    };

    namespace Detail {
        // the checks first (a base is complete before the next one is looked at): a kind the part does not have and
        // a CPU above the part's maximum are said before a kind's own solver complains
        template<typename B, auto C>
        struct TreeChecks {
            static_assert(B::has(C.kind),
                          "this SAM part has no such clock source (Kvasir::Clocks::Sam::Source)");
            static_assert(C.cpu <= B::cpuMax(C),
                          "f_CPU above the part's maximum (SAM D21 Table 37-7; SAM C21 Tables "
                          "45-8/45-9 by speed "
                          "grade; SAM E5x Table 54-6)");
        };
    }   // namespace Detail

    template<Builders B, typename B::Config C>
    struct TreeOf
      : Detail::TreeChecks<B, C>
      , B::template Kind<C> {
        static constexpr unsigned waitStates = B::template waitStates<C>();

        using Provides
          = brigand::append<brigand::list<Hal::Clk<ClkMain, C.cpu>, Startup::ProcessorClock<C.cpu>>,
                            typename B::template Kind<C>::ExtraProvides>;

        static void coreClockInit() { B::template Kind<C>::template start<waitStates>(); }
    };
}}}   // namespace Kvasir::Clocks::Sam
