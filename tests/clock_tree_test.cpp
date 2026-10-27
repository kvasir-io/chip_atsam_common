// ClockTree.hpp: Clocks::Sam::TreeOf over fake builders - the clocks it provides, the wait states it takes from the
// chip's table, and its two refusals before any source kind's own checks: a source the part does not have, and an
// f_CPU above the part's maximum. Every check is a static_assert: building is the test. The refusals are must_fail
// cases (CMakeLists.txt): MUST_FAIL=n turns one on.
#include "chip/atsam_common/ClockTree.hpp"

#include <cstdint>
#include <type_traits>

namespace {
namespace S = Kvasir::Clocks::Sam;

// A part with an XOSC + FDPLL tree only, up to 48 MHz, one wait state per 24 MHz started (the chip's table stands in)
struct Fake {
    struct Config {
        std::uint64_t cpu{};
        S::Source     kind = S::Source::xoscFdpll;
        S::Xosc       source{};
    };

    static consteval bool has(S::Source s) { return s == S::Source::xoscFdpll; }

    static consteval std::uint64_t cpuMax(Config const&) { return 48'000'000; }

    template<Config C>
    static consteval unsigned waitStates() {
        return static_cast<unsigned>((C.cpu - 1) / 24'000'000);
    }

    struct Marker : Kvasir::Hal::ClockTag {};

    template<Config C>
    struct Kind {
        using ExtraProvides = brigand::list<Kvasir::Hal::Clk<Marker, C.source.hz>>;

        template<unsigned>
        static void start() {}
    };
};

static_assert(S::Builders<Fake>);

using Tree = S::TreeOf<Fake, Fake::Config{.cpu = 48'000'000, .source = {.hz = 8'000'000}}>;
static_assert(Tree::waitStates == 1,
              "the chip's wait states for 48 MHz");
static_assert(std::is_same_v<Tree::Provides,
                             brigand::list<Kvasir::Hal::Clk<Kvasir::Clocks::ClkMain,
                                                            48'000'000>,
                                           Kvasir::Startup::ProcessorClock<48'000'000>,
                                           Kvasir::Hal::Clk<Fake::Marker,
                                                            8'000'000>>>,
              "generator 0, the processor clock, then the kind's own clocks");
using Slow = S::TreeOf<Fake, Fake::Config{.cpu = 24'000'000, .source = {.hz = 8'000'000}}>;
static_assert(Slow::waitStates == 0);

#if MUST_FAIL == 1
// a source this part has no builder for
using NoSuchSource = S::TreeOf<
  Fake,
  Fake::Config{.cpu = 48'000'000, .kind = S::Source::dfllOpenLoop, .source = {.hz = 8'000'000}}>;
constexpr unsigned w = NoSuchSource::waitStates;
#elif MUST_FAIL == 2
// f_CPU above the part's maximum
using TooFast = S::TreeOf<Fake, Fake::Config{.cpu = 48'000'001, .source = {.hz = 8'000'000}}>;
constexpr unsigned w = TooFast::waitStates;
#endif
}   // namespace

int main() { return 0; }
