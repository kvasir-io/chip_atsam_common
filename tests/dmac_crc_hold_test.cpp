// The DMAC CRC engine's debug check (DmacCrcHold.hpp): a second start while one user holds the engine - a DmacEngine
// alive, a calcCrc running - panics with checkFailed where debug checks are on, and one user after the other never
// does. Built twice (CMakeLists.txt): KVASIR_DCHECK_LEVEL 2 (the debug and sanitize variants) and 0 (release,
// release_log), where nothing panics and nothing is tracked.
#include "DmacCrcHold.hpp"
#include "harness/PanicCatcher.hpp"
#include "test_harness.hpp"

#include <optional>

template<>
struct Kvasir::Panic::PanicReaction<Kvasir::Tag::User> {
    using type = Kvasir::Test::CatchPanic;
};

namespace {
using Kvasir::DmaCrc::detail::Held;
using Kvasir::DmaCrc::detail::Hold;
using Kvasir::Panic::Cause;

template<typename F>
std::optional<Cause> causeOf(F&& f) {
    auto const info = Kvasir::Test::panicOf(static_cast<F&&>(f));
    return info ? std::optional<Cause>{info->cause} : std::nullopt;
}

constexpr bool checksOn = KVASIR_DCHECK_LEVEL > 0;
}   // namespace

int main() {
    using Kvasir::Test::test;
#ifdef KVASIR_TEST_NO_LONGJMP
    return 0;
#endif

    test("one user after the other: no panic, the engine free afterwards");
    CHECK(!causeOf([] {
        { Held const a{}; }
        { Held const b{}; }
        Hold::take();
        Hold::give();
    }));
    CHECK(!Hold::taken());

    test("held while it lives");
    {
        Held const a{};
        CHECK(Hold::taken() == checksOn);
    }
    CHECK(!Hold::taken());

    // (no holder object inside a lambda that is left by longjmp: its destructor would be skipped)
    test("started again while held: an interrupt's calcCrc or engine");
    {
        auto const cause = causeOf([] {
            Hold::take();   // an engine holds the DMAC's CRC engine
            Held const
              intruder{};   // a second user: its constructor's check panics before it exists
            static_cast<void>(intruder);
        });
        if constexpr(checksOn) {
            CHECK(cause == Cause::checkFailed);
        } else {
            CHECK(!cause);
        }
    }
    Hold::give();
    CHECK(!Hold::taken());

    test("a take without a give, then another take");
    {
        auto const cause = causeOf([] {
            Hold::take();
            Hold::take();
        });
        if constexpr(checksOn) {
            CHECK(cause == Cause::checkFailed);
        } else {
            CHECK(!cause);
        }
    }
    Hold::give();
    CHECK(!Hold::taken());

    return Kvasir::Test::report();
}
