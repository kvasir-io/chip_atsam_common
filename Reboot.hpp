#pragma once
// Kvasir::Chip::reboot() on the SAM D21, C21 and D5x/E5x: a system reset request (AIRCR = VECTKEY | SYSRESETREQ,
// DDI0419E B3.2.6 / DDI0403E B3.2.6). It resets everything but the debug logic, the RTC, the 32 kHz sources, a WDT
// with ALWAYSON and a GCLK with WRTLOCK (D21 DS40001882L Table 16-2, C21 DS60001479M Table 18-1, E5x DS60001507N
// Table 16-1); the next boot reads `syst` (PM::resetCause(), resetKind() softwareRequest).
//
// The log printer is told first (kvasir/Runtime/AnnouncedReset.hpp): with one armed it waits until the printer is off
// the chip. That wait counts polls at the chip's fastest clock (Register::CpuHzCeiling<Tag::User>, chip/WaitBounds.hpp),
// so an armed printer that never acks costs ~0.5 s at the ceiling, longer below it. A BootGuard's plannedReset() stays
// the caller's (Runtime/BootGuard.hpp).
//
// Not for the start code: Startup::Sam::restartIfNotFromReset() (StartUp.hpp) resets before RAM is initialised and
// must not announce.
#include "chip/WaitBounds.hpp"   // Register::CpuHzCeiling<Tag::User>: announce()'s polls
#include "core/core.hpp"
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Runtime/AnnouncedReset.hpp"

namespace Kvasir::Chip {
[[noreturn]] inline void reboot() {
    (void)AnnouncedReset::announce();
    Kvasir::SystemControl::
      resetNow();   // dsb; AIRCR; dsb; loop (core_cortex_common SystemControl.hpp)
}
}   // namespace Kvasir::Chip

/// reboot() as the port's hook (kvasir/Port/Hooks.hpp), for what the SDK ends in: Port::LateT<Port::Reboot<Tag::User>,
/// Dep>{}().
template<typename... Ts>
struct Kvasir::Port::Reboot<Kvasir::Tag::User, Ts...> {
    [[noreturn]] void operator()() const { Kvasir::Chip::reboot(); }
};
