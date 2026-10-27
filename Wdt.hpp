#pragma once
// The SAM D21 / C21 / D5x-E5x watchdog (D21 DS40001882L chapter 18, C21 DS60001479M chapter 23, E5x DS60001507N
// chapter 20), in the RP driver's shape (chip_rp_common/watchdog.hpp): the same Config names, the same Health
// interface.
//
//   struct WatchdogConfig {
//       static constexpr auto clockSpeed  = 1'024;   // D21: the GCLK_WDT the firmware routes to the WDT, in Hz
//       static constexpr auto overrunTime = 2s;
//       static constexpr bool gatedFeed   = true;    // optional, default false
//   };
//   using Watchdog = Kvasir::Watchdog<WatchdogConfig>;   // ungated: in the Startup list (or arm() it)
//   Watchdog::feed();
//
// The clock: on the D21 a generic clock the firmware sets up, GCLK_WDT, "intended to be sourced from the ... (ULP)
// oscillator" (D21 18.5.3) - `clockSpeed` must be what that generator gives (nothing checks it: a wrong value only
// mis-states Timeout). On the C21 and E5x a fixed 1.024 kHz CLK_WDT_OSC from the OSCULP32K (C21 23.5.3, E5x 20.5.3):
// `clockSpeed` may be left out, and anything but 1024 is refused.
//
// The period: the smallest CONFIG.PER that is at least `overrunTime` (WdtPeriod.hpp, which also has the
// oscillator's tolerance); `Timeout` is that nominal period.
//
// Gated (`gatedFeed = true`): only Kvasir::Health::Supervisor arms, feeds and disarms it (Health/HealthKey.hpp) - the
// plain arm(), feed() and disarm() do not compile, the FeedKey ones do. An ungated one arms itself as a Startup
// entry (runtimeInit()).
//
// "When the CPU is halted in debug mode the WDT will halt normal operation" (D21 18.5.7, C21 23.5.7, E5x 20.5.7): a
// halt under a debugger never bites, and there is no switch for it.
//
// ALWAYSON (set by the NVM user row or by software, cleared only by a power-on reset): the WDT runs whatever
// CTRLA.ENABLE says, and CONFIG and EWCTRL are read-only (D21 18.6.3.1, C21 23.6.8.1, E5x 20.6.8.1). arm() and
// disarm() then only clear it, and the period is the user row's, not `Timeout`: alwaysOn() tells, the caller may log
// it (Health's period checks compare with `Timeout` and cannot know).
#include "WdtPeriod.hpp"
#include "chip/PM.hpp"
#include "chip/Package.hpp"   // the WDT's sync register and clock
#include "chip/WaitBounds.hpp"
#include "kvasir/Health/WatchdogFront.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Wait.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/Util/Config.hpp"
#include "peripherals/WDT.hpp"

#include <chrono>
#include <cstdint>
#include <type_traits>

namespace Kvasir {

// Startup resource: the one watchdog (kvasir/StartUp/Resources.hpp).
struct WatchdogTag {};

namespace Sam::Detail {
    // D21: one STATUS.SYNCBUSY for every write-synchronised register, CTRL and CLEAR (18.6.5); a write while it is
    // set stalls the bus until the synchronisation is done. Which one the chip has: Sam::Package::wdtSyncBusyInStatus.
    template<typename R, bool InStatus = Sam::Package::wdtSyncBusyInStatus>
    struct WdtSync {
        static constexpr auto enable = R::STATUS::syncbusy;
        static constexpr auto clear  = R::STATUS::syncbusy;
    };

    // C21 / E5x: SYNCBUSY.ENABLE and SYNCBUSY.CLEAR (C21 23.8.7, E5x 20.8.7); a second write to a register while it
    // synchronises "is discarded and an error is reported through the PAC" (C21 15.3.2, E5x 13.3.2).
    template<typename R>
    struct WdtSync<R, false> {
        static constexpr auto enable = R::SYNCBUSY::enable;
        static constexpr auto clear  = R::SYNCBUSY::clear;
    };
}   // namespace Sam::Detail

// arm(), feed() and disarm(), gated or not: the SDK's one front (kvasir/Health/WatchdogFront.hpp).
template<typename Config>
struct Watchdog : Health::WatchdogFront<Watchdog<Config>, Health::gatedFeedOf<Config>()> {
    using Regs = Kvasir::Peripheral::WDT::Registers<>;
    using Sync = Sam::Detail::WdtSync<Regs>;

    using Provides = brigand::list<Startup::Resource<WatchdogTag, 0>>;

    // the clock the counter counts: the D21's GCLK_WDT (the config names it), the C21's and E5x's fixed CLK_WDT_OSC
    // (Sam::Package::wdtFixedClockHz)
    static constexpr std::uint64_t clockHz = static_cast<std::uint64_t>(
      KVASIR_CONFIG_OR(Config, clockSpeed, Sam::Package::wdtFixedClockHz));
    static_assert(
      Sam::Package::wdtFixedClockHz == 0 || clockHz == Sam::Package::wdtFixedClockHz,
      "the C21's and E5x's WDT counts the fixed 1.024 kHz CLK_WDT_OSC (C21 23.5.3, E5x 20.5.3)");
    static_assert(clockHz > 0,
                  "Watchdog: clockSpeed in Hz");

    static constexpr bool gatedFeed = Health::gatedFeedOf<Config>();

    static_assert(
      static_cast<bool>(KVASIR_CONFIG_OR(Config,
                                         pauseOnDebug,
                                         true)),
      "the SAM WDT always halts while the CPU is halted in debug mode (D21 18.5.7, C21 23.5.7, E5x "
      "20.5.7): pauseOnDebug = false cannot be had");

    static constexpr Sam::Wdt::Period setting = Sam::Wdt::periodFor(
      clockHz,
      static_cast<std::uint64_t>(
        std::chrono::ceil<std::chrono::microseconds>(Config::overrunTime).count()));

    /// The nominal period of the setting (CONFIG.PER cycles at ClockHz).
    static constexpr std::chrono::nanoseconds timeout{
      static_cast<std::int64_t>(Sam::Wdt::periodNs(clockHz, setting))};

    using SyncBound = Kvasir::Register::MicrosecondsBound<Sam::Wdt::syncBoundUs(clockHz)>;

private:
    // the generated field types (the SVDs' enums: their enumerator names differ per SVD, the numbers do not)
    using PerType   = typename std::remove_cvref_t<decltype(Regs::CONFIG::per)>::DataType;
    using ClearType = typename std::remove_cvref_t<decltype(Regs::CLEAR::clear)>::DataType;

public:
    /// The user row or an earlier run set ALWAYSON: the WDT runs with the period it has, until a power-on reset.
    [[nodiscard]] static bool alwaysOn() { return apply(read(Regs::CTRLA::alwayson)) != 0; }

    /// The ungated watchdog as a Startup entry: armed after the enables. A gated one is armed by the supervisor.
    static void runtimeInit()
        requires(!gatedFeed)
    {
        armNow();
    }

    /// Reset now: "Writing any other value than 0xA5 to CLEAR will issue an immediate system reset" (D21 18.6.2.4,
    /// C21 23.6.2.4, E5x 20.6.2.4). Only while the WDT runs (enabled or ALWAYSON): a write to CLEAR of a stopped WDT
    /// is not described, so this arms it first. The next boot's reset cause is whatever RCAUSE then says.
    [[noreturn]] static void trigger() {
        if(!alwaysOn() && !apply(read(Regs::CTRLA::enable))) { armNow(); }
        waitSync(Sync::clear);
        apply(write(Regs::CLEAR::clear,
                    Kvasir::Register::value<ClearType, static_cast<ClearType>(0x00)>()));
        while(true) { asm volatile("" ::: "memory"); }
    }

    /// Whether the last reset was the watchdog's (RCAUSE.WDT).
    [[nodiscard]] static bool causedReset() { return PM::resetCause() == PM::ResetCause::wdt; }

private:
    friend struct Health::WatchdogFront<Watchdog, gatedFeed>;

    template<typename Field>
    static void waitSync(Field const& f) {
        (void)Kvasir::Register::waitUntil<SyncBound>(Kvasir::Register::isClear(f));
    }

    // CONFIG is enable-protected: writes while the WDT is enabled "will be discarded" (D21 18.6.2.1), and on the
    // C21/E5x CONFIG may not be written together with CTRLA.ENABLE = 0 (C21 23.6.2.1, E5x 20.6.2.1). So: off and
    // synchronised first - a start that was no reset (a bootloader's or a debugger's jump) finds it running - then
    // the period, then on: a fresh period from here. With ALWAYSON CONFIG is read-only: only the clear.
    static void armNow() {
        if(alwaysOn()) {
            feedNow();
            return;
        }
        disarmNow();
        apply(write(Regs::CONFIG::per,
                    Kvasir::Register::value<PerType, static_cast<PerType>(setting.per)>()));
        waitSync(
          Sync::
            enable);   // D21: CONFIG is write-synchronised (18.8.2); C21/E5x: nothing pending, no wait
        apply(set(Regs::CTRLA::enable));
        waitSync(Sync::enable);
    }

    // ENABLE "can only be written if CTRLA.ALWAYSON=0" (C21 23.8.1; D21 18.6.2.3): an always-on one only gets a
    // clear.
    static void disarmNow() {
        if(alwaysOn()) {
            feedNow();
            return;
        }
        if(apply(read(Regs::CTRLA::enable))) {
            apply(clear(Regs::CTRLA::enable));
            waitSync(Sync::enable);
        }
    }

    // feed(): a clear. A clear while the last one still synchronises is skipped (the next feed brings another): on the C21/E5x it
    // would be discarded with a PAC error, on the D21 it would stall the bus (D21 18.6.5).
    static void feedNow() {
        if(!apply(read(Sync::clear))) { apply(write(Regs::CLEAR::CLEARValC::key)); }
    }
};

}   // namespace Kvasir
