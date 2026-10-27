#pragma once

#include "chip/Interrupt.hpp"   // the vector table below reads InterruptOffsetTraits
#include "chip/Package.hpp"
#include "chip/Start.hpp"
#include "chip/atsam_common/ChipTraits.hpp"
#include "core/core.hpp"
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Port/LinkerSymbols.hpp"
#include "kvasir/Util/attributes.hpp"

#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace Startup { namespace Sam {
    // Set just before the restart below, read and replaced on the next start: survives SysResetReq (RAM is not
    // reset) and is not touched by initMemory (.noInit).
    inline constexpr std::uint32_t restartRequested = 0x4B52'5354U;   // "TSRK"
    inline constexpr std::uint32_t restartDone      = 0x4B52'5344U;   // "DSRK"
    [[KVASIR_NOINIT]] inline std::uint32_t volatile restartMark;

    /// Whether this start is the one the restart below made: an image at flash address 0 was entered without a
    /// reset (a debugger's or its own jump) and reset the chip first. Valid from main() on, until the next start.
    inline bool restartedAfterJump() { return restartMark == restartDone; }
}}}   // namespace Kvasir::Startup::Sam

// For Kvasir::Boot::logBoot (kvasir/Runtime/BootLog.hpp), which adds a line when the reset it reports was
// restartIfNotFromReset()'s: the SDK's late hook (kvasir/Port/Hooks.hpp).
template<>
struct Kvasir::Port::RestartedAfterJump<Kvasir::Tag::User> {
    bool operator()() const { return Kvasir::Startup::Sam::restartedAfterJump(); }
};

namespace Kvasir { namespace Startup { namespace Sam {
    // An image at flash address 0 has no bootloader in front of it, so a start that is no reset can only be a
    // debugger's or the image's own jump. Such a start finds every peripheral as the run before left it - and on
    // SAM an access to an enable-protected register of an enabled peripheral is a HardFault ("an attempt to
    // access these registers will result generation of hard fault exception", D21 DS40001882 11.7; measured: an
    // SPI's PeripheryInit writing SERCOM.BAUD of a SERCOM a jumped-from run left enabled faulted, and the same
    // write from a debugger was refused with a bus error). A system reset gives it the state a power-on would
    // (D21 Table 16-2, C21 Table 18-1) for a few bytes; once, guarded by restartMark, so a state the reset does
    // not clear (a WRTLOCKed GCLK0) cannot loop. An image behind a bootloader (not at 0) must live with the
    // jump: parkMainClock() is for it. AIRCR = VECTKEY 0x05FA | SYSRESETREQ (DDI0419E B3.2.6).
    inline void restartIfNotFromReset() {
        bool const restarted = restartMark == restartRequested;
        restartMark          = restarted ? restartDone : 0U;
        // the table's address through a register: the compiler takes a function's address for never null
        std::uintptr_t vectors = 0;
        asm("" : "=r"(vectors) : "0"(&_LINKER_vectors_start_));
        if(restarted || vectors != 0U || StartTraits::looksLikeAfterReset()) { return; }
        restartMark = restartRequested;
        Kvasir::SystemControl::resetNow();   // dsb; AIRCR = VECTKEY | SYSRESETREQ; dsb; loop
    }
}}}   // namespace Kvasir::Startup::Sam

// every name the shared drivers use, checked once here instead of deep in a driver (ChipTraits.hpp lists them)
static_assert(Kvasir::Sam::ChipPackage<Kvasir::Sam::Package>,
              "the SAM chip package lacks a name the shared drivers use "
              "(chip_atsam_common/ChipTraits.hpp lists them)");

namespace Kvasir { namespace Startup {
    template<typename... Ts>
    struct FirstInitStep<Tag::User, Ts...> {
        void operator()() {
            Core::startup();
            Sam::restartIfNotFromReset();
            // GCLK0 off whatever clock the image before left it on, before the clock tree is rewritten (the
            // chip's Start.hpp says why and how; the start after a real reset only reads one register)
            Sam::StartTraits::parkMainClock();
        }
    };
}}   // namespace Kvasir::Startup

// An address no bus slave answers at, for Kvasir::Fault::Inject (kvasir/Runtime/FaultInject.hpp): 0xF0000000 is in the
// vendor system region (Armv6-M ARM DDI0419E B3.1 / Armv7-M DDI0403E B3.1 Table B3-1: 0xE0100000-0xFFFFFFFF
// Vendor_SYS) where no SAM part maps anything: D21 DS40001882 Figure 9-1 (Product Mapping) shows 0xE0100000 -
// 0xFFFFFFFF "Reserved", C21 Table 9-1 and E5x Table 9-1 list no memory or bridge there. The M0+ routes it to the
// bus matrix (Cortex-M0+ TRM DDI0484C 3.4), and an access "at an address where no client (bridge or peripheral) is
// mapped" is an AHB client bus error (C21 11.5.2.7, E5x 27.5.2.6; the D21 datasheet does not say it in words).
template<>
struct Kvasir::Port::UnmappedAddress<Kvasir::Tag::User> {
    static constexpr std::uintptr_t value = 0xF000'0000U;
};
