#pragma once

#include "core/core.hpp"
#include "kvasir/Common/Core.hpp"
#include "kvasir/StartUp/LinkerSymbols.hpp"
#include "kvasir/Util/attributes.hpp"

#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace Startup { namespace Sam {
    // Set just before the restart below, read and replaced on the next start: survives SysResetReq (RAM is not
    // reset) and is not touched by initMemory (.noInit).
    inline constexpr std::uint32_t RestartRequested = 0x4B52'5354U;   // "TSRK"
    inline constexpr std::uint32_t RestartDone      = 0x4B52'5344U;   // "DSRK"
    [[KVASIR_SECTION(".noInit")]] inline std::uint32_t volatile restartMark;

    /// Whether this start is the one the restart below made: an image at flash address 0 was entered without a
    /// reset (a debugger's or its own jump) and reset the chip first. Valid from main() on, until the next start.
    inline bool restartedAfterJump() { return restartMark == RestartDone; }
}}}   // namespace Kvasir::Startup::Sam

namespace Kvasir { namespace PM {
    /// For Kvasir::Boot::logBoot (kvasir/Util/BootLog.hpp), which finds it next to the chip's reset cause type
    /// (argument-dependent lookup) and adds a line when the reset it reports was restartIfNotFromReset()'s.
    template<typename Cause>
        requires std::is_enum_v<Cause>
    bool restartedAfterJump(Cause) {
        return Startup::Sam::restartedAfterJump();
    }
}}   // namespace Kvasir::PM

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
        bool const restarted = restartMark == RestartRequested;
        restartMark          = restarted ? RestartDone : 0U;
        // the table's address through a register: the compiler takes a function's address for never null
        std::uintptr_t vectors = 0;
        asm("" : "=r"(vectors) : "0"(&_LINKER_vectors_start_));
        if(restarted || vectors != 0U || Kvasir::GCLK::looksLikeAfterReset()) { return; }
        restartMark = RestartRequested;
        asm volatile("dsb" ::: "memory");
        *reinterpret_cast<std::uint32_t volatile*>(0xE000'ED0CU) = 0x05FA'0004U;
        asm volatile("dsb" ::: "memory");
        while(true) {}
    }
}}}   // namespace Kvasir::Startup::Sam

namespace Kvasir { namespace Startup {
    template<typename... Ts>
    struct FirstInitStep<Tag::User, Ts...> {
        void operator()() {
            Core::startup();
            Sam::restartIfNotFromReset();
            // GCLK0 off whatever clock the image before left it on, before the clock tree is rewritten (the
            // chip's GCLK.hpp says why and how; the start after a real reset only reads one register)
            Kvasir::GCLK::parkMainClock();
        }
    };
}}   // namespace Kvasir::Startup

#include "kvasir/StartUp/StartUp.hpp"
