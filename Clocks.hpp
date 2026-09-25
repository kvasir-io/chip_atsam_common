#pragma once

// Clocks as Startup resources (kvasir/StartUp/Resources.hpp), as far as the SAM packages need
// them: a driver whose block only works from one particular clock *claims* it, the application's
// clock settings *provide* what they program, and a claim nothing provides is refused by Startup
// at compile time. The names are chip_rp_common's (Clocks.hpp there), so that code written against
// the one reads the same against the other.
//
//     struct ClockSettings {
//         using Provides = brigand::list<Kvasir::Clocks::Clk<Kvasir::Clocks::ClkUsb, 48'000'000>>;
//         static void coreClockInit();        // DFLL::enableUsbRecovery(), ...
//         static void peripheryClockInit();   // ... and GCLK_USB routed to the USB
//     };
//
// Unlike on the RP, ClkUsb is NOT an optional provider here: on a SAM nothing sets GCLK_USB up by
// default, and a USB module without its clock never finishes its reset. So a firmware with the
// USB in its Startup list and clock settings that do not say they provide it does not build.

#include "kvasir/StartUp/Resources.hpp"

namespace Kvasir { namespace Clocks {
    struct ClockTag {
        static constexpr bool sharedClaim      = true;
        static constexpr bool coreLocal        = false;
        static constexpr bool optionalProvider = false;
    };

    // GCLK_USB: 48 MHz +-0.25 % (SAM D21 datasheet DS40001882, 32.5.3).
    struct ClkUsb : ClockTag {};

    template<typename Which, auto Hz>
    using Clk = Startup::Resource<Which, Hz>;

    template<typename Which, auto Hz>
    using Claim = brigand::list<Clk<Which, Hz>>;
}}   // namespace Kvasir::Clocks
