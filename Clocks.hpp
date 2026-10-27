#pragma once

// Clocks as Startup resources (kvasir/StartUp/Resources.hpp), as far as the SAM packages need
// them: a driver whose block only works from one particular clock *claims* it, the application's
// clock settings *provide* what they program, and a claim nothing provides is refused by Startup
// at compile time. The mechanism (Clk, Claim, hzOf) is the SDK's, kvasir/Hal/Clocks.hpp, as on
// the RP; the tags are this family's.
//
//     struct ClockSettings {
//         using Provides = brigand::list<Kvasir::Hal::Clk<Kvasir::Clocks::ClkUsb, 48'000'000>>;
//         static void coreClockInit();        // DFLL::enableUsbRecovery(), ...
//         static void peripheryClockInit();   // ... and GCLK_USB routed to the USB
//     };
//
// Every claim needs a provider (as on the RP and the STM32), and here for a hardware reason too: on a
// SAM nothing sets GCLK_USB up by default, and a USB module without its clock never finishes its
// reset. So a firmware with the USB in its Startup list and clock settings that do not say they
// provide it does not build.
//
// The same holds for every SERCOM and CAN: a SERCOM's core clock (GCLK_SERCOMn_CORE) and a CAN's
// (GCLK_CANn) are off after a reset (the GCLK CLKCTRL / PCHCTRL reset value: CLKEN / CHEN = 0), so
// a driver on one runs on nothing until the firmware's peripheryClockInit routes a generator
// there - and its BAUD is computed from the `clockSpeed` its config says. Each SERCOM/CAN driver
// therefore claims `ClkSercomCore<n>` / `ClkCan<n>` at its `clockSpeed`, and the ClockSettings
// provides each one at the rate of the generator it routes to it:
//
//     using Provides = brigand::list<Kvasir::Hal::Clk<Kvasir::Clocks::ClkSercomCore<3>, ClockSpeed / 32>>;
//
// A ClockSettings that routes no clock, or one at another rate, does not build (no flag turns
// this off).

#include "kvasir/Hal/Clocks.hpp"

namespace Kvasir { namespace Clocks {
    // GCLK_USB: 48 MHz +-0.25 % (SAM D21 datasheet DS40001882, 32.5.3).
    struct ClkUsb : Hal::ClockTag {};

    // GCLK_MAIN: generic clock generator 0, the clock of the CPU and the buses (provided by the
    // chip's ClockTree.hpp Tree next to Startup::ProcessorClock).
    struct ClkMain : Hal::ClockTag {};

    // GCLK_SERCOMn_CORE: the clock a SERCOM's BAUD counts (SERCOM I2C/SPI/USART).
    template<unsigned Instance>
    struct ClkSercomCore : Hal::ClockTag {};

    // GCLK_CANn: the clock the CAN's bit timing counts (C21, E5x).
    template<unsigned Instance>
    struct ClkCan : Hal::ClockTag {};
}}   // namespace Kvasir::Clocks
