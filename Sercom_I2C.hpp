#pragma once

#include "chip/PM.hpp"
#include "chip/Package.hpp"
#include "chip/atsam_common/Clocks.hpp"
#include "chip/atsam_common/Io.hpp"
#include "chip/atsam_common/SercomBaud.hpp"
#include "kvasir/Hal/Clocks.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "peripherals/SERCOM_I2CM.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <ratio>

namespace Kvasir { namespace Sercom { namespace I2C { namespace Detail {

    template<unsigned SercomInstance,
             int      Port,
             int      Pin>
    constexpr bool isValidPinLocationSDA(Kvasir::Io::PinLocation<Port,
                                                                 Pin>) {
        return Sam::Package::Sercom::validIfPOVal<SercomInstance, Port, Pin>(0);
    }

    template<unsigned SercomInstance,
             int      Port,
             int      Pin>
    constexpr bool isValidPinLocationSCL(Kvasir::Io::PinLocation<Port,
                                                                 Pin>) {
        return Sam::Package::Sercom::validIfPOVal<SercomInstance, Port, Pin>(1);
    }

    template<typename Regs, unsigned ClockSpeed, unsigned BaudRate>
    struct GetBaudConfig {
        static constexpr auto baudConfig = calcBaudConfig(ClockSpeed, BaudRate);

        using config = decltype(list(
          write(Regs::BAUD::baud, Register::value<unsigned char, baudConfig.baud>()),
          write(Regs::BAUD::baudlow, Register::value<unsigned char, baudConfig.baudlow>()),
          write(Regs::BAUD::hsbaud, Register::value<unsigned char, baudConfig.hsbaud>()),
          write(Regs::BAUD::hsbaudlow, Register::value<unsigned char, baudConfig.hsbaudlow>())));
    };

    template<typename Regs,
             unsigned Baudrate>
    constexpr auto getSpeedConfig() {
        if constexpr(Baudrate <= maxSpeedFast) {
            return Regs::CTRLA::SPEEDValC::standard_and_fast_mode;
        } else if constexpr(Baudrate <= maxSpeedFastPlus) {
            return Regs::CTRLA::SPEEDValC::fastplus_mode;
        } else {
            // unreachable until high-speed mode is implemented: I2CBase refuses it (kept for that driver)
            return Regs::CTRLA::SPEEDValC::high_speed_mode;
        }
    }

    template<typename I2CConfig_>
    struct I2CBase {
        struct I2CConfig : I2CConfig_ {
            static constexpr auto userConfigOverride
              = KVASIR_CONFIG_OR(I2CConfig_, userConfigOverride, brigand::list<>{});

            static constexpr auto maxBaudRateError
              = KVASIR_CONFIG_OR(I2CConfig_, maxBaudRateError, std::ratio<1, 100>{});

            /// Each device at its own clock, switched between transfers (Sercom_I2CQueued's
            /// timing()); `baudRate` is then the fastest any device gets and the rate the
            /// block starts with.
            static constexpr bool perDeviceClock
              = static_cast<bool>(KVASIR_CONFIG_OR(I2CConfig_, perDeviceClock, false));

            /// The slowest rate a device on this bus runs at: what the idle watchdog of
            /// LineRecovery scales its threshold with. Only perDeviceClock makes it differ.
            static constexpr std::uint32_t minBaudRate = static_cast<std::uint32_t>(
              KVASIR_CONFIG_OR(I2CConfig_,
                               minBaudRate,
                               static_cast<std::uint32_t>(I2CConfig_::baudRate)));
        };

        // needed config
        // clockSpeed
        // baudRate
        // maxBaudRateError
        // i2cInstance
        // SdaPinLocation
        // SclPinLocation
        // userConfigOverride
        static constexpr auto instance = I2CConfig::instance;
        using Regs                     = Kvasir::Peripheral::SERCOM_I2CM::Registers<instance>;

        // Startup: this SERCOM (one owner across I2C/SPI/USART - the three register views share
        // the base address) and the core clock BAUD is computed from (Clocks.hpp: the
        // ClockSettings must route a generator at clockSpeed to GCLK_SERCOMn_CORE and provide it).
        using Provides = brigand::list<Hal::Owns<Regs>>;
        using Claims   = Hal::Claim<Clocks::ClkSercomCore<instance>, I2CConfig::clockSpeed>;

        using InterruptIndexs = decltype(Sam::Block<Regs>::irqs());

        // High-speed mode (above fast-mode plus) needs a host code sent in full speed, the client address
        // with ADDR.HS = 1 on every (repeated) START, and CTRLA.SCLSM = 1 (SAM D21 DS40001882L 28.6.2.4.6,
        // SAM C21 DS60001479M 33.6.2.4.6, SAM D5x/E5x DS60001507N 36.6.2.4.6). startNext_ writes ADDR alone
        // and SCLSM stays 0, so a bus above 1 MHz is refused here, as clockTiming() refuses it per device.
        static_assert(
          I2CConfig::baudRate <= maxSpeedFastPlus,
          "SAM I2C: baudRate above 1 MHz is high-speed mode, which needs a host code, ADDR.HS "
          "and CTRLA.SCLSM; this driver does not implement it");
        // the achieved SCL against maxBaudRateError; a failure prints wanted, got and ppm
        static constexpr bool baudInTolerance = [] {
            Prescaler::assertInTolerance<achievedRate(I2CConfig::clockSpeed,
                                                      calcBaudConfig(I2CConfig::clockSpeed,
                                                                     I2CConfig::baudRate)),
                                         I2CConfig::baudRate,
                                         Prescaler::Tolerance{I2CConfig::maxBaudRateError},
                                         "I2C SCL">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(baudInTolerance);
        static_assert(I2CConfig::minBaudRate <= I2CConfig::baudRate
                        && (I2CConfig::perDeviceClock
                            || I2CConfig::minBaudRate == I2CConfig::baudRate),
                      "minBaudRate is the slowest device on a perDeviceClock bus, at most "
                      "baudRate");
        static_assert(!I2CConfig::perDeviceClock || I2CConfig::baudRate <= maxSpeedFastPlus,
                      "a perDeviceClock bus switches between standard, fast and fast-mode plus "
                      "only");
        static_assert(isValidPinLocationSDA<instance>(I2CConfig::sdaPinLocation),
                      "invalid SDAPin");
        static_assert(isValidPinLocationSCL<instance>(I2CConfig::sclPinLocation),
                      "invalid SCLPin");

        static constexpr auto powerClockEnable
          = list(typename PM::Enable<Regs::baseAddr>::action{});

        static constexpr auto initStepPinConfig = list(
          Sam::PinFunctionOf<Sam::Package::Sercom, instance, decltype(I2CConfig::sdaPinLocation)>{},
          Sam::
            PinFunctionOf<Sam::Package::Sercom, instance, decltype(I2CConfig::sclPinLocation)>{});

        static constexpr auto initStepPeripheryConfig = list(
          Regs::BAUD::overrideDefaults(
            typename GetBaudConfig<Regs, I2CConfig::clockSpeed, I2CConfig::baudRate>::config{}),

          Regs::CTRLA::overrideDefaults(write(Regs::CTRLA::MODEValC::i2c_master),
                                        write(getSpeedConfig<Regs, I2CConfig::baudRate>())),
          set(Regs::INTENSET::mb),
          set(Regs::INTENSET::sb),
          set(Regs::INTENSET::error),
          I2CConfig::userConfigOverride);

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<I2CConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable
          = list(set(Regs::CTRLA::enable), Nvic::makeEnable(InterruptIndexs{}));

        /// What a transfer of `numBytes` may take before it is given up: four times its length
        /// on the wire (9 bits a byte, the address byte on top) and 10 ms -- the same rule as
        /// the RP driver's, so a part that stretches the clock is given the same patience on
        /// either chip.
        static constexpr auto calcTransferTimeout(std::size_t   numBytes,
                                                  std::uint32_t microsecondsPerDataByte
                                                  = usPerDataByte(I2CConfig::baudRate)) {
            using namespace std::chrono_literals;
            constexpr auto baseTimeout = 10ms;

            auto const timeoutUs
              = static_cast<std::uint32_t>(numBytes + 1) * microsecondsPerDataByte;

            return std::chrono::microseconds(timeoutUs) + baseTimeout;
        }

        /// What Kvasir::I2C::LineRecovery applies before it takes the two pins over: the block
        /// disabled, so that it lets go of both lines and does not take the recovery's clocks
        /// for a bus of its own. The driver puts it through a software reset afterwards.
        static constexpr auto softAbortRequest = list(clear(Regs::CTRLA::enable));
    };
}}}}   // namespace Kvasir::Sercom::I2C::Detail
