#pragma once

#include "chip/Io.hpp"
#include "chip/PM.hpp"
#include "chip/Sercom_Traits.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/literals.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/SERCOM_I2CM.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <ratio>

namespace Kvasir { namespace Sercom { namespace I2C { namespace Detail {

    template<unsigned SercomInstance,
             int      Port,
             int      Pin>
    constexpr bool isValidPinLocationSDA(Kvasir::Register::PinLocation<Port,
                                                                       Pin>) {
        return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(0);
    }

    template<unsigned SercomInstance,
             int      Port,
             int      Pin>
    constexpr bool isValidPinLocationSCL(Kvasir::Register::PinLocation<Port,
                                                                       Pin>) {
        return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(1);
    }

    template<unsigned SercomInstance, typename SDAPIN>
    struct GetSdaPinConfig;

    template<unsigned SercomInstance, int Port, int Pin>
    struct GetSdaPinConfig<SercomInstance, Kvasir::Register::PinLocation<Port, Pin>> {
        using pinConfig = decltype(action(
          Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<SercomInstance>(
            Register::PinLocation<Port, Pin>{})>{},
          Register::PinLocation<Port, Pin>{}));
    };

    template<unsigned SercomInstance, typename SCLPIN>
    struct GetSclPinConfig;

    template<unsigned SercomInstance, int Port, int Pin>
    struct GetSclPinConfig<SercomInstance, Kvasir::Register::PinLocation<Port, Pin>> {
        using pinConfig = decltype(action(
          Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<SercomInstance>(
            Register::PinLocation<Port, Pin>{})>{},
          Register::PinLocation<Port, Pin>{}));
    };

    struct BaudConfig {
        unsigned char baud;
        unsigned char baudlow;
        unsigned char hsbaud;
        unsigned char hsbaudlow;
    };

    struct BaudConfigRaw {
        unsigned char baud;
        unsigned char baudlow;
    };

    static constexpr unsigned maxSpeedStandard  = 100'000;
    static constexpr unsigned maxSpeedFast      = 400'000;
    static constexpr unsigned maxSpeedFastPlus  = 1'000'000;
    static constexpr unsigned maxSpeedHighSpeed = 3'400'000;

    static constexpr BaudConfigRaw calcBaudConfigRaw(std::uint32_t f_clockSpeed,
                                                     std::uint32_t f_baud,
                                                     double        val,
                                                     double        lowtime,
                                                     double        hightime) {
        double     baudraw = ((double(f_clockSpeed) / double(f_baud)) - val);
        auto const baud    = std::int64_t(
          (baudraw * (hightime / (lowtime + hightime)))   //NOLINT(bugprone-incorrect-roundings)
          + 0.5);
        auto const baudlow = std::int64_t((baudraw * (lowtime / (lowtime + hightime)))
                                          + 0.5);   //NOLINT(bugprone-incorrect-roundings)
        auto const baudReg = static_cast<unsigned char>(std::clamp<std::int64_t>(baud, 0, 255));
        auto const baudlowReg
          = static_cast<unsigned char>(std::clamp<std::int64_t>(baudlow, 0, 255));
        return {baudReg, baudlowReg};
    }

    constexpr BaudConfig calcBaudConfig(std::uint32_t f_clockSpeed,
                                        std::uint32_t f_baud) {
        if(f_baud <= maxSpeedStandard) {
            auto const raw = calcBaudConfigRaw(f_clockSpeed, f_baud, 10, 4.7, 4.0);
            return {raw.baud, raw.baudlow, 0, 0};
        }
        if(f_baud <= maxSpeedFast) {
            auto const raw = calcBaudConfigRaw(f_clockSpeed, f_baud, 10, 1.3, 0.6);
            return {raw.baud, raw.baudlow, 0, 0};
        }
        if(f_baud <= maxSpeedFastPlus) {
            auto const raw = calcBaudConfigRaw(f_clockSpeed, f_baud, 10, 0.5, 0.26);
            return {raw.baud, raw.baudlow, 0, 0};
        }
        auto const raw = calcBaudConfigRaw(f_clockSpeed, f_baud, 2, 2.0, 1.0);
        return {0, 0, raw.baud, raw.baudlow};
    }

    constexpr double calcf_Baud(std::uint32_t f_clockSpeed,
                                BaudConfig    baudConfig) {
        if(baudConfig.baud == 0 && baudConfig.baudlow == 0) {
            // high_speed
            if(baudConfig.hsbaudlow == 0) {
                return double(f_clockSpeed) / (2.0 + 2.0 * double(baudConfig.hsbaud));
            }
            return double(f_clockSpeed)
                 / (2.0 + double(baudConfig.hsbaud) + double(baudConfig.hsbaudlow));
        }
        if(baudConfig.hsbaud == 0 && baudConfig.hsbaudlow == 0) {
            // other
            if(baudConfig.baudlow == 0) {
                return double(f_clockSpeed) / (10.0 + 2.0 * double(baudConfig.baud));
            }
            return double(f_clockSpeed)
                 / (10.0 + double(baudConfig.baud) + double(baudConfig.baudlow));
        }
        return std::numeric_limits<double>::min();
    }

    template<std::uint32_t f_clockSpeed,
             std::uint32_t f_baud,
             std::intmax_t Num,
             std::intmax_t Denom>
    constexpr bool isValidBaudConfig(std::ratio<Num,
                                                Denom>) {
        static_assert(f_baud <= maxSpeedHighSpeed, "baudRate to big");

        constexpr auto baudConfig   = calcBaudConfig(f_clockSpeed, f_baud);
        constexpr auto f_baudCalced = calcf_Baud(f_clockSpeed, baudConfig);
        constexpr auto err          = f_baudCalced - double(f_baud);
        constexpr auto absErr       = err > 0.0 ? err : -err;
        constexpr auto ret          = absErr <= (double(f_baud) * (double(Num) / (double(Denom))));
        return ret;
    }

    template<typename Regs, unsigned clockSpeed, unsigned baudRate>
    struct GetBaudConfig {
        static constexpr auto baudConfig = calcBaudConfig(clockSpeed, baudRate);

        using config = decltype(list(
          write(Regs::BAUD::baud, Register::value<unsigned char, baudConfig.baud>()),
          write(Regs::BAUD::baudlow, Register::value<unsigned char, baudConfig.baudlow>()),
          write(Regs::BAUD::hsbaud, Register::value<unsigned char, baudConfig.hsbaud>()),
          write(Regs::BAUD::hsbaudlow, Register::value<unsigned char, baudConfig.hsbaudlow>())));
    };

    template<typename Regs,
             unsigned baudrate>
    constexpr auto getSpeedConfig() {
        if constexpr(baudrate <= maxSpeedFast) {
            return Regs::CTRLA::SPEEDValC::standard_and_fast_mode;
        } else if constexpr(baudrate <= maxSpeedFastPlus) {
            return Regs::CTRLA::SPEEDValC::fastplus_mode;
        } else {
            return Regs::CTRLA::SPEEDValC::high_speed_mode;
        }
    }

    /// One device's BAUD on a bus that runs each device at its own clock
    /// (I2CConfig::perDeviceClock), plus CTRLA.SPEED and the transfer timeout's time per byte.
    struct ClockTiming {
        std::uint8_t  baud{};
        std::uint8_t  baudlow{};
        std::uint8_t  speed{};   ///< CTRLA.SPEED: 0 standard and fast, 1 fast-mode plus
        std::uint16_t usPerByte{};

        constexpr bool operator==(ClockTiming const&) const = default;
    };

    // Not constexpr: reaching one in clockTiming() is the compile error that says why.
    inline void i2cDeviceClockAboveFastModePlus() {}

    inline void i2cDeviceClockErrorAboveMaxBaudRateError() {}

    /// The transfer timeout's time per byte: 9 bits, 4 times over.
    constexpr std::uint32_t usPerDataByte(std::uint32_t f_baud) {
        constexpr std::uint32_t bitsPerDataByte = 9;
        constexpr std::uint32_t safetyFactor    = 4;
        return (bitsPerDataByte * 1'000'000 * safetyFactor) / f_baud;
    }

    /// calcBaudConfig and isValidBaudConfig for a rate known only per device. High-speed mode
    /// is not offered per device: it needs a master code and CTRLA.SCLSM (28.6.2.4.6).
    template<std::intmax_t Num,
             std::intmax_t Denom>
    consteval ClockTiming clockTiming(std::uint32_t f_clockSpeed,
                                      std::uint32_t f_baud,
                                      std::ratio<Num,
                                                 Denom>) {
        if(f_baud == 0 || f_baud > maxSpeedFastPlus) { i2cDeviceClockAboveFastModePlus(); }
        auto const cfg    = calcBaudConfig(f_clockSpeed, f_baud);
        auto const err    = calcf_Baud(f_clockSpeed, cfg) - double(f_baud);
        auto const absErr = err > 0.0 ? err : -err;
        if(absErr > double(f_baud) * (double(Num) / double(Denom))) {
            i2cDeviceClockErrorAboveMaxBaudRateError();
        }
        return ClockTiming{.baud      = cfg.baud,
                           .baudlow   = cfg.baudlow,
                           .speed     = static_cast<std::uint8_t>(f_baud <= maxSpeedFast ? 0 : 1),
                           .usPerByte = static_cast<std::uint16_t>(usPerDataByte(f_baud))};
    }

    template<typename I2CConfig_>
    struct I2CBase {
        struct I2CConfig : I2CConfig_ {
            static constexpr auto userConfigOverride = [] {
                if constexpr(requires { I2CConfig_::userConfigOverride; }) {
                    return I2CConfig_::userConfigOverride;
                } else {
                    return brigand::list<>{};
                }
            }();

            static constexpr auto maxBaudRateError = [] {
                if constexpr(requires { I2CConfig_::maxBaudRateError; }) {
                    return I2CConfig_::maxBaudRateError;
                } else {
                    return std::ratio<1, 100>{};
                }
            }();

            /// Each device at its own clock, switched between transfers (Sercom_I2CQueued's
            /// timing()); `baudRate` is then the fastest any device gets and the rate the
            /// block starts with.
            static constexpr bool perDeviceClock = [] {
                if constexpr(requires { I2CConfig_::perDeviceClock; }) {
                    return static_cast<bool>(I2CConfig_::perDeviceClock);
                } else {
                    return false;
                }
            }();

            /// The slowest rate a device on this bus runs at: what the idle watchdog of
            /// LineRecovery scales its threshold with. Only perDeviceClock makes it differ.
            static constexpr std::uint32_t minBaudRate = [] {
                if constexpr(requires { I2CConfig_::minBaudRate; }) {
                    return static_cast<std::uint32_t>(I2CConfig_::minBaudRate);
                } else {
                    return static_cast<std::uint32_t>(I2CConfig_::baudRate);
                }
            }();
        };

        // needed config
        // clockSpeed
        // baudRate
        // maxBaudRateError
        // i2cInstance
        // SdaPinLocation
        // SclPinLocation
        // userConfigOverride
        static constexpr auto Instance = I2CConfig::instance;
        using Regs                     = Kvasir::Peripheral::SERCOM_I2CM::Registers<Instance>;

        using InterruptIndexs = decltype(Traits::SercomTraits::getSercomIsrIndexs<Instance>());

        static_assert(isValidBaudConfig<I2CConfig::clockSpeed,
                                        I2CConfig::baudRate>(I2CConfig::maxBaudRateError),
                      "invalid baud configuration baudRate error to big");
        static_assert(I2CConfig::minBaudRate <= I2CConfig::baudRate
                        && (I2CConfig::perDeviceClock
                            || I2CConfig::minBaudRate == I2CConfig::baudRate),
                      "minBaudRate is the slowest device on a perDeviceClock bus, at most "
                      "baudRate");
        static_assert(!I2CConfig::perDeviceClock || I2CConfig::baudRate <= maxSpeedFastPlus,
                      "a perDeviceClock bus switches between standard, fast and fast-mode plus "
                      "only");
        static_assert(isValidPinLocationSDA<Instance>(I2CConfig::sdaPinLocation),
                      "invalid SDAPin");
        static_assert(isValidPinLocationSCL<Instance>(I2CConfig::sclPinLocation),
                      "invalid SCLPin");

        static constexpr auto powerClockEnable
          = list(typename PM::enable<Regs::baseAddr>::action{});

        static constexpr auto initStepPinConfig = list(
          typename GetSdaPinConfig<Instance,
                                   std::decay_t<decltype(I2CConfig::sdaPinLocation)>>::pinConfig{},
          typename GetSclPinConfig<Instance,
                                   std::decay_t<decltype(I2CConfig::sclPinLocation)>>::pinConfig{});

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
