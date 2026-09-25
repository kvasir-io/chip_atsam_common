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
        static constexpr auto calcTransferTimeout(std::size_t numBytes) {
            using namespace std::chrono_literals;
            constexpr std::uint32_t bitsPerDataByte = 9;
            constexpr std::uint32_t safetyFactor    = 4;
            constexpr auto          baseTimeout     = 10ms;
            constexpr std::uint32_t microsecondsPerDataByte
              = (bitsPerDataByte * 1'000'000 * safetyFactor) / I2CConfig::baudRate;

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
