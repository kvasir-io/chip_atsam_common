#pragma once

#include "chip/Io.hpp"
#include "chip/PM.hpp"
#include "chip/Sercom_Traits.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Prescaler.hpp"
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

    /// The SCL rate the registers give, TRISE left out: f_SCL = f_GCLK / (10 + 2 BAUD) with BAUDLOW
    /// 0, f_GCLK / (10 + BAUD + BAUDLOW) otherwise; high-speed f_GCLK / (2 + 2 HSBAUD) or
    /// f_GCLK / (2 + HSBAUD + HSBAUDLOW) (SAM D21 DS40001882L 28.6.2.4.1).
    constexpr Prescaler::Rational achievedRate(std::uint32_t f_clockSpeed,
                                               BaudConfig    baudConfig) {
        if(baudConfig.baud == 0 && baudConfig.baudlow == 0) {
            // high_speed
            if(baudConfig.hsbaudlow == 0) {
                return {f_clockSpeed, 2U + 2U * std::uint64_t{baudConfig.hsbaud}};
            }
            return {f_clockSpeed, 2U + std::uint64_t{baudConfig.hsbaud} + baudConfig.hsbaudlow};
        }
        if(baudConfig.hsbaud == 0 && baudConfig.hsbaudlow == 0) {
            // other
            if(baudConfig.baudlow == 0) {
                return {f_clockSpeed, 10U + 2U * std::uint64_t{baudConfig.baud}};
            }
            return {f_clockSpeed, 10U + std::uint64_t{baudConfig.baud} + baudConfig.baudlow};
        }
        return {0, 1};
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

    // Not constexpr: reaching it in clockTiming() is the compile error that says why.
    inline void i2cDeviceClockAboveFastModePlus() {}

    /// The transfer timeout's time per byte: 9 bits, 4 times over.
    constexpr std::uint32_t usPerDataByte(std::uint32_t f_baud) {
        constexpr std::uint32_t bitsPerDataByte = 9;
        constexpr std::uint32_t safetyFactor    = 4;
        return (bitsPerDataByte * 1'000'000 * safetyFactor) / f_baud;
    }

    /// calcBaudConfig and the tolerance check for a rate known only per device. High-speed mode
    /// is not offered per device: it needs a master code and CTRLA.SCLSM (28.6.2.4.6).
    template<std::intmax_t Num,
             std::intmax_t Denom>
    consteval ClockTiming clockTiming(std::uint32_t f_clockSpeed,
                                      std::uint32_t f_baud,
                                      std::ratio<Num,
                                                 Denom>) {
        if(f_baud == 0 || f_baud > maxSpeedFastPlus) { i2cDeviceClockAboveFastModePlus(); }
        auto const cfg = calcBaudConfig(f_clockSpeed, f_baud);
        // out of tolerance: the note "in call to 'rateOutOfTolerance(...)'" has the numbers
        Prescaler::requireInTolerance(achievedRate(f_clockSpeed, cfg),
                                      f_baud,
                                      Prescaler::Tolerance{std::ratio<Num, Denom>{}});
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

            /// Count the requests accepted for the wire (Sercom_I2CQueued's transfers()). Off,
            /// neither the counter nor its increment is in the image.
            static constexpr bool countTransfers = [] {
                if constexpr(requires { I2CConfig_::countTransfers; }) {
                    return static_cast<bool>(I2CConfig_::countTransfers);
                } else {
                    return false;
                }
            }();

            /// Tickets and cancel() (Sercom_I2CQueued's submitTracked/cancel; kvasir_devices BusTypes.hpp).
            static constexpr bool cancellable = [] {
                if constexpr(requires { I2CConfig_::cancellable; }) {
                    return static_cast<bool>(I2CConfig_::cancellable);
                } else {
                    return false;
                }
            }();

            /// Request::deadline: done by then, queue wait included, else timedOut.
            static constexpr bool requestDeadlines = [] {
                if constexpr(requires { I2CConfig_::requestDeadlines; }) {
                    return static_cast<bool>(I2CConfig_::requestDeadlines);
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

        static_assert(I2CConfig::baudRate <= maxSpeedHighSpeed,
                      "baudRate to big");
        // the achieved SCL against maxBaudRateError; a failure prints wanted, got and ppm
        static constexpr bool BaudInTolerance = [] {
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
        static_assert(BaudInTolerance);
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
