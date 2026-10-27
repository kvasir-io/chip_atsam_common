#pragma once
// The SERCOM baud maths: the I2C host's BAUD/BAUDLOW (and the high-speed HSBAUD/HSBAUDLOW) with the
// SCL rate they give and the per-device clock timing, the USART's arithmetic-mode BAUD and the SPI's
// synchronous BAUD. Sercom_I2C.hpp, Sercom_Usart.hpp and Sercom_SPI.hpp write them into the
// registers. Standard headers and Kvasir/Util/Prescaler.hpp only: host-tested in
// tests/sercom_baud_test.cpp.
#include "kvasir/Util/Prescaler.hpp"

#include <algorithm>
#include <cstdint>
#include <ratio>

namespace Kvasir { namespace Sercom { namespace I2C { namespace Detail {

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

    /// A count of clock periods to the nearest whole one (a half rounds up), as an 8-bit register
    /// field: at or below 0 it is 0, above 255 it is 255.
    constexpr unsigned char nearestByte(double v) {
        constexpr double half{0.5};
        if(v <= 0.0) { return 0; }
        return static_cast<unsigned char>(std::min<std::int64_t>(std::int64_t(v + half), 255));
    }

    static constexpr BaudConfigRaw calcBaudConfigRaw(std::uint32_t fClockSpeed,
                                                     std::uint32_t fBaud,
                                                     double        val,
                                                     double        lowtime,
                                                     double        hightime) {
        double const baudraw = ((double(fClockSpeed) / double(fBaud)) - val);
        return {nearestByte(baudraw * (hightime / (lowtime + hightime))),
                nearestByte(baudraw * (lowtime / (lowtime + hightime)))};
    }

    // Not constexpr: reaching it in calcBaudConfig() is the compile error.
    inline void i2cBaudAndBaudlowBothZeroRefusedTheClockIsTooSlowForThisRate() {}

    /// BAUD = BAUDLOW = 0 is refused: "At a minimum, BAUD.BAUD and/or BAUD.BAUDLOW must be
    /// non-zero" (SAM D21 DS40001882L 28.6.2.4.1, SAM C20/C21 DS60001479M 33.6.2.4.1, SAM D5x/E5x
    /// DS60001507N 36.6.2.4.1, the Note under the SCL formulas). The clock is then too slow for
    /// the rate: f_SCL needs at least 10 + 1 GCLK periods. The note "in call to
    /// 'refuseBaudZero(...)'" has the numbers.
    constexpr void refuseBaudZero(std::uint32_t fClockSpeed,
                                  std::uint32_t fBaud) {
        static_cast<void>(fClockSpeed);
        static_cast<void>(fBaud);
        i2cBaudAndBaudlowBothZeroRefusedTheClockIsTooSlowForThisRate();
    }

    constexpr BaudConfig calcBaudConfig(std::uint32_t fClockSpeed,
                                        std::uint32_t fBaud) {
        if(fBaud <= maxSpeedStandard) {
            auto const raw = calcBaudConfigRaw(fClockSpeed, fBaud, 10, 4.7, 4.0);
            if(raw.baud == 0 && raw.baudlow == 0) { refuseBaudZero(fClockSpeed, fBaud); }
            return {raw.baud, raw.baudlow, 0, 0};
        }
        if(fBaud <= maxSpeedFast) {
            auto const raw = calcBaudConfigRaw(fClockSpeed, fBaud, 10, 1.3, 0.6);
            if(raw.baud == 0 && raw.baudlow == 0) { refuseBaudZero(fClockSpeed, fBaud); }
            return {raw.baud, raw.baudlow, 0, 0};
        }
        if(fBaud <= maxSpeedFastPlus) {
            auto const raw = calcBaudConfigRaw(fClockSpeed, fBaud, 10, 0.5, 0.26);
            if(raw.baud == 0 && raw.baudlow == 0) { refuseBaudZero(fClockSpeed, fBaud); }
            return {raw.baud, raw.baudlow, 0, 0};
        }
        // unreachable until high-speed mode is implemented: I2CBase refuses it (kept for that driver)
        auto const raw = calcBaudConfigRaw(fClockSpeed, fBaud, 2, 2.0, 1.0);
        return {0, 0, raw.baud, raw.baudlow};
    }

    /// The SCL rate the registers give, TRISE left out: f_SCL = f_GCLK / (10 + 2 BAUD) with BAUDLOW
    /// 0, f_GCLK / (10 + BAUD + BAUDLOW) otherwise; high-speed f_GCLK / (2 + 2 HSBAUD) or
    /// f_GCLK / (2 + HSBAUD + HSBAUDLOW) (SAM D21 DS40001882L 28.6.2.4.1).
    constexpr Prescaler::Rational achievedRate(std::uint32_t fClockSpeed,
                                               BaudConfig    baudConfig) {
        if(baudConfig.baud == 0 && baudConfig.baudlow == 0) {
            // high_speed
            if(baudConfig.hsbaudlow == 0) {
                return {fClockSpeed, 2U + 2U * std::uint64_t{baudConfig.hsbaud}};
            }
            return {fClockSpeed, 2U + std::uint64_t{baudConfig.hsbaud} + baudConfig.hsbaudlow};
        }
        if(baudConfig.hsbaud == 0 && baudConfig.hsbaudlow == 0) {
            // other
            if(baudConfig.baudlow == 0) {
                return {fClockSpeed, 10U + 2U * std::uint64_t{baudConfig.baud}};
            }
            return {fClockSpeed, 10U + std::uint64_t{baudConfig.baud} + baudConfig.baudlow};
        }
        return {0, 1};
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
    constexpr std::uint32_t usPerDataByte(std::uint32_t fBaud) {
        constexpr std::uint32_t bitsPerDataByte = 9;
        constexpr std::uint32_t safetyFactor    = 4;
        return (bitsPerDataByte * 1'000'000 * safetyFactor) / fBaud;
    }

    /// calcBaudConfig and the tolerance check for a rate known only per device. High-speed mode
    /// is not offered per device: it needs a master code and CTRLA.SCLSM (28.6.2.4.6).
    template<std::intmax_t Num,
             std::intmax_t Denom>
    consteval ClockTiming clockTiming(std::uint32_t fClockSpeed,
                                      std::uint32_t fBaud,
                                      std::ratio<Num,
                                                 Denom>) {
        if(fBaud == 0 || fBaud > maxSpeedFastPlus) { i2cDeviceClockAboveFastModePlus(); }
        auto const cfg = calcBaudConfig(fClockSpeed, fBaud);
        // out of tolerance: the note "in call to 'rateOutOfTolerance(...)'" has the numbers
        Prescaler::requireInTolerance(achievedRate(fClockSpeed, cfg),
                                      fBaud,
                                      Prescaler::Tolerance{std::ratio<Num, Denom>{}});
        return ClockTiming{.baud      = cfg.baud,
                           .baudlow   = cfg.baudlow,
                           .speed     = static_cast<std::uint8_t>(fBaud <= maxSpeedFast ? 0 : 1),
                           .usPerByte = static_cast<std::uint16_t>(usPerDataByte(fBaud))};
    }
}}}}   // namespace Kvasir::Sercom::I2C::Detail

namespace Kvasir { namespace Sercom { namespace Usart { namespace Detail {
    // Arithmetic mode, 16 samples per bit: f_baud = f_ref / 16 x (1 - BAUD / 65536), BAUD 16
    // bits (SAM D21 DS40001882L 25.6.2.3 Table 25-2; SAM C20/C21 Table 30-2; SAM D5x/E5x
    // Table 33-2). As a multiplier: f_baud = f_ref x n / 2^20 with n = 65536 - BAUD, 1..65536.
    static constexpr Prescaler::Multiplier arithmeticBaud{.first = 1,
                                                          .last  = 65536,
                                                          .den   = std::uint64_t{1} << 20};

    // the nearest BAUD; on a tie the larger BAUD (the slower rate), as round-half-up gave it
    constexpr auto baudDivider(std::uint32_t fClockSpeed,
                               std::uint32_t fBaud) {
        return Prescaler::fromMultiplier(fClockSpeed, fBaud, arithmeticBaud);
    }

    constexpr std::uint32_t calcBaudReg(std::uint32_t fClockSpeed,
                                        std::uint32_t fBaud) {
        return 65536U - baudDivider(fClockSpeed, fBaud).setting;
    }
}}}}   // namespace Kvasir::Sercom::Usart::Detail

namespace Kvasir { namespace Sercom { namespace SPI { namespace Detail {
    // f_sck = f_ref / (2 (BAUD + 1)) (SAM D21 DS40001882L 25.6.2.3 Table 25-2, synchronous;
    // SAM C20/C21 Table 30-2; SAM D5x/E5x Table 33-2); BAUD is 8 bits (D21 27.8.3). As a
    // divider 2 x (BAUD + 1), BAUD + 1 in 1..256, the closest SCK not above the request, as
    // Sercom_SPIQueued does it: the clock never runs faster than asked, unless even BAUD 255
    // is too fast.
    static constexpr Prescaler::Fixed sckDivider{.intMin   = 1,
                                                 .intMax   = 256,
                                                 .fracBits = 0,
                                                 .scale    = 2};

    constexpr auto baudDivider(std::uint32_t fClockSpeed,
                               std::uint32_t fBaud) {
        return Prescaler::fromFixedPoint(fClockSpeed, fBaud, sckDivider, Prescaler::Pick::notAbove);
    }

    constexpr std::uint32_t calcBaudReg(std::uint32_t fClockSpeed,
                                        std::uint32_t fBaud) {
        return baudDivider(fClockSpeed, fBaud).setting.integer - 1U;
    }
}}}}   // namespace Kvasir::Sercom::SPI::Detail
