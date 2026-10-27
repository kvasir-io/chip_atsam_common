// SercomBaud.hpp: the SERCOM BAUD maths, against the data sheet's formulas written out here.
//  - I2C host, standard/fast/fast-mode plus: f_SCL = f_GCLK / (10 + 2 BAUD + f_GCLK T_RISE) with BAUDLOW 0,
//    f_GCLK / (10 + BAUD + BAUDLOW + f_GCLK T_RISE) otherwise; high speed f_GCLK / (2 + 2 HSBAUD) or
//    f_GCLK / (2 + HSBAUD + HSBAUDLOW) (SAM D21 DS40001882L 28.6.2.4.1, "Clock Generation" and "Host Clock
//    Generation (High-Speed Mode)"). achievedRate and these checks leave the T_RISE term out.
//  - USART, asynchronous arithmetic mode, 16 samples: f_BAUD = f_ref / 16 x (1 - BAUD / 65536), BAUD 16 bits;
//    SPI (synchronous): f_BAUD = f_ref / (2 (BAUD + 1)), BAUD 8 bits (DS40001882L 25.6.2.3 Table 25-2).
// Needs Kvasir_SDK's Util/Prescaler.hpp only.
#include "../SercomBaud.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <ratio>

namespace {
namespace I2C   = Kvasir::Sercom::I2C::Detail;
namespace Usart = Kvasir::Sercom::Usart::Detail;
namespace SPI   = Kvasir::Sercom::SPI::Detail;
using Kvasir::Prescaler::Rational;

// 28.6.2.4.1 without T_RISE, as a rational f_GCLK / quanta
constexpr Rational sclRate(std::uint32_t   f,
                           I2C::BaudConfig c) {
    if(c.baudlow == 0) { return {f, 10U + 2U * std::uint64_t{c.baud}}; }
    return {f, 10U + std::uint64_t{c.baud} + c.baudlow};
}

constexpr Rational hsSclRate(std::uint32_t   f,
                             I2C::BaudConfig c) {
    if(c.hsbaudlow == 0) { return {f, 2U + 2U * std::uint64_t{c.hsbaud}}; }
    return {f, 2U + std::uint64_t{c.hsbaud} + c.hsbaudlow};
}

constexpr bool is(I2C::BaudConfig c,
                  unsigned        baud,
                  unsigned        baudlow,
                  unsigned        hsbaud,
                  unsigned        hsbaudlow) {
    return c.baud == baud && c.baudlow == baudlow && c.hsbaud == hsbaud && c.hsbaudlow == hsbaudlow;
}

// 8 MHz: 100 kHz = 80 GCLK periods, split 4.0 : 4.7 high : low (BAUD : BAUDLOW, minus the 10 fixed);
// 400 kHz = 20 periods, 0.6 : 1.3
constexpr auto s8 = I2C::calcBaudConfig(8'000'000, 100'000);
static_assert(is(s8,
                 32,
                 38,
                 0,
                 0));
static_assert(I2C::achievedRate(8'000'000,
                                s8)
              == Rational{8'000'000,
                          80});
static_assert(I2C::achievedRate(8'000'000,
                                s8)
              == sclRate(8'000'000,
                         s8));
constexpr auto f8 = I2C::calcBaudConfig(8'000'000, 400'000);
static_assert(is(f8,
                 3,
                 7,
                 0,
                 0));
static_assert(I2C::achievedRate(8'000'000,
                                f8)
              == Rational{8'000'000,
                          20});
static_assert(I2C::achievedRate(8'000'000,
                                f8)
              == sclRate(8'000'000,
                         f8));
// BAUD = BAUDLOW = 0 is refused at compile time: "At a minimum, BAUD.BAUD and/or BAUD.BAUDLOW must
// be non-zero" (28.6.2.4.1, the Note under the formulas). An SCL period needs at least 10 + 1 GCLK
// periods; 8 MHz and 10 MHz at 1 MHz (8 and 10) are must_fail cases (CMakeLists.txt), so is 8 MHz at
// 1 MHz through clockTiming (refused before its tolerance check). Just accepted: 11 MHz at 1 MHz,
// BAUD 0 and BAUDLOW 1, exactly 1 MHz.
constexpr auto p11 = I2C::calcBaudConfig(11'000'000, 1'000'000);
static_assert(is(p11,
                 0,
                 1,
                 0,
                 0));
static_assert(I2C::achievedRate(11'000'000,
                                p11)
              == Rational{11'000'000,
                          11});
static_assert(I2C::achievedRate(11'000'000,
                                p11)
              == sclRate(11'000'000,
                         p11));

// 48 MHz: 480, 120 and 48 periods
constexpr auto s48 = I2C::calcBaudConfig(48'000'000, 100'000);
static_assert(is(s48,
                 216,
                 254,
                 0,
                 0));
static_assert(I2C::achievedRate(48'000'000,
                                s48)
              == Rational{48'000'000,
                          480});
constexpr auto f48 = I2C::calcBaudConfig(48'000'000, 400'000);
static_assert(is(f48,
                 35,
                 75,
                 0,
                 0));
static_assert(I2C::achievedRate(48'000'000,
                                f48)
              == Rational{48'000'000,
                          120});
constexpr auto p48 = I2C::calcBaudConfig(48'000'000, 1'000'000);
static_assert(is(p48,
                 13,
                 25,
                 0,
                 0));
static_assert(I2C::achievedRate(48'000'000,
                                p48)
              == Rational{48'000'000,
                          48});
static_assert(I2C::achievedRate(48'000'000,
                                p48)
              == sclRate(48'000'000,
                         p48));

// High speed (above 1 MHz; the driver refuses it, the maths is kept for a high-speed driver): the
// HSBAUD fields, 2 fixed periods, high : low 1 : 2
constexpr auto hs48 = I2C::calcBaudConfig(48'000'000, 3'400'000);
static_assert(is(hs48,
                 0,
                 0,
                 4,
                 8));
static_assert(I2C::achievedRate(48'000'000,
                                hs48)
              == hsSclRate(48'000'000,
                           hs48));
static_assert(I2C::achievedRate(48'000'000,
                                hs48)
              == Rational{48'000'000,
                          14});
// both normal and high-speed fields set: no rate
static_assert(I2C::achievedRate(48'000'000,
                                I2C::BaudConfig{1,
                                                2,
                                                3,
                                                4})
              == Rational{0,
                          1});
// one BAUD for both halves when BAUDLOW is 0
static_assert(I2C::achievedRate(8'000'000,
                                I2C::BaudConfig{35,
                                                0,
                                                0,
                                                0})
              == Rational{8'000'000,
                          80});

// the rate classes
static_assert(I2C::maxSpeedStandard == 100'000 && I2C::maxSpeedFast == 400'000
              && I2C::maxSpeedFastPlus == 1'000'000 && I2C::maxSpeedHighSpeed == 3'400'000);

// clockTiming: the per-device BAUD, CTRLA.SPEED (1 only above fast mode) and 4 x 9 bit times per byte
constexpr auto t100 = I2C::clockTiming(48'000'000, 100'000, std::ratio<1, 100>{});
static_assert(t100.baud == 216 && t100.baudlow == 254 && t100.speed == 0 && t100.usPerByte == 360);
constexpr auto t400 = I2C::clockTiming(48'000'000, 400'000, std::ratio<1, 100>{});
static_assert(t400.baud == 35 && t400.baudlow == 75 && t400.speed == 0 && t400.usPerByte == 90);
constexpr auto t1m = I2C::clockTiming(48'000'000, 1'000'000, std::ratio<1, 100>{});
static_assert(t1m.baud == 13 && t1m.baudlow == 25 && t1m.speed == 1 && t1m.usPerByte == 36);
static_assert(I2C::usPerDataByte(100'000) == 360 && I2C::usPerDataByte(400'000) == 90);

// USART: 48 MHz / 115 200 -> n = 115200 x 2^20 / 48e6 = 2516.58 -> 2517, BAUD 63019; 8 MHz / 9600 ->
// 1258.29 -> 1258, BAUD 64278
static_assert(Usart::calcBaudReg(48'000'000,
                                 115'200)
              == 63019);
static_assert(Usart::calcBaudReg(8'000'000,
                                 9600)
              == 64278);
// a tie (n = 1.5 exactly): the larger BAUD, the slower rate
static_assert(Usart::calcBaudReg(1U << 21,
                                 3)
              == 65535);
// f_ref / 16, the fastest: BAUD 0
static_assert(Usart::calcBaudReg(16'000'000,
                                 1'000'000)
              == 0);

// SPI: 48 MHz / 1 MHz = 2 x 24; 7 MHz is not reachable, the next slower is 6 MHz (BAUD 3); 30 MHz ->
// 24 MHz (BAUD 0); 50 kHz would need BAUD 479: BAUD 255 (93.75 kHz), the one case above the request
static_assert(SPI::calcBaudReg(48'000'000,
                               1'000'000)
              == 23);
static_assert(SPI::calcBaudReg(48'000'000,
                               7'000'000)
              == 3);
static_assert(SPI::calcBaudReg(48'000'000,
                               30'000'000)
              == 0);
static_assert(SPI::calcBaudReg(48'000'000,
                               50'000)
              == 255);

// The formulas, the other way round: f_BAUD of a register value as a rational
constexpr Rational usartRate(std::uint64_t f,
                             std::uint64_t baud) {
    return {f * (65536U - baud), std::uint64_t{16U} * 65536U};
}

constexpr Rational spiRate(std::uint64_t f,
                           std::uint64_t baud) {
    return {f, 2U * (baud + 1U)};
}

// |r - want| compared across two rationals: a closer than b (or equal)
constexpr bool notFurther(Rational      a,
                          Rational      b,
                          std::uint64_t want) {
    auto const da = a.num > want * a.den ? a.num - want * a.den : want * a.den - a.num;
    auto const db = b.num > want * b.den ? b.num - want * b.den : want * b.den - b.num;
    return da * b.den <= db * a.den;
}

constexpr bool fasterThan(Rational      r,
                          std::uint64_t want) {
    return r.num > want * r.den;
}

int failures = 0;

void check(bool          ok,
           char const*   what,
           std::uint32_t f,
           std::uint32_t want,
           std::uint32_t got) {
    if(!ok) {
        ++failures;
        std::printf("FAIL %s: f_ref %u, wanted %u, BAUD %u\n", what, f, want, got);
    }
}
}   // namespace

int main() {
    constexpr std::array<std::uint32_t, 5> refs{1'000'000,
                                                8'000'000,
                                                16'000'000,
                                                32'768'000,
                                                48'000'000};
    constexpr std::array<std::uint32_t, 10>
      usartBauds{300, 1200, 9600, 19200, 31250, 57600, 115200, 250'000, 460'800, 921'600};
    constexpr std::array<std::uint32_t, 8>
        spiBauds{1'000, 50'000, 100'000, 400'000, 1'000'000, 4'000'000, 12'000'000, 24'000'000};
    int cases = 0;
    for(auto const f : refs) {
        for(auto const want : usartBauds) {
            if(std::uint64_t{want} * 16U > f) {
                continue;
            }   // above f_ref / 16: not arithmetic mode
            ++cases;
            auto const b = Usart::calcBaudReg(f, want);
            // nearest: BAUD - 1 is not closer; BAUD + 1 (the slower rate) is strictly further, since
            // it would win a tie
            bool ok = b <= 65535U;
            if(ok && b > 0) { ok = notFurther(usartRate(f, b), usartRate(f, b - 1), want); }
            if(ok && b < 65535U) { ok = !notFurther(usartRate(f, b + 1), usartRate(f, b), want); }
            check(ok, "usart nearest BAUD", f, want, b);
        }
        for(auto const want : spiBauds) {
            if(std::uint64_t{want} * 2U > f) { continue; }   // above f_ref / 2
            ++cases;
            auto const b  = SPI::calcBaudReg(f, want);
            bool       ok = b <= 255U;
            if(ok && b < 255U) {
                // never above the request, and BAUD - 1 would be
                ok = !fasterThan(spiRate(f, b), want)
                  && (b == 0 || fasterThan(spiRate(f, b - 1), want));
            } else if(ok) {
                // BAUD 255: whether it fits or nothing does, BAUD 254 is too fast
                ok = fasterThan(spiRate(f, 254), want);
            }
            check(ok, "spi slowest-not-above BAUD", f, want, b);
        }
    }
    std::printf("sercom baud: %d run-time cases, %d failures\n", cases, failures);
    return failures == 0 ? 0 : 1;
}
