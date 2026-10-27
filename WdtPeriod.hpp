#pragma once
// The SAM watchdog's time-out setting for a clock and a wanted overrun time, without registers (host-tested in
// tests/wdt_period_test.cpp); Wdt.hpp writes it.
//
// CONFIG.PER n is 8 << n cycles of the WDT clock, n = 0x0 .. 0xB (8 .. 16384 cycles; 0xC-0xF reserved) - D21
// DS40001882L 18.8.2, C21 DS60001479M 23.8.2, E5x DS60001507N 20.8.2. The setting is the SHORTEST period that is at
// least the overrun time asked for (the RP watchdog rounds up too); a time longer than 16384 cycles is refused at
// compile time. The period is the nominal one: the WDT clock comes from the OSCULP32K, which is not trimmed for the
// temperature and supply range (D21 Table 37-56: 25.559 .. 38.011 kHz calibrated at 25 C, so the real period is
// -14 % .. +28 % of the nominal one there). That tolerance is documented, not applied.
#include <cstdint>

namespace Kvasir::Sam::Wdt {
inline constexpr std::uint8_t maxPer = 0xB;

[[nodiscard]] constexpr std::uint32_t cyclesOf(std::uint8_t per) { return std::uint32_t{8} << per; }

struct Period {
    std::uint8_t  per;      // CONFIG.PER
    std::uint32_t cycles;   // 8 << per
};

// Not constexpr: a call in a constant evaluation is the compile error, and clang's note "in call to
// 'refuseOverrunTooLong(clock Hz, overrun us, longest us)'" has the numbers.
void wdtOverrunTimeLongerThanTheLongestPeriodLowerTheClockOrTheTime();

constexpr void refuseOverrunTooLong(std::uint64_t clockHz,
                                    std::uint64_t overrunUs,
                                    std::uint64_t longestUs) {
    static_cast<void>(clockHz);
    static_cast<void>(overrunUs);
    static_cast<void>(longestUs);
    wdtOverrunTimeLongerThanTheLongestPeriodLowerTheClockOrTheTime();
}

/// The smallest PER whose period at clockHz is at least overrunUs.
[[nodiscard]] constexpr Period periodFor(std::uint64_t clockHz,
                                         std::uint64_t overrunUs) {
    for(std::uint8_t per = 0; per <= maxPer; ++per) {
        if(std::uint64_t{cyclesOf(per)} * 1'000'000 >= overrunUs * clockHz) {
            return {per, cyclesOf(per)};
        }
    }
    refuseOverrunTooLong(clockHz, overrunUs, std::uint64_t{cyclesOf(maxPer)} * 1'000'000 / clockHz);
    return {maxPer, cyclesOf(maxPer)};
}

/// The nominal period of a setting, in nanoseconds.
[[nodiscard]] constexpr std::uint64_t periodNs(std::uint64_t clockHz,
                                               Period        p) {
    return std::uint64_t{p.cycles} * 1'000'000'000ULL / clockHz;
}

/// The bound of a WDT register synchronisation (CTRLA.ENABLE, CLEAR), in microseconds: 5 P_GCLK + 2 P_APB < D <
/// 6 P_GCLK + 3 P_APB (D21 DS40001882L 14.3.1.8, C21 DS60001479M 15.3.7, E5x DS60001507N 13.3.7), P_GCLK the WDT
/// clock's period. That the generic-clock formula holds for the C21's and E5x's CLK_WDT_OSC, which is no GCLK, is
/// unchecked; 3 P_APB is taken as 3 us (an APB clock of at least 1 MHz); the result times ten for both. At 1.024 kHz
/// 5.9 ms -> 58.6 ms.
[[nodiscard]] constexpr std::uint64_t syncBoundUs(std::uint64_t clockHz) {
    return 10 * ((6'000'000 + clockHz - 1) / clockHz + 3);
}
}   // namespace Kvasir::Sam::Wdt
