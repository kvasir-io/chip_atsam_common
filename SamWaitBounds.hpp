#pragma once
// The bounds of the shared SAM drivers' register waits (Kvasir_SDK kvasir/Register/Wait.hpp), each with its source.
// Each SAM chip's chip/WaitBounds.hpp includes this file after its CpuHzCeiling<Tag::User> specialisation - drivers
// include that one, not this file: MicrosecondsBound converts at the chip's ceiling. "l." = line of the converted
// datasheet (D21 DS40001882L, C21 DS60001479M, E5x DS60001507N). "Unchecked" = the datasheet gives no time; the value
// is a choice. Under the default wait policy (Unbounded) a site that does not name its policy is the loop it was.
#include "kvasir/Register/Wait.hpp"

#include <cstdint>

namespace Kvasir::Chip::Sam {
// A register synchronisation (SYNCBUSY / STATUS.SYNCBUSY after a write, ENABLE, SWRST, a SERCOM command's SYSOP):
// 5 P_GCLK + 2 P_APB < D < 6 P_GCLK + 3 P_APB (D21 14.3.1.8 l.4421 and 14.3.2.7 l.4488, C21 15.3.7 l.5706, E5x
// 13.3.7 l.5853). With GCLK and APB at 32.768 kHz that is 275 us; 1 ms. ENABLE and SWRST are tied to it only by
// "cleared by hardware when the operation is complete" (D21 14.3.2.5/14.3.2.6 l.4476/4480): no time of their own
// (unchecked), and a generator divided below 32 kHz waits longer than this bound.
using SyncBound = Kvasir::Register::MicrosecondsBound<1'000>;

// SERCOM I2C host, STATUS.BUSSTATE back to IDLE after the last transfer's STOP: "the I2C host can issue a Stop
// condition, which will change the Bus state back to IDLE" (D21 28.6.2.3 l.22391, C21 33.6.2.3 l.26423, E5x
// 36.6.2.3 l.42289), no time given (unchecked): 1 ms, ten SCL periods of a 10 kHz bus.
using I2cStopBound = Kvasir::Register::MicrosecondsBound<1'000>;

// DMAC CHCTRLA.ENABLE reading 0 after it was written 0: "disabled when the ongoing beat transfer is completed and the
// write-back memory section is updated" (D21 20.6.3.6 l.13123, C21 25.6.3.6 l.15969, E5x 22.6.3.7 l.16503; the
// register text says "burst", D21 20.8.18 l.14118), no cycle count given (unchecked): 1 ms.
using DmaChannelStopBound = Kvasir::Register::MicrosecondsBound<1'000>;

// USB SYNCBUSY after CTRLA.SWRST / ENABLE: the synchronisation delay above with GCLK_USB at 48 MHz (D21 32.5.3
// l.32802, E5x 38.5.3 l.46532; the C21 has no USB): 6 / 48 MHz + 3 P_APB. No minimum APB clock is given (only "the
// CLK_USB_AHB clock should be at minimum 8 MHz"): at a 32.768 kHz APB 92 us (unchecked); 1 ms. A missing GCLK_USB
// never synchronises - that is what the wait finds.
using UsbSyncBound = Kvasir::Register::MicrosecondsBound<1'000>;

// SERCOM SPI host, INTFLAG.TXC after the last character: "set when the data have been shifted out and there are no
// new data in DATA" (D21 27.8.6 l.21889, C21 32.8.6 l.25909, E5x 35.8.7 l.41699); "single buffered for
// transmitting" (D21 27.6.1 l.20924, C21 32.6.1 l.24920, E5x 35.6.1 l.40615), so DATA and the shift register are
// all that can be pending: two 8-bit characters at the slowest SCK, f_ref / 512 (BAUD 255; f_sck = f_ref / (2 (BAUD
// + 1)), D21 27.6.2.3).
template<std::uint64_t RefHz>
using SpiShifterBound = Kvasir::Register::MicrosecondsBound<2 * 8 * 512 * 1'000'000ULL / RefHz + 1>;

// XOSC ready after its enable: STARTUP n counts 2^n OSCULP32K cycles (+ 3 XOSC cycles) before XOSCRDY, 15 = 32768
// cycles, ~1 s (D21 17.8.5 SYSCTRL.XOSC l.8456-8487, C21 20.8.5 Table 20-5 l.9470-9490, E5x 28.8.6 XOSCCTRLn Table
// 28-6 l.33033 ff.); + 1 ms for the oscillator itself.
template<unsigned Startup>
    requires(Startup <= 15)
using XoscReadyBound
  = Kvasir::Register::MicrosecondsBound<(std::uint64_t{1} << Startup) * 1'000'000 / 32'768 + 1'000>;

// DSU CRC32 over the array (STATUSA.DONE): no time given (D21 13.11.3 l.3214, C21 13.12.3 l.4070, E5x 12.11.3
// l.4456; unchecked). Measured on a C21 at 48 MHz: ~22 MB/s, 256 KiB in ~12 ms; ImageCheck::SamDsu's default.
inline constexpr std::uint32_t dsuCrcTimeoutUs = 100'000;
}   // namespace Kvasir::Chip::Sam
