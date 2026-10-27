#pragma once
// Kvasir::Chip::debuggerPresent() on the SAM D21, C21 and D5x/E5x: DSU STATUSB.DBGPRES, "set when a debugger probe
// is detected. This bit is never cleared" (D21 DS40001882L 13.13.3, C21 DS60001479M 13.14.3, E5x DS60001507N
// 12.13.3). The debug logic is reset by a power-on or an external reset only, not by a WDT reset or SYSRESETREQ
// (D21 Table 16-2, C21 Table 18-1, E5x Table 16-1). So it means "a probe was seen since the last power-on or external
// reset", not "a probe reads now": a board that had a probe on it keeps logging over RTT into an empty buffer until
// the next power cycle. Its declaration and the functor for uc_log are in DebuggerPresent.hpp (no includes).
#include "DebuggerPresent.hpp"
#include "peripherals/DSU.hpp"

// "There are no restrictions when accessing DSU registers from the CPU" (D21 DS40001882L 13.11.2.3): no PAC unlock for
// a read
inline bool Kvasir::Chip::debuggerPresent() {
    return apply(read(Kvasir::Peripheral::DSU::Registers<>::STATUSB::dbgpres)) != 0;
}
