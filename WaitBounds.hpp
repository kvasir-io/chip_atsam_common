#pragma once
// The bounds of the shared SAM drivers' register waits (Kvasir_SDK kvasir/Register/Wait.hpp). Under the default
// wait policy (Unbounded) every site is the loop it was.
#include "kvasir/Register/Wait.hpp"

#include <cstdint>

namespace Kvasir::Chip::Sam {
// A register synchronisation: 5 P_GCLK + 2 P_APB < D < 6 P_GCLK + 3 P_APB (SAM D21 DS40001882L 14.3.1.8, md
// l.4417-4423; C21/E5x the same scheme). At a 32 kHz GCLK and the fastest SAM core (E5x, 120 MHz) that is ~23 000
// cycles, ~5 800 polls at 4 cycles each.
using SyncBound = Kvasir::Register::Bound::Polls<100'000>;
}   // namespace Kvasir::Chip::Sam
