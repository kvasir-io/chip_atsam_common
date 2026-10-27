// WdtPeriod.hpp: the smallest CONFIG.PER whose period is at least the overrun time asked for (8 << PER cycles,
// PER 0..11), its nominal period, and the synchronisation bound. All static_asserts: building is the test; the
// refusal of a too long time is must_fail 15 (must_fail.cpp).
#include "../WdtPeriod.hpp"

namespace {
using namespace Kvasir::Sam::Wdt;

static_assert(cyclesOf(0) == 8 && cyclesOf(maxPer) == 16'384);

// 1.024 kHz, 2 s: 2048 cycles, PER 8, exactly 2 s
static_assert(periodFor(1'024,
                        2'000'000)
                  .per
                == 8
              && periodFor(1'024,
                           2'000'000)
                     .cycles
                   == 2'048);
static_assert(periodNs(1'024,
                       periodFor(1'024,
                                 2'000'000))
              == 2'000'000'000ULL);
// one microsecond more takes the next period
static_assert(periodFor(1'024,
                        2'000'001)
                .per
              == 9);
// 4.096 kHz, 4 s: PER 11, 16384 cycles
static_assert(periodFor(4'096,
                        4'000'000)
                  .per
                == 11
              && periodFor(4'096,
                           4'000'000)
                     .cycles
                   == 16'384);
// 1.024 kHz, 8 s: PER 10, 8192 cycles
static_assert(periodFor(1'024,
                        8'000'000)
                .per
              == 10);
// 1.024 kHz, 1 s: 1024 cycles, PER 7
static_assert(periodFor(1'024,
                        1'000'000)
                  .per
                == 7
              && periodNs(1'024,
                          periodFor(1'024,
                                    1'000'000))
                   == 1'000'000'000ULL);
// rounded up, never down: 1.5 s at 1.024 kHz is 1536 cycles -> 2048 (2 s)
static_assert(periodFor(1'024,
                        1'500'000)
                .per
              == 8);
// the shortest: anything up to 8 cycles
static_assert(periodFor(1'024,
                        0)
                  .per
                == 0
              && periodFor(1'024,
                           7'812)
                     .per
                   == 0);
// the longest at 1.024 kHz: 16 s
static_assert(periodFor(1'024,
                        16'000'000)
                .per
              == maxPer);

// 6 P_GCLK + 3 us, times ten: 1.024 kHz -> 58.63 ms
static_assert(syncBoundUs(1'024) == 58'630);
static_assert(syncBoundUs(32'768) == 1'870);   // 184 + 3
}   // namespace

int main() { return 0; }
