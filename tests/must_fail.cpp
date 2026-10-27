// One compile error per MUST_FAIL value; CMakeLists.txt checks that the compiler prints the message
// with the numbers. Without MUST_FAIL the file compiles (and is not built).
#include "../CanBitTiming.hpp"
#include "../ChipTraits.hpp"
#include "../SercomBaud.hpp"
#include "../WdtPeriod.hpp"
#include "TestLimits.hpp"

#include <array>
#include <cstdint>
#include <ratio>

namespace L = Kvasir::ClockLimits::Test;
using Kvasir::ClockLimits::Supply;

// a chip package as atsam_common/ChipTraits.hpp wants it, for the two cases below to take one name away from
namespace Fake {
struct Bound {};

struct NvmWithoutEraseBound {
    static constexpr bool        hasRwweeArray = false;
    static constexpr std::size_t pageSize      = 64;
    static constexpr std::size_t pagesPerRow   = 4;
    using WriteBound                           = Bound;

    static bool ready() { return true; }

    static std::uint8_t takeErrors() { return 0; }

    static bool inArray(std::uint32_t,
                        std::uint32_t,
                        bool) {
        return true;
    }

    static void setAddr(std::uint32_t) {}

    static void commandWritePage() {}

    static void commandEraseRow() {}

    static void commandPageBufferClear() {}

    static void disableCache() {}

    static void enableCache() {}
};

struct Nvm : NvmWithoutEraseBound {
    using EraseBound = Bound;
};

struct Start {
    static bool looksLikeAfterReset() { return true; }

    static void parkMainClock() {}
};

inline constexpr std::array<std::uint32_t, 4> serial{};

template<unsigned>
struct Registers {};

// what every fake package has; WithSpi false leaves out the SERCOM SPI register set
template<bool WithSpi>
struct Common {
    struct Sercom {};

    struct Dmac {};

    template<unsigned>
    using EicVector = int;
    template<unsigned I>
    using SercomUsart                                    = Registers<I>;
    using SercomPads                                     = Kvasir::Sam::Pads::SamD2xC2x;
    static constexpr auto const&   serialNumberAddresses = serial;
    static constexpr bool          hasCan                = false;
    static constexpr bool          eicSyncBusyInStatus   = true;
    static constexpr bool          wdtSyncBusyInStatus   = true;
    static constexpr std::uint32_t wdtFixedClockHz       = 0;
    static constexpr bool          dmacHasCrcEnable      = true;
};

template<>
struct Common<true> : Common<false> {
    template<unsigned I>
    using SercomSpi = Registers<I>;
};

template<typename N, bool WithStart, bool WithSpi = true>
struct Package : Common<WithSpi> {
    using Nvm   = N;
    using Start = Fake::Start;
};

template<typename N, bool WithSpi>
struct Package<N, false, WithSpi> : Common<WithSpi> {
    using Nvm = N;
};

static_assert(Kvasir::Sam::ChipPackage<Package<Nvm,
                                               true>>);
}   // namespace Fake

#if MUST_FAIL == 1
// a hand-written RWS too small for the supply
consteval void f() {
    Kvasir::Nvm::assertWaitStatesFrom<48'000'000, Supply::from1V62, 1, L::waitStates>();
}
#elif MUST_FAIL == 2
// a frequency the table has no row for
constexpr unsigned ws = Kvasir::Nvm::waitStatesFrom<49'000'000, Supply::from2V7, L::waitStates>();
#elif MUST_FAIL == 3
// DIV 128: reference 31 007.75 Hz
constexpr auto c
  = Kvasir::DPLL::assertXoscSetting<L::plain, L::plainWhere, 8'000'000, 128, 3095, 0>();
#elif MUST_FAIL == 4
// nothing reaches 200 MHz
constexpr auto s
  = Kvasir::DPLL::solveXosc<L::plain, L::plainWhere, 8'000'000, 200'000'000, 0, false>();
#elif MUST_FAIL == 5
// a reference taken directly, above f_IN
constexpr auto s
  = Kvasir::DPLL::solveReference<L::plain, L::plainWhere, 4'000'000, 48'000'000, 0, false>();
#elif MUST_FAIL == 6
// a field wider than its bits
constexpr auto c = Kvasir::DPLL::assertXoscSetting<L::plain, L::plainWhere, 8'000'000, 3, 47, 16>();
#elif MUST_FAIL == 7
// a DFLL closed loop outside the tolerance asked for: 1465 x 32768 Hz
consteval void f() {
    constexpr auto loop = Kvasir::DFLL::closedLoop<L::dfll48>(32'768, 48'000'000);
    Kvasir::Prescaler::assertInTolerance<loop.achieved,
                                         48'000'000,
                                         Kvasir::Prescaler::Tolerance::ppm(100),
                                         "DFLL48M closed loop">();
}
#elif MUST_FAIL == 8
// CAN: 8 MHz, 1 Mbit/s, 87.5 %: phase segment 2 of one quantum (NTSEG2 0, the erratum)
constexpr auto t = Kvasir::CAN::Detail::bitTiming(8'000'000, 1'000'000, 875);
#elif MUST_FAIL == 9
// CAN: the sample point at the very end of the bit, 8 MHz, 500 kbit/s
constexpr auto t = Kvasir::CAN::Detail::bitTiming(8'000'000, 500'000, 1000);
#elif MUST_FAIL == 10
// I2C: 8 MHz is too slow for 1 MHz, BAUD and BAUDLOW would both be 0
constexpr auto c = Kvasir::Sercom::I2C::Detail::calcBaudConfig(8'000'000, 1'000'000);
#elif MUST_FAIL == 11
// I2C: 10 MHz at 1 MHz, exactly 10 periods: still both 0
constexpr auto c = Kvasir::Sercom::I2C::Detail::calcBaudConfig(10'000'000, 1'000'000);
#elif MUST_FAIL == 12
// I2C, per device: refused before the tolerance check
constexpr auto c
  = Kvasir::Sercom::I2C::Detail::clockTiming(8'000'000, 1'000'000, std::ratio<1, 100>{});
#elif MUST_FAIL == 13
// CAN: 8 MHz, 500 kbit/s, 87.5 %: phase segment 2 of two quanta (NTSEG2 1)
constexpr auto t = Kvasir::CAN::Detail::bitTiming(8'000'000, 500'000, 875);
#elif MUST_FAIL == 14
// CAN: 8 MHz, 1 Mbit/s, 75 %: NTSEG2 1
constexpr auto t = Kvasir::CAN::Detail::bitTiming(8'000'000, 1'000'000, 750);
#elif MUST_FAIL == 15
// 20 s at 1.024 kHz: longer than 16384 cycles (16 s)
constexpr auto p = Kvasir::Sam::Wdt::periodFor(1'024, 20'000'000);
#elif MUST_FAIL == 16
// a chip package whose NVMCTRL traits lack EraseBound (atsam_common/StartUp.hpp makes the same assert)
static_assert(Kvasir::Sam::ChipPackage<Fake::Package<Fake::NvmWithoutEraseBound,
                                                     true>>,
              "the SAM chip package lacks a name the shared drivers use");
#elif MUST_FAIL == 17
// a chip package without its start traits
static_assert(Kvasir::Sam::ChipPackage<Fake::Package<Fake::Nvm,
                                                     false>>,
              "the SAM chip package lacks a name the shared drivers use");
#elif MUST_FAIL == 18
// ClockGate.hpp: a driver asks the interrupt lines of a block whose row states none
    #include "FakeGates.hpp"
using Lines = decltype(Kvasir::Sam::Block<Fake::PlainRegs>::irqs());
#elif MUST_FAIL == 19
// ... and the DMAC trigger sources of a block whose row states only its line
    #include "FakeGates.hpp"
constexpr auto rx = Kvasir::Sam::Block<Fake::TwoGateRegs>::dmaRx();
#elif MUST_FAIL == 20
// ... a register set at a base with no row at all
    #include "FakeGates.hpp"
using Row = Kvasir::Sam::Block<Fake::NoRowRegs>::Row;
#elif MUST_FAIL == 21
// a chip package without the SERCOM SPI register set the SPI drivers name (Sam::Package::SercomSpi)
static_assert(Kvasir::Sam::ChipPackage<Fake::Package<Fake::Nvm,
                                                     true,
                                                     false>>,
              "the SAM chip package lacks a name the shared drivers use");
#endif

int main() { return 0; }
