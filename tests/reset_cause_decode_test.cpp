// ResetCauseDecode.hpp: exactly one RCAUSE bit is that cause; no bit or several bits read as `unknown`. All
// static_asserts (causeOfMask) plus oneCauseOf() over a tuple standing in for a register read.
#include "../ResetCauseDecode.hpp"

#include <array>
#include <cstdint>
#include <tuple>

namespace {
enum class Cause : std::uint8_t { por, bod12, bod33, ext, wdt, syst, unknown };

constexpr std::array causes{Cause::por,
                            Cause::bod12,
                            Cause::bod33,
                            Cause::ext,
                            Cause::wdt,
                            Cause::syst};

constexpr Cause decode(unsigned mask) {
    return Kvasir::Sam::causeOfMask(mask, causes, Cause::unknown);
}

// each single bit is its own cause
static_assert(decode(0b000001) == Cause::por);
static_assert(decode(0b000010) == Cause::bod12);
static_assert(decode(0b000100) == Cause::bod33);
static_assert(decode(0b001000) == Cause::ext);
static_assert(decode(0b010000) == Cause::wdt);
static_assert(decode(0b100000) == Cause::syst);
// none, two, all: no cause the datasheet describes
static_assert(decode(0) == Cause::unknown);
static_assert(decode(0b100001) == Cause::unknown);
static_assert(decode(0b011000) == Cause::unknown);
static_assert(decode(0b111111) == Cause::unknown);
}   // namespace

int main() {
    // a read result: get<I> found by argument-dependent lookup, as on Kvasir's register read
    auto const wdt = std::
      tuple<std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t>{0,
                                                                                                0,
                                                                                                0,
                                                                                                0,
                                                                                                1,
                                                                                                0};
    auto const several = std::
      tuple<std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t>{1,
                                                                                                0,
                                                                                                0,
                                                                                                0,
                                                                                                1,
                                                                                                0};
    if(Kvasir::Sam::oneCauseOf(wdt, causes, Cause::unknown) != Cause::wdt) { return 1; }
    if(Kvasir::Sam::oneCauseOf(several, causes, Cause::unknown) != Cause::unknown) { return 2; }
    return 0;
}
