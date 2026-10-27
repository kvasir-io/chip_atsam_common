#pragma once
// The reset cause out of RCAUSE's bits, without registers (host-tested in tests/reset_cause_decode_test.cpp): each
// chip's PM.hpp reads its generated RCAUSE fields in one apply() and hands the result here with its own causes.
//
// RCAUSE "When a Reset occurs, the bit corresponding to the Reset source is set to '1' and all other bits are written
// to '0'" (C21 DS60001479M 18.8.1, E5x DS60001507N 16.8.1); the D21 register has one bit per source the same way
// (DS40001882L 16.8.14, reset value 0x01 = POR). So exactly one bit is the cause; no bit or several bits is no cause
// the datasheets describe, and reads as the chip's `unknown`.
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace Kvasir::Sam {
namespace Detail {
    template<typename Cause,
             std::size_t N,
             std::size_t... I>
    [[nodiscard,
      gnu::always_inline]] constexpr Cause
    causeOfMask(std::uint32_t        mask,
                std::array<Cause,
                           N> const& causes,
                Cause                unknown,
                std::index_sequence<I...>) {
        // one compare per cause, unrolled (a loop over the array stays a loop at -Oz)
        Cause found = unknown;
        static_cast<void>(
          ((mask == std::uint32_t{1} << I ? (found = causes[I], true) : false) || ...));
        return found;
    }
}   // namespace Detail

/// The cause whose bit is the one set in `mask` (bit I = causes[I]), or `unknown` when none or more than one is.
template<typename Cause,
         std::size_t N>
[[nodiscard,
  gnu::always_inline]] constexpr Cause
causeOfMask(std::uint32_t        mask,
            std::array<Cause,
                       N> const& causes,
            Cause                unknown) {
    static_assert(N <= 32);
    return Detail::causeOfMask(mask, causes, unknown, std::make_index_sequence<N>{});
}

/// causeOfMask() over a register read of N fields (`apply(read(R::a, R::b, ...))`), field I belonging to causes[I].
template<typename Cause,
         std::size_t N,
         typename Read>
[[nodiscard,
  gnu::always_inline]] inline Cause
oneCauseOf(Read const&          read,
           std::array<Cause,
                      N> const& causes,
           Cause                unknown) {
    return [&]<std::size_t... I>(std::index_sequence<I...>) {
        std::uint32_t const mask = ((std::uint32_t{get<I>(read) != 0} << I) | ...);
        return causeOfMask(mask, causes, unknown);
    }(std::make_index_sequence<N>{});
}
}   // namespace Kvasir::Sam
