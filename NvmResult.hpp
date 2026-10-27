#pragma once
// The result of a SAM NVM controller command (NVMCTRL.hpp), chip-neutral: each chip's NVMCTRL::Traits::takeErrors()
// reads and clears its error flags and hands them over as the mask ErrorBits::bits() builds. No chip header here (only the
// SDK's Util/Result.hpp), so the host tests (tests/nvm_result_test.cpp) can include it.
#include "kvasir/Util/Result.hpp"

#include <cstdint>
#include <expected>

namespace Kvasir::Nvm {

enum class Error : std::uint8_t {
    lock,   ///< LOCKE: the row is in a locked region or in the BOOTPROT section; nothing was done (D21 22.6.4.5)
    program,   ///< PROGE: a bad key, an invalid command, or a command while busy (D21 22.8.1); E5x also ADDRE
    nvm,       ///< NVME: an error from the NVM block itself (D21 22.8.7)
    timeout,   ///< READY did not come back within the datasheet time (x2)
    verify,    ///< the flash does not hold what was written (the read-back after every page)
};

template<typename T = void>
using Result = Kvasir::Result<T, Error>;

/// The chip-neutral error mask: what a chip's takeErrors() returns, and its decoding.
namespace ErrorBits {
    inline constexpr std::uint8_t lockBit    = 1U;
    inline constexpr std::uint8_t programBit = 2U;
    inline constexpr std::uint8_t nvmBit     = 4U;

    constexpr std::uint8_t bits(bool lock,
                                bool program,
                                bool nvm) {
        return static_cast<std::uint8_t>((lock ? lockBit : 0U) | (program ? programBit : 0U)
                                         | (nvm ? nvmBit : 0U));
    }

    // lock before program: an E5x erase in a locked region sets LOCKE "and INTFLAG.PROGE will also be set"
    // (DS60001507N 25.6.6.6), and that is a lock. Then program, then nvm.
    constexpr Result<> fromBits(std::uint8_t b) {
        if((b & lockBit) != 0) { return std::unexpected{Error::lock}; }
        if((b & programBit) != 0) { return std::unexpected{Error::program}; }
        if((b & nvmBit) != 0) { return std::unexpected{Error::nvm}; }
        return {};
    }
}   // namespace ErrorBits
}   // namespace Kvasir::Nvm
