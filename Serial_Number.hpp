#pragma once
#include "chip/Serial_Number_Traits.hpp"
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Util/HexString.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Kvasir {

// The chip's 128-bit serial number: its four words (chip/Serial_Number_Traits.hpp; NVM rows that never change), each
// copied in memory byte order. Memory::loadPlain: an aligned, non-volatile load as the cast and memcpy before it
// (Memory::load, a volatile access, made an E51 caller 80 B larger; loadBytes from an untyped address an
// __aeabi_uread4 call on the M0+). Internal linkage as before: an `inline` one changed how LTO folds it into its
// caller (E51 MainImpl +16 / -8 B).
static inline std::array<std::byte,
                         16>
serialNumber() {
    std::array<std::byte, 16> ret;   // every byte written below
    std::byte*                it = ret.data();
    for(auto const address : Kvasir::SerialNumber::Traits::addresses) {
        auto const word = Kvasir::Memory::loadPlain<std::uint32_t>(address);
        std::memcpy(it, &word, sizeof(word));
        it += sizeof(word);
    }
    return ret;
}

/// serialNumber() as 32 upper-case hex digits (Util/HexString.hpp, every chip's form).
static inline auto serialNumberString() { return Kvasir::hexString(Kvasir::serialNumber()); }
}   // namespace Kvasir
