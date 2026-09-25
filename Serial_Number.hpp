#include "chip/Serial_Number_Traits.hpp"

#include <array>
#include <cstddef>

namespace Kvasir {

static inline std::array<std::byte,
                         16>
serial_number() {
    std::array<std::byte, 16> ret;
    auto                      it = ret.data();

    auto add = [&](std::uint32_t address) {
        auto v = reinterpret_cast<std::uint32_t const*>(address);
        std::memcpy(it, v, 4);
        it += 4;
    };

    for(auto address : Kvasir::SerialNumber::Traits::addresses) { add(address); }

    return ret;
};
}   // namespace Kvasir
