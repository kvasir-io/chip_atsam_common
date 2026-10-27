#pragma once
// The block index of an EepromEmulator page (NVMCTRL.hpp): a page's valid-block word starts at all
// ones (erased flash) and loses one more low bit per value written into the page, so value i is
// marked by the i + 1 low bits cleared. Chip-neutral, no chip header: host-tested in
// tests/nvm_layout_test.cpp.
#include <algorithm>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace Kvasir::Nvm {

// Is [addr, addr + bytes) inside the D21/C21 main array (mainArray) or the RWWEE array, given NVMCTRL.PARAM's fields?
// NVMP pages of 8 << PSZ bytes from address 0, RWWEEP pages of the same size from 0x00400000 (D21 DS40001882L 22.8.3,
// Table 10-1; C21 DS60001479M 27.8.3, Table 9-1). The controller checks nothing: an Erase Row past the end of a 128 KB
// part wraps around and erases a row of the main array (0x30000 -> 0x10000) with no flag set.
constexpr bool rangeInArray(std::uint32_t nvmp,
                            std::uint32_t psz,
                            std::uint32_t rwweep,
                            std::uint32_t addr,
                            std::uint32_t bytes,
                            bool          mainArray) {
    std::uint32_t const page = 8U << psz;
    std::uint32_t const base = mainArray ? 0U : 0x0040'0000U;
    std::uint32_t const size = page * (mainArray ? nvmp : rwweep);
    return addr >= base && addr - base <= size && bytes <= size - (addr - base);
}

template<typename Block, std::size_t NumTypes>
struct BlockIndex {
    using BlockType                              = Block;
    static constexpr auto        blockResetValue = std::numeric_limits<BlockType>::max();
    static constexpr std::size_t numTypes        = NumTypes;

    static constexpr BlockType blockFromIndex(std::size_t index) {
        assert(index == std::clamp<decltype(index)>(index, 0, numTypes));
        BlockType v = blockResetValue;

        for(std::size_t i = 0; i < index + 1; ++i) { v = v & (~(BlockType{1} << i)); }
        return v;
    }

    static constexpr bool validIndex(BlockType v) {
        auto const vv = (std::popcount(blockResetValue) - 1) - std::popcount(v);
        if(vv != std::clamp<decltype(vv)>(vv, 0, numTypes - 1)) { return false; }

        return blockFromIndex(static_cast<std::size_t>(vv)) == v;
    }

    static constexpr std::size_t getIndex(BlockType v) {
        assert(validIndex(v));
        auto const vv = (std::popcount(blockResetValue) - 1) - std::popcount(v);
        return static_cast<std::size_t>(vv);
    }

    static constexpr BlockType nextBlockValue(BlockType v) {
        if(validIndex(v)) {
            auto const newIndex = getIndex(v) + 1;
            if(newIndex >= numTypes) { return 0; }
            return blockFromIndex(newIndex);
        }
        if(v == blockResetValue) { return blockFromIndex(0); }
        return 0;
    }
};

}   // namespace Kvasir::Nvm
