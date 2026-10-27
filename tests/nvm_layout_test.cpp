// NvmLayout.hpp: the D2x/C2x array range check (compile time), then the EepromEmulator's block index. A page's valid-block word is all ones when erased,
// value i is marked by its i + 1 low bits cleared, the word after the last value is 0 (page full).
// Compile-time checks for the shapes, then every 8- and 16-bit word at run time. Needs nothing but the
// standard library.
#include "../NvmLayout.hpp"

#include <cstdint>
#include <cstdio>

namespace {
using Kvasir::Nvm::BlockIndex;

// rangeInArray: a 128 KB D21 (NVMP 2048 pages of 64 B, PSZ 3) with a 4 KB RWWEE array
// (RWWEEP 64 pages) at 0x00400000
constexpr bool inMain(std::uint32_t a,
                      std::uint32_t n) {
    return Kvasir::Nvm::rangeInArray(2048, 3, 64, a, n, true);
}

constexpr bool inRwwee(std::uint32_t a,
                       std::uint32_t n) {
    return Kvasir::Nvm::rangeInArray(2048, 3, 64, a, n, false);
}

static_assert(inMain(0,
                     256)
              && inMain(0x1FF00,
                        256)
              && inMain(0x20000,
                        0));
// the wrap that erased a row of the running image: ER at 0x30000 on the 128 KB part
static_assert(!inMain(0x30000,
                      256)
              && !inMain(0x1FF01,
                         256)
              && !inMain(0x20000,
                         1));
// a length that would overflow addr + bytes
static_assert(!inMain(0x100,
                      0xFFFF'FF80U));
static_assert(inRwwee(0x0040'0000,
                      256)
              && inRwwee(0x0040'0F00,
                         256));
static_assert(!inRwwee(0x0040'1000,
                       256)
              && !inRwwee(0x0000'0000,
                          256)
              && !inRwwee(0x003F'FF00,
                          256));
// a 256 KB C21 has 4096 pages; PSZ 3 again
static_assert(Kvasir::Nvm::rangeInArray(4096,
                                        3,
                                        128,
                                        0x3FF00,
                                        256,
                                        true));
static_assert(!Kvasir::Nvm::rangeInArray(4096,
                                         3,
                                         128,
                                         0x40000,
                                         256,
                                         true));

using I8 = BlockIndex<std::uint8_t, 7>;
static_assert(I8::blockResetValue == 0xFF);
static_assert(I8::blockFromIndex(0) == 0xFE && I8::blockFromIndex(1) == 0xFC
              && I8::blockFromIndex(6) == 0x80);
// one past the last index is allowed and gives the full-page word
static_assert(I8::blockFromIndex(7) == 0x00);
static_assert(I8::validIndex(0xFE) && I8::validIndex(0x80));
static_assert(!I8::validIndex(0xFF) && !I8::validIndex(0x00));
// bit 1 cleared without bit 0: not a block word
static_assert(!I8::validIndex(0xFD) && !I8::validIndex(0x7F));
static_assert(I8::getIndex(0xFE) == 0 && I8::getIndex(0xC0) == 5 && I8::getIndex(0x80) == 6);
// erased -> index 0 -> ... -> index 6 -> 0 (full); anything else -> 0
static_assert(I8::nextBlockValue(0xFF) == 0xFE);
static_assert(I8::nextBlockValue(0xFE) == 0xFC);
static_assert(I8::nextBlockValue(0x80) == 0x00);
static_assert(I8::nextBlockValue(0x00) == 0x00);
static_assert(I8::nextBlockValue(0xFD) == 0x00);

using I16 = BlockIndex<std::uint16_t, 15>;
static_assert(I16::blockFromIndex(0) == 0xFFFE && I16::blockFromIndex(14) == 0x8000);
static_assert(I16::validIndex(0x8000) && !I16::validIndex(0x0000) && !I16::validIndex(0xFFFF));
static_assert(I16::nextBlockValue(0xC000) == 0x8000 && I16::nextBlockValue(0x8000) == 0);

using I32 = BlockIndex<std::uint32_t, 31>;
static_assert(I32::blockFromIndex(0) == 0xFFFF'FFFEU && I32::blockFromIndex(30) == 0x8000'0000U);
static_assert(I32::getIndex(0x8000'0000U) == 30 && I32::nextBlockValue(0x8000'0000U) == 0);
static_assert(I32::nextBlockValue(0xFFFF'FFFFU) == 0xFFFF'FFFEU);

// fewer values than bits: the index ends at NumTypes - 1, the words below it are not valid
using Small = BlockIndex<std::uint8_t, 3>;
static_assert(Small::validIndex(0xF8) && Small::getIndex(0xF8) == 2);
static_assert(!Small::validIndex(0xF0) && !Small::validIndex(0x80));
static_assert(Small::nextBlockValue(0xFC) == 0xF8 && Small::nextBlockValue(0xF8) == 0);

int failures = 0;

// validIndex(v) holds exactly for blockFromIndex(0 .. N-1); nextBlockValue of each word is the next
// index's word, 0 after the last, 0 for any other word than these and the erased one
template<typename Index>
void exhaustive(char const* name) {
    using W                   = typename Index::BlockType;
    constexpr std::uint32_t n = 1U << (8U * sizeof(W));
    for(std::uint32_t raw = 0; raw != n; ++raw) {
        auto const  v     = static_cast<W>(raw);
        std::size_t index = Index::numTypes;
        for(std::size_t k = 0; k != Index::numTypes; ++k) {
            if(Index::blockFromIndex(k) == v) { index = k; }
        }
        bool const valid = index != Index::numTypes;
        bool       ok    = Index::validIndex(v) == valid;
        W const    next  = Index::nextBlockValue(v);
        if(valid) {
            ok = ok && Index::getIndex(v) == index
              && next == (index + 1 == Index::numTypes ? W{0} : Index::blockFromIndex(index + 1));
        } else if(v == Index::blockResetValue) {
            ok = ok && next == Index::blockFromIndex(0);
        } else {
            ok = ok && next == 0;
        }
        if(!ok) {
            ++failures;
            std::printf("FAIL %s: word 0x%x\n", name, static_cast<unsigned>(raw));
        }
    }
}
}   // namespace

int main() {
    exhaustive<I8>("uint8_t/7");
    exhaustive<Small>("uint8_t/3");
    exhaustive<BlockIndex<std::uint8_t, 1>>("uint8_t/1");
    exhaustive<I16>("uint16_t/15");
    exhaustive<BlockIndex<std::uint16_t, 9>>("uint16_t/9");
    std::printf("nvm layout: %d failures\n", failures);
    return failures == 0 ? 0 : 1;
}
