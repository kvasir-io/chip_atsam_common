// Kvasir::Nvm::Detail::fromBits (NvmResult.hpp): which error a set of NVM controller flags is. lock wins over program
// (an E5x erase in a locked region raises both, DS60001507N 25.6.6.6), program over nvm; no flag is success. And each
// chip's flag positions, as its NVMCTRL_Traits::takeErrors() reads them: D21/C21 STATUS PROGE 2, LOCKE 3, NVME 4
// (DS40001882L 22.8.7, DS60001479M 27.8.7); E5x INTFLAG ADDRE 1, PROGE 2, LOCKE 3, NVME 6 (DS60001507N 25.8.6).
#include "../NvmResult.hpp"

#include <cstdint>
#include <cstdio>

namespace N = Kvasir::Nvm;
namespace D = Kvasir::Nvm::Detail;

namespace {
constexpr bool is(N::Result<> r,
                  N::Error    e) {
    return !r && r.error() == e;
}

// the masks the traits apply, kept here as numbers to check the decoding against the data sheets' bit tables
constexpr std::uint8_t samD21C21(std::uint16_t status) {
    return D::bits((status & 0x08U) != 0, (status & 0x04U) != 0, (status & 0x10U) != 0);
}

constexpr std::uint8_t samE5x(std::uint16_t intflag) {
    return D::bits((intflag & 0x08U) != 0, (intflag & 0x06U) != 0, (intflag & 0x40U) != 0);
}

static_assert(D::fromBits(0).has_value());
static_assert(is(D::fromBits(D::LockBit),
                 N::Error::lock));
static_assert(is(D::fromBits(D::ProgramBit),
                 N::Error::program));
static_assert(is(D::fromBits(D::NvmBit),
                 N::Error::nvm));
static_assert(is(D::fromBits(D::LockBit | D::ProgramBit),
                 N::Error::lock));
static_assert(is(D::fromBits(D::ProgramBit | D::NvmBit),
                 N::Error::program));
static_assert(is(D::fromBits(D::LockBit | D::ProgramBit | D::NvmBit),
                 N::Error::lock));

static_assert(D::fromBits(samD21C21(0x0002)).has_value());   // LOAD alone is no error
static_assert(D::fromBits(samD21C21(0x0001)).has_value());   // PRM
static_assert(is(D::fromBits(samD21C21(0x0004)),
                 N::Error::program));
static_assert(is(D::fromBits(samD21C21(0x0008)),
                 N::Error::lock));
static_assert(is(D::fromBits(samD21C21(0x0010)),
                 N::Error::nvm));
static_assert(D::fromBits(samD21C21(0x0100)).has_value());   // SB, the security bit

static_assert(D::fromBits(samE5x(0x0001)).has_value());   // DONE
static_assert(is(D::fromBits(samE5x(0x0002)),
                 N::Error::program));   // ADDRE
static_assert(is(D::fromBits(samE5x(0x0004)),
                 N::Error::program));
static_assert(is(D::fromBits(samE5x(0x000C)),
                 N::Error::lock));   // a locked erase: LOCKE and PROGE
static_assert(is(D::fromBits(samE5x(0x0040)),
                 N::Error::nvm));
static_assert(
  D::fromBits(samE5x(0x0030)).has_value());   // ECCSE/ECCDE: read errors, not command errors
}   // namespace

int main() {
    // the same at run time, so a sanitizer sees the code once
    int failed = 0;
    for(unsigned b = 0; b != 8; ++b) {
        auto const r  = D::fromBits(static_cast<std::uint8_t>(b));
        bool const ok = b == 0                   ? r.has_value()
                      : (b & D::LockBit) != 0    ? is(r, N::Error::lock)
                      : (b & D::ProgramBit) != 0 ? is(r, N::Error::program)
                                                 : is(r, N::Error::nvm);
        if(!ok) {
            std::printf("FAIL fromBits(%u)\n", b);
            ++failed;
        }
    }
    return failed == 0 ? 0 : 1;
}
