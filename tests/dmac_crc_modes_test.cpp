// DmacCrcModes.hpp: which Kvasir::Crc presets the DMAC's CRC engine computes, its seeds, and the CRC from a CRCCHKSUM
// read - against Kvasir::Crc::Engine and the datasheets' own numbers. The engine's register is modelled the way its
// read rule describes it (CRC-32: the CRC is the bit-reversed, complemented "actual content"; D21 DS40001882L 20.8.4
// md l.13548): CRC-32 as an MSB-first register over bit-reversed bytes, CRC-16 as an MSB-first register. And the pace
// between two CRCDATAIN writes against both busy-cycle rules. What the silicon does is not modelled here: it takes a
// board.
#include "../DmacCrcModes.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

namespace D = Kvasir::DmaCrc;
namespace P = Kvasir::Crc::Presets;

namespace {
constexpr std::uint8_t reversedByte(std::uint8_t b) {
    return static_cast<std::uint8_t>(D::bitReversed(b) >> 24U);
}

// CRC-32 0x04C11DB7 MSB first over bit-reversed bytes: the register the read rule bit-reverses and complements
constexpr std::uint32_t engineRegister32(std::span<std::uint8_t const> data,
                                         std::uint32_t                 seed) {
    std::uint32_t r = seed;
    for(std::uint8_t const b : data) {
        r ^= std::uint32_t{reversedByte(b)} << 24U;
        for(int i = 0; i < 8; ++i) {
            r = (r & 0x8000'0000U) != 0 ? (r << 1U) ^ 0x04C1'1DB7U : r << 1U;
        }
    }
    return r;
}

// CRC-16 0x1021 MSB first: the register CRCCHKSUM reads as it is
constexpr std::uint32_t engineRegister16(std::span<std::uint8_t const> data,
                                         std::uint32_t                 seed) {
    std::uint16_t r = static_cast<std::uint16_t>(seed);
    for(std::uint8_t const b : data) {
        r = static_cast<std::uint16_t>(r ^ (b << 8U));
        for(int i = 0; i < 8; ++i) {
            r = static_cast<std::uint16_t>((r & 0x8000U) != 0 ? (r << 1U) ^ 0x1021U : r << 1U);
        }
    }
    return r;
}

constexpr std::string_view check = "123456789";

constexpr auto checkBytes = [] {
    std::array<std::uint8_t, 9> b{};
    for(std::size_t i = 0; i < b.size(); ++i) { b[i] = static_cast<std::uint8_t>(check[i]); }
    return b;
}();

// 300 bytes of a pattern, then room for the CRC appended little endian
constexpr std::size_t patternBytes = 300;

constexpr auto pattern = [] {
    std::array<std::uint8_t, patternBytes + 4> b{};
    std::uint32_t                              x = 0x1234'5678U;
    for(std::size_t i = 0; i < patternBytes; ++i) {
        x ^= x << 13U;
        x ^= x >> 17U;
        x ^= x << 5U;
        b[i] = static_cast<std::uint8_t>(x);
    }
    return b;
}();

template<auto Params>
constexpr std::uint32_t software(std::span<std::uint8_t const> data) {
    return Kvasir::Crc::Engine<Params, 0>::compute(data);
}

constexpr auto withCrcAppended(std::uint32_t crc) {
    auto b = pattern;
    for(std::size_t i = 0; i < 4; ++i) {
        b[patternBytes + i] = static_cast<std::uint8_t>(crc >> (8U * i));
    }
    return b;
}

// ---- which presets, which seeds
static_assert(D::dmacTypeFor<P::crc32IsoHdlc>() == D::CrcType::crc32);
static_assert(D::dmacTypeFor<P::crc16Xmodem>() == D::CrcType::crc16
              && D::dmacTypeFor<P::crc16Ibm3740>() == D::CrcType::crc16);
static_assert(D::dmacSeedFor<P::crc32IsoHdlc>() == 0xFFFF'FFFFU
              && D::dmacSeedFor<P::crc16Ibm3740>() == 0xFFFF'FFFFU
              && D::dmacSeedFor<P::crc16Xmodem>() == 0U);
static_assert(!D::dmacComputes<P::crc8MaximDow>() && !D::dmacComputes<P::crc8Nrsc5>()
              && !D::dmacComputes<P::crc8Htu21d>() && !D::dmacComputes<P::crc7Mmc>());

// ---- bitReversed
static_assert(D::bitReversed(0x0000'0001U) == 0x8000'0000U
              && D::bitReversed(0x8000'0000U) == 0x0000'0001U);
static_assert(D::bitReversed(0x04C1'1DB7U)
              == 0xEDB8'8320U);   // CRC-32's polynomial, normal -> reflected form
static_assert(D::bitReversed(D::bitReversed(0x1234'5678U)) == 0x1234'5678U);

// ---- the CRC from the engine's register: CRC-32 while CRCBUSY is set, CRC-16 always
static_assert(D::checksumOf<D::CrcType::crc32>(engineRegister32(checkBytes,
                                                                D::dmacSeedFor<P::crc32IsoHdlc>()),
                                               true)
              == P::crc32IsoHdlc.check);
static_assert(D::checksumOf<D::CrcType::crc16>(engineRegister16(checkBytes,
                                                                D::dmacSeedFor<P::crc16Ibm3740>()),
                                               true)
              == P::crc16Ibm3740.check);
static_assert(D::checksumOf<D::CrcType::crc16>(engineRegister16(checkBytes,
                                                                D::dmacSeedFor<P::crc16Xmodem>()),
                                               true)
              == P::crc16Xmodem.check);
static_assert(D::checksumOf<D::CrcType::crc32>(engineRegister32(pattern,
                                                                0xFFFF'FFFFU),
                                               true)
              == software<P::crc32IsoHdlc>(pattern));
static_assert(D::checksumOf<D::CrcType::crc16>(engineRegister16(pattern,
                                                                0xFFFF'FFFFU),
                                               false)
              == software<P::crc16Ibm3740>(pattern));
// CRCBUSY clear: CRC-32 reads as the CRC already, nothing to turn
static_assert(D::checksumOf<D::CrcType::crc32>(P::crc32IsoHdlc.check,
                                               false)
              == P::crc32IsoHdlc.check);
// nothing taken (a CRC of no bytes): the seed alone gives the software's empty CRC
static_assert(D::checksumOf<D::CrcType::crc32>(0xFFFF'FFFFU,
                                               true)
              == software<P::crc32IsoHdlc>({}));
static_assert(D::checksumOf<D::CrcType::crc16>(0xFFFF'FFFFU,
                                               true)
              == software<P::crc16Ibm3740>({}));

// ---- the datasheets' CRC-32 residue (CRCSTATUS.CRCZERO, D21 20.8.5 md l.13597, C21 25.8.5 md l.16495): the CRC
// appended little endian makes the final checksum 0x2144df1c; the complemented CRC appended makes the register 0
constexpr std::uint32_t patternCrc
  = software<P::crc32IsoHdlc>(std::span{pattern}.first(patternBytes));
static_assert(software<P::crc32IsoHdlc>(withCrcAppended(patternCrc)) == 0x2144'DF1CU);
static_assert(engineRegister32(withCrcAppended(~patternCrc),
                               0xFFFF'FFFFU)
              == 0);

// ---- the pace between two CRCDATAIN writes (crcDataInPace)
// CRCCTRL.CRCBEATSIZE: BYTE 0x0, HWORD 0x1, WORD 0x2 (D21 20.8.2, E5x 22.8.2 md l.17484-17486)
static_assert(D::crcBeatSize(1) == 0U && D::crcBeatSize(2) == 1U && D::crcBeatSize(4) == 2U);
// busy cycles: one a byte (the I/O paragraph: "a 32-bit data ... takes four cycles") and (CRCBEAT+1) (CRCDATAIN),
// whichever is more - a word is 4, not CRCDATAIN's 3
static_assert(D::crcDataInBusyCycles(1) == 1U && D::crcDataInBusyCycles(2) == 2U
              && D::crcDataInBusyCycles(4) == 4U);

// The engine's clock runs at the core's rate or faster, so a write is safe when at least `busy` core cycles lie between
// the last one's completion and it: the paced NOPs, then the store's own cycle. A byte needs only the store's own cycle
// (two bus writes are a cycle apart at least) and, where the errata ask, one instruction between two writes.
constexpr unsigned coreCyclesBetween(D::CrcDataInPace p) { return p.nops + 1U; }

template<std::size_t Bytes>
constexpr bool pacedEnough(bool gapErratum) {
    auto const p            = D::crcDataInPace(Bytes, gapErratum);
    bool const enoughCycles = coreCyclesBetween(p) >= D::crcDataInBusyCycles(Bytes)
                           && coreCyclesBetween(p) >= static_cast<unsigned>(Bytes)
                           && coreCyclesBetween(p) >= D::crcBeatSize(Bytes) + 1U;
    bool const erratumGap   = !gapErratum || p.nops >= 1U;   // an instruction between two writes
    // a store still in a write buffer would let the NOPs count before it reached the DMAC
    bool const fromCompletion = Bytes == 1 || p.barrier;
    return enoughCycles && erratumGap && fromCompletion;
}

static_assert(pacedEnough<1>(false) && pacedEnough<1>(true));
static_assert(pacedEnough<2>(false) && pacedEnough<2>(true));
static_assert(pacedEnough<4>(false) && pacedEnough<4>(true));
// and no more than that: the byte writes every engine user makes stay unpaced (E5x) or one NOP (D21/C21)
static_assert(!D::crcDataInPace(1,
                                false)
                 .barrier
              && D::crcDataInPace(1,
                                  false)
                     .nops
                   == 0U);
static_assert(!D::crcDataInPace(1,
                                true)
                 .barrier
              && D::crcDataInPace(1,
                                  true)
                     .nops
                   == 1U);
static_assert(D::crcDataInPace(2,
                               false)
                .barrier
              && D::crcDataInPace(2,
                                  false)
                     .nops
                   == 2U);
static_assert(D::crcDataInPace(4,
                               true)
                .barrier
              && D::crcDataInPace(4,
                                  true)
                     .nops
                   == 4U);
}   // namespace

int main() {
    // the same at run time, so a constexpr-only path cannot hide a difference
    int fail                    = 0;
    std::uint32_t volatile seed = 0xFFFF'FFFFU;
    std::uint32_t const raw     = engineRegister32(pattern, seed);
    std::uint32_t const crc     = D::checksumOf<D::CrcType::crc32>(raw, true);
    std::uint32_t const want
      = Kvasir::Crc::Engine<P::crc32IsoHdlc, 16>::compute(std::span{pattern});
    if(crc != want) {
        std::printf("FAIL: CRC-32 from the register %08x, software %08x\n", crc, want);
        ++fail;
    }
    seed = 0U;
    std::uint32_t const x16
      = D::checksumOf<D::CrcType::crc16>(engineRegister16(pattern, seed), true);
    std::uint32_t const w16 = Kvasir::Crc::Engine<P::crc16Xmodem, 16>::compute(std::span{pattern});
    if(x16 != w16) {
        std::printf("FAIL: CRC-16/XMODEM from the register %04x, software %04x\n", x16, w16);
        ++fail;
    }
    // the pace at run time too
    for(std::size_t const width : {std::size_t{1}, std::size_t{2}, std::size_t{4}}) {
        std::size_t volatile bytes = width;
        auto const p               = D::crcDataInPace(bytes, true);
        if(p.nops + 1U < D::crcDataInBusyCycles(bytes) || (width != 1 && !p.barrier) || p.nops < 1U)
        {
            std::printf("FAIL: pace of a %zu-byte write: barrier %d, %u NOPs\n",
                        width,
                        p.barrier ? 1 : 0,
                        p.nops);
            ++fail;
        }
    }
    std::printf("dmac_crc_modes: %s\n", fail == 0 ? "ok" : "FAILED");
    return fail;
}
