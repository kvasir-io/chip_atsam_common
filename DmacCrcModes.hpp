#pragma once
// The DMAC's CRC engine without its registers (DmacCrc.hpp drives them; host test tests/dmac_crc_modes_test.cpp): which
// Kvasir::Crc::Params it computes, the seed for each, how CRCCHKSUM's content becomes the CRC, and the pace between two
// CRCDATAIN writes (crcDataInPace). The same on the
// D21, the C21 and the D5x/E5x (D21 DS40001882L 20.6.3.7 md l.13131-13184 and 20.8.2-20.8.5 md l.13429-13603; C21
// DS60001479M 25.6.3.7 md l.15977-16034 and 25.8.2-25.8.5 md l.16325-16499; E5x DS60001507N 22.6.3.8 md
// l.16511-16564 and 22.8.2-22.8.5 md l.17382-17604):
//   - two polynomials, CRCCTRL.CRCPOLY: CRC-16 0x1021 ("CRC-CCITT") and CRC-32 0x04C11DB7 ("IEEE 802.3");
//   - the seed is CRCCHKSUM, written while the CRC is off: "reset to zero by default, but it is possible to reset all
//     bits to one by writing the CRCCHKSUM register directly" - seed 0 or all ones, nothing else documented;
//   - CRC-32 is read "bit reversed ... and complemented" once CRCSTATUS.CRCBUSY is clear, and "the actual content"
//     while it is set (checksumOf below); CRC-16 is read as it is, "the 16 MSB bits are always read zero". So no
//     output reflection and no final xor for CRC-16, and only the complement for CRC-32.
// CRC-32 is CRC-32/ISO-HDLC: the datasheets give its residue - "appending the checksum at the end of the packet (as
// little endian), the final checksum should be 0x2144df1c" (CRCSTATUS.CRCZERO, D21 20.8.5 md l.13597, C21 25.8.5 md
// l.16495), which the host test checks against Kvasir::Crc. In which bit order CRC-16 takes a byte, the datasheets do
// not say: MSB first is the reading this driver has always made, which makes seed 0 CRC-16/XMODEM and seed 0xFFFF
// CRC-16/IBM-3740 ("CCITT-FALSE") - an assumption until silicon has been compared with the software, no datasheet fact.
// CRC-16/KERMIT and X-25 need a reflected result, which the engine never gives; they, CRC-32/BZIP2 and MPEG-2 (CRC-32
// MSB first), CRC-32C and every 7- or 8-bit CRC stay on the software.
#include "kvasir/Util/Crc.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace Kvasir::DmaCrc {

// CRCCTRL.CRCPOLY's two polynomials: crc16 is 0x1021, crc32 0x04C11DB7.
enum class CrcType { crc16, crc32 };

/// Whether the DMAC's CRC engine computes the CRC `P` describes (a Kvasir::Crc::Params): CRC-32/ISO-HDLC, or CRC-16
/// 0x1021 MSB first with seed 0 or 0xFFFF and no final xor (CRC-16/XMODEM, CRC-16/IBM-3740).
template<auto P>
consteval bool dmacComputes() {
    using T          = std::remove_cvref_t<decltype(P.poly)>;
    T const    all   = Kvasir::Crc::widthMask<T>(P.width);
    bool const crc32 = P.width == 32 && P.poly == 0x04C1'1DB7U && P.refin && P.refout
                    && P.init == all && P.xorout == all;
    bool const crc16 = P.width == 16 && P.poly == 0x1021U && !P.refin && !P.refout
                    && (P.init == 0 || P.init == all) && P.xorout == 0;
    return crc32 || crc16;
}

template<auto P>
consteval CrcType dmacTypeFor() {
    static_assert(
      dmacComputes<P>(),
      "the DMAC's CRC engine computes CRC-32/ISO-HDLC, CRC-16/XMODEM and CRC-16/IBM-3740 only");
    return P.width == 32 ? CrcType::crc32 : CrcType::crc16;
}

/// What CRCCHKSUM is set to before the first byte: 0 or all ones, the two seeds the datasheets name.
template<auto P>
consteval std::uint32_t dmacSeedFor() {
    static_assert(dmacComputes<P>());
    return P.init == 0 ? 0U : 0xFFFF'FFFFU;
}

/// Bit 31 swapped with bit 0, bit 30 with bit 1, ... (CRCCHKSUM's "bit reversed").
constexpr std::uint32_t bitReversed(std::uint32_t v) {
    v = ((v >> 1U) & 0x5555'5555U) | ((v & 0x5555'5555U) << 1U);
    v = ((v >> 2U) & 0x3333'3333U) | ((v & 0x3333'3333U) << 2U);
    v = ((v >> 4U) & 0x0F0F'0F0FU) | ((v & 0x0F0F'0F0FU) << 4U);
    v = ((v >> 8U) & 0x00FF'00FFU) | ((v & 0x00FF'00FFU) << 8U);
    return std::rotl(v, 16);
}

/// The CRC of what the engine has taken so far, from a CRCCHKSUM read and the CRCSTATUS.CRCBUSY read before it:
/// CRC-32 reads bit reversed and complemented with CRCBUSY clear and as "the actual content" with it set - turned into
/// the CRC here the same way, so the engine can go on; CRC-16 reads as it is (D21 20.8.4 md l.13548, C21 25.8.4 md
/// l.16446, E5x 22.8.4 md l.17539).
template<CrcType Type>
constexpr std::uint32_t checksumOf(std::uint32_t chksum,
                                   bool          busy) {
    if constexpr(Type == CrcType::crc16) {
        return chksum & 0xFFFFU;
    } else {
        return busy ? ~bitReversed(chksum) : chksum;
    }
}

/// CRCCTRL.CRCBEATSIZE for a CRCDATAIN write of `bytes` (1, 2 or 4): BYTE 0x0, HWORD 0x1, WORD 0x2 (D21 20.8.2 md
/// l.13481, C21 25.8.2 md l.16377, E5x 22.8.2 md l.17478-17488).
constexpr std::uint32_t crcBeatSize(std::size_t bytes) {
    return bytes == 1 ? 0U : bytes == 2 ? 1U : 2U;
}

/// The engine clock cycles a CRCDATAIN write of `bytes` keeps the CRC engine busy, the larger of the two rules the
/// datasheets give: the I/O interface's "CRC is done continuously for each byte. This means if a 32-bit data is written
/// to the CRCDATAIN register the CRC engine takes four cycles" (one a byte: D21 20.6.3.7 md l.13184, C21 25.6.3.7 md
/// l.16034, E5x 22.6.3.8 md l.16564) and CRCDATAIN's "A new CRC Checksum is ready (CRCBEAT+1) clock cycles after the
/// CRCDATAIN register is written" (D21 20.8.3 md l.13534, C21 25.8.3 md l.16432, E5x 22.8.3 md l.17525): 1, 2, 4.
constexpr unsigned crcDataInBusyCycles(std::size_t bytes) {
    auto const perByte = static_cast<unsigned>(bytes);
    auto const perBeat = static_cast<unsigned>(crcBeatSize(bytes)) + 1U;
    return perByte > perBeat ? perByte : perBeat;
}

/// What follows a CRCDATAIN write before the next one (DmacCrc.hpp detail::put()). A write of one busy cycle needs no
/// pace: two bus writes are at least one cycle of the module clock apart, which never runs slower than the core (the
/// engine is the DMAC's, clocked by CLK_DMAC_AHB - D21 20.5.3 md l.12399, C21 25.5.3 md l.15185, E5x 22.5.3 md l.15718
/// -, the AHB clock at the CPU clock's rate or above - D21 md l.5730 "always equal to the CPU clock", C21 md l.6748 one
/// synchronous domain at fCPU, E5x fHS >= fCPU md l.7316) - only the errata's instruction between two writes
/// (`gapErratum`: D21 DS80000760M 1.7.1, C21 DS80000740T 1.10.1). A longer one is paced: a DSB, so the write has reached
/// the DMAC before anything counts ("The DSB operation completes when all explicit memory accesses before this
/// instruction complete", ARMv6-M DDI0419E A3.7.3 md l.2122; Armv7-M DDI0403E A7.7.24 md l.8346), then one NOP for every
/// busy cycle (one cycle each: Cortex-M0+ TRM DDI0484C Table 3-1 md l.944, Cortex-M4 TRM 100166 Table 3-1 md l.1430),
/// and the next write after them. No wait on CRCSTATUS.CRCBUSY instead: by the bit's own description it does not clear
/// by itself on the I/O interface (DmacCrc.hpp).
struct CrcDataInPace {
    bool     barrier;   // a DSB right after the write
    unsigned nops;      // then this many NOPs
};

constexpr CrcDataInPace crcDataInPace(std::size_t bytes,
                                      bool        gapErratum) {
    unsigned const cycles = crcDataInBusyCycles(bytes);
    if(cycles <= 1U) { return {false, gapErratum ? 1U : 0U}; }
    return {true, cycles};
}

static_assert(dmacComputes<Kvasir::Crc::Presets::crc32IsoHdlc>()
              && dmacComputes<Kvasir::Crc::Presets::crc16Xmodem>()
              && dmacComputes<Kvasir::Crc::Presets::crc16Ibm3740>());
static_assert(!dmacComputes<Kvasir::Crc::Presets::crc16Kermit>()
              && !dmacComputes<Kvasir::Crc::Presets::crc16X25>()
              && !dmacComputes<Kvasir::Crc::Presets::crc32Bzip2>()
              && !dmacComputes<Kvasir::Crc::Presets::crc32Mpeg2>()
              && !dmacComputes<Kvasir::Crc::Presets::crc32c>()
              && !dmacComputes<Kvasir::Crc::Presets::crc8Smbus>());

}   // namespace Kvasir::DmaCrc
