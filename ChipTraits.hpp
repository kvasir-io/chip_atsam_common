#pragma once
// What a SAM chip package gives the shared drivers of atsam_common, as one concept. Each chip names its parts in one
// `Kvasir::Sam::Package` (src/chip/Package.hpp, aliases to what it has), and atsam_common/StartUp.hpp asserts
//
//     static_assert(Kvasir::Sam::ChipPackage<Kvasir::Sam::Package>, "...");
//
// so a chip that lacks a name fails there with the concept's note, not deep in a driver. The shared drivers name the
// chip's traits through it only (Sam::Package::Sercom, Dmac, Nvm, EicVector, Can), never by their chip-file names.
// Chip-free: the host tests (tests/must_fail.cpp) check it with fake packages.
//
// The members:
//   Sercom, Dmac            the SERCOM and DMAC instance traits (Sercom::Traits::SercomTraits, DMAC::Traits::DmacTraits)
//   SercomSpi<I>,           the register sets of SERCOMn in SPI host and USART mode (the D21/E51 SERCOM_SPI /
//   SercomUsart<I>          SERCOM_USART, the C21 SERCOM_SPIM / SERCOM_USART_INT): Sercom_SPI.hpp, Sercom_SPIQueued.hpp
//                           and Sercom_Usart.hpp name them, never a peripherals/SERCOM_*.hpp of their own
//   SercomPads              the pads DOPO / TXPO can put MOSI+SCK / TxD on (SercomPads.hpp: Pads::SamD2xC2x, SamE5x)
//   Nvm                     NVMCTRL::Traits: ready, takeErrors, inArray, setAddr, the page/row commands, the cache
//                           switch, EraseBound/WriteBound, pageSize, pagesPerRow, hasRwweeArray (the RWWEE commands
//                           only where it is true)
//   Start                   Startup::Sam::StartTraits: looksLikeAfterReset(), parkMainClock() (chip/Start.hpp)
//   EicVector<Line>         the interrupt vector of an EXTINT line (Interrupt::EicExtIntVector)
//   serialNumberAddresses   the four words of the serial number (SerialNumber::Traits::addresses)
//   hasCan, Can             CAN instance traits where the part has a CAN (CAN::Traits::CanTraits)
//   eicSyncBusyInStatus,    the chip differences of the shared drivers, read with if constexpr (a register name the
//   wdtSyncBusyInStatus,    other chip lacks is no test; a declaration that picks a register type only one family
//   wdtFixedClockHz,        has stays an #if): the EIC's and the WDT's write synchronisation in one STATUS.SYNCBUSY
//   dmacHasCrcEnable        bit (D21) or a SYNCBUSY register, the WDT's fixed clock (0: the GCLK the config names),
//                           the DMAC CRC's CTRL.CRCENABLE (D21/C21; the E5x turns it on by CRCSRC)

#include "SercomPads.hpp"

#include <concepts>
#include <cstddef>
#include <cstdint>

namespace Kvasir { namespace Sam {
    template<typename N>
    concept NvmTraits = requires(std::uint32_t a) {
        { N::ready() } -> std::same_as<bool>;
        { N::takeErrors() } -> std::same_as<std::uint8_t>;
        { N::inArray(a, a, true) } -> std::same_as<bool>;
        N::setAddr(a);
        N::commandWritePage();
        N::commandEraseRow();
        N::commandPageBufferClear();
        N::disableCache();
        N::enableCache();
        typename N::EraseBound;
        typename N::WriteBound;
        { N::pageSize } -> std::convertible_to<std::size_t>;
        { N::pagesPerRow } -> std::convertible_to<std::size_t>;
        { N::hasRwweeArray } -> std::convertible_to<bool>;
    } && (!N::hasRwweeArray || requires(std::uint32_t a) {
                            N::commandWriteRww();
                            N::commandEraseRww();
                            N::prepareEraseRww(a);
                        });

    template<typename S>
    concept StartTraits = requires {
        { S::looksLikeAfterReset() } -> std::same_as<bool>;
        { S::parkMainClock() } -> std::same_as<void>;
    };

    template<typename P>
    concept ChipPackage = requires {
        typename P::Sercom;
        typename P::Dmac;
        typename P::template EicVector<0>;
        typename P::template SercomSpi<0>;
        typename P::template SercomUsart<0>;
        { P::serialNumberAddresses.size() } -> std::convertible_to<std::size_t>;
        { P::hasCan } -> std::convertible_to<bool>;
        { P::eicSyncBusyInStatus } -> std::convertible_to<bool>;
        { P::wdtSyncBusyInStatus } -> std::convertible_to<bool>;
        { P::wdtFixedClockHz } -> std::convertible_to<std::uint32_t>;
        { P::dmacHasCrcEnable } -> std::convertible_to<bool>;
    } && NvmTraits<typename P::Nvm> && StartTraits<typename P::Start> && Pads::PadTable<typename P::SercomPads> && (!P::hasCan || requires {
                              typename P::Can;
                          });
}}   // namespace Kvasir::Sam
