#pragma once
// Which SERCOM pads the SPI host and the USART can put their signals on, as per-chip tables: CTRLA.DOPO places data
// out (MOSI), SCK and the hardware SS of an SPI host, CTRLA.TXPO places TxD of a USART. Each chip package names its
// table (`Sam::Package::SercomPads`); Sercom_SPI.hpp and Sercom_Usart.hpp take DOPO and TXPO from it, and a pad choice
// the chip does not offer is a compile error naming the pads and the chip. DIPO and RXPO are the pad number on every
// chip (any of PAD[0..3]).
//
// Register-free (host test tests/sercom_pads_test.cpp, must_fail cases).

#include "kvasir/Io/Utility.hpp"
#include "kvasir/Util/Diagnostic.hpp"

#include <array>
#include <concepts>
#include <cstdint>
#include <optional>
#include <string_view>

namespace Kvasir { namespace Sam { namespace Pads {

    /// CTRLA.DOPO: data out (MOSI in host operation), SCK and the host's SS when CTRLB.MSSEN = 1
    struct DopoRow {
        int dopo;
        int dataOut;
        int sck;
        int ss;
    };

    /// CTRLA.TXPO: the pad of TxD. Only the settings without flow control or RS485 are listed: the driver drives no
    /// XCK, RTS, TE or CTS pad.
    struct TxpoRow {
        int txpo;
        int txd;
    };

    /// SAM D21 and C21: every DOPO; TXPO 0x0 (TxD PAD[0]) and 0x1 (TxD PAD[2]).
    /// D21 DS40001882L 27.8.1 CTRLA.DOPO (md l.21489-21492), 26.8.1 CTRLA.TXPO (md l.19961-19964);
    /// C21 DS60001479M 32.8.1 CTRLA.DOPO (md l.25494-25497), 31.8.1 CTRLA.TXPO (md l.23849-23852).
    struct SamD2xC2x {
        static constexpr std::string_view name = "SAM D21/C21";
        static constexpr std::array       dopo{
          DopoRow{0, 0, 1, 2},
          DopoRow{1, 2, 3, 1},
          DopoRow{2, 3, 1, 2},
          DopoRow{3, 0, 3, 1},
        };
        static constexpr std::array txpo{
          TxpoRow{0, 0},
          TxpoRow{1, 2},
        };
    };

    /// SAM D5x/E5x: DOPO 0x1 and 0x3 are Reserved, TXPO 0x1 is Reserved - TxD is on PAD[0] only.
    /// DS60001507N 35.8.1 CTRLA.DOPO (md l.41271-41274), 34.8.1 CTRLA.TXPO (md l.39328-39331).
    struct SamE5x {
        static constexpr std::string_view name = "SAM D5x/E5x";
        static constexpr std::array       dopo{
          DopoRow{0, 0, 1, 2},
          DopoRow{2, 3, 1, 2},
        };
        static constexpr std::array txpo{
          TxpoRow{0, 0},
        };
    };

    /// What a chip package's `SercomPads` must give.
    template<typename T>
    concept PadTable = requires {
        { T::name } -> std::convertible_to<std::string_view>;
        T::dopo.size();
        T::txpo.size();
    };

    template<PadTable Table>
    constexpr std::optional<DopoRow> findDopo(int dataOut,
                                              int sck) {
        for(auto const& row : Table::dopo) {
            if(row.dataOut == dataOut && row.sck == sck) { return row; }
        }
        return std::nullopt;
    }

    template<PadTable Table>
    constexpr std::optional<TxpoRow> findTxpo(int txd) {
        for(auto const& row : Table::txpo) {
            if(row.txd == txd) { return row; }
        }
        return std::nullopt;
    }

    namespace Detail {
        template<PadTable Table, int MosiPad, int SckPad>
        struct DopoRefused {
            consteval Diagnostic::Text operator()() const {
                Diagnostic::Text t;
                t << "SPI pads refused on " << Table::name << ": no CTRLA.DOPO puts MOSI on PAD[";
                t.dec(MosiPad) << "] with SCK on PAD[";
                t.dec(SckPad) << "]; this chip offers";
                for(auto const& row : Table::dopo) {
                    t << " (MOSI PAD[";
                    t.dec(static_cast<std::uint64_t>(row.dataOut)) << "], SCK PAD[";
                    t.dec(static_cast<std::uint64_t>(row.sck)) << "])";
                }
                return t;
            }
        };

        template<PadTable Table, int TxPad>
        struct TxpoRefused {
            consteval Diagnostic::Text operator()() const {
                Diagnostic::Text t;
                t << "USART pads refused on " << Table::name << ": no CTRLA.TXPO puts TxD on PAD[";
                t.dec(TxPad) << "]; this chip offers";
                for(auto const& row : Table::txpo) {
                    t << " PAD[";
                    t.dec(static_cast<std::uint64_t>(row.txd)) << "]";
                }
                return t;
            }
        };

        template<int SsPad, int CsPad>
        struct SsRefused {
            consteval Diagnostic::Text operator()() const {
                Diagnostic::Text t;
                t << "SPI pads refused: the hardware SS (csPinLocation) is on PAD[";
                t.dec(CsPad) << "], the chosen DOPO puts it on PAD[";
                t.dec(SsPad) << "]";
                return t;
            }
        };
    }   // namespace Detail

    /// The DOPO row for MOSI on MosiPad and SCK on SckPad; a pair the chip does not offer is a compile error.
    template<PadTable Table,
             int      MosiPad,
             int      SckPad>
    constexpr DopoRow dopo() {
        constexpr auto row     = findDopo<Table>(MosiPad, SckPad);
        constexpr bool offered = row.has_value();
        KVASIR_STATIC_ASSERT(offered, (Detail::DopoRefused<Table, MosiPad, SckPad>));
        return row.value_or(DopoRow{});
    }

    /// The hardware SS on CsPad must be where the DOPO row puts SS.
    template<DopoRow Row,
             int     CsPad>
    constexpr void checkSs() {
        constexpr bool same = Row.ss == CsPad;
        KVASIR_STATIC_ASSERT(same, (Detail::SsRefused<Row.ss, CsPad>));
    }

    /// TXPO for TxD on TxPad; a pad the chip does not offer is a compile error.
    template<PadTable Table,
             int      TxPad>
    constexpr int txpo() {
        constexpr auto row     = findTxpo<Table>(TxPad);
        constexpr bool offered = row.has_value();
        KVASIR_STATIC_ASSERT(offered, (Detail::TxpoRefused<Table, TxPad>));
        return row.value_or(TxpoRow{}).txpo;
    }

    /// Loop-back (D21 27.6.3.4 / 26.6.3.6, C21 32.6.3.4 / 31.6.3.8, E5x 35.6.3.4 / 34.6.3.9: "configure ... to use the
    /// same data pins for transmit and receive. The loop-back is through the pad"): with `loopBack` the receive pin
    /// is the transmit pin, so DIPO/RXPO name the data-out / TxD pad and the pin is configured once; without it the
    /// two must differ. D21 DS40001882L md l.21193 / l.19658, C21 DS60001479M l.25188 / l.23524, E5x DS60001507N
    /// l.40882 / l.38962.
    template<bool LoopBack,
             typename ReceivePin,
             typename TransmitPin>
    constexpr bool receivePinFits() {
        bool const same = Io::pinLocationEqual(ReceivePin{}, TransmitPin{});
        return LoopBack ? same : !same;
    }

    namespace Detail {
        template<bool LoopBack, Diagnostic::FixedString What>
        struct ReceivePinRefused {
            consteval Diagnostic::Text operator()() const {
                Diagnostic::Text t;
                t << What.view();
                if constexpr(LoopBack) {
                    t << ": loopBack is set, so the receive pin must be the transmit pin (the "
                         "loop-back is through "
                         "the pad)";
                } else {
                    t << ": the receive and the transmit pin are the same pin (set loopBack for a "
                         "loop-back through "
                         "the pad)";
                }
                return t;
            }
        };
    }   // namespace Detail

    /// receivePinFits as a compile error naming What ("SPI misoPinLocation/mosiPinLocation"); true, for a
    /// static_assert at class scope
    template<bool LoopBack,
             typename ReceivePin,
             typename TransmitPin,
             Diagnostic::FixedString What>
    constexpr bool checkReceivePin() {
        constexpr bool fits = receivePinFits<LoopBack, ReceivePin, TransmitPin>();
        KVASIR_STATIC_ASSERT(fits, (Detail::ReceivePinRefused<LoopBack, What>));
        return true;
    }

}}}   // namespace Kvasir::Sam::Pads
