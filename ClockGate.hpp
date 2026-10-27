#pragma once

// The SAM peripheral clock gates - the bus clock mask bits of PM (SAM D21) or MCLK (SAM C21, E51) - over the chip's
// gate table (chip/Blocks.hpp, kvasir/Hal/Block.hpp's hook). A row names the generated register set of a block and
// the generated mask field that clocks it; a block on several clock domains (an AHB and an APB bit) has a row per
// bit. The front every driver uses:
//
//     static constexpr auto powerClockEnable = list(typename PM::enable<Regs::baseAddr>::action{});
//
// `enable<Base>::action` sets the block's mask bit(s): one action, or a brigand::list of them for two rows;
// `disable<Base>::action` clears them; `isEnabled<Base>()` / `isEnabled(base)` read them back (all set). A base with
// no row is a compile error. From the table, Startup also checks that every Startup-phase write into a block whose
// gate is off out of reset follows a listed `enable` of it (rows marked open at boot claim nothing).
//
// A block a shared driver serves (SERCOM, CAN) states its interrupt lines and DMAC trigger sources in its row too,
// as the RP rows do (chip_rp_common Block.hpp), and the driver reads them from `Sam::Block<Regs>` instead of a
// per-instance ladder of its own:
//
//     using InterruptIndexs = decltype(Sam::Block<Regs>::irqs());
//     static constexpr auto RxDmaTrigger = Sam::Block<Regs>::dmaRx();
//
// The lookup is by base address, so the I2CM, SPI and USART views of one SERCOM find one row; asking a row that does
// not state the column is a compile error naming the base.

#include "kvasir/Hal/Block.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Diagnostic.hpp"

#include <bit>
#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace Sam {
    /// The gate id of a mask field: its bit counted from the mask register block's base (`MaskBase`, PM's or MCLK's).
    template<std::uint32_t MaskBase,
             auto const&   Field>
    constexpr unsigned gateId() {
        using F = std::remove_cvref_t<decltype(Field)>;
        static_assert(std::has_single_bit(F::mask), "a clock mask field is one bit");
        return (Register::RegisterTraits<F>::address - MaskBase) * 8U
             + static_cast<unsigned>(std::countr_zero(F::mask));
    }

    /// The DMAC trigger column of a row whose block has none (GateRow's default).
    struct NoDmaTrigger {};

    /// One gate: the block at `Base` (a generated register set's `baseAddr`, 1 KiB) is clocked by `Field`. A block a
    /// shared driver serves also names its interrupt lines (`Interrupts`: a brigand::list of the generated
    /// `decltype(Interrupt::x)`) and its DMAC trigger sources (`DmaRx`/`DmaTx`: the generated TRIGSRC enumerators); a
    /// two-gate block names them on its first row.
    template<std::uint32_t MaskBase,
             std::uint32_t Base,
             auto const&   Field,
             bool          OpenAtBoot = false,
             typename Interrupts      = brigand::list<>,
             auto DmaRx               = NoDmaTrigger{},
             auto DmaTx               = NoDmaTrigger{}>
    struct GateRow
      : Hal::BlockRow<Base, 0x400, gateId<MaskBase, Field>(), decltype(set(Field)), OpenAtBoot> {
        static constexpr auto const& field = Field;
        using Irqs                         = Interrupts;
        static constexpr auto dmaRx        = DmaRx;
        static constexpr auto dmaTx        = DmaTx;
        static constexpr bool hasIrqs      = !std::is_same_v<Interrupts, brigand::list<>>;
        static constexpr bool hasDma       = !std::is_same_v<decltype(DmaRx), NoDmaTrigger>;
        static_assert(std::is_same_v<decltype(DmaRx),
                                     NoDmaTrigger>
                        == std::is_same_v<decltype(DmaTx),
                                          NoDmaTrigger>,
                      "a gate row names both DMAC trigger sources (RX and TX) or neither");
    };

    namespace Detail {
        template<typename... Dep>
        using GateTable = typename Hal::BlockGatesOf<Dep...>::type;

        template<typename Row>
        using ClearOf = decltype(clear(Row::field));

        template<std::uint32_t Base, typename Table>
        struct RowsAt;

        template<std::uint32_t Base, typename... Rows>
        struct RowsAt<Base, Hal::BlockTable<Rows...>> {
            using type = brigand::flatten<brigand::list<
              std::conditional_t<Rows::base == Base, brigand::list<Rows>, brigand::list<>>...>>;
        };

        template<typename L>
        struct OneOrList {
            using type = L;
        };

        template<typename A>
        struct OneOrList<brigand::list<A>> {
            using type = A;
        };

        template<typename Rows>
        struct Disable;

        template<typename... Rows>
        struct Disable<brigand::list<Rows...>> {
            using type = typename OneOrList<brigand::list<ClearOf<Rows>...>>::type;
        };

        template<typename Rows>
        struct AllSet;

        template<typename... Rows>
        struct AllSet<brigand::list<Rows...>> {
            static bool allSet() { return ((apply(read(Rows::field)) != 0U) && ...); }
        };

        // the error of enable<>/disable<> for a base with no row: "... for 0x42001800"
        template<std::uint32_t Base>
        struct NoGate {
            consteval Diagnostic::Text operator()() const {
                Diagnostic::Text t;
                t << "invalid PeripheryAddress: no clock gate row in chip/Blocks.hpp for ";
                t.hex(Base);
                return t;
            }
        };

        // what Block<> reads where a base has no row at all (NoGate is the error then)
        struct NoRow {
            using Irqs                    = brigand::list<>;
            static constexpr auto dmaRx   = NoDmaTrigger{};
            static constexpr auto dmaTx   = NoDmaTrigger{};
            static constexpr bool hasIrqs = false;
            static constexpr bool hasDma  = false;
        };

        template<typename Rows>
        struct FirstRow {
            using type = NoRow;
        };

        template<typename Row, typename... Rows>
        struct FirstRow<brigand::list<Row, Rows...>> {
            using type = Row;
        };

        // the error of Block<>::irqs()/dmaRx()/dmaTx() for a row without the column: "the chip/Blocks.hpp row of
        // 0x42000800 names no interrupt lines"
        template<std::uint32_t Base, bool Dma>
        struct NoColumn {
            consteval Diagnostic::Text operator()() const {
                Diagnostic::Text t;
                t << "the chip/Blocks.hpp row of ";
                t.hex(Base);
                t << (Dma ? " names no DMAC trigger sources" : " names no interrupt lines");
                return t;
            }
        };
    }   // namespace Detail

    /// The driver columns of the block a generated register set `Regs` names (its `baseAddr`), from the chip's gate
    /// table: `irqs()` (a value of the brigand::list of its interrupt lines, for decltype), `dmaRx()`, `dmaTx()`.
    /// Each is a compile error naming the base when the block's (first) row does not state it.
    template<typename Regs>
    struct Block {
        static constexpr std::uint32_t base = Regs::baseAddr;
        using Rows                   = typename Detail::RowsAt<base, Detail::GateTable<Regs>>::type;
        static constexpr bool hasRow = brigand::size<Rows>::value > 0;
        KVASIR_STATIC_ASSERT(hasRow,
                             (Detail::NoGate<base>));
        using Row = typename Detail::FirstRow<Rows>::type;
        // no row: NoGate above is the one error
        static constexpr bool hasIrqs = !hasRow || Row::hasIrqs;
        static constexpr bool hasDma  = !hasRow || Row::hasDma;

        static consteval auto irqs() {
            KVASIR_STATIC_ASSERT(hasIrqs, (Detail::NoColumn<base, false>));
            return typename Row::Irqs{};
        }

        static consteval auto dmaRx() {
            KVASIR_STATIC_ASSERT(hasDma, (Detail::NoColumn<base, true>));
            return Row::dmaRx;
        }

        static consteval auto dmaTx() {
            KVASIR_STATIC_ASSERT(hasDma, (Detail::NoColumn<base, true>));
            return Row::dmaTx;
        }
    };
}}   // namespace Kvasir::Sam

namespace Kvasir { namespace PM {
    template<unsigned PeripheryAddress>
    struct Enable {
        using Table                  = Sam::Detail::GateTable<Enable>;
        static constexpr bool hasRow = Table::rowsOf(PeripheryAddress) > 0;
        KVASIR_STATIC_ASSERT(hasRow,
                             (Sam::Detail::NoGate<PeripheryAddress>));
        using action = typename Table::template OpenActions<PeripheryAddress>;
    };

    template<unsigned PeripheryAddress>
    struct Disable {
        using Table                  = Sam::Detail::GateTable<Disable>;
        static constexpr bool hasRow = Table::rowsOf(PeripheryAddress) > 0;
        KVASIR_STATIC_ASSERT(hasRow,
                             (Sam::Detail::NoGate<PeripheryAddress>));
        using action = typename Sam::Detail::Disable<
          typename Sam::Detail::RowsAt<PeripheryAddress, Table>::type>::type;
    };

    /// Whether every mask bit that gates the block at `PeripheryAddress` is set: what enable<> writes, read back.
    template<unsigned PeripheryAddress>
    bool isEnabled() {
        using Table = Sam::Detail::GateTable<Enable<PeripheryAddress>>;
        KVASIR_STATIC_ASSERT(Table::rowsOf(PeripheryAddress) > 0,
                             (Sam::Detail::NoGate<PeripheryAddress>));
        return Sam::Detail::AllSet<
          typename Sam::Detail::RowsAt<PeripheryAddress, Table>::type>::allSet();
    }

    namespace Detail {
        template<typename Table>
        struct IsEnabledAt;

        template<typename... Rows>
        struct IsEnabledAt<Hal::BlockTable<Rows...>> {
            static bool allSetAt(unsigned base) {
                bool any = false;
                bool all = true;
                (((Rows::base == base) ? (any = true, all = all && apply(read(Rows::field)) != 0U)
                                       : false),
                 ...);
                return any && all;
            }
        };
    }   // namespace Detail

    /// isEnabled<>() for a base known at run time; false for a base with no row.
    template<typename... Dep>
    bool isEnabled(unsigned peripheryAddress) {
        return Detail::IsEnabledAt<Sam::Detail::GateTable<Dep...>>::allSetAt(peripheryAddress);
    }
}}   // namespace Kvasir::PM
