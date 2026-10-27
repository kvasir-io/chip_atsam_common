#pragma once
// A chip-free gate table for ClockGate.hpp's tests (gate_row_test.cpp, must_fail.cpp 18-20): a mask register at
// 0x40000820 with three gate bits, a SERCOM-like block that states its interrupt lines and DMAC trigger sources, a
// block on two gates that states its line on the first row, and a plain block that states neither.
#include "../ClockGate.hpp"

#include <cstdint>

namespace Fake {
inline constexpr std::uint32_t maskBase = 0x40000800;

using MaskAddr = Kvasir::Register::Address<maskBase + 0x20, 0xFFFFFFF0, 0x00000000, std::uint32_t>;

template<unsigned Bit>
inline constexpr Kvasir::Register::FieldLocation<MaskAddr,
                                                 Kvasir::Register::maskFromRange(Bit, Bit),
                                                 Kvasir::Register::ReadWriteAccess,
                                                 std::uint32_t>
  gate{};

struct SercomRegs {
    static constexpr std::uint32_t baseAddr = 0x42000800;
};

// another register view of the same block (a SERCOM's I2CM, SPI and USART views share the base)
struct SercomSpiRegs {
    static constexpr std::uint32_t baseAddr = 0x42000800;
};

struct TwoGateRegs {
    static constexpr std::uint32_t baseAddr = 0x42000C00;
};

struct PlainRegs {
    static constexpr std::uint32_t baseAddr = 0x42001000;
};

struct NoRowRegs {
    static constexpr std::uint32_t baseAddr = 0x42001400;
};

// stand-ins for the generated `decltype(Interrupt::x)` and TRIGSRC enumerators
template<unsigned N>
struct Line {};

enum class Trigger : std::uint8_t { disable = 0, sercomRx = 3, sercomTx = 4 };

using Table = Kvasir::Hal::BlockTable<
  Kvasir::Sam::GateRow<maskBase,
                       SercomRegs::baseAddr,
                       gate<0>,
                       false,
                       brigand::list<Line<9> const, Line<10> const>,
                       Trigger::sercomRx,
                       Trigger::sercomTx>,
  Kvasir::Sam::
    GateRow<maskBase, TwoGateRegs::baseAddr, gate<1>, true, brigand::list<Line<11> const>>,
  Kvasir::Sam::GateRow<maskBase, TwoGateRegs::baseAddr, gate<2>, true>,
  Kvasir::Sam::GateRow<maskBase, PlainRegs::baseAddr, gate<3>>>;
}   // namespace Fake

template<>
struct Kvasir::Hal::BlockGates<Kvasir::Tag::User> {
    using type = Fake::Table;
};
