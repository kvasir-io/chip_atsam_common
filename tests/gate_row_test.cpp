// ClockGate.hpp's driver columns over a chip-free table (FakeGates.hpp): Sam::Block<Regs> finds a block's row by its
// base, so every register view of one block reads the same interrupt lines and DMAC trigger sources; a two-gate block
// states them on its first row; the gate front ends (PM::enable) take rows with columns as before. Building is the
// test; the refusals are must_fail.cpp 18-20.
#include "FakeGates.hpp"

#include <type_traits>

namespace {
using Kvasir::Sam::Block;

static_assert(std::is_same_v<decltype(Block<Fake::SercomRegs>::irqs()),
                             brigand::list<Fake::Line<9> const,
                                           Fake::Line<10> const>>);
static_assert(Block<Fake::SercomRegs>::dmaRx() == Fake::Trigger::sercomRx);
static_assert(Block<Fake::SercomRegs>::dmaTx() == Fake::Trigger::sercomTx);

// another view of the same base: the same row
static_assert(std::is_same_v<Block<Fake::SercomSpiRegs>::Row,
                             Block<Fake::SercomRegs>::Row>);
static_assert(Block<Fake::SercomSpiRegs>::dmaTx() == Fake::Trigger::sercomTx);

// two gates: the columns of the first row, both gates still opened together
static_assert(std::is_same_v<decltype(Block<Fake::TwoGateRegs>::irqs()),
                             brigand::list<Fake::Line<11> const>>);
static_assert(!Block<Fake::TwoGateRegs>::Row::hasDma);
static_assert(brigand::size<Kvasir::PM::Enable<Fake::TwoGateRegs::baseAddr>::action>::value == 2);

// a plain row: a gate, no columns
static_assert(!Block<Fake::PlainRegs>::Row::hasIrqs && !Block<Fake::PlainRegs>::Row::hasDma);
static_assert(std::is_same_v<Kvasir::PM::Enable<Fake::PlainRegs::baseAddr>::action,
                             decltype(set(Fake::gate<3>))>);
}   // namespace

int main() { return 0; }
