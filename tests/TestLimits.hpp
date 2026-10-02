#pragma once
// Limits for the solver's own tests. They are test data: the numbers mirror the three FDPLL
// shapes the SAM parts have (D21: no output prescaler; C21: PRESC /1 /2 /4; E5x: LDRFRAC in
// 1/32), but nothing here is checked against a datasheet. Each chip package tests its real
// limits (chip/ClockLimits.hpp) in its own tests/.
#include "../ClockSolver.hpp"

namespace Kvasir::ClockLimits::Test {
// D21-shaped: f_IN 32..2000 kHz, DCO 48..96 MHz, LDR 12 bits, LDRFRAC 4 bits, DIV 11 bits
inline constexpr Dpll                   Plain{.refMin   = 32'000,
                                              .refMax   = 2'000'000,
                                              .outMin   = 48'000'000,
                                              .outMax   = 96'000'000,
                                              .ldrBits  = 12,
                                              .fracBits = 4,
                                              .divBits  = 11,
                                              .prescMax = 0};
inline constexpr Prescaler::FixedString PlainWhere = "test DPLL";

// C21-shaped: the same with an output prescaler up to /4
inline constexpr Dpll WithPresc{.refMin   = 32'000,
                                .refMax   = 2'000'000,
                                .outMin   = 48'000'000,
                                .outMax   = 96'000'000,
                                .ldrBits  = 12,
                                .fracBits = 4,
                                .divBits  = 11,
                                .prescMax = 2};

// E5x-shaped: f_IN 32..3200 kHz, DCO 96..200 MHz, LDR 13 bits, LDRFRAC 5 bits
inline constexpr Dpll Fine{.refMin   = 32'000,
                           .refMax   = 3'200'000,
                           .outMin   = 96'000'000,
                           .outMax   = 200'000'000,
                           .ldrBits  = 13,
                           .fracBits = 5,
                           .divBits  = 11,
                           .prescMax = 0};

// two supply rows, one with a short list
inline constexpr WaitStateTable WaitStates{
  "test table",
  {{
    {Supply::from1V62, {14'000'000, 28'000'000, 42'000'000, 48'000'000}},
    {Supply::from2V7, {24'000'000, 48'000'000}},
  }}};
// one row only
inline constexpr WaitStateTable WaitStatesOneRow{"test table, one row",
                                                 {{
                                                   {Supply::from2V7, {24'000'000, 40'000'000}},
                                                   {},
                                                 }}};

inline constexpr Dfll Dfll48{.refMin     = 732,
                             .refMax     = 33'000,
                             .mulBits    = 16,
                             .coarseBits = 6,
                             .fineBits   = 10};
}   // namespace Kvasir::ClockLimits::Test
