cmake_minimum_required(VERSION 3.28)

# Host tests of the SAM clock solver (ClockSolver.hpp) against the test limits in TestLimits.hpp, and of the NVM
# controller's error decoding (NvmResult.hpp). Standalone, needs only Kvasir_SDK's headers and its tests/ harness (the
# debug-check test) (KVASIR_ROOT: the checkout next to this one, else fetched):
#
# cmake -S tests -B tests/build -G Ninja && cmake --build tests/build && ctest --test-dir tests/build
#
# Most checks are static_asserts: building is the test. The must_fail_* tests compile must_fail.cpp with -DMUST_FAIL=n
# and look for the message, numbers included, in the compiler's output. The chips' real limits (chip/ClockLimits.hpp)
# are tested in each chip package's tests/, against the chip_atsam_common commit it records.
project(chip_atsam_common_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# Kvasir_SDK: KVASIR_ROOT, else the checkout next to this repository, else fetched; its submodules and the other
# packages: kvasir_locate (Kvasir_SDK cmake/kvasir_locate.cmake)
if(NOT KVASIR_ROOT AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK" ABSOLUTE)
elseif(NOT KVASIR_ROOT)
    include(FetchContent)
    FetchContent_Declare(
        kvasir_sdk
        GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_SUBMODULES "" SOURCE_SUBDIR fetch-only)
    FetchContent_MakeAvailable(kvasir_sdk)
    set(KVASIR_ROOT "${kvasir_sdk_SOURCE_DIR}")
endif()
include("${KVASIR_ROOT}/cmake/kvasir_locate.cmake")
kvasir_locate(SDK_SUBMODULES svd_converter)
# kvasir::base: the SDK's src/ (Util/Prescaler.hpp, Register/, ...); kvasir::test_support: its tests/ harness
include(${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake)

add_executable(clock_solver clock_solver_test.cpp)
target_link_libraries(clock_solver PRIVATE kvasir::base)
target_compile_options(clock_solver PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME clock_solver COMMAND clock_solver)

# NvmResult.hpp: the NVM controller's error flags as Nvm::Error, needs nothing but the standard library
add_executable(nvm_result nvm_result_test.cpp)
target_link_libraries(nvm_result PRIVATE kvasir::base) # Util/Result.hpp
target_compile_options(nvm_result PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME nvm_result COMMAND nvm_result)

set(_must_fail_messages
    "flash wait states: 48000000 Hz at 1.62-2.7 V needs 3 \\(test table\\), RWS is 1"
    "flash wait states: no count for 49000000 Hz at 2.7 V and above in test table"
    "FDPLL reference: wanted 32000..2000000 Hz, got 31007.8 Hz \\(test DPLL\\)"
    "test DPLL: no setting makes 200000000 Hz from 8000000 Hz \\(integer mode, 0 ppm\\)"
    "FDPLL reference: wanted 32000..2000000 Hz, got 4000000.0 Hz \\(test DPLL\\)"
    "FDPLL: DIV, LDR, LDRFRAC or PRESC wider than its field"
    "DFLL48M closed loop: wanted 48000000 Hz, got 48005120.0 Hz \\(\\+107 ppm\\), allowed 100 ppm")
# The refusals made by a call to a non-constexpr function (CanBitTiming.hpp, SercomBaud.hpp): clang's note names the
# numbers ("in call to 'refuseNtseg2BelowTwo(8000000, ...)'"), gcc's names only the refusing function.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    list(
        APPEND
        _must_fail_messages
        "in call to 'refuseNtseg2BelowTwo\\(8000000, 1000000, 875\\)'"
        "in call to 'refuseNtseg2BelowTwo\\(8000000, 500000, 1000\\)'"
        "in call to 'refuseBaudZero\\(8000000, 1000000\\)'"
        "in call to 'refuseBaudZero\\(10000000, 1000000\\)'"
        "in call to 'refuseBaudZero\\(8000000, 1000000\\)'"
        "in call to 'refuseNtseg2BelowTwo\\(8000000, 500000, 875\\)'"
        "in call to 'refuseNtseg2BelowTwo\\(8000000, 1000000, 750\\)'"
        "in call to 'refuseOverrunTooLong\\(1024, 20000000, 16000000\\)'")
else()
    list(
        APPEND
        _must_fail_messages
        "canPhaseSeg2UnderThreeQuantaRefusedMoveTheSamplePointEarlier"
        "canPhaseSeg2UnderThreeQuantaRefusedMoveTheSamplePointEarlier"
        "i2cBaudAndBaudlowBothZeroRefusedTheClockIsTooSlowForThisRate"
        "i2cBaudAndBaudlowBothZeroRefusedTheClockIsTooSlowForThisRate"
        "i2cBaudAndBaudlowBothZeroRefusedTheClockIsTooSlowForThisRate"
        "canPhaseSeg2UnderThreeQuantaRefusedMoveTheSamplePointEarlier"
        "canPhaseSeg2UnderThreeQuantaRefusedMoveTheSamplePointEarlier"
        "wdtOverrunTimeLongerThanTheLongestPeriodLowerTheClockOrTheTime")
endif()
# ChipTraits.hpp: fake chip packages without Nvm::EraseBound and without Start (static_assert on the concept)
list(APPEND _must_fail_messages "the SAM chip package lacks a name the shared drivers use"
     "the SAM chip package lacks a name the shared drivers use")
# ClockGate.hpp: Sam::Block<Regs> asked for a column its row does not state, or for a base with no row (FakeGates.hpp)
list(APPEND _must_fail_messages "the chip/Blocks.hpp row of 0x42001000 names no interrupt lines"
     "the chip/Blocks.hpp row of 0x42000C00 names no DMAC trigger sources"
     "invalid PeripheryAddress: no clock gate row in chip/Blocks.hpp for 0x42001400")
# ChipTraits.hpp: a fake chip package without the SERCOM SPI register set (Sam::Package::SercomSpi)
list(APPEND _must_fail_messages "the SAM chip package lacks a name the shared drivers use")
set(_n 0)
foreach(_message IN LISTS _must_fail_messages)
    math(EXPR _n "${_n} + 1")
    add_test(
        NAME must_fail_${_n}
        COMMAND
            ${CMAKE_CXX_COMPILER} -std=c++26 -fsyntax-only
            -I$<TARGET_PROPERTY:kvasir_base,INTERFACE_INCLUDE_DIRECTORIES> -DMUST_FAIL=${_n}
            ${CMAKE_CURRENT_LIST_DIR}/must_fail.cpp)
    set_tests_properties(must_fail_${_n} PROPERTIES PASS_REGULAR_EXPRESSION "${_message}")
endforeach()

# ClockGate.hpp: the gate rows' driver columns (interrupt lines, DMAC triggers) through Sam::Block<Regs>, over a fake
# chip table (FakeGates.hpp); needs Hal/Block.hpp and the register layer
add_executable(gate_row gate_row_test.cpp)
target_link_libraries(gate_row PRIVATE kvasir::base)
target_compile_options(gate_row PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME gate_row COMMAND gate_row)

# PinMux.hpp: a pin's row in a pin-multiplexing table and its PORT function action (an unused pin: none), over fake
# tables; needs the SDK's Io pin factories and brigand
add_executable(pin_mux pin_mux_test.cpp)
target_link_libraries(pin_mux PRIVATE kvasir::base)
target_compile_options(pin_mux PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME pin_mux COMMAND pin_mux)

# SercomPads.hpp: the per-chip DOPO/TXPO tables and the loop-back pin rule; the refusals compile sercom_pads_test.cpp
# with -DMUST_FAIL=n (the message names the pads and the chip). Needs the SDK's Io and Diagnostic.
add_executable(sercom_pads sercom_pads_test.cpp)
target_link_libraries(sercom_pads PRIVATE kvasir::base)
target_compile_options(sercom_pads PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME sercom_pads COMMAND sercom_pads)
set(_pads_must_fail_messages
    "SPI pads refused on SAM D5x/E5x: no CTRLA.DOPO puts MOSI on PAD\\[2\\] with SCK on PAD\\[3\\]"
    "SPI pads refused on SAM D5x/E5x: no CTRLA.DOPO puts MOSI on PAD\\[0\\] with SCK on PAD\\[3\\]"
    "USART pads refused on SAM D5x/E5x: no CTRLA.TXPO puts TxD on PAD\\[2\\]"
    "SPI misoPinLocation/mosiPinLocation: loopBack is set, so the receive pin must be the transmit pin"
    "SPI pads refused: the hardware SS \\(csPinLocation\\) is on PAD\\[1\\], the chosen DOPO puts it on PAD\\[2\\]")
set(_n 0)
foreach(_message IN LISTS _pads_must_fail_messages)
    math(EXPR _n "${_n} + 1")
    add_test(
        NAME sercom_pads_must_fail_${_n}
        COMMAND
            ${CMAKE_CXX_COMPILER} -std=c++26 -fsyntax-only
            -I$<TARGET_PROPERTY:kvasir_base,INTERFACE_INCLUDE_DIRECTORIES> -DMUST_FAIL=${_n}
            ${CMAKE_CURRENT_LIST_DIR}/sercom_pads_test.cpp)
    set_tests_properties(sercom_pads_must_fail_${_n} PROPERTIES PASS_REGULAR_EXPRESSION "${_message}")
endforeach()

# ClockTree.hpp: Clocks::Sam::TreeOf over fake builders (its Provides, the wait states) and its two refusals as
# must_fail cases. ClockTree.hpp includes its sibling as chip/atsam_common/Clocks.hpp, the path a chip package has it
# under: a link in the build tree gives this checkout that name.
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/chip)
file(CREATE_LINK ${CMAKE_CURRENT_LIST_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/include/chip/atsam_common SYMBOLIC)
add_executable(clock_tree clock_tree_test.cpp)
target_include_directories(clock_tree PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/include)
target_link_libraries(clock_tree PRIVATE kvasir::base)
target_compile_options(clock_tree PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME clock_tree COMMAND clock_tree)
set(_tree_must_fail_messages "this SAM part has no such clock source" "f_CPU above the part's maximum")
set(_n 0)
foreach(_message IN LISTS _tree_must_fail_messages)
    math(EXPR _n "${_n} + 1")
    add_test(
        NAME clock_tree_must_fail_${_n}
        COMMAND
            ${CMAKE_CXX_COMPILER} -std=c++26 -fsyntax-only -I${CMAKE_CURRENT_BINARY_DIR}/include
            -I$<TARGET_PROPERTY:kvasir_base,INTERFACE_INCLUDE_DIRECTORIES> -DMUST_FAIL=${_n}
            ${CMAKE_CURRENT_LIST_DIR}/clock_tree_test.cpp)
    set_tests_properties(clock_tree_must_fail_${_n} PROPERTIES PASS_REGULAR_EXPRESSION "${_message}")
endforeach()

# EicLines.hpp: the EXTINT line of every port A/B pad, D21/C21 and D5x/E5x rules; needs nothing but the standard library
add_executable(eic_lines eic_lines_test.cpp)
target_compile_options(eic_lines PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME eic_lines COMMAND eic_lines)

# CanBitTiming.hpp: the M_CAN's NBTP fields for a clock, a bit rate and a sample point; needs Util/Prescaler.hpp
add_executable(can_bit_timing can_bit_timing_test.cpp)
target_link_libraries(can_bit_timing PRIVATE kvasir::base)
target_compile_options(can_bit_timing PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME can_bit_timing COMMAND can_bit_timing)

# SercomBaud.hpp: the I2C host's BAUD fields and SCL rate, the USART's and the SPI's BAUD; needs Util/Prescaler.hpp
add_executable(sercom_baud sercom_baud_test.cpp)
target_link_libraries(sercom_baud PRIVATE kvasir::base)
target_compile_options(sercom_baud PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME sercom_baud COMMAND sercom_baud)

# NvmLayout.hpp: the EepromEmulator's block index; needs nothing but the standard library
add_executable(nvm_layout nvm_layout_test.cpp)
target_compile_options(nvm_layout PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME nvm_layout COMMAND nvm_layout)

# ResetCauseDecode.hpp: one RCAUSE bit -> its cause, none or several -> unknown; needs nothing but the standard library
add_executable(reset_cause_decode reset_cause_decode_test.cpp)
target_compile_options(reset_cause_decode PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME reset_cause_decode COMMAND reset_cause_decode)

# DmacCrcModes.hpp: the CRCs the DMAC's CRC engine computes, its seeds and its checksum read; needs Util/Crc.hpp
add_executable(dmac_crc_modes dmac_crc_modes_test.cpp)
target_link_libraries(dmac_crc_modes PRIVATE kvasir::base)
target_compile_options(dmac_crc_modes PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME dmac_crc_modes COMMAND dmac_crc_modes)

# DmacDescriptor.hpp: the transfer descriptor's builder against BTCTRL's bit table and the constructor it replaced
add_executable(dmac_descriptor dmac_descriptor_test.cpp)
target_link_libraries(dmac_descriptor PRIVATE kvasir::base)
target_compile_options(dmac_descriptor PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME dmac_descriptor COMMAND dmac_descriptor)

# GclkDivision.hpp: a GCLK generator's DIVSEL + DIV for a divisor, the D21/C2x/E5x widths (static_asserts)
add_executable(gclk_division gclk_division_test.cpp)
target_compile_options(gclk_division PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME gclk_division COMMAND gclk_division)

# WdtPeriod.hpp: the watchdog's CONFIG.PER for a clock and an overrun time, and its sync bound; standard library only
add_executable(wdt_period wdt_period_test.cpp)
target_compile_options(wdt_period PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME wdt_period COMMAND wdt_period)

# DmacCrcHold.hpp: the DMAC CRC engine's debug check (two users at once), at the debug variants' level and with debug
# checks off; the SDK's test harness and PanicCatcher (tests/ of the same Kvasir_SDK)
foreach(_level 2 0)
    add_executable(dmac_crc_hold_${_level} dmac_crc_hold_test.cpp)
    target_link_libraries(dmac_crc_hold_${_level} PRIVATE kvasir::test_support)
    target_compile_definitions(dmac_crc_hold_${_level} PRIVATE KVASIR_DCHECK_LEVEL=${_level})
    target_include_directories(dmac_crc_hold_${_level} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
    target_compile_options(dmac_crc_hold_${_level} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME dmac_crc_hold_${_level} COMMAND dmac_crc_hold_${_level})
endforeach()
