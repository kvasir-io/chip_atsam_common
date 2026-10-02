cmake_minimum_required(VERSION 3.28)

# Host tests of the SAM clock solver (ClockSolver.hpp) against the test limits in TestLimits.hpp. Standalone, needs only
# Kvasir_SDK's Util/Prescaler.hpp (KVASIR_ROOT, default: the checkout next to this one):
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

if(NOT KVASIR_ROOT)
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK" ABSOLUTE)
endif()

add_executable(clock_solver clock_solver_test.cpp)
target_include_directories(clock_solver PRIVATE ${KVASIR_ROOT}/src)
target_compile_options(clock_solver PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME clock_solver COMMAND clock_solver)

set(_must_fail_messages
    "flash wait states: 48000000 Hz at 1.62-2.7 V needs 3 \\(test table\\), RWS is 1"
    "flash wait states: no count for 49000000 Hz at 2.7 V and above in test table"
    "FDPLL reference: wanted 32000..2000000 Hz, got 31007.8 Hz \\(test DPLL\\)"
    "test DPLL: no setting makes 200000000 Hz from 8000000 Hz \\(integer mode, 0 ppm\\)"
    "FDPLL reference: wanted 32000..2000000 Hz, got 4000000.0 Hz \\(test DPLL\\)"
    "FDPLL: DIV, LDR, LDRFRAC or PRESC wider than its field"
    "DFLL48M closed loop: wanted 48000000 Hz, got 48005120.0 Hz \\(\\+107 ppm\\), allowed 100 ppm")
set(_n 0)
foreach(_message IN LISTS _must_fail_messages)
    math(EXPR _n "${_n} + 1")
    add_test(NAME must_fail_${_n} COMMAND ${CMAKE_CXX_COMPILER} -std=c++26 -fsyntax-only -I${KVASIR_ROOT}/src
                                          -DMUST_FAIL=${_n} ${CMAKE_CURRENT_LIST_DIR}/must_fail.cpp)
    set_tests_properties(must_fail_${_n} PROPERTIES PASS_REGULAR_EXPRESSION "${_message}")
endforeach()
