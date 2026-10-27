#pragma once
// The NVM software calibration area: words the factory measured for this die (ADC, oscillators, USB pads, ...),
// written in production and read only. Its address and its layout differ per family (SAM D21 DS40001882L 10.3.2
// Table 10-8 at 0x806020; SAM C20/C21 DS60001479M 9.4 Table 9-5 at the same address with other fields; SAM D5x/E5x
// DS60001507N 9.5 Table 9-3 at 0x00800080), so each chip package defines the layout as a trait,
// `Kvasir::Nvm::CalibrationLayout` (the D21's: chip_atsamd21 chip/NvmCalibration_Traits.hpp), and this file reads it:
//
//     auto const cal = Kvasir::Nvm::calibration();     // the chip's decoded fields, e.g. cal.adcLinearity
//
// A layout trait has
//     static constexpr std::uintptr_t address;         // the area's first word
//     static constexpr std::size_t    words;           // how many 32-bit words its fields need
//     using Fields = <a struct of the decoded fields>;
//     static constexpr Fields decode(std::array<std::uint32_t, words> const&);
//
// A field that reads all ones was never programmed; what to load then is the user's choice (the DFLL, the USB pads).
// The words are loaded through Kvasir::Memory (mocked in host tests) into a local: nothing goes into .data or .bss,
// so the read may run inside coreClockInit(), before memory init.
#include "kvasir/Register/Memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace Nvm {
    /// Defined by the chip package that has a layout (a chip without one has no calibration()).
    struct CalibrationLayout;

    template<typename L>
    concept CalibrationLayoutTrait = requires(std::array<std::uint32_t, L::words> const& w) {
        { L::address } -> std::convertible_to<std::uintptr_t>;
        { L::decode(w) } -> std::same_as<typename L::Fields>;
    };

    /// What the factory wrote into this part: `L::words` word loads, decoded.
    template<CalibrationLayoutTrait L = CalibrationLayout>
    [[nodiscard]] inline typename L::Fields calibration() {
        std::array<std::uint32_t, L::words> w{};
        Kvasir::Memory::loadWords(L::address, w.data(), L::words);
        return L::decode(w);
    }
}}   // namespace Kvasir::Nvm
