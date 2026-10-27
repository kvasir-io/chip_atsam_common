#pragma once
// The SAM D2x/C2x NVM controller's command set (NVMCTRL.CTRLA commands with the key, INTFLAG.READY, STATUS errors, a
// half-word ADDR, PARAM's geometry) for the shared driver (NVMCTRL.hpp), over the chip's errata policy:
//
//     struct Errata {   // the chip's NVMCTRL errata, each with its citation (empty hooks where there is none)
//         template<typename T> static void prepareEraseRww(T addr);   // before an RWWEE row erase
//         static void disableCache(); static void enableCache();     // around every command
//     };
//     struct Traits : Sam::NvmCommandsDC<Peripheral::NVMCTRL::Registers<>, Errata> { bounds, PageSize, PagesPerRow };
//
// The D5x/E5x controller commands through CTRLB, counts in blocks and has no RWWEE array: its chip writes its own
// Traits with `hasRwweeArray = false`.

#include "chip/atsam_common/NvmLayout.hpp"
#include "chip/atsam_common/NvmResult.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"

#include <cstdint>

namespace Kvasir { namespace Sam {
    template<typename NVM, typename Errata>
    struct NvmCommandsDC {
        static constexpr bool hasRwweeArray = true;

        static bool ready() { return 0 != apply(read(NVM::INTFLAG::ready)); }

        static void commandWritePage() {
            apply(write(NVM::CTRLA::CMDValC::wp), write(NVM::CTRLA::CMDEXValC::key));
        }

        static void commandEraseRow() {
            apply(write(NVM::CTRLA::CMDValC::er), write(NVM::CTRLA::CMDEXValC::key));
        }

        static void commandPageBufferClear() {
            apply(write(NVM::CTRLA::CMDValC::pbc), write(NVM::CTRLA::CMDEXValC::key));
        }

        static void commandWriteRww() {
            apply(write(NVM::CTRLA::CMDValC::rwweewp), write(NVM::CTRLA::CMDEXValC::key));
        }

        static void commandEraseRww() {
            apply(write(NVM::CTRLA::CMDValC::rwweeer), write(NVM::CTRLA::CMDEXValC::key));
        }

        template<typename T>
        static void prepareEraseRww(T addr) {
            Errata::prepareEraseRww(addr);
        }

        static void enableCache() { Errata::enableCache(); }

        static void disableCache() { Errata::disableCache(); }

        // STATUS.PROGE (bit 2), LOCKE (3), NVME (4) and INTFLAG.ERROR (1) are cleared by writing a one (D21
        // DS40001882L 22.8.7, 22.8.6; C21 DS60001479M 27.8.7, 27.8.6); LOAD (STATUS bit 1) is left alone. Only what
        // was read is written back, so a flag that comes up in between stays for the next look. Neither register is
        // PAC-protected (D21 22.5.5).
        static std::uint8_t takeErrors() {
            static constexpr std::uint16_t errors
              = NVM::STATUS::proge.mask | NVM::STATUS::locke.mask | NVM::STATUS::nvme.mask;
            auto const status
              = static_cast<std::uint16_t>(get<0>(apply(read(NVM::STATUS::FULLREGISTER))) & errors);
            if(status != 0) {
                apply(write(NVM::STATUS::FULLREGISTER, status));
                apply(set(NVM::INTFLAG::error));
            }
            return Kvasir::Nvm::ErrorBits::bits(
              Kvasir::Register::extract(NVM::STATUS::locke, status) != 0,
              Kvasir::Register::extract(NVM::STATUS::proge, status) != 0,
              Kvasir::Register::extract(NVM::STATUS::nvme, status) != 0);
        }

        // ADDR is a half-word address, 22 bits (D21 22.8.8, C21 27.8.8)
        template<typename T>
        static void setAddr(T addr) {
            apply(write(NVM::ADDR::addr,
                        (reinterpret_cast<std::uint32_t>(addr) / 2)
                          & Kvasir::Register::maskFromRange(21, 0)));
        }

        // Is [addr, addr + bytes) in the main array (mainArray) or the RWWEE array? PARAM's NVMP, PSZ and RWWEEP,
        // judged by Nvm::rangeInArray (NvmLayout.hpp). The driver asks this before every command.
        static bool inArray(std::uint32_t addr,
                            std::uint32_t bytes,
                            bool          mainArray) {
            auto const param = apply(read(NVM::PARAM::nvmp, NVM::PARAM::psz, NVM::PARAM::rwweep));
            return Kvasir::Nvm::rangeInArray(static_cast<std::uint32_t>(get<0>(param)),
                                             static_cast<std::uint32_t>(get<1>(param)),
                                             static_cast<std::uint32_t>(get<2>(param)),
                                             addr,
                                             bytes,
                                             mainArray);
        }
    };
}}   // namespace Kvasir::Sam
