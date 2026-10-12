#pragma once

#include "chip/Fuse_Traits.hpp"
#include "chip/atsam_common/NVMCTRL.hpp"
#include "kvasir/Util/BitField.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/NVMCTRL.hpp"
#include "uc_log/uc_log.hpp"

#include <array>
#include <cstdint>
#include <cstring>

namespace Kvasir { namespace Fuses {

    struct Bod33 : Traits::Bod33 {
        using Traits::Bod33::Bod33;

        template<std::size_t N>
        static constexpr bool isValid(std::array<std::byte,
                                                 N> const& a) {
            return decltype(action)::isValid(a) && decltype(enable)::isValid(a)
                && decltype(hysteresis)::isValid(a) && decltype(level)::isValid(a);
        }

        constexpr bool operator==(Bod33 const& rhs) const {
            return action == rhs.action && enable == rhs.enable && hysteresis == rhs.hysteresis
                && level == rhs.level;
        }

        template<std::size_t N>
        constexpr void combineInto(std::array<std::byte,
                                              N>& a) const {
            action.combineInto(a);
            enable.combineInto(a);
            hysteresis.combineInto(a);
            level.combineInto(a);
        }
    };

    struct Wdt : Traits::Wdt {
        using Traits::Wdt::Wdt;

        template<std::size_t N>
        static constexpr bool isValid(std::array<std::byte,
                                                 N> const& a) {
            return decltype(enable)::isValid(a) && decltype(alwaysOn)::isValid(a)
                && decltype(windowEnable)::isValid(a) && decltype(period)::isValid(a)
                && decltype(window)::isValid(a) && decltype(offset)::isValid(a);
        }

        constexpr bool operator==(Wdt const& rhs) const {
            return enable == rhs.enable && alwaysOn == rhs.alwaysOn
                && windowEnable == rhs.windowEnable && period == rhs.period && window == rhs.window
                && offset == rhs.offset;
        }

        template<std::size_t N>
        constexpr void combineInto(std::array<std::byte,
                                              N>& a) const {
            enable.combineInto(a);
            alwaysOn.combineInto(a);
            windowEnable.combineInto(a);
            period.combineInto(a);
            window.combineInto(a);
            offset.combineInto(a);
        }
    };

    struct Nvm : Traits::Nvm {
        using Traits::Nvm::Nvm;

        template<std::size_t N>
        static constexpr bool isValid(std::array<std::byte,
                                                 N> const& a) {
            return decltype(bootprot)::isValid(a) && decltype(eeprom)::isValid(a)
                && decltype(lock)::isValid(a);
        }

        constexpr bool operator==(Nvm const& rhs) const {
            return bootprot == rhs.bootprot && eeprom == rhs.eeprom && lock == rhs.lock;
        }

        template<std::size_t N>
        constexpr void combineInto(std::array<std::byte,
                                              N>& a) const {
            bootprot.combineInto(a);
            eeprom.combineInto(a);
            lock.combineInto(a);
        }
    };

    struct FuseBits {
        Bod33 bod33;
        Wdt   wdt;
        Nvm   nvm;

        constexpr FuseBits(Bod33 bod33_,
                           Wdt   wdt_,
                           Nvm   nvm_)
          : bod33(bod33_)
          , wdt(wdt_)
          , nvm{nvm_} {}

        template<std::size_t N>
        constexpr explicit FuseBits(std::array<std::byte,
                                               N> const& a)
          : bod33{a}
          , wdt{a}
          , nvm{a} {}

        template<std::size_t N>
        static constexpr bool isValid(std::array<std::byte,
                                                 N> const& a) {
            return decltype(bod33)::isValid(a) && decltype(wdt)::isValid(a)
                && decltype(nvm)::isValid(a);
        }

        constexpr bool operator==(FuseBits const& rhs) const {
            return bod33 == rhs.bod33 && wdt == rhs.wdt && nvm == rhs.nvm;
        }

        constexpr bool operator!=(FuseBits const& rhs) const { return !(*this == rhs); }

        template<std::size_t N>
        constexpr void combineInto(std::array<std::byte,
                                              N>& a) const {
            bod33.combineInto(a);
            wdt.combineInto(a);
            nvm.combineInto(a);
        }
    };

    enum class FuseWrite : std::uint8_t {
        upToDate,         ///< the user row already held them
        written,          ///< written; they take effect after the next reset (DS40001882L 22.6.3)
        securityBitSet,   ///< not written: the security bit is set (only a chip erase clears it)
    };

    /// Writes the user row's fuse fields where they differ from `soll` (the other bits kept) and, with `lock`, sets
    /// the security bit. Every NVM command is checked (Nvm::Detail::command in NVMCTRL.hpp: EAR, PBC, WAP, SSB, each after READY),
    /// and the two words are read back after WAP (Error::verify). Callers run it at every boot, so a failed
    /// write is tried again at the next one.
    template<typename Clock>
    [[nodiscard]] ::Kvasir::Nvm::Result<FuseWrite> setFuseBits(FuseBits const& soll,
                                                               bool            lock = false) {
        using NVME   = Kvasir::Peripheral::NVMCTRL::Registers<>;
        using Erase  = NVMCTRL::Traits::EraseBound;
        using Write  = NVMCTRL::Traits::WriteBound;
        using Result = ::Kvasir::Nvm::Result<FuseWrite>;

        static constexpr std::uint32_t fuseBitAddr = 0x00804000;
        auto* const                    row = reinterpret_cast<std::uint32_t volatile*>(fuseBitAddr);
        std::array<std::byte, 8> const oldFuseBits = [row]() {
            std::array<std::uint32_t, 2> const bits{row[0], row[1]};
            std::array<std::byte, 8>           buff{};
            std::memcpy(buff.data(), bits.data(), buff.size());
            return buff;
        }();

        if(FuseBits::isValid(oldFuseBits) && soll == FuseBits{oldFuseBits}) {
            UC_LOG_D("fuse bits already up to date");
            return FuseWrite::upToDate;
        }
        if(apply(read(NVME::STATUS::sb))) {
            UC_LOG_E("want to write fuse bits but securityBit set");
            return FuseWrite::securityBitSet;
        }

        std::array<std::byte, 8> newFuseBits = oldFuseBits;
        soll.combineInto(newFuseBits);
        std::array<std::uint32_t, 2> bits{};
        std::memcpy(bits.data(), newFuseBits.data(), newFuseBits.size());

        ::Kvasir::Nvm::Detail::CacheOff const cacheOff{};
        auto const                            cmd
          = [](auto value) { apply(write(value), write(NVME::CTRLA::CMDEXValC::key)); };
        if(auto const r = ::Kvasir::Nvm::Detail::command<Erase>([&] {
               NVMCTRL::Traits::set_addr(fuseBitAddr);
               cmd(NVME::CTRLA::CMDValC::ear);
           });
           !r)
        {
            return Result{std::unexpect, r.error()};
        }
        if(auto const r
           = ::Kvasir::Nvm::Detail::command<Write>([&] { cmd(NVME::CTRLA::CMDValC::pbc); });
           !r)
        {
            return Result{std::unexpect, r.error()};
        }
        row[0] = bits[0];
        row[1] = bits[1];
        if(auto const r = ::Kvasir::Nvm::Detail::command<Write>([&] {
               NVMCTRL::Traits::set_addr(fuseBitAddr);
               cmd(NVME::CTRLA::CMDValC::wap);
           });
           !r)
        {
            return Result{std::unexpect, r.error()};
        }
        if(row[0] != bits[0] || row[1] != bits[1]) {
            return Result{std::unexpect, ::Kvasir::Nvm::Error::verify};
        }

        // "After issuing the SSB command, the PROGE error bit can be checked" (22.6.6)
        if(lock) {
            if(auto const r
               = ::Kvasir::Nvm::Detail::command<Write>([&] { cmd(NVME::CTRLA::CMDValC::ssb); });
               !r)
            {
                return Result{std::unexpect, r.error()};
            }
        }

        UC_LOG_D("fuse bits written, they take effect after the next reset");
        return FuseWrite::written;
    }

}}   // namespace Kvasir::Fuses
