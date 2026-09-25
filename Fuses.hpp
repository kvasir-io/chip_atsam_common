#pragma once

#include "chip/Fuse_Traits.hpp"
#include "kvasir/Util/BitField.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/NVMCTRL.hpp"
#include "uc_log/uc_log.hpp"

#include <cassert>
#include <cstdint>

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

    template<typename Clock>
    void setFuseBits(FuseBits const& soll,
                     bool            lock = false) {
        using NVME = Kvasir::Peripheral::NVMCTRL::Registers<>;

        static constexpr std::uint32_t fuseBitAddr = 0x00804000;
        std::array<std::byte, 8> const oldFuseBits = []() {
            std::array<std::uint32_t, 2> const bits{
              *reinterpret_cast<std::uint32_t volatile*>(fuseBitAddr),
              *reinterpret_cast<std::uint32_t volatile*>(fuseBitAddr + 4)};
            std::array<std::byte, 8> buff{};
            std::memcpy(buff.data(), bits.data(), buff.size());
            return buff;
        }();

        bool needToWrite = false;
        if(!FuseBits::isValid(oldFuseBits) || soll != FuseBits{oldFuseBits}) { needToWrite = true; }

        if(needToWrite) {
            bool securityBit = apply(read(NVME::STATUS::sb));
            if(!securityBit) {
                std::array<std::byte, 8> newFuseBits = oldFuseBits;
                soll.combineInto(newFuseBits);

                std::array<std::uint32_t, 2> bits{};
                std::memcpy(bits.data(), newFuseBits.data(), newFuseBits.size());

                auto waitForReady = []() {
                    auto const timeout = Clock::now() + 100ms;
                    while(!apply(read(NVME::INTFLAG::ready))) {
                        if(Clock::now() > timeout) { assert(false); }
                    }
                };

                apply(set(NVME::CTRLB::cachedis));
                apply(write(NVME::ADDR::addr,
                            Kvasir::Register::value<(fuseBitAddr / 2)
                                                    & Kvasir::Register::maskFromRange(21, 0)>()));

                apply(write(NVME::CTRLA::CMDValC::ear), write(NVME::CTRLA::CMDEXValC::key));
                waitForReady();
                apply(write(NVME::CTRLA::CMDValC::pbc), write(NVME::CTRLA::CMDEXValC::key));
                waitForReady();
                *reinterpret_cast<std::uint32_t volatile*>(fuseBitAddr)     = bits[0];
                *reinterpret_cast<std::uint32_t volatile*>(fuseBitAddr + 4) = bits[1];
                apply(write(NVME::CTRLA::CMDValC::wap), write(NVME::CTRLA::CMDEXValC::key));
                apply(clear(NVME::CTRLB::cachedis));

                if(lock) {
                    apply(write(NVME::CTRLA::CMDValC::ssb), write(NVME::CTRLA::CMDEXValC::key));
                }

                UC_LOG_D("fuse bits written need power cycle to get active");

            } else {
                UC_LOG_E("want to write fuse bits but securityBit set");
            }
        } else {
            UC_LOG_D("fuse bits already up to date");
        }
    }

}}   // namespace Kvasir::Fuses
