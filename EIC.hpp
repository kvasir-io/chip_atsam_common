#pragma once
// The SAM EIC: external interrupt lines as Startup entries.
//
// Each user of an EXTINT line (button, encoder, zero-cross, PPS) is or contains an EIC::ExtInt,
// which brings its pin config, CONFIGn/EVCTRL settings and a sub-interrupt (SubIsrs). Startup
// merges the lines of one vector into one generated ISR (SharedIsr.hpp). EicBase (bus clock and
// CTRL.ENABLE) must be in the same Startup; two users of one EXTINT number are a compile error.
//
// Data sheet facts (D21 DS40001882L, C21 DS60001479M):
//   - INTFLAG.EXTINT[x] is write-one-to-clear (D21 21.8.8, md 15492; C21 26.8.8, md 18709).
//   - The request needs flag AND enable (D21 21.6.6, md 15158), hence the INTENSET test.
//   - D21/C21 share one NVIC line for all lines (D21 Table 11-3, md 1856; C21 Table 10-3,
//     md 2193); D5x/E5x has one per line (Table 10-1, md 2785). See EicExtIntVector<x>.
//   - CONFIGn and EVCTRL are written while the EIC is disabled (D21 21.6.2.1, md 15036).
//   - Spurious INTFLAG at enable (D21 errata 1.9.1, C21 errata 1.11.3): handled in EicBase.

#include "core/Nvic.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/StartUp/SharedIsr.hpp"
#include "peripherals/EIC.hpp"

#include <cstdint>
#include <utility>

namespace Kvasir { namespace EIC {
    enum class InterruptType {
        None     = 0,
        EdgeRise = 1,
        EdgeFall = 2,
        EdgeBoth = 3,

        LevelHigh = 4,
        LevelLow  = 5
    };

    namespace detail {
        using Regs = Kvasir::Peripheral::EIC::Registers<>;

        // One EXTINT bit of INTFLAG / INTENSET, typed as the register's own EXTINT field.
        template<unsigned Line, typename Org>
        using ExtIntBit = Register::FieldLocation<typename Org::Addr,
                                                  Register::maskFromRange(Line, Line),
                                                  typename decltype(Org::extint)::Access,
                                                  typename decltype(Org::extint)::DataType>;

        template<unsigned bit, typename Org>
        struct EVCTRL {
            static_assert(bit < 16,
                          "not a valid pin");
            using Addr     = typename Org::Addr;
            using AddrT    = typename Addr::RegType;
            using Loc      = decltype(Org::extinteo);
            using Access   = typename Loc::Access;
            using DataType = typename Loc::DataType;

            template<bool Event>
            static constexpr auto setEvent() {
                if constexpr(Event) {
                    return Kvasir::Register::write(
                      Register::FieldLocation<
                        Kvasir::Register::Address<Addr::value, 0xFFFFFFFF, 0, AddrT>,
                        Register::maskFromRange(bit, bit),
                        Access,
                        DataType>{},
                      Register::value<DataType, static_cast<DataType>(1)>());
                } else {
                    return brigand::list<>();
                }
            }
        };

        template<unsigned bit, template<unsigned> typename Org_>
        struct CONFIG {
            static_assert(bit < 16,
                          "not a valid pin");

            using Org                   = Org_<(bit > 7 ? 1 : 0)>;
            using Addr                  = typename Org::Addr;
            using AddrT                 = typename Addr::RegType;
            using TLoc                  = decltype(Org::sense0);
            using TAccess               = typename TLoc::Access;
            static constexpr auto TMask = TLoc::Mask;
            using TDataType             = typename TLoc::DataType;

            using FLoc                  = decltype(Org::filten0);
            using FAccess               = typename FLoc::Access;
            static constexpr auto FMask = FLoc::Mask;
            using FDataType             = typename FLoc::DataType;

            static constexpr int bitPos = (bit % 8) * 4;

            static_assert(TMask
                          == Register::maskFromRange(2,
                                                     0));
            static_assert(FMask
                          == Register::maskFromRange(3,
                                                     3));

            static_assert(InterruptType::None == static_cast<InterruptType>(Org::SENSE0Val::none));
            static_assert(InterruptType::EdgeRise
                          == static_cast<InterruptType>(Org::SENSE0Val::rise));
            static_assert(InterruptType::EdgeFall
                          == static_cast<InterruptType>(Org::SENSE0Val::fall));
            static_assert(InterruptType::EdgeBoth
                          == static_cast<InterruptType>(Org::SENSE0Val::both));
            static_assert(InterruptType::LevelHigh
                          == static_cast<InterruptType>(Org::SENSE0Val::high));
            static_assert(InterruptType::LevelLow
                          == static_cast<InterruptType>(Org::SENSE0Val::low));

            // A plain store of the merged CONFIGn value (every bit write-ignored-if-zero): the
            // lines of one CONFIGn register, whoever declares them, are written together by
            // Startup's single apply of initStepPeripheryConfig.
            template<InterruptType type>
            static constexpr auto setInterruptType() {
                return Kvasir::Register::write(
                  Register::FieldLocation<
                    Kvasir::Register::Address<Addr::value, 0xFFFFFFFF, 0, AddrT>,
                    Register::maskFromRange(bitPos + 2, bitPos),
                    TAccess,
                    TDataType>{},
                  Register::value<TDataType, static_cast<TDataType>(type)>());
            }

            template<bool enable>
            static constexpr auto setFilter() {
                if constexpr(enable) {
                    return Kvasir::Register::write(
                      Register::FieldLocation<
                        Kvasir::Register::Address<Addr::value, 0xFFFFFFFF, 0, AddrT>,
                        Register::maskFromRange(bitPos + 3, bitPos + 3),
                        FAccess,
                        FDataType>{},
                      Register::value<FDataType, static_cast<FDataType>(1)>());
                } else {
                    return brigand::list<>();
                }
            }
        };

        // The EXTINT line a pin drives on function A: pin % 16, except PA24/PA25 -> 12/13,
        // PA27 -> 15, PA28 -> 8, PA30/PA31 -> 10/11, and PA08 is the NMI (SAM D21 Table 7-1,
        // md 1032-1093; SAM C21 Table 6-2, md 1253-1316). The D5x/E5x differs (PA24 on EXTINT[8],
        // DS60001507N md 1401) and would need its own table.
        inline constexpr unsigned noExtInt = 16;

        template<int Port,
                 int Pin>
        consteval unsigned extIntOf(Register::PinLocation<Port,
                                                          Pin>) {
            if constexpr(Port == 0) {
                switch(Pin) {
                case 8:  return noExtInt;
                case 24: return 12;
                case 25: return 13;
                case 27: return 15;
                case 28: return 8;
                case 30: return 10;
                case 31: return 11;
                default: break;
                }
            }
            return static_cast<unsigned>(Pin % 16);
        }
    }   // namespace detail

    // An EXTINT number: the line that configures it provides it, so a second user of the same
    // line (another pin on that EXTINT, D21 21.6.6 note 2, md 15164: "only one will be active")
    // is a compile error.
    struct ExtIntLineTag {};

    // The EIC block: its bus clock and CTRL.ENABLE, after every line's configuration. The GCLK
    // channel that clocks edge detection and the filter (GCLK_EIC) is the firmware's clock
    // settings' business.
    struct EicBase {
        using Regs                   = detail::Regs;
        static constexpr unsigned ba = Regs::baseAddr;

        static constexpr auto powerClockEnable = list(typename PM::enable<ba>::action{});

        // Enabling the EIC with the filter on can raise a spurious INTFLAG.EXTINTx for RISE, BOTH
        // or LOW lines (SAM D21 errata DS80000760 1.9.1, SAM C20/C21 DS80000740 1.11.3). So:
        // enable, wait for sync, clear every flag and the vector's pending bit; Startup's
        // PeripheryEnable step enables the vector afterwards.
        static void preEnableRuntimeInit() {
            apply(Regs::CTRLA::overrideDefaults(set(Regs::CTRLA::enable)));
            waitForSync<Regs>();
            using Flags = typename decltype(Regs::INTFLAG::FULLREGISTER)::DataType;
            apply(write(Regs::INTFLAG::FULLREGISTER, static_cast<Flags>(~Flags{})));
            clearPending(std::make_index_sequence<16>{});
        }

    private:
        template<typename R>
        static void waitForSync() {
            if constexpr(requires { R::STATUS::syncbusy; }) {
                while(apply(read(R::STATUS::syncbusy))) {}   // D21: STATUS.SYNCBUSY
            } else {
                while(apply(read(R::SYNCBUSY::enable))) {}   // C21/E5x: SYNCBUSY.ENABLE
            }
        }

        template<std::size_t... Lines>
        static void clearPending(std::index_sequence<Lines...>) {
            (apply(Nvic::makeClearPending(Kvasir::Interrupt::EicExtIntVector<Lines>{})), ...);
        }
    };

    // One EXTINT line: Pin on the EIC, Type its sense, Callback (void()) run in the interrupt for
    // each detection, at NVIC priority Priority (every line of one vector has to agree). Filter:
    // the majority filter (CONFIGn.FILTENx); Event: EVCTRL.EXTINTEOx, the line's event output.
    template<typename Pin,
             InterruptType         Type,
             auto                  Callback,
             int                   Priority,
             Io::PullConfiguration Pull   = Io::PullConfiguration::PullUp,
             bool                  Filter = true,
             bool                  Event  = false>
    struct ExtInt {
        static constexpr unsigned line = detail::extIntOf(Pin{});
        static_assert(line != detail::noExtInt,
                      "PA08 is the NMI pin: it drives no EXTINT line");
        static_assert(Priority >= 0,
                      "an EXTINT line needs its NVIC priority");

        using Regs   = detail::Regs;
        using Vector = Kvasir::Interrupt::EicExtIntVector<line>;

        static constexpr detail::ExtIntBit<line, typename Regs::INTFLAG>  flag{};
        static constexpr detail::ExtIntBit<line, typename Regs::INTENSET> enable{};

        // the line is this entry's; the EIC has to be enabled by an EicBase in the list
        using Provides = brigand::list<Startup::Resource<ExtIntLineTag, line>>;
        using Claims   = brigand::list<Startup::Listed<EicBase>>;

        using SubIsrs = brigand::list<Nvic::SubIsr<Vector,
                                                   Callback,
                                                   Nvic::RawStatus<flag, enable>,
                                                   Nvic::ClearLast<flag>,
                                                   Nvic::Enable<enable>,
                                                   Priority>>;

        static constexpr auto initStepPinConfig
          = list(action(Kvasir::Io::Action::PinFunction<0,
                                                        Io::OutputType::PushPull,
                                                        Io::OutputSpeed::Low,
                                                        Io::OutputInit::Low,
                                                        Pull>{},
                        Pin{}));

        static constexpr auto initStepPeripheryConfig
          = list(detail::CONFIG<line, Regs::CONFIG>::template setInterruptType<Type>(),
                 detail::CONFIG<line, Regs::CONFIG>::template setFilter<Filter>(),
                 detail::EVCTRL<line, typename Regs::EVCTRL>::template setEvent<Event>());
    };
}}   // namespace Kvasir::EIC
