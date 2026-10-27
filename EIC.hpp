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
//     md 2193); D5x/E5x has one per line (Table 10-1, md 2785). See Sam::Package::EicVector<x> (the chip's
//     Interrupt::EicExtIntVector).
//   - CONFIGn and EVCTRL are written while the EIC is disabled (D21 21.6.2.1, md 15036).
//   - Spurious INTFLAG at enable (D21 errata 1.9.1, C21 errata 1.11.3): handled in EicBase.
//   - D5x/E5x (DS60001507N, errata DS80000748T): "SYNCBUSY.ENABLE is released before EIC is fully enabled. Edge
//     detection can be done only after three cycles of the selected GCLK" (errata 2.12.1, all revisions A-G, no
//     workaround): an edge in the first three GCLK_EIC cycles after EicBase's enable is not seen. Whether the
//     spurious flag above can rise on the E5x too is not known; EicBase's clear covers it either way.
//   - Which EXTINT a pad drives is the chip's rule: chip/EIC_Traits.hpp (EicLines.hpp has both families' tables).

#include "chip/EIC_Traits.hpp"
#include "chip/Interrupt.hpp"
#include "chip/PM.hpp"
#include "chip/Package.hpp"
#include "chip/WaitBounds.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/StartUp/SharedIsr.hpp"
#include "kvasir/Util/Config.hpp"
#include "peripherals/EIC.hpp"

#include <cstdint>
#include <utility>

namespace Kvasir { namespace EIC {
    enum class InterruptType {
        none     = 0,
        edgeRise = 1,
        edgeFall = 2,
        edgeBoth = 3,

        levelHigh = 4,
        levelLow  = 5
    };

    namespace detail {
        using Regs = Kvasir::Peripheral::EIC::Registers<>;

        // One EXTINT bit of INTFLAG / INTENSET, typed as the register's own EXTINT field.
        template<unsigned Line, typename Org>
        using ExtIntBit = Register::FieldLocation<typename Org::Addr,
                                                  Register::maskFromRange(Line, Line),
                                                  typename decltype(Org::extint)::Access,
                                                  typename decltype(Org::extint)::DataType>;

        template<unsigned Bit, typename Org>
        struct EVCTRL {
            static_assert(Bit < 16,
                          "not a valid pin");
            using Addr     = typename Org::Addr;
            using AddrT    = typename Addr::RegType;
            using Loc      = decltype(Org::extinteo);
            using Access   = typename Loc::Access;
            using DataType = typename Loc::DataType;

            // the line's bit of EXTINTEO, under Register::fromReset (see CONFIG below): the event outputs of every line
            // in the list are one store, the others at their reset 0
            template<bool Event>
            static constexpr auto setEvent() {
                if constexpr(Event) {
                    return Kvasir::Register::fromReset<Org>(
                      Kvasir::Register::set(Kvasir::Register::bit<Bit>(Loc{})));
                } else {
                    return brigand::list<>();
                }
            }
        };

        template<unsigned Bit, template<unsigned> typename Org_>
        struct CONFIG {
            static_assert(Bit < 16,
                          "not a valid pin");

            using Org                   = Org_<(Bit > 7 ? 1 : 0)>;
            using Addr                  = typename Org::Addr;
            using AddrT                 = typename Addr::RegType;
            using TLoc                  = decltype(Org::sense0);
            using TAccess               = typename TLoc::Access;
            static constexpr auto tMask = TLoc::mask;
            using TDataType             = typename TLoc::DataType;

            using FLoc                  = decltype(Org::filten0);
            using FAccess               = typename FLoc::Access;
            static constexpr auto fMask = FLoc::mask;
            using FDataType             = typename FLoc::DataType;

            static constexpr int bitPos = (Bit % 8) * 4;

            static_assert(tMask
                          == Register::maskFromRange(2,
                                                     0));
            static_assert(fMask
                          == Register::maskFromRange(3,
                                                     3));

            static_assert(InterruptType::none == static_cast<InterruptType>(Org::SENSE0Val::none));
            static_assert(InterruptType::edgeRise
                          == static_cast<InterruptType>(Org::SENSE0Val::rise));
            static_assert(InterruptType::edgeFall
                          == static_cast<InterruptType>(Org::SENSE0Val::fall));
            static_assert(InterruptType::edgeBoth
                          == static_cast<InterruptType>(Org::SENSE0Val::both));
            static_assert(InterruptType::levelHigh
                          == static_cast<InterruptType>(Org::SENSE0Val::high));
            static_assert(InterruptType::levelLow
                          == static_cast<InterruptType>(Org::SENSE0Val::low));

            // The line's SENSEn / FILTENn - the generated fields at its nibble - under Register::fromReset<CONFIGn>:
            // the lines of one CONFIGn register, whoever declares them, are written together by Startup's single
            // apply of initStepPeripheryConfig, as ONE store with every line nobody configures at its reset value
            // (none, no filter) - whatever a bootloader left there.
            using Sense  = Register::FieldAt<Org, static_cast<unsigned>(bitPos)>;
            using Filter = Register::FieldAt<Org, static_cast<unsigned>(bitPos + 3)>;

            template<InterruptType Type>
            static constexpr auto setInterruptType() {
                using V = typename Sense::DataType;
                return Kvasir::Register::fromReset<Org>(
                  Kvasir::Register::write(Sense{}, Register::value<V, static_cast<V>(Type)>()));
            }

            template<bool Enable>
            static constexpr auto setFilter() {
                if constexpr(Enable) {
                    return Kvasir::Register::fromReset<Org>(Kvasir::Register::set(Filter{}));
                } else {
                    return brigand::list<>();
                }
            }
        };

        // The EXTINT line a pin drives on function A, by the chip's rule (chip/EIC_Traits.hpp)
        inline constexpr unsigned noExtInt   = Traits::none;
        inline constexpr unsigned notEncoded = Traits::notEncoded;

        template<int Port,
                 int Pin>
        consteval unsigned extIntOf(Io::PinLocation<Port,
                                                    Pin>) {
            return Traits::extIntOf(Port, Pin);
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

        static constexpr auto powerClockEnable = list(typename PM::Enable<ba>::action{});

        // Enabling the EIC with the filter on can raise a spurious INTFLAG.EXTINTx for RISE, BOTH
        // or LOW lines (SAM D21 errata DS80000760 1.9.1, SAM C20/C21 DS80000740 1.11.3). So:
        // enable, wait for sync, clear every flag and the vector's pending bit; Startup's
        // PeripheryEnable step enables the vector afterwards.
        static void preEnableRuntimeInit() {
            apply(Regs::CTRLA::overrideDefaults(set(Regs::CTRLA::enable)));
            waitForSync<Regs>();
            apply(reset(Regs::INTFLAG::extint));
            clearPending(std::make_index_sequence<16>{});
        }

    private:
        template<typename R>
        static void waitForSync() {
            if constexpr(Kvasir::Sam::Package::eicSyncBusyInStatus) {
                Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound>(
                  Kvasir::Register::isClear(R::STATUS::syncbusy));   // D21: STATUS.SYNCBUSY
            } else {
                Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound>(
                  Kvasir::Register::isClear(R::SYNCBUSY::enable));   // C21/E5x: SYNCBUSY.ENABLE
            }
        }

        template<std::size_t... Lines>
        static void clearPending(std::index_sequence<Lines...>) {
            (apply(Nvic::makeClearPending(Kvasir::Sam::Package::EicVector<Lines>{})), ...);
        }
    };

    // One EXTINT line: Pin on the EIC, Type its sense, Callback (void()) run in the interrupt for
    // each detection, at NVIC priority Priority (every line of one vector has to agree). Filter:
    // the majority filter (CONFIGn.FILTENx); Event: EVCTRL.EXTINTEOx, the line's event output.
    template<typename Pin,
             InterruptType         Type,
             auto                  Callback,
             int                   Priority,
             Io::PullConfiguration Pull   = Io::PullConfiguration::pullUp,
             bool                  Filter = true,
             bool                  Event  = false>
    struct ExtInt {
        static constexpr unsigned line = detail::extIntOf(Pin{});
        static_assert(
          line != detail::notEncoded,
          "this chip's EXTINT rule does not encode the pad's port (atsam_common/EicLines.hpp)");
        static_assert(line != detail::noExtInt,
                      "the pad drives no EXTINT line (PA08 is the NMI pin)");
        static_assert(Priority >= 0,
                      "an EXTINT line needs its NVIC priority");

        using Regs   = detail::Regs;
        using Vector = Kvasir::Sam::Package::EicVector<line>;

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
                                                        Io::OutputType::pushPull,
                                                        Io::OutputSpeed::low,
                                                        Io::OutputInit::low,
                                                        Pull>{},
                        Pin{}));

        static constexpr auto initStepPeripheryConfig
          = list(detail::CONFIG<line, Regs::CONFIG>::template setInterruptType<Type>(),
                 detail::CONFIG<line, Regs::CONFIG>::template setFilter<Filter>(),
                 detail::EVCTRL<line, typename Regs::EVCTRL>::template setEvent<Event>());
    };

    // The pull of an EXTINT helper's pins (SamPushButton, SamRotaryEncoder, SamQuadratureEncoder): PullUp unless the
    // helper's Config says `pull` - pullNone or pullDown for a line with an external pull resistor, which the 20-60 k
    // internal pull-up (SAM C21 DS60001479M RPULL md l.50199) would otherwise fight.
    template<typename Config>
    inline constexpr auto pullOf = KVASIR_CONFIG_OR(Config, pull, Io::PullConfiguration::pullUp);

    // The NVIC priority of a helper's lines: its Config's `eicPriority` (every line of the EIC vector shares one).
    template<typename Config>
    consteval int priorityOf() {
        static_assert(
          requires { Config::eicPriority; },
          "an EXTINT helper's config needs eicPriority, the NVIC priority of the EIC interrupt");
        return static_cast<int>(Config::eicPriority);
    }

    // An EXTINT line as the helpers set it up: Config's priority and pull, the filter on, no event output.
    template<typename Pin, InterruptType Type, auto Callback, typename Config>
    using HelperLine
      = ExtInt<Pin, Type, Callback, priorityOf<Config>(), pullOf<Config>, true, false>;

    // A Startup entry made of EXTINT lines, for a helper to derive from: the lines' resources, sub-interrupts and init
    // steps. A helper that extends one of them declares it again (name hiding: SamRotaryEncoder's initStepPinConfig,
    // a wrapper's SubIsrs with its own callback).
    template<typename Line, typename... More>
    struct LinesEntry {
        using Provides = brigand::append<typename Line::Provides, typename More::Provides...>;
        using Claims   = brigand::append<typename Line::Claims, typename More::Claims...>;
        using SubIsrs  = brigand::append<typename Line::SubIsrs, typename More::SubIsrs...>;

        static constexpr auto initStepPinConfig = [] {
            if constexpr(sizeof...(More) == 0) {
                return Line::initStepPinConfig;
            } else {
                return list(Line::initStepPinConfig, More::initStepPinConfig...);
            }
        }();
        static constexpr auto initStepPeripheryConfig = [] {
            if constexpr(sizeof...(More) == 0) {
                return Line::initStepPeripheryConfig;
            } else {
                return list(Line::initStepPeripheryConfig, More::initStepPeripheryConfig...);
            }
        }();
    };
}}   // namespace Kvasir::EIC
