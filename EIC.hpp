#pragma once

#include "core/Nvic.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/EIC.hpp"

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

        template<unsigned bit, typename Org>
        struct INTENSET {
            static_assert(bit < 16,
                          "not a valid pin");
            using Addr     = typename Org::Addr;
            using Loc      = decltype(Org::extint);
            using Access   = typename Loc::Access;
            using DataType = typename Loc::DataType;

            static constexpr auto set() {
                return Kvasir::Register::set(
                  Register::
                    FieldLocation<Addr, Register::maskFromRange(bit, bit), Access, DataType>{});
            }
        };

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

        template<unsigned bit, typename Org>
        struct INTFLAG {
            using Addr     = typename Org::Addr;
            using AddrT    = typename Addr::RegType;
            using Loc      = decltype(Org::extint);
            using Access   = typename Loc::Access;
            using DataType = typename Loc::DataType;

            static_assert(bit < 16,
                          "not a valid pin");

            static constexpr auto clear() {
                return Kvasir::Register::set(
                  Register::
                    FieldLocation<Addr, Register::maskFromRange(bit, bit), Access, DataType>{});
            }

            static constexpr auto runtimeClear(bool b) {
                return Kvasir::Register::write(
                  Register::
                    FieldLocation<Addr, Register::maskFromRange(bit, bit), Access, DataType>{},
                  static_cast<DataType>(b ? 1U : 0U));
            }

            static constexpr auto get() {
                return Kvasir::Register::read(
                  Register::
                    FieldLocation<Addr, Register::maskFromRange(bit, bit), Access, DataType>{});
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

        template<typename Reg,
                 int Port,
                 int Pin>
        constexpr auto enablePinInterrupt(Register::PinLocation<Port,
                                                                Pin>) {
            return INTENSET<Pin % 16, Reg>::set();
        }

        template<template<unsigned> typename Reg,
                 InterruptType Type,
                 int           Port,
                 int           Pin>
        constexpr auto setInterruptType(Register::PinLocation<Port,
                                                              Pin>) {
            // use with care
            return CONFIG<Pin % 16, Reg>::template setInterruptType<Type>();
        }

        template<template<unsigned> typename Reg,
                 bool Filter,
                 int  Port,
                 int  Pin>
        constexpr auto setFilter(Register::PinLocation<Port,
                                                       Pin>) {
            // use with care
            return CONFIG<Pin % 16, Reg>::template setFilter<Filter>();
        }

        template<typename Reg,
                 bool Event,
                 int  Port,
                 int  Pin>
        constexpr auto setEvent(Register::PinLocation<Port,
                                                      Pin>) {
            // use with care
            return EVCTRL<Pin % 16, Reg>::template setEvent<Event>();
        }

        template<typename Reg,
                 int Port,
                 int Pin>
        constexpr auto RuntimeClearInterrupt(Register::PinLocation<Port,
                                                                   Pin>,
                                             bool v) {
            return INTFLAG<Pin % 16, Reg>::runtimeClear(v);
        }

        template<typename Reg,
                 int Port,
                 int Pin>
        constexpr auto getInterruptFlag(Register::PinLocation<Port,
                                                              Pin>) {
            return INTFLAG<Pin % 16, Reg>::get();
        }

    }   // namespace detail

    template<typename Clock, typename EICConfig, typename... PinConfigs>
    struct EicBase {
        using Regs                   = Kvasir::Peripheral::EIC::Registers<>;
        static constexpr unsigned ba = Regs::baseAddr;
        using InterruptIndex         = decltype(Kvasir::Interrupt::eic);

        static constexpr auto powerClockEnable = list(typename PM::enable<ba>::action{});

        static constexpr auto initStepPinConfig
          = list(action(Kvasir::Io::Action::PinFunction<0,
                                                        Io::OutputType::PushPull,
                                                        Io::OutputSpeed::Low,
                                                        Io::OutputInit::Low,
                                                        PinConfigs::pull>{},
                        PinConfigs::pin)...);

        static constexpr auto initStepPeripheryConfig
          = list(detail::enablePinInterrupt<Regs::INTENSET>(PinConfigs::pin)...,
                 detail::setInterruptType<Regs::CONFIG, PinConfigs::type>(PinConfigs::pin)...,
                 detail::setFilter<Regs::CONFIG, PinConfigs::filter>(PinConfigs::pin)...,
                 detail::setEvent<Regs::EVCTRL, PinConfigs::enableEvent>(PinConfigs::pin)...);

        static constexpr auto initStepInterruptConfig = list(
          action(Kvasir::Nvic::Action::SetPriority<EICConfig::IsrPriority>{}, InterruptIndex{}),
          action(Kvasir::Nvic::Action::clearPending, InterruptIndex{}));

        static constexpr auto initStepPeripheryEnable
          = list(Regs::CTRLA::overrideDefaults(set(Regs::CTRLA::enable)),
                 makeEnable(InterruptIndex{}));

        template<std::size_t I,
                 typename Flags,
                 typename CBS>
        static void callIfTrue(Flags flags,
                               CBS   cbs) {
            if(Kvasir::Register::get<I>(flags)) { std::get<I>(cbs)(); }
        }

        template<typename Flags,
                 typename CBS,
                 std::size_t... Is>
        static void callIfTrue(Flags flags,
                               CBS   cbs,
                               std::index_sequence<Is...>) {
            return ((callIfTrue<Is>(flags, cbs)), ...);
        }

        template<std::size_t I,
                 typename Flags,
                 typename Pins>
        static auto clearIfTrue(Flags flags,
                                Pins  pins) {
            return detail::RuntimeClearInterrupt<Regs::INTFLAG>(std::get<I>(pins),
                                                                Kvasir::Register::get<I>(flags));
        }

        template<typename Flags,
                 typename Pins,
                 std::size_t... Is>
        static void clearIfTrue(Flags flags,
                                Pins  pins,
                                std::index_sequence<Is...>) {
            return Kvasir::Register::apply(clearIfTrue<Is>(flags, pins)...);
        }

        // ISR
        static void onIsr() {
            static constexpr auto pins = std::tuple<decltype(PinConfigs::pin)...>{};
            static constexpr auto callbacks
              = std::tuple<decltype(PinConfigs::callback)...>{PinConfigs::callback...};
            auto const flags = apply(detail::getInterruptFlag<Regs::INTFLAG>(PinConfigs::pin)...);

            callIfTrue(flags, callbacks, std::make_index_sequence<sizeof...(PinConfigs)>{});
            clearIfTrue(flags, pins, std::make_index_sequence<sizeof...(PinConfigs)>{});
        }

        static constexpr Kvasir::Nvic::Isr<std::addressof(onIsr),
                                           std::remove_const_t<InterruptIndex>>
          isr{};
    };
}}   // namespace Kvasir::EIC
