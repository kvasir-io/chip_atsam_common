#pragma once

#include "chip/CAN_Traits.hpp"
#include "chip/Interrupt.hpp"
#include "chip/Io.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "kvasir/Util/attributes.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/CAN.hpp"
#include "uc_log/uc_log.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <optional>

namespace Kvasir { namespace CAN {

    struct CanMessage {
        constexpr std::uint32_t id() const {
            if((F0 & 0x40000000) != 0) { return F0 & 0x1FFFFFFF; }
            return (F0 & 0x1FFFFFFF) >> 18;
        }

        constexpr std::uint32_t size() const { return (F1 & 0x000F0000_u32) >> 16_u32; }

        constexpr void setId(std::uint32_t id) {
            if(id > 0x7FF) {
                F0 = (id & 0x1FFFFFFF) | 0x40000000;
            } else {
                F0 = (id & 0x7FF) << 18;
            }
        }

        constexpr void setSize(std::uint8_t size) {
            F1 = static_cast<std::uint32_t>(size << 16_u32);
        }

        constexpr auto begin() { return data.begin(); }

        constexpr auto end() { return std::next(data.begin(), static_cast<int>(size())); }

        constexpr auto begin() const { return data.begin(); }

        constexpr auto end() const { return std::next(data.begin(), static_cast<int>(size())); }

        std::uint32_t            F0{};
        std::uint32_t            F1{};
        std::array<std::byte, 8> data{};
    };

    static_assert(sizeof(CanMessage) == 16);

    struct CanFilter {
        enum class FilterType { Range = 0, Dual = 1, Classic = 3 };

        enum class FilterConfig {
            Disable     = 0,
            StoreFifo0  = 1,
            StoreFifo1  = 2,
            Reject      = 3,
            Priority    = 4,
            PrioFifi0   = 5,
            PrioFifi1   = 6,
            StoreBuffer = 7
        };

        std::uint32_t value{};

        constexpr void setId(std::uint32_t id) {
            assert(2047 >= id);
            value = (static_cast<std::uint32_t>(FilterType::Range) << 30)
                  | (static_cast<std::uint32_t>(FilterConfig::StoreFifo0) << 27) | id << 16 | id;
        }

        constexpr void disable() {
            value = static_cast<std::uint32_t>(FilterConfig::Disable) << 27;
        }
    };

    static_assert(sizeof(CanFilter) == 4);

    namespace Detail {
        /// The M_CAN's nominal bit timing (NBTP; SAM C20/C21 DS60001479 and D5x/E5x data sheets, CAN
        /// NBTP): a time quantum is NBRP + 1 GCLK_CAN periods, a bit is 1 + (NTSEG1 + 1) + (NTSEG2
        /// + 1) quanta, 4 to 385; NBRP 0..511, NTSEG1 1..255, NTSEG2 0..127, NSJW 0..127, each field
        /// used as one more than written.
        struct BitTiming {
            std::uint32_t       nbrp{};
            std::uint32_t       ntseg1{};
            std::uint32_t       ntseg2{};
            std::uint32_t       nsjw{};
            Prescaler::Rational achieved{};
            bool                found{};
        };

        /// The prescaler whose bit is closest to `baud` in rate, the smallest prescaler (most
        /// quanta) on a tie; the sample point at `samplePointPermille` of the bit (sync segment
        /// included), rounded to a quantum; the jump width as wide as phase segment 2. 8 MHz,
        /// 500 kbit/s, 750 give the reset value 0x06000A03 (16 quanta, 12 before the sample point).
        consteval BitTiming bitTiming(std::uint32_t clock,
                                      std::uint32_t baud,
                                      std::uint32_t samplePointPermille) {
            BitTiming best{};
            for(std::uint64_t pre = 1; pre <= 512; ++pre) {
                std::uint64_t const quanta = (clock + pre * baud / 2) / (pre * baud);
                if(quanta < 4 || quanta > 385) { continue; }
                std::uint64_t const beforeSample = (quanta * samplePointPermille + 500) / 1000;
                std::uint64_t const seg2 = std::clamp<std::uint64_t>(quanta - beforeSample, 1, 128);
                std::uint64_t const seg1 = quanta - 1 - seg2;
                if(seg1 < 2 || seg1 > 256) { continue; }
                Prescaler::Rational const achieved{clock, pre * quanta};
                auto const                error = [&](Prescaler::Rational const& r) {
                    // |r - baud| * den, compared across candidates as |num - baud den| / den
                    return r.num > baud * r.den ? r.num - baud * r.den : baud * r.den - r.num;
                };
                if(best.found
                   && error(achieved) * best.achieved.den >= error(best.achieved) * achieved.den)
                {
                    continue;
                }
                best = BitTiming{.nbrp     = static_cast<std::uint32_t>(pre - 1),
                                 .ntseg1   = static_cast<std::uint32_t>(seg1 - 1),
                                 .ntseg2   = static_cast<std::uint32_t>(seg2 - 1),
                                 .nsjw     = static_cast<std::uint32_t>(std::min(seg1, seg2) - 1),
                                 .achieved = achieved,
                                 .found    = true};
            }
            return best;
        }

        template<unsigned CanInstance, typename RXPIN>
        struct GetRxPinConfig;

        template<unsigned CanInstance, int Port, int Pin>
        struct GetRxPinConfig<CanInstance, Kvasir::Register::PinLocation<Port, Pin>> {
            using pinConfig = decltype(action(
              Kvasir::Io::Action::PinFunction<Traits::CanTraits::GetPinFunction<CanInstance>(
                Register::PinLocation<Port, Pin>{})>{},
              Register::PinLocation<Port, Pin>{}));
        };

        template<unsigned CanInstance, typename TXPIN>
        struct GetTxPinConfig;

        template<unsigned CanInstance, int Port, int Pin>
        struct GetTxPinConfig<CanInstance, Kvasir::Register::PinLocation<Port, Pin>> {
            using pinConfig = decltype(action(
              Kvasir::Io::Action::PinFunction<Traits::CanTraits::GetPinFunction<CanInstance>(
                Register::PinLocation<Port, Pin>{})>{},
              Register::PinLocation<Port, Pin>{}));
        };

        template<typename CANConfig_>
        struct CANBase {
            struct CANConfig : CANConfig_ {
                static constexpr auto userConfigOverride = [] {
                    if constexpr(requires { CANConfig_::userConfigOverride; }) {
                        return CANConfig_::userConfigOverride;
                    } else {
                        return brigand::list<>{};
                    }
                }();

                // the achieved bit rate against this (the bus needs every node within ~0.5 %)
                static constexpr auto maxBaudRateError = [] {
                    if constexpr(requires { CANConfig_::maxBaudRateError; }) {
                        return CANConfig_::maxBaudRateError;
                    } else {
                        return std::ratio<1, 1000>{};
                    }
                }();
                // where the bus is sampled, in 1/1000 of a bit; 750 is the reset value's
                static constexpr std::uint32_t samplePointPermille = [] {
                    if constexpr(requires { CANConfig_::samplePointPermille; }) {
                        return CANConfig_::samplePointPermille;
                    } else {
                        return 750U;
                    }
                }();
                static constexpr auto rejectNonMatching = [] {
                    if constexpr(requires { CANConfig_::rejectNonMatching; }) {
                        return CANConfig_::rejectNonMatching;
                    } else {
                        return false;
                    }
                }();
            };

            static constexpr auto Instance = CANConfig::instance;
            using Regs                     = Peripheral::CAN::Registers<Instance>;
            using InterruptIndexs = decltype(Traits::CanTraits::getCanIsrIndexs<Instance>());

            // clockSpeed is GCLK_CAN's
            static constexpr BitTiming Timing = bitTiming(CANConfig::clockSpeed,
                                                          CANConfig::baudRate,
                                                          CANConfig::samplePointPermille);
            static_assert(Timing.found,
                          "no CAN bit timing: baudRate needs 4 to 385 time quanta of 1 to 512 "
                          "GCLK_CAN periods");
            static constexpr bool BitRateInTolerance = [] {
                Prescaler::assertInTolerance<Timing.achieved,
                                             CANConfig::baudRate,
                                             Prescaler::Tolerance{CANConfig::maxBaudRateError},
                                             "CAN bit rate">();
                return true;
            }();
            // a static data member of a class template is initialised only when used: this use
            // is what runs the check
            static_assert(BitRateInTolerance);

            static constexpr auto rejectConfig = []() {
                if constexpr(CANConfig::rejectNonMatching) {
                    return list(write(Regs::GFC::ANFEValC::reject),
                                write(Regs::GFC::ANFSValC::reject),
                                set(Regs::GFC::rrfs),
                                set(Regs::GFC::rrfe));
                } else {
                    return Kvasir::MPL::list();
                }
            }();

            static constexpr auto powerClockEnable
              = list(typename PM::enable<Regs::baseAddr>::action{});

            static_assert(Traits::CanTraits::isRX<Instance>(CANConfig::rxPinLocation));
            static_assert(Traits::CanTraits::isTX<Instance>(CANConfig::txPinLocation));

            static constexpr auto initStepPinConfig = list(
              typename GetRxPinConfig<Instance, std::decay_t<decltype(CANConfig::rxPinLocation)>>::
                pinConfig{},
              typename GetTxPinConfig<Instance, std::decay_t<decltype(CANConfig::txPinLocation)>>::
                pinConfig{});

            static constexpr auto initStepPeripheryConfig = list(
              //       Regs::BAUD::overrideDefaults(
              //       typename GetBaudConfig<Regs, CANConfig::clockSpeed, CANConfig::baudRate>::config{}),

              //  Regs::CTRLA::overrideDefaults(
              //  write(Regs::CTRLA::MODEValC::i2c_master),
              //  write(getSpeedConfig<Regs, I2CConfig::baudRate>())),
              //  set(Regs::INTENSET::mb),
              //  set(Regs::INTENSET::sb),
              //  set(Regs::INTENSET::error),

              set(Regs::CCCR::init),
              Register::SequencePoint{},
              set(Regs::CCCR::cce),
              Register::SequencePoint{},
              // NBTP is writable only with CCCR.CCE and INIT set (NBTP, "Write-restricted")
              Regs::NBTP::overrideDefaults(
                write(Regs::NBTP::nbrp, Register::value<Timing.nbrp>()),
                write(Regs::NBTP::ntseg1, Register::value<Timing.ntseg1>()),
                write(Regs::NBTP::ntseg2, Register::value<Timing.ntseg2>()),
                write(Regs::NBTP::nsjw, Register::value<Timing.nsjw>())),
              set(Regs::IE::rf0le),
              rejectConfig,
              //set(Regs::IE::rf0ne),
              /*     set(Regs::IE::rf0ne),
              set(Regs::IE::rf0we),
              set(Regs::IE::rf0fe),
              set(Regs::IE::rf0le),
              set(Regs::IE::rf1ne),
              set(Regs::IE::rf1we),
              set(Regs::IE::rf1fe),
              set(Regs::IE::rf1le),
              set(Regs::IE::hpme),
              set(Regs::IE::tce),
              set(Regs::IE::tcfe),
              set(Regs::IE::tfee),
              set(Regs::IE::tefne),
              set(Regs::IE::tefne),
              set(Regs::IE::tefwe),
              set(Regs::IE::tefle),
              set(Regs::IE::tswe),
              set(Regs::IE::mrafe),
              set(Regs::IE::tooe),
              set(Regs::IE::drxe),
              set(Regs::IE::bece),
              set(Regs::IE::beue),
              set(Regs::IE::eloe),
              set(Regs::IE::epe),
              set(Regs::IE::ewe),
              set(Regs::IE::boe),
              set(Regs::IE::wdie),
              set(Regs::IE::peae),
              set(Regs::IE::pede),
              set(Regs::IE::arae),
*/
              set(Regs::ILE::eint0),
              CANConfig::userConfigOverride);
            static constexpr auto initStepInterruptConfig
              = list(Nvic::makeSetPriority<CANConfig::isrPriority>(InterruptIndexs{}),
                     Nvic::makeClearPending(InterruptIndexs{}));

            static constexpr auto initStepPeripheryEnable
              = list(Nvic::makeEnable(InterruptIndexs{}));
        };
    }   // namespace Detail

    template<typename CANConfig_, typename Clock>
    struct CANBehavior : Detail::CANBase<CANConfig_> {
        using base       = Detail::CANBase<CANConfig_>;
        using Regs       = typename base::Regs;
        using tp         = typename Clock::time_point;
        using CanMessage = ::Kvasir::CAN::CanMessage;

        static constexpr auto MaxRxSize     = 64;
        static constexpr auto MaxTxSize     = 32;
        static constexpr auto MaxFilterSize = 128;

        struct CANConfig : CANConfig_ {
            static constexpr auto rxSize = [] {
                if constexpr(requires { CANConfig_::rxSize; }) {
                    return CANConfig_::rxSize;
                } else {
                    return MaxRxSize;
                }
            }();

            static constexpr auto txSize = [] {
                if constexpr(requires { CANConfig_::txSize; }) {
                    return CANConfig_::txSize;
                } else {
                    return MaxTxSize;
                }
            }();
            static constexpr auto filterSize = [] {
                if constexpr(requires { CANConfig_::filterSize; }) {
                    return CANConfig_::filterSize;
                } else {
                    if constexpr(requires { CANConfig_::rejectNonMatching; }) {
                        return MaxFilterSize;
                    } else {
                        return 0;
                    }
                }
            }();
        };

        static constexpr std::size_t RxSize     = CANConfig::rxSize;
        static constexpr std::size_t TxSize     = CANConfig::txSize;
        static constexpr std::size_t FilterSize = CANConfig::filterSize;

        static_assert(MaxRxSize >= RxSize,
                      "rx size to big");
        static_assert(MaxTxSize >= TxSize,
                      "tx size to big");
        static_assert(MaxFilterSize >= FilterSize,
                      "filter size to big");

        [[KVASIR_SECTION_MEMBER(
          ".noInitLowRam")]] static inline std::array<CanMessage, RxSize>    RxFIFO{};
        [[KVASIR_SECTION_MEMBER(
          ".noInitLowRam")]] static inline std::array<CanMessage, TxSize>    TxFIFO{};
        [[KVASIR_SECTION_MEMBER(
          ".noInitLowRam")]] static inline std::array<CanFilter, FilterSize> Filter{};

        static void runtimeInit() {
            std::uint32_t const rx
              = reinterpret_cast<std::uint32_t>(std::addressof(RxFIFO)) & 0x0000FFFF_u32;
            std::uint32_t const tx
              = reinterpret_cast<std::uint32_t>(std::addressof(TxFIFO)) & 0x0000FFFF_u32;
            std::uint32_t const filter
              = reinterpret_cast<std::uint32_t>(std::addressof(Filter)) & 0x0000FFFF_u32;

            apply(write(Regs::RXF0C::f0sa, rx),
                  write(Regs::RXF0C::f0s, Kvasir::Register::value<RxSize>()));

            apply(write(Regs::TXBC::tbsa, tx),
                  write(Regs::TXBC::tfqs, Kvasir::Register::value<TxSize>()));

            apply(write(Regs::SIDFC::flssa, filter),
                  write(Regs::SIDFC::lss, Kvasir::Register::value<FilterSize>()));

            for(auto& f : Filter) { f.disable(); }

            apply(clear(Regs::CCCR::init));
        }

        static void setFilter(std::uint8_t  number,
                              std::uint32_t id) {
            assert(FilterSize > number);
            Filter[number].setId(id);
        }

        static bool send(CanMessage const& msg) {
            auto const state = apply(read(Regs::TXFQS::tfqpi), read(Regs::TXFQS::tffl));
            if(get<1>(state) == 0) { return false; }
            TxFIFO[get<0>(state)] = msg;
            apply(write(Regs::TXBAR::ar, 1_u32 << get<0>(state)));
            return true;
        }

        static bool transmissonComplete() {
            auto const state = apply(read(Regs::TXFQS::tffl));
            return get<0>(state) == TxSize;
        }

        static std::optional<CanMessage> recv() {
            std::optional<CanMessage> ret{};
            auto const state = apply(read(Regs::RXF0S::f0gi), read(Regs::RXF0S::f0fl));
            if(get<1>(state) == 0) { return ret; }

            ret = RxFIFO[get<0>(state)];

            apply(write(Regs::RXF0A::f0ai, get<0>(state)));

            return ret;
        }

        // ISR
        static void onIsr() {
            auto const state = apply(read(Regs::IR::rf0l));
            if(get<0>(state)) { UC_LOG_T("CAN ISR overrrun {}", get<0>(state)); }
            apply(set(Regs::IR::rf0l));
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));
    };
}}   // namespace Kvasir::CAN
