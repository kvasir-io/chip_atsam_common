#pragma once
#include "DMAC.hpp"
//#include "chip/MCLK.hpp"
#include "chip/PM.hpp"
#include "chip/Sercom_Traits.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "peripherals/SERCOM_USART.hpp"

#include <algorithm>
#include <cassert>
#include <memory>
#include <optional>
#include <ratio>

namespace Kvasir { namespace Sercom { namespace Usart {
    template<typename = void>
    struct NotUsed {};

    enum class DataBits {
        _5,
        _6,
        _7,
        _8,
        _9,
    };

    enum class Parity {
        none,
        odd,
        even,
    };

    enum class StopBits {
        _1,
        _2,
    };

    namespace Detail {

        template<unsigned SercomInstance>
        struct Config {
            using Regs = Kvasir::Peripheral::SERCOM_USART::Registers<SercomInstance>;

            static constexpr bool isValidPinLocationRX(NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationRX(Kvasir::Register::PinLocation<Port,
                                                                                     Pin>) {
                return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(0, 1, 2, 3);
            }

            static constexpr bool isValidPinLocationTX(NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationTX(Kvasir::Register::PinLocation<Port,
                                                                                     Pin>) {
                return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(0, 2);
            }

            template<int LPort,
                     int LPin,
                     int RPort,
                     int RPin>
            static constexpr bool PinLocationAreTheSame(Kvasir::Register::PinLocation<LPort,
                                                                                      LPin> l,
                                                        Kvasir::Register::PinLocation<RPort,
                                                                                      RPin> r) {
                return Io::Detail::PinLocationEqual(l, r);
            }

            template<int LPort,
                     int LPin>
            static constexpr bool PinLocationAreTheSame(Kvasir::Register::PinLocation<LPort,
                                                                                      LPin>,
                                                        NotUsed<>) {
                return false;
            }

            template<int RPort,
                     int RPin>
            static constexpr bool PinLocationAreTheSame(NotUsed<>,
                                                        Kvasir::Register::PinLocation<RPort,
                                                                                      RPin>) {
                return false;
            }

            template<int Port,
                     int Pin>
            static constexpr int GetRxPadValue(Kvasir::Register::PinLocation<Port,
                                                                             Pin>) {
                if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) {
                    return std::numeric_limits<int>::max();
                }
                for(auto mi : Traits::SercomTraits::pinMuxInfos) {
                    if(mi.sercomInstance != SercomInstance) { continue; }
                    if(mi.port != Port) { continue; }
                    if(mi.pin != Pin) { continue; }
                    return mi.POVal;
                }
                return std::numeric_limits<int>::max();
            }

            template<int Port,
                     int Pin>
            static constexpr int GetTxPadValue(Kvasir::Register::PinLocation<Port,
                                                                             Pin>) {
                if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) {
                    return std::numeric_limits<int>::max();
                }
                for(auto mi : Traits::SercomTraits::pinMuxInfos) {
                    if(mi.sercomInstance != SercomInstance) { continue; }
                    if(mi.port != Port) { continue; }
                    if(mi.pin != Pin) { continue; }
                    if(mi.POVal == 1 || mi.POVal == 3) { continue; }

                    if(mi.POVal == 0) { return mi.POVal; }
                    return mi.POVal - 1;
                }
                return std::numeric_limits<int>::max();
            }

            template<typename RXPIN>
            struct GetRxPinConfig;

            template<typename dummy>
            struct GetRxPinConfig<NotUsed<dummy>> {
                using pad       = decltype(write(Regs::CTRLA::rxpo, Register::value<0>()));
                using enable    = decltype(clear(Regs::CTRLB::rxen));
                using pinConfig = brigand::list<>;
                using interrupt = brigand::list<>;
                template<typename InterruptIndex>
                using interruptEnable = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetRxPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pad       = decltype(write(
                  Regs::CTRLA::rxpo,
                  Register::value<GetRxPadValue(Register::PinLocation<Port, Pin>{})>()));
                using enable    = decltype(set(Regs::CTRLB::rxen));
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<
                    SercomInstance>(Register::PinLocation<Port, Pin>{})>{},
                  Register::PinLocation<Port, Pin>{}));
                using interrupt = brigand::list<decltype(set(Regs::INTENSET::rxc)),
                                                decltype(set(Regs::INTENSET::error))>;
                template<typename InterruptIndex>
                using interruptEnable = decltype(Kvasir::Nvic::makeEnable(InterruptIndex{}));
            };

            template<typename TXPIN>
            struct GetTxPinConfig;

            template<typename dummy>
            struct GetTxPinConfig<NotUsed<dummy>> {
                using pad       = decltype(write(Regs::CTRLA::txpo, Register::value<0>()));
                using enable    = decltype(clear(Regs::CTRLB::txen));
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetTxPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pad       = decltype(write(
                  Regs::CTRLA::txpo,
                  Register::value<GetTxPadValue(Register::PinLocation<Port, Pin>{})>()));
                using enable    = decltype(set(Regs::CTRLB::txen));
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<
                    SercomInstance>(Register::PinLocation<Port, Pin>{})>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<DataBits dbits>
            struct GetDataBitConfig {
                static constexpr auto config_ = []() {
                    if constexpr(dbits == DataBits::_5) {
                        return write(Regs::CTRLB::CHSIZEValC::_5_bit);
                    } else if constexpr(dbits == DataBits::_6) {
                        return write(Regs::CTRLB::CHSIZEValC::_6_bit);
                    } else if constexpr(dbits == DataBits::_7) {
                        return write(Regs::CTRLB::CHSIZEValC::_7_bit);
                    } else if constexpr(dbits == DataBits::_8) {
                        return write(Regs::CTRLB::CHSIZEValC::_8_bit);
                    } else if constexpr(dbits == DataBits::_9) {
                        return write(Regs::CTRLB::CHSIZEValC::_9_bit);
                    }
                }();
                using config = decltype(config_);
            };

            template<StopBits sbits>
            struct GetStopBitConfig {
                static constexpr auto config_ = []() {
                    if constexpr(sbits == StopBits::_1) {
                        return clear(Regs::CTRLB::sbmode);
                    } else if constexpr(sbits == StopBits::_2) {
                        return set(Regs::CTRLB::sbmode);
                    }
                }();
                using config = decltype(config_);
            };

            template<Parity pa>
            struct GetParityConfig {
                static constexpr auto config_ = []() {
                    if constexpr(pa == Parity::none) {
                        return brigand::list<decltype(write(Regs::CTRLA::form,
                                                            Register::value<0>())),
                                             decltype(clear(Regs::CTRLB::pmode))>{};
                    } else if constexpr(pa == Parity::odd) {
                        return brigand::list<decltype(write(Regs::CTRLA::form,
                                                            Register::value<1>())),
                                             decltype(set(Regs::CTRLB::pmode))>{};
                    } else if constexpr(pa == Parity::even) {
                        return brigand::list<decltype(write(Regs::CTRLA::form,
                                                            Register::value<1>())),
                                             decltype(clear(Regs::CTRLB::pmode))>{};
                    }
                }();
                using config = decltype(config_);
            };
        };

        // Arithmetic mode, 16 samples per bit: f_baud = f_ref / 16 x (1 - BAUD / 65536), BAUD 16
        // bits (SAM D21 DS40001882L 25.6.2.3 Table 25-2; SAM C20/C21 Table 30-2; SAM D5x/E5x
        // Table 33-2). As a multiplier: f_baud = f_ref x n / 2^20 with n = 65536 - BAUD, 1..65536.
        static constexpr Prescaler::Multiplier ArithmeticBaud{.first = 1,
                                                              .last  = 65536,
                                                              .den   = std::uint64_t{1} << 20};

        // the nearest BAUD; on a tie the larger BAUD (the slower rate), as round-half-up gave it
        constexpr auto baudDivider(std::uint32_t f_clockSpeed,
                                   std::uint32_t f_baud) {
            return Prescaler::fromMultiplier(f_clockSpeed, f_baud, ArithmeticBaud);
        }

        constexpr std::uint32_t calcBaudReg(std::uint32_t f_clockSpeed,
                                            std::uint32_t f_baud) {
            return 65536U - baudDivider(f_clockSpeed, f_baud).setting;
        }

    }   // namespace Detail

    template<typename UsartConfig_>
    struct UsartBase {
        struct UsartConfig : UsartConfig_ {
            static constexpr auto userConfigOverride = [] {
                if constexpr(requires { UsartConfig_::userConfigOverride; }) {
                    return UsartConfig_::userConfigOverride;
                } else {
                    return brigand::list<>{};
                }
            }();

            static constexpr auto maxBaudRateError = [] {
                if constexpr(requires { UsartConfig_::maxBaudRateError; }) {
                    return UsartConfig_::maxBaudRateError;
                } else {
                    return std::ratio<5, 1000>{};
                }
            }();
        };

        // needed config
        // clockSpeed
        // baudRate
        // usartInstance
        // txPinLocation
        // rxPinLocation
        // userConfigOverride
        // dataBits
        // parity
        // stopBits
        static constexpr auto Instance = UsartConfig::instance;
        using Regs                     = Kvasir::Peripheral::SERCOM_USART::Registers<Instance>;

        using InterruptIndexs = decltype(Traits::SercomTraits::getSercomIsrIndexs<Instance>());

        using Config = Detail::Config<Instance>;

        static constexpr auto RxDmaTrigger = Traits::SercomTraits::DmaRX_Trigger<Instance>();
        static constexpr auto TxDmaTrigger = Traits::SercomTraits::DmaTX_Trigger<Instance>();

        // the achieved baud rate against maxBaudRateError; a failure prints wanted, got and ppm
        static constexpr bool BaudInTolerance = [] {
            Prescaler::assertInTolerance<
              Detail::baudDivider(UsartConfig::clockSpeed, UsartConfig::baudRate).achieved,
              UsartConfig::baudRate,
              Prescaler::Tolerance{UsartConfig::maxBaudRateError},
              "USART baud rate">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(BaudInTolerance);
        static_assert(Config::isValidPinLocationTX(UsartConfig::txPinLocation),
                      "invalid TXPin");
        static_assert(Config::isValidPinLocationRX(UsartConfig::rxPinLocation),
                      "invalid RXPin");
        static_assert(!Config::PinLocationAreTheSame(UsartConfig::txPinLocation,
                                                     UsartConfig::rxPinLocation),
                      "TX and RX are the same pin");

        static constexpr auto powerClockEnable
          = list(typename PM::enable<Regs::baseAddr>::action{});

        static constexpr auto initStepPinConfig
          = list(typename Config::template GetTxPinConfig<
                   std::decay_t<decltype(UsartConfig::txPinLocation)>>::pinConfig{},
                 typename Config::template GetRxPinConfig<
                   std::decay_t<decltype(UsartConfig::rxPinLocation)>>::pinConfig{});

        static constexpr auto initStepPeripheryConfig = list(
          typename Config::template GetTxPinConfig<
            std::decay_t<decltype(UsartConfig::txPinLocation)>>::enable{},
          typename Config::template GetRxPinConfig<
            std::decay_t<decltype(UsartConfig::rxPinLocation)>>::enable{},
          typename Config::template GetTxPinConfig<
            std::decay_t<decltype(UsartConfig::txPinLocation)>>::pad{},
          typename Config::template GetRxPinConfig<
            std::decay_t<decltype(UsartConfig::rxPinLocation)>>::pad{},

          typename Config::template GetDataBitConfig<UsartConfig::dataBits>::config{},
          typename Config::template GetStopBitConfig<UsartConfig::stopBits>::config{},
          typename Config::template GetParityConfig<UsartConfig::parity>::config{},

          write(
            Regs::BAUD::baud,
            Register::value<std::uint16_t,
                            Detail::calcBaudReg(UsartConfig::clockSpeed, UsartConfig::baudRate)>()),
          clear(Regs::CTRLA::enable),
          clear(Regs::CTRLA::swrst),
          // The following parameters are only supported via userConfigOverride
          write(Regs::CTRLA::MODEValC::usart_int_clk),
          clear(Regs::CTRLA::runstdby),

          clear(Regs::CTRLA::ibon),
          write(Regs::CTRLA::SAMPRValC::_16x_arithmetic),
          write(Regs::CTRLA::sampa, Register::value<0>()),
          clear(Regs::CTRLA::cmode),
          clear(Regs::CTRLA::cpol),
          set(Regs::CTRLA::dord),

          clear(Regs::CTRLB::colden),
          clear(Regs::CTRLB::sfde),
          clear(Regs::CTRLB::enc),

          typename Config::template GetRxPinConfig<
            std::decay_t<decltype(UsartConfig::rxPinLocation)>>::interrupt{},
          UsartConfig::userConfigOverride);

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<UsartConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable = list(
          set(Regs::CTRLA::enable),
          typename Config::template GetRxPinConfig<std::decay_t<
            decltype(UsartConfig::rxPinLocation)>>::template interruptEnable<InterruptIndexs>{});
    };

    template<typename UartConfig,
             typename Dma,
             DMAC::DMAChannel  DmaChannel,
             DMAC::DMAPriority DmaPriority,
             std::size_t       BufferSize>
    struct UartBehaviorImpl : Kvasir::Sercom::Usart::UsartBase<UartConfig> {
        using base                           = Kvasir::Sercom::Usart::UsartBase<UartConfig>;
        using Regs                           = typename base::Regs;
        using DmaRegs                        = typename Dma::Regs;
        static constexpr std::size_t DmaChId = std::size_t(DmaChannel);
        static constexpr std::size_t DmaLvl  = std::size_t(DmaPriority);

        static_assert(Dma::numberOfChannels > DmaChId);

        inline static Kvasir::Atomic::Queue<std::optional<std::byte>, BufferSize> rxbuffer_{};

        enum class OperationState { succeeded, failed, ongoing };

        inline static bool b = false;

        // inline static std::atomic<OperationState> operationState_ = OperationState::succeeded;
        static OperationState operationState() {
            if(!b) { return OperationState::succeeded; }
            if(!Dma::template wd<DmaChannel>().isValid() && apply(read(Regs::INTFLAG::txc))) {
                b = false;
                return OperationState::succeeded;
            }
            // TODO timeout
            return OperationState::ongoing;
        }

        static void startDma() {
            apply(Dma::template start<DmaChannel, DmaPriority, base::TxDmaTrigger>());
            b = true;   // TODO
        }

        template<typename C>
        static void send_nocopy(C const& c) {
            send_nocopy(c.begin(), c.end());
        }

        template<typename InputIt>
        static void send_nocopy(InputIt first,
                                InputIt last) {
            static_assert(sizeof(*first) == 1, "only bytes");
            Dma::template rd<DmaChannel>()
              = DMAC::DmacDescriptor(true,
                                     DMAC::DmacDescriptor::stepsize::x1,
                                     DMAC::DmacDescriptor::stepsel::src,
                                     DMAC::DmacDescriptor::dstinc::no_increment,
                                     DMAC::DmacDescriptor::srcinc::increment,
                                     DMAC::DmacDescriptor::beatsize::byte,
                                     DMAC::DmacDescriptor::blockact::noact,
                                     DMAC::DmacDescriptor::evosel::disabled,
                                     std::uint16_t(std::distance(first, last)),
                                     // to_address: a container's iterator need not be a pointer
                                     reinterpret_cast<std::uint32_t>(std::to_address(last)),
                                     Regs::DATA8::Addr::value);
            // set timeout
            startDma();
        }

        /* template<typename C>
            static void send(C const& c) {
            // TODO copy to DMA buffer
         }*/
    };

    template<typename UartConfig,
             typename Dma,
             DMAC::DMAChannel  DmaChannel,
             DMAC::DMAPriority DmaPriority,
             std::size_t       BufferSize,
             bool              isr>
    struct UartBehaviorSelector;

    template<typename UartConfig,
             typename Dma,
             DMAC::DMAChannel  DmaChannel,
             DMAC::DMAPriority DmaPriority,
             std::size_t       BufferSize>
    struct UartBehaviorSelector<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize, false>
      : UartBehaviorImpl<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize> {};

    template<typename UartConfig,
             typename Dma,
             DMAC::DMAChannel  DmaChannel,
             DMAC::DMAPriority DmaPriority,
             std::size_t       BufferSize>
    struct UartBehaviorSelector<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize, true>
      : UartBehaviorImpl<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize> {
        using base = UartBehaviorImpl<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize>;
        using Regs = typename base::Regs;

        static void onIsr() {
            auto const intflag = apply(read(Regs::INTFLAG::error, Regs::INTFLAG::rxc));

            if(intflag.template get<0>()) {
                apply(set(Regs::INTFLAG::error));
                base::rxbuffer_.push(std::nullopt);
            } else if(intflag.template get<1>()) {
                std::byte data = std::byte(apply(read(Regs::DATA8::data)).template get<0>());
                base::rxbuffer_.push(data);
            } else {
                assert(false);
            }
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));
    };

    template<typename UartConfig,
             typename Dma,
             DMAC::DMAChannel  DmaChannel,
             DMAC::DMAPriority DmaPriority,
             std::size_t       BufferSize>
    struct UartBehavior
      : UartBehaviorSelector<
          UartConfig,
          Dma,
          DmaChannel,
          DmaPriority,
          BufferSize,
          !std::is_same_v<std::remove_cvref_t<decltype(UartConfig::rxPinLocation)>,
                          std::remove_cvref_t<decltype(Kvasir::Sercom::Usart::NotUsed<>{})>>> {};

}}}   // namespace Kvasir::Sercom::Usart
