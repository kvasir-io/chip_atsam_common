#pragma once
#include "DMAC.hpp"
#include "chip/PM.hpp"
#include "chip/Package.hpp"
#include "chip/WaitBounds.hpp"
#include "chip/atsam_common/Clocks.hpp"
#include "chip/atsam_common/SercomBaud.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Hal/Clocks.hpp"
#include "kvasir/Hal/DmaHandoff.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Hal/Uart.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Prescaler.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <ratio>
#include <span>
#include <type_traits>

namespace Kvasir { namespace Sercom { namespace Usart {
    namespace Detail {

        template<unsigned SercomInstance>
        struct Config {
            using Regs = Sam::Package::SercomUsart<SercomInstance>;

            static constexpr bool isValidPinLocationRX(Io::NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationRX(Kvasir::Io::PinLocation<Port,
                                                                               Pin>) {
                return Sam::Package::Sercom::validIfPOVal<SercomInstance, Port, Pin>(0, 1, 2, 3);
            }

            static constexpr bool isValidPinLocationTX(Io::NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationTX(Kvasir::Io::PinLocation<Port,
                                                                               Pin>) {
                return Sam::Package::Sercom::validIfPOVal<SercomInstance, Port, Pin>(0, 2);
            }

            // TXPO from the chip's pad table (SercomPads.hpp): TxD on a pad the chip does not offer is a compile
            // error naming the pad and the chip
            template<int Port,
                     int Pin>
            static constexpr int getTxPadValue(Kvasir::Io::PinLocation<Port,
                                                                       Pin>) {
                return Sam::Pads::txpo<Sam::Package::SercomPads,
                                       Sam::Package::Sercom::getPOVal<SercomInstance>(
                                         Io::PinLocation<Port, Pin>{})>();
            }

            template<typename RXPIN>
            struct GetRxPinConfig;

            template<typename Dummy>
            struct GetRxPinConfig<Io::NotUsed<Dummy>> {
                using pad       = decltype(write(Regs::CTRLA::rxpo,
                                                 fieldValue<decltype(Regs::CTRLA::rxpo), 0>()));
                using enable    = decltype(clear(Regs::CTRLB::rxen));
                using PinConfig = brigand::list<>;
                using interrupt = brigand::list<>;
                template<typename InterruptIndex>
                using InterruptEnable = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetRxPinConfig<Kvasir::Io::PinLocation<Port, Pin>> {
                using pad
                  = decltype(write(Regs::CTRLA::rxpo,
                                   fieldValue<decltype(Regs::CTRLA::rxpo),
                                              Sam::Package::Sercom::getPOVal<SercomInstance>(
                                                Io::PinLocation<Port, Pin>{})>()));
                using enable    = decltype(set(Regs::CTRLB::rxen));
                using PinConfig = Sam::
                  PinFunctionOf<Sam::Package::Sercom, SercomInstance, Io::PinLocation<Port, Pin>>;
                using interrupt = brigand::list<decltype(set(Regs::INTENSET::rxc)),
                                                decltype(set(Regs::INTENSET::error))>;
                template<typename InterruptIndex>
                using InterruptEnable = decltype(Kvasir::Nvic::makeEnable(InterruptIndex{}));
            };

            template<typename TXPIN>
            struct GetTxPinConfig;

            template<typename Dummy>
            struct GetTxPinConfig<Io::NotUsed<Dummy>> {
                using pad       = decltype(write(Regs::CTRLA::txpo,
                                                 fieldValue<decltype(Regs::CTRLA::txpo), 0>()));
                using enable    = decltype(clear(Regs::CTRLB::txen));
                using PinConfig = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetTxPinConfig<Kvasir::Io::PinLocation<Port, Pin>> {
                using pad
                  = decltype(write(Regs::CTRLA::txpo,
                                   fieldValue<decltype(Regs::CTRLA::txpo),
                                              getTxPadValue(Io::PinLocation<Port, Pin>{})>()));
                using enable    = decltype(set(Regs::CTRLB::txen));
                using PinConfig = Sam::
                  PinFunctionOf<Sam::Package::Sercom, SercomInstance, Io::PinLocation<Port, Pin>>;
            };

            template<Hal::Uart::DataBits Dbits>
            struct GetDataBitConfig {
                static constexpr auto config_ = []() {
                    if constexpr(Dbits == Hal::Uart::DataBits::bits5) {
                        return write(Regs::CTRLB::CHSIZEValC::_5_bit);
                    } else if constexpr(Dbits == Hal::Uart::DataBits::bits6) {
                        return write(Regs::CTRLB::CHSIZEValC::_6_bit);
                    } else if constexpr(Dbits == Hal::Uart::DataBits::bits7) {
                        return write(Regs::CTRLB::CHSIZEValC::_7_bit);
                    } else if constexpr(Dbits == Hal::Uart::DataBits::bits8) {
                        return write(Regs::CTRLB::CHSIZEValC::_8_bit);
                    } else if constexpr(Dbits == Hal::Uart::DataBits::bits9) {
                        return write(Regs::CTRLB::CHSIZEValC::_9_bit);
                    }
                }();
                using config = decltype(config_);
            };

            template<Hal::Uart::StopBits Sbits>
            struct GetStopBitConfig {
                static constexpr auto config_ = []() {
                    if constexpr(Sbits == Hal::Uart::StopBits::stop1) {
                        return clear(Regs::CTRLB::sbmode);
                    } else if constexpr(Sbits == Hal::Uart::StopBits::stop2) {
                        return set(Regs::CTRLB::sbmode);
                    }
                }();
                using config = decltype(config_);
            };

            template<Hal::Uart::Parity Pa>
            struct GetParityConfig {
                static constexpr auto config_ = []() {
                    if constexpr(Pa == Hal::Uart::Parity::none) {
                        return brigand::list<decltype(write(
                                               Regs::CTRLA::form,
                                               fieldValue<decltype(Regs::CTRLA::form), 0>())),
                                             decltype(clear(Regs::CTRLB::pmode))>{};
                    } else if constexpr(Pa == Hal::Uart::Parity::odd) {
                        return brigand::list<decltype(write(
                                               Regs::CTRLA::form,
                                               fieldValue<decltype(Regs::CTRLA::form), 1>())),
                                             decltype(set(Regs::CTRLB::pmode))>{};
                    } else if constexpr(Pa == Hal::Uart::Parity::even) {
                        return brigand::list<decltype(write(
                                               Regs::CTRLA::form,
                                               fieldValue<decltype(Regs::CTRLA::form), 1>())),
                                             decltype(clear(Regs::CTRLB::pmode))>{};
                    }
                }();
                using config = decltype(config_);
            };
        };

    }   // namespace Detail

    template<typename UsartConfig_>
    struct UsartBase {
        struct UsartConfig : UsartConfig_ {
            static constexpr auto userConfigOverride
              = KVASIR_CONFIG_OR(UsartConfig_, userConfigOverride, brigand::list<>{});

            static constexpr auto maxBaudRateError
              = KVASIR_CONFIG_OR(UsartConfig_, maxBaudRateError, std::ratio<5, 1000>{});

            // loop-back through the pad: rxPinLocation is the txPinLocation, RXPO names its pad
            // (SercomPads.hpp receivePinFits)
            static constexpr bool loopBack = KVASIR_CONFIG_OR(UsartConfig_, loopBack, false);
        };

        // needed config: clockSpeed, baudRate, instance, txPinLocation, rxPinLocation (Io::NotUsed<>: send only),
        // dataBits, parity, stopBits; optional (above): userConfigOverride, maxBaudRateError, loopBack
        static constexpr auto instance = UsartConfig::instance;
        using Regs                     = Sam::Package::SercomUsart<instance>;

        // Startup: this SERCOM (one owner across I2C/SPI/USART - the three register views share
        // the base address) and the core clock BAUD is computed from (Clocks.hpp: the
        // ClockSettings must route a generator at clockSpeed to GCLK_SERCOMn_CORE and provide it).
        using Provides = brigand::list<Hal::Owns<Regs>>;
        using Claims   = Hal::Claim<Clocks::ClkSercomCore<instance>, UsartConfig::clockSpeed>;

        using InterruptIndexs = decltype(Sam::Block<Regs>::irqs());

        using Config = Detail::Config<instance>;

        static constexpr auto rxDmaTrigger = Sam::Block<Regs>::dmaRx();
        static constexpr auto txDmaTrigger = Sam::Block<Regs>::dmaTx();

        // the achieved baud rate against maxBaudRateError; a failure prints wanted, got and ppm
        static constexpr bool baudInTolerance = [] {
            Prescaler::assertInTolerance<
              Detail::baudDivider(UsartConfig::clockSpeed, UsartConfig::baudRate).achieved,
              UsartConfig::baudRate,
              Prescaler::Tolerance{UsartConfig::maxBaudRateError},
              "USART baud rate">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(baudInTolerance);
        static_assert(Config::isValidPinLocationTX(UsartConfig::txPinLocation),
                      "invalid TXPin");
        static_assert(Config::isValidPinLocationRX(UsartConfig::rxPinLocation),
                      "invalid RXPin");
        static_assert(
          Sam::Pads::checkReceivePin<UsartConfig::loopBack,
                                     std::remove_cvref_t<decltype(UsartConfig::rxPinLocation)>,
                                     std::remove_cvref_t<decltype(UsartConfig::txPinLocation)>,
                                     "USART rxPinLocation/txPinLocation">());

        static constexpr auto powerClockEnable
          = list(typename PM::Enable<Regs::baseAddr>::action{});

        static constexpr auto initStepPinConfig = list(
          typename Config::template GetTxPinConfig<
            std::decay_t<decltype(UsartConfig::txPinLocation)>>::PinConfig{},
          std::conditional_t<UsartConfig::loopBack,   // loop-back: the TX pin, configured once
                             brigand::list<>,
                             typename Config::template GetRxPinConfig<
                               std::decay_t<decltype(UsartConfig::rxPinLocation)>>::PinConfig>{});

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
            decltype(UsartConfig::rxPinLocation)>>::template InterruptEnable<InterruptIndexs>{});

        // CTRLA.ENABLE is write-synchronized and SYNCBUSY.ENABLE clears when the enable is done (D21 DS40001882L
        // 26.6.6, 14.3.2.5); what a DATA write during that time does is not stated, so nothing is sent before
        // it. As Sercom_SPIQueued does.
        static void runtimeInit() {
            Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound>(
              Kvasir::Register::isClear(Regs::SYNCBUSY::enable));
        }
    };

    // A transmit that never finishes - the SERCOM or the DMAC reset or disabled under it, a descriptor lost - is
    // given up after twice its time on the wire plus 1 ms (txTimeout(), on TimeClock): the channel stopped, the bytes
    // the caller's again, the SERCOM set up again if it was reset (CTRLA.SWRST "resets all registers ... and the
    // peripheral is disabled", D21 DS40001882L 26.6.2.2, md l.19309), the DMAC set up again if it is off;
    // operationState() then says `failed` until the next send(), and the next send() starts afresh. txTimeouts()
    // counts them. Nothing waits for a person or a reset.
    template<typename UartConfig,
             typename TimeClock,
             typename Dma,
             DMAC::DMAChannel  DmaChannel,
             DMAC::DMAPriority DmaPriority,
             std::size_t       BufferSize>
    struct UartBehaviorImpl : Kvasir::Sercom::Usart::UsartBase<UartConfig> {
        using base = Kvasir::Sercom::Usart::UsartBase<UartConfig>;
        using Regs = typename base::Regs;

        static_assert(Dma::numberOfChannels > std::size_t(DmaChannel));

        using Claims = brigand::append<typename base::Claims, DMAC::Claims<Dma, DmaChannel>>;

        enum class OperationState { succeeded, failed, ongoing };

        /// One character on the wire: the start bit, the data bits, the parity bit, the stop bits.
        static constexpr std::uint32_t bitsPerCharacter
          = 1U + 5U + static_cast<std::uint32_t>(UartConfig::dataBits)
          + (UartConfig::parity == Hal::Uart::Parity::none ? 0U : 1U)
          + (UartConfig::stopBits == Hal::Uart::StopBits::stop2 ? 2U : 1U);

        /// How long a transmit of `bytes` may take before it counts as stuck: twice its time on the wire, plus 1 ms
        /// for the DMAC's start and the clock's tick.
        static constexpr std::chrono::microseconds txTimeout(std::size_t bytes) {
            return std::chrono::microseconds{2ULL * bytes * bitsPerCharacter * 1'000'000ULL
                                               / static_cast<std::uint64_t>(UartConfig::baudRate)
                                             + 1'000ULL};
        }

        static OperationState operationState() {
            if(state_ != OperationState::ongoing) { return state_; }
            if(!Dma::template wd<DmaChannel>().isValid() && apply(read(Regs::INTFLAG::txc))) {
                // the bytes are the caller's again (Kvasir/Hal/DmaHandoff.hpp)
                Hal::takeFromHardware();
                state_ = OperationState::succeeded;
                return state_;
            }
            if(TimeClock::now() < deadline_) { return OperationState::ongoing; }
            abandon();
            return state_;
        }

        /// Transmits given up since boot (header comment above).
        [[nodiscard]] static std::uint32_t txTimeouts() { return txTimeouts_; }

    private:
        inline static OperationState                 state_{OperationState::succeeded};
        inline static typename TimeClock::time_point deadline_{};
        inline static std::uint32_t volatile txTimeouts_{};

        static void abandon() {
            static_cast<void>(
              Dma::template stopAndWait<DmaChannel>());   // takes the bytes back itself
            // the DMAC reset or disabled under it: its descriptor bases and enable again (DmaBase::runtimeInit)
            if(apply(read(Dma::Regs::CTRL::dmaenable)) == 0) { Dma::runtimeInit(); }
            // the SERCOM reset or disabled under it: Startup's steps again, after a reset still running (SYNCBUSY.SWRST)
            if(apply(read(Regs::CTRLA::enable)) == 0) {
                static_cast<void>(
                  Register::waitUntil<Kvasir::Chip::Sam::SyncBound, Register::OnTimeout::Report>(
                    Register::isClear(Regs::SYNCBUSY::swrst)));
                apply(base::initStepPeripheryConfig);
                apply(base::initStepPeripheryEnable);
                base::runtimeInit();
            }
            txTimeouts_ = txTimeouts_ + 1;
            state_      = OperationState::failed;
        }

        static void startDma(std::size_t bytes) {
            deadline_ = TimeClock::now()
                      + std::chrono::ceil<typename TimeClock::duration>(txTimeout(bytes)) +
                        typename TimeClock::duration{1};   // the tick in progress
            // the descriptor and the bytes are the DMAC's from here
            Hal::handToHardware();
            apply(Dma::template start<DmaChannel, DmaPriority, base::txDmaTrigger>());
            state_ = OperationState::ongoing;
        }

    public:
        template<typename C>
        static void sendNocopy(C const& c) {
            sendNocopy(c.begin(), c.end());
        }

        template<typename InputIt>
        static void sendNocopy(InputIt first,
                               InputIt last) {
            static_assert(sizeof(*first) == 1, "only bytes");
            Dma::template rd<DmaChannel>() = DMAC::DmacDescriptor{
              {.stepSel = DMAC::DmacDescriptor::Stepsel::src,
               .srcInc  = DMAC::DmacDescriptor::Srcinc::increment,
               .count   = std::uint16_t(std::distance(first, last)),
               // to_address: a container's iterator need not be a pointer
               .src = reinterpret_cast<std::uint32_t>(std::to_address(last)),
               .dst = Regs::DATA8::Addr::value}
            };
            startDma(static_cast<std::size_t>(std::distance(first, last)));
        }

        /// Hal::Uart::ByteSource (kvasir/Hal/Uart.hpp): the next received byte, if one is
        /// waiting; an empty optional inside `out` is a frame error the interrupt recorded.
        static bool receive(std::optional<std::byte>& out) { return rxbuffer_.popInto(out); }

        /// Hal::Uart::ByteSink: all of `s` while nothing is on the way, else nothing (an empty `s`
        /// starts no transfer). No copy: the bytes stay the caller's until txIdle().
        static std::size_t send(std::span<std::byte const> s) {
            if(s.empty() || operationState() == OperationState::ongoing) { return 0; }
            sendNocopy(s.begin(), s.end());
            return s.size();
        }

        /// The DMA's descriptor is done and INTFLAG.TXC says the whole frame, stop bits included,
        /// has left the shift register with nothing new in DATA (SAM D21 DS40001882 26.6.2.5).
        [[nodiscard]] static bool txIdle() { return operationState() != OperationState::ongoing; }

    protected:
        // BufferSize 0 = a send-only UART: no receive queue (a Queue needs at least one slot), receive() finds nothing
        struct NoRxBuffer {
            static bool popInto(std::optional<std::byte>&) { return false; }

            static void push(std::optional<std::byte>) {}
        };

        // filled by the receive interrupt (UartBehavior::onIsr)
        inline static std::conditional_t<
          BufferSize == 0,
          NoRxBuffer,
          Kvasir::Atomic::Queue<std::optional<std::byte>, BufferSize>>
          rxbuffer_{};
    };

    /// A SERCOM USART: receive interrupt into a queue of BufferSize (0: send only), transmit by DMA on DmaChannel,
    /// given up after its timeout on TimeClock (Clock::now(); the queued I2C master's Clock) - UartBehaviorImpl.
    /// Its receive half is there when the config names an rxPinLocation (not Io::NotUsed<>): the receive interrupt
    /// (Isr, an empty list for a send-only UART - Startup installs nothing then) and spuriousInterrupts().
    template<typename UartConfig,
             typename TimeClock,
             typename Dma,
             DMAC::DMAChannel  DmaChannel,
             DMAC::DMAPriority DmaPriority,
             std::size_t       BufferSize>
    struct UartBehavior
      : UartBehaviorImpl<UartConfig, TimeClock, Dma, DmaChannel, DmaPriority, BufferSize> {
        using base
          = UartBehaviorImpl<UartConfig, TimeClock, Dma, DmaChannel, DmaPriority, BufferSize>;
        using Regs = typename base::Regs;

        static constexpr bool receives
          = !std::is_same_v<std::remove_cvref_t<decltype(UartConfig::rxPinLocation)>,
                            Kvasir::Io::NotUsed<>>;

        // The receive interrupt. Not constrained like spuriousInterrupts(): Isr below names its address in both arms
        // of the choice, and an address of a function whose constraint fails is an error - a send-only UART simply
        // never installs it.
        static void onIsr() {
            auto const intflag = apply(read(Regs::INTFLAG::error, Regs::INTFLAG::rxc));

            if(intflag.template get<0>()) {
                apply(set(Regs::INTFLAG::error));
                base::rxbuffer_.push(std::nullopt);
            } else if(intflag.template get<1>()) {
                std::byte data = std::byte(apply(read(Regs::DATA8::data)).template get<0>());
                base::rxbuffer_.push(data);
            } else {
                // Neither ERROR nor RXC: an entry with nothing to do (a spurious or late one). Only these two are
                // enabled, and reading DATA clears RXC (SAM D21 DS 26.8.7, INTFLAG bit 2): nothing to clear. Class E
                // (Kvasir_SDK Runtime/Check.hpp): counted, never a panic.
                spurious_ = spurious_ + 1U;
            }
        }

        /// Receive interrupts that found neither ERROR nor RXC set (onIsr's "nothing to do").
        [[nodiscard]] static std::uint32_t spuriousInterrupts()
            requires receives
        {
            return spurious_;
        }

        using Isr
          = std::conditional_t<receives,
                               Nvic::IsrsFor<std::addressof(onIsr), typename base::InterruptIndexs>,
                               brigand::list<>>;

    private:
        // written by onIsr only, read anywhere: one word, volatile for the reader outside the interrupt
        inline static std::uint32_t volatile spurious_{};
    };

}}}   // namespace Kvasir::Sercom::Usart
