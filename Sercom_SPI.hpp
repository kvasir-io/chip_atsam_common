#pragma once

#include "chip/Io.hpp"
#include "chip/PM.hpp"
#include "chip/Sercom_Traits.hpp"
#include "chip/atsam_common/DMAC.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/literals.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/SERCOM_SPI.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <ratio>

namespace Kvasir { namespace Sercom { namespace SPI {
    template<typename = void>
    struct NotUsed {};

    enum class Mode {
        _0,   //CPOL0_CPHA0
        _1,   //CPOL0_CPHA1
        _2,   //CPOL1_CPHA0
        _3    //CPOL1_CPHA1
    };

    namespace Detail {

        template<unsigned SercomInstance>
        struct Config {
            using Regs = Kvasir::Peripheral::SERCOM_SPI::Registers<SercomInstance>;

            static constexpr bool isValidPinLocationMISO(NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationMISO(Kvasir::Register::PinLocation<Port,
                                                                                       Pin>) {
                return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(0, 1, 2, 3);
            }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationMOSI(Kvasir::Register::PinLocation<Port,
                                                                                       Pin>) {
                return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(0, 2, 3);
            }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationSCLK(Kvasir::Register::PinLocation<Port,
                                                                                       Pin>) {
                return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(1, 3);
            }

            static constexpr bool isValidPinLocationCS(NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationCS(Kvasir::Register::PinLocation<Port,
                                                                                     Pin>) {
                return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(1, 2);
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

            static constexpr bool PinLocationAreTheSame(NotUsed<>,
                                                        NotUsed<>) {
                return false;
            }

            template<typename MISOPIN>
            struct GetMISOPinConfig;

            template<typename dummy>
            struct GetMISOPinConfig<NotUsed<dummy>> {
                using enable    = decltype(clear(Regs::CTRLB::rxen));
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetMISOPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using enable    = decltype(set(Regs::CTRLB::rxen));
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<
                    SercomInstance>(Register::PinLocation<Port, Pin>{})>{},
                  Register::PinLocation<Port, Pin>{}));
            };
            template<typename CSPIN>
            struct GetCSPinConfig;

            template<typename dummy>
            struct GetCSPinConfig<NotUsed<dummy>> {
                using enable    = decltype(clear(Regs::CTRLB::mssen));
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetCSPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using enable    = decltype(set(Regs::CTRLB::mssen));
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<
                    SercomInstance>(Register::PinLocation<Port, Pin>{})>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<typename MOSIPIN>
            struct GetMOSIPinConfig;

            template<int Port, int Pin>
            struct GetMOSIPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using enable    = brigand::list<>;
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<
                    SercomInstance>(Register::PinLocation<Port, Pin>{})>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<typename SCLKPIN>
            struct GetSCLKPinConfig;

            template<int Port, int Pin>
            struct GetSCLKPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using enable    = brigand::list<>;
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<
                    SercomInstance>(Register::PinLocation<Port, Pin>{})>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<typename MISOPIN>
            static constexpr auto getDIPO() {
                if constexpr(std::is_same_v<MISOPIN, NotUsed<>>) {
                    return brigand::list<>{};
                } else {
                    return write(Regs::CTRLA::dipo,
                                 Kvasir::Register::value<static_cast<std::uint32_t>(
                                   Traits::SercomTraits::GetPOVal<SercomInstance>(MISOPIN{}))>());
                }
            }

            template<typename T>
            struct Fail {
                static_assert(std::is_void_v<T>,
                              "invalid Pin config");
                static_assert(!std::is_void_v<T>,
                              "invalid Pin config");
            };

            template<typename MOSIPIN,
                     typename SCLKPIN,
                     typename CSPIN>
            static constexpr auto getDOPO() {
                constexpr auto MOSIPAD = Traits::SercomTraits::GetPOVal<SercomInstance>(MOSIPIN{});
                constexpr auto SCLKPAD = Traits::SercomTraits::GetPOVal<SercomInstance>(SCLKPIN{});

                constexpr auto DOPO = []() {
                    if constexpr(MOSIPAD == 0) {
                        if constexpr(SCLKPAD == 1) {
                            return 0;
                        } else if constexpr(SCLKPAD == 3) {
                            return 3;
                        } else {
                            Fail<void>{};
                        }

                    } else if constexpr(MOSIPAD == 2) {
                        static_assert(SCLKPAD == 3);
                        return 1;
                    } else if constexpr(MOSIPAD == 3) {
                        static_assert(SCLKPAD == 1);
                        return 2;
                    } else {
                        Fail<void>{};
                    }
                }();

                if constexpr(!std::is_same_v<CSPIN, NotUsed<>>) {
                    constexpr auto CSPAD = Traits::SercomTraits::GetPOVal<SercomInstance>(CSPIN{});
                    if constexpr(DOPO == 0 || DOPO == 2) {
                        static_assert(CSPAD == 2);
                    } else {
                        static_assert(CSPAD == 1);
                    }
                }
                return write(Regs::CTRLA::dopo, Kvasir::Register::value<DOPO>());
            }

            template<typename MISOPIN, typename MOSIPIN, typename SCLKPIN, typename CSPIN>
            struct GetPadValueConfig {
                static constexpr auto config_
                  = []() { return list(getDIPO<MISOPIN>(), getDOPO<MOSIPIN, SCLKPIN, CSPIN>()); }();
                using config = decltype(config_);
            };

            template<Mode mode>
            struct GetModeConfig {
                static constexpr auto config_ = []() {
                    if constexpr(mode == Mode::_0) {
                        return brigand::list<
                          decltype(write(Regs::CTRLA::cpol, Kvasir::Register::value<0>())),
                          decltype(write(Regs::CTRLA::cpha, Kvasir::Register::value<0>()))>{};
                    } else if constexpr(mode == Mode::_1) {
                        return brigand::list<
                          decltype(write(Regs::CTRLA::cpol, Kvasir::Register::value<0>())),
                          decltype(write(Regs::CTRLA::cpha, Kvasir::Register::value<1>()))>{};
                    } else if constexpr(mode == Mode::_2) {
                        return brigand::list<
                          decltype(write(Regs::CTRLA::cpol, Kvasir::Register::value<1>())),
                          decltype(write(Regs::CTRLA::cpha, Kvasir::Register::value<0>()))>{};
                    } else if constexpr(mode == Mode::_3) {
                        return brigand::list<
                          decltype(write(Regs::CTRLA::cpol, Kvasir::Register::value<1>())),
                          decltype(write(Regs::CTRLA::cpha, Kvasir::Register::value<1>()))>{};
                    }
                }();
                using config = decltype(config_);
            };
        };

        constexpr std::uint32_t calcBaudReg(std::uint32_t f_clockSpeed,
                                            std::uint32_t f_baud) {
            auto baudReg = std::int64_t((double(f_clockSpeed) / (2.0 * double(f_baud))) - 1.0);
            return static_cast<std::uint32_t>(std::clamp<std::int64_t>(baudReg, 0, 256));
        }

        constexpr double calcf_Baud(std::uint32_t f_clockSpeed,
                                    std::uint32_t baudReg) {
            return (double(f_clockSpeed) / (2.0 * (double(baudReg) + 1.0)));
        }

        template<std::uint32_t f_clockSpeed,
                 std::uint32_t f_baud,
                 std::intmax_t Num,
                 std::intmax_t Denom>
        constexpr bool isValidBaudConfig(std::ratio<Num,
                                                    Denom>) {
            constexpr auto baudReg      = calcBaudReg(f_clockSpeed, f_baud);
            constexpr auto f_baudCalced = calcf_Baud(f_clockSpeed, baudReg);
            constexpr auto err          = f_baudCalced - double(f_baud);
            constexpr auto absErr       = err > 0.0 ? err : -err;
            constexpr auto ret = absErr <= (double(f_baud) * (double(Num) / (double(Denom))));
            return ret;
        }

    }   // namespace Detail

    template<typename SPIConfig_>
    struct SPIBase {
        struct SPIConfig : SPIConfig_ {
            static constexpr auto userConfigOverride = [] {
                if constexpr(requires { SPIConfig_::userConfigOverride; }) {
                    return SPIConfig_::userConfigOverride;
                } else {
                    return brigand::list<>{};
                }
            }();

            static constexpr auto maxBaudRateError = [] {
                if constexpr(requires { SPIConfig_::maxBaudRateError; }) {
                    return SPIConfig_::maxBaudRateError;
                } else {
                    return std::ratio<1, 100>{};
                }
            }();
        };

        // needed config
        // clockSpeed
        // baudRate
        // instance
        // mode
        // misoPinLocation
        // mosiPinLocation
        // sclkPinLocation
        // csPinLocation
        // userConfigOverride
        static constexpr auto Instance = SPIConfig::instance;
        using Regs                     = Kvasir::Peripheral::SERCOM_SPI::Registers<Instance>;

        using InterruptIndex = decltype(Traits::SercomTraits::getSercomIsrIndexs<Instance>());

        using Config = Detail::Config<Instance>;

        static constexpr auto RxDmaTrigger = Traits::SercomTraits::DmaRX_Trigger<Instance>();
        static constexpr auto TxDmaTrigger = Traits::SercomTraits::DmaTX_Trigger<Instance>();

        static_assert(Detail::isValidBaudConfig<SPIConfig::clockSpeed,
                                                SPIConfig::baudRate>(SPIConfig::maxBaudRateError),
                      "invalid baud configuration baudRate error to big");
        static_assert(Config::isValidPinLocationMISO(SPIConfig::misoPinLocation),
                      "invalid MISOPin");
        static_assert(Config::isValidPinLocationMOSI(SPIConfig::mosiPinLocation),
                      "invalid MOSIPin");
        static_assert(Config::isValidPinLocationSCLK(SPIConfig::sclkPinLocation),
                      "invalid SCLKPin");
        static_assert(Config::isValidPinLocationCS(SPIConfig::csPinLocation),
                      "invalid CSPin");
        static_assert(!Config::PinLocationAreTheSame(SPIConfig::misoPinLocation,
                                                     SPIConfig::mosiPinLocation),
                      "MISO and MOSI are the same pin");
        static_assert(!Config::PinLocationAreTheSame(SPIConfig::misoPinLocation,
                                                     SPIConfig::sclkPinLocation),
                      "MISO and SCLK are the same pin");
        static_assert(!Config::PinLocationAreTheSame(SPIConfig::misoPinLocation,
                                                     SPIConfig::csPinLocation),
                      "MISO and CS are the same pin");

        static constexpr auto powerClockEnable
          = list(typename PM::enable<Regs::baseAddr>::action{});

        static constexpr auto initStepPinConfig
          = list(typename Config::template GetMISOPinConfig<
                   std::decay_t<decltype(SPIConfig::misoPinLocation)>>::pinConfig{},
                 typename Config::template GetMOSIPinConfig<
                   std::decay_t<decltype(SPIConfig::mosiPinLocation)>>::pinConfig{},
                 typename Config::template GetSCLKPinConfig<
                   std::decay_t<decltype(SPIConfig::sclkPinLocation)>>::pinConfig{},
                 typename Config::template GetCSPinConfig<
                   std::decay_t<decltype(SPIConfig::csPinLocation)>>::pinConfig{});

        static constexpr auto initStepPeripheryConfig = list(
          typename Config::template GetMISOPinConfig<
            std::decay_t<decltype(SPIConfig::misoPinLocation)>>::enable{},
          typename Config::template GetMOSIPinConfig<
            std::decay_t<decltype(SPIConfig::mosiPinLocation)>>::enable{},
          typename Config::template GetSCLKPinConfig<
            std::decay_t<decltype(SPIConfig::sclkPinLocation)>>::enable{},
          typename Config::template GetCSPinConfig<
            std::decay_t<decltype(SPIConfig::csPinLocation)>>::enable{},

          write(Regs::BAUD::baud,
                Register::value<std::uint8_t,
                                Detail::calcBaudReg(SPIConfig::clockSpeed, SPIConfig::baudRate)>()),

          typename Config::template GetModeConfig<SPIConfig::mode>::config{},

          typename Config::template GetPadValueConfig<
            std::decay_t<decltype(SPIConfig::misoPinLocation)>,
            std::decay_t<decltype(SPIConfig::mosiPinLocation)>,
            std::decay_t<decltype(SPIConfig::sclkPinLocation)>,
            std::decay_t<decltype(SPIConfig::csPinLocation)>>::config{},
          clear(Regs::CTRLA::enable),
          clear(Regs::CTRLA::swrst),

          // The following parameters are only supported via userConfigOverride
          write(Regs::CTRLA::MODEValC::spi_master),
          clear(Regs::CTRLA::runstdby),
          write(Regs::CTRLA::FORMValC::spi_frame),
          clear(Regs::CTRLA::ibon),
          write(Regs::CTRLA::DORDValC::msb),
          write(Regs::CTRLB::AMODEValC::mask),
          clear(Regs::CTRLB::ssde),
          clear(Regs::CTRLB::ploaden),
          write(Regs::CTRLB::CHSIZEValC::_8_bit),

          SPIConfig::userConfigOverride);

        /*   static constexpr auto initStepInterruptConfig = list(
          action(Kvasir::Nvic::Action::SetPriority<SPIConfig::isrPriority>{}, InterruptIndex{}),
          action(Kvasir::Nvic::Action::clearPending, InterruptIndex{}));*/

        static constexpr auto initStepPeripheryEnable = list(set(Regs::CTRLA::enable));
    };

    template<typename SPIConfig,
             typename Dma,
             DMAC::DMAChannel  DmaChannelA,
             DMAC::DMAChannel  DmaChannelB,
             DMAC::DMAPriority DmaPriority>
    struct SPIBehavior : SPIBase<SPIConfig> {
        using base    = SPIBase<SPIConfig>;
        using Regs    = typename base::Regs;
        using DmaRegs = typename Dma::Regs;

        enum class OperationState { succeeded, failed, ongoing };

        inline static bool a = false;
        inline static bool b = false;

        static OperationState operationState() {
            if(!a && !b) { return OperationState::succeeded; }

            if(!Dma::template wd<DmaChannelA>().isValid() && apply(read(Regs::INTFLAG::txc))) {
                a = false;
            }
            if(!Dma::template wd<DmaChannelB>().isValid()) { b = false; }

            if(!a && !b) { return OperationState::succeeded; }
            // TODO timeout
            return OperationState::ongoing;
        }

        // The bookkeeping a shared-bus device driver (kvasir_devices SPIDeviceBase) needs, with
        // the RP2040 / RP2350 SPI's names. This behaviour has no completion interrupt: a
        // transfer is over when operationState() says so, and SPIDeviceBase polls it.
        inline static std::atomic<bool> booked{false};

        /// Book the bus for one frame; false while another device has it.
        static bool acquire() { return !booked.exchange(true, std::memory_order_acquire); }

        static void release() { booked.store(false, std::memory_order_release); }

        /// A transfer is still moving bytes: operationState() asked the other way round.
        [[nodiscard]] static bool transferInProgress() {
            return operationState() == OperationState::ongoing;
        }

        /// Nothing is latched here to acknowledge: a transfer completes or is aborted.
        static void clearError() {}

        /// Abandon an in-flight transfer: stop both DMA channels, forget them, and drop what
        /// the receiver holds so it is not read as the start of the next transfer.
        static void abortTransfer() {
            apply(Dma::template stop<DmaChannelA>());
            apply(Dma::template stop<DmaChannelB>());
            a = false;
            b = false;
            for(int i = 0; i < 4 && apply(read(Regs::INTFLAG::rxc)); ++i) {
                apply(read(Regs::DATA8::data));
            }
        }

        template<DMAC::DMAChannel Channel,
                 bool             isRx>
        static void startDma() {
            apply(Dma::template start<Channel,
                                      DmaPriority,
                                      isRx ? base::RxDmaTrigger : base::TxDmaTrigger>());
        }

        template<typename C>
        static void send_nocopy(C& c) {
            send_nocopy(c.begin(), c.end());
        }

        template<typename InputIt>
        static void send_nocopy(InputIt first,
                                InputIt last) {
            static_assert(sizeof(*first) == 1, "only bytes");
            send_nocopy_impl(last, static_cast<std::size_t>(std::distance(first, last)), true);
        }

        template<typename T>
        static void send_nocopy_static(T const*    v,
                                       std::size_t size) {
            static_assert(sizeof(*v) == 1, "only bytes");
            send_nocopy_impl(v, size, false);
        }

        template<typename InputIt>
        static void send_nocopy_impl(InputIt     last,
                                     std::size_t size,
                                     bool        increment) {
            Dma::template rd<DmaChannelA>()
              = DMAC::DmacDescriptor(true,
                                     DMAC::DmacDescriptor::stepsize::x1,
                                     DMAC::DmacDescriptor::stepsel::dst,
                                     DMAC::DmacDescriptor::dstinc::no_increment,
                                     increment ? DMAC::DmacDescriptor::srcinc::increment
                                               : DMAC::DmacDescriptor::srcinc::no_increment,
                                     DMAC::DmacDescriptor::beatsize::byte,
                                     DMAC::DmacDescriptor::blockact::noact,
                                     DMAC::DmacDescriptor::evosel::disabled,
                                     std::uint16_t(size),
                                     std::uint32_t(std::addressof(*last)),
                                     Regs::DATA8::Addr::value);
            // set timeout
            startDma<DmaChannelA, false>();
            a = true;
            b = false;
        }

        template<typename C>
        static void send_receive_nocopy(C& c) {
            send_receive_nocopy(c.begin(), c.end(), c.begin(), c.end());
        }

        template<typename C1,
                 typename C2>
        static void send_receive_nocopy(C1 const& c1,
                                        C2&       c2) {
            send_receive_nocopy(c1.begin(), c1.end(), c2.begin(), c2.end());
        }

        template<typename InputOutputIt>
        static void send_receive_nocopy(InputOutputIt first,
                                        InputOutputIt last) {
            send_receive_nocopy(first, last, first, last);
        }

        template<typename InputIt,
                 typename OutputIt>
        static void send_receive_nocopy(InputIt  first,
                                        InputIt  last,
                                        OutputIt firstOut,
                                        OutputIt lastOut) {
            static_assert(sizeof(*first) == 1, "only bytes");
            static_assert(sizeof(*firstOut) == 1, "only bytes");
            auto const inSize  = std::uint16_t(std::distance(first, last));
            auto const outSize = std::uint16_t(std::distance(firstOut, lastOut));
            assert(inSize == outSize);

            while(apply(read(Regs::INTFLAG::rxc))) { apply(read(Regs::DATA8::data)); }
            apply(set(Regs::STATUS::bufovf));

            Dma::template rd<DmaChannelA>()
              = DMAC::DmacDescriptor(true,
                                     DMAC::DmacDescriptor::stepsize::x1,
                                     DMAC::DmacDescriptor::stepsel::src,
                                     DMAC::DmacDescriptor::dstinc::no_increment,
                                     DMAC::DmacDescriptor::srcinc::increment,
                                     DMAC::DmacDescriptor::beatsize::byte,
                                     DMAC::DmacDescriptor::blockact::noact,
                                     DMAC::DmacDescriptor::evosel::disabled,
                                     inSize,
                                     std::uint32_t(std::addressof(*last)),
                                     Regs::DATA8::Addr::value);
            Dma::template rd<DmaChannelB>()
              = DMAC::DmacDescriptor(true,
                                     DMAC::DmacDescriptor::stepsize::x1,
                                     DMAC::DmacDescriptor::stepsel::dst,
                                     DMAC::DmacDescriptor::dstinc::increment,
                                     DMAC::DmacDescriptor::srcinc::no_increment,
                                     DMAC::DmacDescriptor::beatsize::byte,
                                     DMAC::DmacDescriptor::blockact::noact,
                                     DMAC::DmacDescriptor::evosel::disabled,
                                     outSize,
                                     Regs::DATA8::Addr::value,
                                     std::uint32_t(std::addressof(*lastOut)));
            // set timeout
            startDma<DmaChannelA, false>();
            startDma<DmaChannelB, true>();
            a = true;
            b = true;
        }
    };

}}}   // namespace Kvasir::Sercom::SPI
