#pragma once

#include "chip/PM.hpp"
#include "chip/Package.hpp"
#include "chip/atsam_common/Clocks.hpp"
#include "chip/atsam_common/Io.hpp"
#include "chip/atsam_common/SercomBaud.hpp"
#include "kvasir/Hal/Clocks.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Hal/Spi.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Prescaler.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <ratio>

namespace Kvasir { namespace Sercom { namespace SPI {
    namespace Detail {

        template<unsigned SercomInstance>
        struct Config {
            using Regs = Sam::Package::SercomSpi<SercomInstance>;

            static constexpr bool isValidPinLocationMISO(Io::NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationMISO(Kvasir::Io::PinLocation<Port,
                                                                                 Pin>) {
                return Sam::Package::Sercom::validIfPOVal<SercomInstance, Port, Pin>(0, 1, 2, 3);
            }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationMOSI(Kvasir::Io::PinLocation<Port,
                                                                                 Pin>) {
                return Sam::Package::Sercom::validIfPOVal<SercomInstance, Port, Pin>(0, 2, 3);
            }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationSCLK(Kvasir::Io::PinLocation<Port,
                                                                                 Pin>) {
                return Sam::Package::Sercom::validIfPOVal<SercomInstance, Port, Pin>(1, 3);
            }

            static constexpr bool isValidPinLocationCS(Io::NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationCS(Kvasir::Io::PinLocation<Port,
                                                                               Pin>) {
                return Sam::Package::Sercom::validIfPOVal<SercomInstance, Port, Pin>(1, 2);
            }

            // the PORT function of a pin (PinMux.hpp), nothing for Io::NotUsed<>
            template<typename Location>
            using PinConfig = Sam::PinFunctionOf<Sam::Package::Sercom, SercomInstance, Location>;

            // MISO and CS may be Io::NotUsed<>: the receiver / the hardware chip select off
            template<typename MISOPIN>
            using MisoEnable = std::conditional_t<std::is_same_v<MISOPIN, Io::NotUsed<>>,
                                                  decltype(clear(Regs::CTRLB::rxen)),
                                                  decltype(set(Regs::CTRLB::rxen))>;

            template<typename CSPIN>
            using CsEnable = std::conditional_t<std::is_same_v<CSPIN, Io::NotUsed<>>,
                                                decltype(clear(Regs::CTRLB::mssen)),
                                                decltype(set(Regs::CTRLB::mssen))>;

            template<typename MISOPIN>
            static constexpr auto getDIPO() {
                if constexpr(std::is_same_v<MISOPIN, Io::NotUsed<>>) {
                    return brigand::list<>{};
                } else {
                    return write(
                      Regs::CTRLA::dipo,
                      fieldValue<decltype(Regs::CTRLA::dipo),
                                 Sam::Package::Sercom::getPOVal<SercomInstance>(MISOPIN{})>());
                }
            }

            // DOPO from the chip's pad table (SercomPads.hpp): a MOSI/SCK pair it does not offer, or a hardware SS
            // on another pad than the row's, is a compile error naming the pads and the chip
            template<typename MOSIPIN,
                     typename SCLKPIN,
                     typename CSPIN>
            static constexpr auto getDOPO() {
                constexpr auto row
                  = Sam::Pads::dopo<Sam::Package::SercomPads,
                                    Sam::Package::Sercom::getPOVal<SercomInstance>(MOSIPIN{}),
                                    Sam::Package::Sercom::getPOVal<SercomInstance>(SCLKPIN{})>();
                if constexpr(!std::is_same_v<CSPIN, Io::NotUsed<>>) {
                    Sam::Pads::checkSs<row,
                                       Sam::Package::Sercom::getPOVal<SercomInstance>(CSPIN{})>();
                }
                return write(Regs::CTRLA::dopo,
                             fieldValue<decltype(Regs::CTRLA::dopo), row.dopo>());
            }

            template<typename MISOPIN, typename MOSIPIN, typename SCLKPIN, typename CSPIN>
            struct GetPadValueConfig {
                static constexpr auto config_
                  = []() { return list(getDIPO<MISOPIN>(), getDOPO<MOSIPIN, SCLKPIN, CSPIN>()); }();
                using config = decltype(config_);
            };

            template<Hal::ClockMode Mode>
            struct GetModeConfig {
                static constexpr auto config_ = []() {
                    if constexpr(Mode == Hal::ClockMode::mode0) {
                        return brigand::list<
                          decltype(write(Regs::CTRLA::cpol,
                                         fieldValue<decltype(Regs::CTRLA::cpol), 0>())),
                          decltype(write(Regs::CTRLA::cpha,
                                         fieldValue<decltype(Regs::CTRLA::cpha), 0>()))>{};
                    } else if constexpr(Mode == Hal::ClockMode::mode1) {
                        return brigand::list<
                          decltype(write(Regs::CTRLA::cpol,
                                         fieldValue<decltype(Regs::CTRLA::cpol), 0>())),
                          decltype(write(Regs::CTRLA::cpha,
                                         fieldValue<decltype(Regs::CTRLA::cpha), 1>()))>{};
                    } else if constexpr(Mode == Hal::ClockMode::mode2) {
                        return brigand::list<
                          decltype(write(Regs::CTRLA::cpol,
                                         fieldValue<decltype(Regs::CTRLA::cpol), 1>())),
                          decltype(write(Regs::CTRLA::cpha,
                                         fieldValue<decltype(Regs::CTRLA::cpha), 0>()))>{};
                    } else if constexpr(Mode == Hal::ClockMode::mode3) {
                        return brigand::list<
                          decltype(write(Regs::CTRLA::cpol,
                                         fieldValue<decltype(Regs::CTRLA::cpol), 1>())),
                          decltype(write(Regs::CTRLA::cpha,
                                         fieldValue<decltype(Regs::CTRLA::cpha), 1>()))>{};
                    }
                }();
                using config = decltype(config_);
            };
        };

    }   // namespace Detail

    template<typename SPIConfig_>
    struct SPIBase {
        struct SPIConfig : SPIConfig_ {
            static constexpr auto userConfigOverride
              = KVASIR_CONFIG_OR(SPIConfig_, userConfigOverride, brigand::list<>{});

            static constexpr auto maxBaudRateError
              = KVASIR_CONFIG_OR(SPIConfig_, maxBaudRateError, std::ratio<1, 100>{});

            // loop-back through the pad: misoPinLocation is the mosiPinLocation, DIPO names its pad
            // (SercomPads.hpp receivePinFits)
            static constexpr bool loopBack = KVASIR_CONFIG_OR(SPIConfig_, loopBack, false);
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
        static constexpr auto instance = SPIConfig::instance;
        using Regs                     = Sam::Package::SercomSpi<instance>;

        // Startup: this SERCOM (one owner across I2C/SPI/USART - the three register views share
        // the base address) and the core clock BAUD is computed from (Clocks.hpp: the
        // ClockSettings must route a generator at clockSpeed to GCLK_SERCOMn_CORE and provide it).
        using Provides = brigand::list<Hal::Owns<Regs>>;
        using Claims   = Hal::Claim<Clocks::ClkSercomCore<instance>, SPIConfig::clockSpeed>;

        using InterruptIndex = decltype(Sam::Block<Regs>::irqs());

        using Config = Detail::Config<instance>;

        static constexpr auto rxDmaTrigger = Sam::Block<Regs>::dmaRx();
        static constexpr auto txDmaTrigger = Sam::Block<Regs>::dmaTx();

        // the achieved SCK against maxBaudRateError; a failure prints wanted, got and ppm
        static constexpr bool baudInTolerance = [] {
            Prescaler::assertInTolerance<
              Detail::baudDivider(SPIConfig::clockSpeed, SPIConfig::baudRate).achieved,
              SPIConfig::baudRate,
              Prescaler::Tolerance{SPIConfig::maxBaudRateError},
              "SPI SCK">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(baudInTolerance);
        static_assert(Config::isValidPinLocationMISO(SPIConfig::misoPinLocation),
                      "invalid MISOPin");
        static_assert(Config::isValidPinLocationMOSI(SPIConfig::mosiPinLocation),
                      "invalid MOSIPin");
        static_assert(Config::isValidPinLocationSCLK(SPIConfig::sclkPinLocation),
                      "invalid SCLKPin");
        static_assert(Config::isValidPinLocationCS(SPIConfig::csPinLocation),
                      "invalid CSPin");
        static_assert(
          Sam::Pads::checkReceivePin<SPIConfig::loopBack,
                                     std::remove_cvref_t<decltype(SPIConfig::misoPinLocation)>,
                                     std::remove_cvref_t<decltype(SPIConfig::mosiPinLocation)>,
                                     "SPI misoPinLocation/mosiPinLocation">());
        static_assert(!Io::pinLocationEqual(SPIConfig::misoPinLocation,
                                            SPIConfig::sclkPinLocation),
                      "MISO and SCLK are the same pin");
        static_assert(!Io::pinLocationEqual(SPIConfig::misoPinLocation,
                                            SPIConfig::csPinLocation),
                      "MISO and CS are the same pin");

        static constexpr auto powerClockEnable
          = list(typename PM::Enable<Regs::baseAddr>::action{});

        static constexpr auto initStepPinConfig
          = list(std::conditional_t<
                   SPIConfig::loopBack,   // loop-back: the MOSI pin, configured once
                   brigand::list<>,
                   typename Config::template PinConfig<decltype(SPIConfig::misoPinLocation)>>{},
                 typename Config::template PinConfig<decltype(SPIConfig::mosiPinLocation)>{},
                 typename Config::template PinConfig<decltype(SPIConfig::sclkPinLocation)>{},
                 typename Config::template PinConfig<decltype(SPIConfig::csPinLocation)>{});

        static constexpr auto initStepPeripheryConfig = list(
          typename Config::template MisoEnable<
            std::decay_t<decltype(SPIConfig::misoPinLocation)>>{},
          brigand::list<>{},   // MOSI and SCLK have no enable bit
          brigand::list<>{},
          typename Config::template CsEnable<std::decay_t<decltype(SPIConfig::csPinLocation)>>{},

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

        static constexpr auto initStepPeripheryEnable = list(set(Regs::CTRLA::enable));
    };

}}}   // namespace Kvasir::Sercom::SPI
