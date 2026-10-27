#pragma once

// The CAN controller exists on the SAM C21 and E5x only: the SAM D21 has none (SAM D21/DA1 family datasheet, 1
// "Description": the peripheral list names no CAN, nor does any chapter), so its chip package says hasCan = false and
// has no Can (atsam_common/ChipTraits.hpp's ChipPackage). Asserted before the other includes: on the D21 it is the
// first error, ahead of the missing peripherals/CAN.hpp.
#include "chip/Package.hpp"

static_assert(
  Kvasir::Sam::Package::hasCan,
  "the CAN exists on the SAM C21/E5x only (SAM D21/DA1 datasheet, 1 Description: no CAN)");

#include "chip/Interrupt.hpp"
#include "chip/PM.hpp"
#include "chip/atsam_common/CanBitTiming.hpp"
#include "chip/atsam_common/Clocks.hpp"
#include "chip/atsam_common/Io.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Hal/Clocks.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "kvasir/Util/attributes.hpp"
#include "kvasir/Util/literals.hpp"
#include "peripherals/CAN.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <optional>

namespace Kvasir { namespace CAN {
    using namespace Kvasir::literals;
    using CanTraits = Kvasir::Sam::Package::Can;

    // One message RAM element: an Rx Buffer/FIFO element (R0, R1, data) or a Tx Buffer element (T0, T1, data) - the
    // same bits in both (C21 DS60001479M 34.9.2 / 34.9.3, md l.33266/33292/33322 and l.33392/33406/33444; E5x
    // DS60001507N 39.9.2 / 39.9.3, md l.54918/54932/54958 and l.55024/55036/55072).
    struct CanMessage {
        static constexpr std::uint32_t xtd    = 1U << 30;   // R0/T0 bit 30 XTD: a 29-bit identifier
        static constexpr std::uint32_t idMask = 0x1FFF'FFFFU;   // R0/T0 bits 28:0 ID
        static constexpr unsigned      standardShift
          = 18;   // "A standard identifier ... into ID[28:18]"
        static constexpr std::uint32_t standardMax = 0x7FFU;   // 11 bits
        static constexpr unsigned      dlcShift    = 16;       // R1/T1 bits 19:16 DLC
        static constexpr std::uint32_t dlcMask     = 0xFU << dlcShift;

        [[nodiscard]] constexpr std::uint32_t id() const {
            if((f0 & xtd) != 0) { return f0 & idMask; }
            return (f0 & idMask) >> standardShift;
        }

        [[nodiscard]] constexpr std::uint32_t size() const { return (f1 & dlcMask) >> dlcShift; }

        constexpr void setId(std::uint32_t id) {
            if(id > standardMax) {
                f0 = (id & idMask) | xtd;
            } else {
                f0 = (id & standardMax) << standardShift;
            }
        }

        constexpr void setSize(std::uint8_t size) {
            f1 = static_cast<std::uint32_t>(size << dlcShift);
        }

        constexpr auto begin() { return data.begin(); }

        constexpr auto end() { return std::next(data.begin(), static_cast<int>(size())); }

        [[nodiscard]] constexpr auto begin() const { return data.begin(); }

        [[nodiscard]] constexpr auto end() const {
            return std::next(data.begin(), static_cast<int>(size()));
        }

        std::uint32_t            f0{};
        std::uint32_t            f1{};
        std::array<std::byte, 8> data{};
    };

    static_assert(sizeof(CanMessage) == 16);

    struct CanFilter {
        enum class FilterType { range = 0, dual = 1, classic = 3 };

        enum class FilterConfig {
            disable       = 0,
            storeFifo0    = 1,
            storeFifo1    = 2,
            reject        = 3,
            priority      = 4,
            priorityFifo0 = 5,
            priorityFifo1 = 6,
            storeBuffer   = 7
        };

        std::uint32_t value{};

        constexpr void setId(std::uint32_t id) {
            assert(2047 >= id);
            value = (static_cast<std::uint32_t>(FilterType::range) << 30)
                  | (static_cast<std::uint32_t>(FilterConfig::storeFifo0) << 27) | id << 16 | id;
        }

        constexpr void disable() {
            value = static_cast<std::uint32_t>(FilterConfig::disable) << 27;
        }
    };

    static_assert(sizeof(CanFilter) == 4);

    namespace Detail {
        template<typename CANConfig_>
        struct CANBase {
            static constexpr auto maxRxSize     = 64;
            static constexpr auto maxTxSize     = 32;
            static constexpr auto maxFilterSize = 128;

            struct CANConfig : CANConfig_ {
                static constexpr auto userConfigOverride
                  = KVASIR_CONFIG_OR(CANConfig_, userConfigOverride, brigand::list<>{});

                // the achieved bit rate against this (the bus needs every node within ~0.5 %)
                static constexpr auto maxBaudRateError
                  = KVASIR_CONFIG_OR(CANConfig_, maxBaudRateError, std::ratio<1, 1000>{});
                // where the bus is sampled, in 1/1000 of a bit; 750 is the reset value's
                static constexpr std::uint32_t samplePointPermille
                  = KVASIR_CONFIG_OR(CANConfig_, samplePointPermille, 750U);
                static constexpr auto rejectNonMatching
                  = KVASIR_CONFIG_OR(CANConfig_, rejectNonMatching, false);

                // message RAM elements: the RX FIFO 0, the TX FIFO and the standard filters; the filters default to all
                // of them when frames that match none are rejected (nothing would be received without a filter), else 0
                static constexpr auto rxSize = KVASIR_CONFIG_OR(CANConfig_, rxSize, maxRxSize);
                static constexpr auto txSize = KVASIR_CONFIG_OR(CANConfig_, txSize, maxTxSize);
                static constexpr auto filterSize
                  = KVASIR_CONFIG_OR(CANConfig_, filterSize, rejectNonMatching ? maxFilterSize : 0);
            };

            static constexpr auto instance = CANConfig::instance;
            using Regs                     = Peripheral::CAN::Registers<instance>;
            // Startup: this CAN, and the clock its bit timing counts (Clocks.hpp: the
            // ClockSettings must route a generator at clockSpeed to GCLK_CANn and provide it).
            using Provides        = brigand::list<Hal::Owns<Regs>>;
            using Claims          = Hal::Claim<Clocks::ClkCan<instance>, CANConfig::clockSpeed>;
            using InterruptIndexs = decltype(Sam::Block<Regs>::irqs());

            // clockSpeed is GCLK_CAN's
            static constexpr BitTiming timing = bitTiming(CANConfig::clockSpeed,
                                                          CANConfig::baudRate,
                                                          CANConfig::samplePointPermille);
            static_assert(timing.found,
                          "no CAN bit timing: baudRate needs 4 to 385 time quanta of 1 to 512 "
                          "GCLK_CAN periods");
            static constexpr bool bitRateInTolerance = [] {
                Prescaler::assertInTolerance<timing.achieved,
                                             CANConfig::baudRate,
                                             Prescaler::Tolerance{CANConfig::maxBaudRateError},
                                             "CAN bit rate">();
                return true;
            }();
            // a static data member of a class template is initialised only when used: this use
            // is what runs the check
            static_assert(bitRateInTolerance);

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
              = list(typename PM::Enable<Regs::baseAddr>::action{});

            static_assert(CanTraits::isRX<instance>(CANConfig::rxPinLocation));
            static_assert(CanTraits::isTX<instance>(CANConfig::txPinLocation));

            static constexpr auto initStepPinConfig
              = list(Sam::PinFunctionOf<CanTraits, instance, decltype(CANConfig::rxPinLocation)>{},
                     Sam::PinFunctionOf<CanTraits, instance, decltype(CANConfig::txPinLocation)>{});

            static constexpr auto initStepPeripheryConfig
              = list(set(Regs::CCCR::init),
                     Register::SequencePoint{},
                     set(Regs::CCCR::cce),
                     Register::SequencePoint{},
                     // NBTP is writable only with CCCR.CCE and INIT set (NBTP, "Write-restricted")
                     Regs::NBTP::overrideDefaults(
                       write(Regs::NBTP::nbrp, Register::value<timing.nbrp>()),
                       write(Regs::NBTP::ntseg1, Register::value<timing.ntseg1>()),
                       write(Regs::NBTP::ntseg2, Register::value<timing.ntseg2>()),
                       write(Regs::NBTP::nsjw, Register::value<timing.nsjw>())),
                     set(Regs::IE::rf0le),
                     rejectConfig,
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
        using CanMessage = ::Kvasir::CAN::CanMessage;

        using CANConfig = typename base::CANConfig;
        using base::maxFilterSize;
        using base::maxRxSize;
        using base::maxTxSize;

        static constexpr std::size_t rxSize     = CANConfig::rxSize;
        static constexpr std::size_t txSize     = CANConfig::txSize;
        static constexpr std::size_t filterSize = CANConfig::filterSize;

        static_assert(maxRxSize >= rxSize,
                      "CAN: rxSize above the RX FIFO elements of the message RAM (MaxRxSize)");
        static_assert(maxTxSize >= txSize,
                      "CAN: txSize above the TX buffer elements of the message RAM (MaxTxSize)");
        static_assert(
          maxFilterSize >= filterSize,
          "CAN: filterSize above the standard filter elements of the message RAM (MaxFilterSize)");

        [[KVASIR_SECTION_MEMBER(
          ".noInitLowRam")]] static inline std::array<CanMessage, rxSize>    rxFIFO{};
        [[KVASIR_SECTION_MEMBER(
          ".noInitLowRam")]] static inline std::array<CanMessage, txSize>    txFIFO{};
        [[KVASIR_SECTION_MEMBER(
          ".noInitLowRam")]] static inline std::array<CanFilter, filterSize> filter{};

        static void runtimeInit() {
            std::uint32_t const rx
              = reinterpret_cast<std::uint32_t>(std::addressof(rxFIFO)) & 0x0000FFFF_u32;
            std::uint32_t const tx
              = reinterpret_cast<std::uint32_t>(std::addressof(txFIFO)) & 0x0000FFFF_u32;
            std::uint32_t const filterAddress
              = reinterpret_cast<std::uint32_t>(std::addressof(filter)) & 0x0000FFFF_u32;

            apply(write(Regs::RXF0C::f0sa, rx),
                  write(Regs::RXF0C::f0s, Kvasir::Register::value<rxSize>()));

            apply(write(Regs::TXBC::tbsa, tx),
                  write(Regs::TXBC::tfqs, Kvasir::Register::value<txSize>()));

            apply(write(Regs::SIDFC::flssa, filterAddress),
                  write(Regs::SIDFC::lss, Kvasir::Register::value<filterSize>()));

            for(auto& f : filter) { f.disable(); }

            apply(clear(Regs::CCCR::init));
        }

        static void setFilter(std::uint8_t  number,
                              std::uint32_t id) {
            assert(filterSize > number);
            filter[number].setId(id);
        }

        static bool send(CanMessage const& msg) {
            auto const state = apply(read(Regs::TXFQS::tfqpi), read(Regs::TXFQS::tffl));
            if(get<1>(state) == 0) { return false; }
            txFIFO[get<0>(state)] = msg;
            apply(write(Regs::TXBAR::ar, 1_u32 << get<0>(state)));
            return true;
        }

        static std::optional<CanMessage> recv() {
            std::optional<CanMessage> ret{};
            auto const state = apply(read(Regs::RXF0S::f0gi), read(Regs::RXF0S::f0fl));
            if(get<1>(state) == 0) { return ret; }

            ret = rxFIFO[get<0>(state)];

            apply(write(Regs::RXF0A::f0ai, get<0>(state)));

            return ret;
        }

        /// Losses of RX FIFO 0 (IR.RF0L, "Rx FIFO 0 Message Lost", C21 DS60001479M md l.31117): counted by the
        /// interrupt, read in thread code - a reader that wants the frames empties the FIFO with recv() faster. One
        /// count per time the flag is found set, at least one frame each: frames lost while the interrupt is masked or
        /// pending count once.
        [[nodiscard]] static std::uint32_t rxOverruns() { return rxOverruns_; }

        // The only interrupt enabled (IE.RF0LE): a frame lost. IR.RF0L is write-one-to-clear, cleared only when it was
        // read set: with an unconditional clear, a loss right after a read that found it clear went uncounted.
        static void onIsr() {
            if(get<0>(apply(read(Regs::IR::rf0l))) != 0) {
                rxOverruns_ = rxOverruns_ + 1U;
                apply(set(Regs::IR::rf0l));
            }
        }

        using Isr = Nvic::IsrsFor<std::addressof(onIsr), typename base::InterruptIndexs>;

    private:
        // written by onIsr only, read anywhere: one word, volatile for the reader outside the interrupt
        inline static std::uint32_t volatile rxOverruns_{};
    };
}}   // namespace Kvasir::CAN
