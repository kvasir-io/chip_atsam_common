#pragma once
// Link kvasir::devices (not only kvasir::bus): the clock ceiling is a Units::Hertz (Quantities.hpp, mp-units).
// Queued SPI master for the SERCOM with DMAC on TX and RX: the SAM policy for kvasir_devices'
// SPI/QueueCore.hpp. Completion is polled (no DMAC interrupt): call turn() every loop turn.
// Section numbers refer to DS40001882L (SAM D21).
#include "chip/WaitBounds.hpp"
#include "chip/atsam_common/DMAC.hpp"
#include "chip/atsam_common/Sercom_SPI.hpp"
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Devices/Quantities.hpp"
#include "kvasir/Devices/SPI/QueueCore.hpp"
#include "kvasir/Devices/SPI/QueuedFront.hpp"
#include "kvasir/Hal/DmaHandoff.hpp"

#include <cstdint>
#include <string_view>

namespace Kvasir { namespace Sercom { namespace SPI {

    namespace Detail {
        /// SPIQueued without its front: the SERCOM registers, the DMAC, the Startup steps (SPIBase) and the
        /// QueueCore over the Hw policy (kvasir_devices SPI/QueuedFront.hpp says why the two are apart).
        template<typename SPIConfig,
                 typename Clock,
                 typename Dma,
                 DMAC::DMAChannel  TxChannel,
                 DMAC::DMAChannel  RxChannel,
                 DMAC::DMAPriority Priority,
                 std::size_t       QueueDepth,
                 std::size_t       CallbackSize,
                 typename Timing>
        struct SPIQueuedHalf : SPIBase<SPIConfig> {
            using base = SPIBase<SPIConfig>;
            using Regs = typename base::Regs;

            using Claims
              = brigand::append<typename base::Claims, DMAC::Claims<Dma, TxChannel, RxChannel>>;

            /// f_sck = f_ref / (2 (BAUD + 1)) (27.6.2.3), rounded down to the device's ceiling.
            struct Setup {
                std::uint8_t  baud{};
                std::uint8_t  cpol{};
                std::uint8_t  cpha{};
                std::uint32_t hz{};
                std::uint32_t usPerBitQ10{};

                constexpr bool operator==(Setup const&) const = default;
            };

            static consteval Setup setup(Kvasir::Hal::ClockMode mode,
                                         Units::Hertz           maxClock) {
                std::uint32_t const ref = SPIConfig::clockSpeed;
                // the slowest SCK is f_ref / 512 (BAUD 255), the fastest f_ref / 2
                auto const max
                  = Kvasir::SPI::ceilingHz(maxClock.numerical_value_in(Units::si::hertz),
                                           static_cast<std::uint32_t>(SPIConfig::baudRate),
                                           ref,
                                           512U,
                                           2U);
                std::uint32_t const div  = (ref + 2U * max - 1U) / (2U * max);   // BAUD + 1, 1..256
                auto const          bits = Kvasir::SPI::modeBits(mode);
                return Setup{.baud        = static_cast<std::uint8_t>(div - 1U),
                             .cpol        = bits.cpol,
                             .cpha        = bits.cpha,
                             .hz          = ref / (2U * div),
                             .usPerBitQ10 = Kvasir::SPI::usPerBitQ10(ref / (2U * div))};
            }

            struct Snapshot {
                std::uint32_t intflag{};
                std::uint32_t status{};
                std::uint32_t ctrla{};
                bool          active{};
                std::uint32_t shifterWaitsExhausted{};
                std::uint32_t dmaStopsExhausted{};
            };

            struct Hw {
                static constexpr unsigned      instance     = base::instance;
                static constexpr bool          supportsWide = false;
                static constexpr std::uint32_t maxFrames    = 65535;
                using Setup                                 = SPIQueuedHalf::Setup;
                using Snapshot                              = SPIQueuedHalf::Snapshot;

                // submit() may come from any interrupt.
                static void mask() { irqs_ = Kvasir::Core::IrqState::saveAndMask(); }

                static void unmask() { irqs_.restore(); }

                static void configure(Setup const& s) {
                    // CTRLA and BAUD are enable-protected (27.8.1, 27.8.3).
                    apply(clear(Regs::CTRLA::enable));
                    waitEnableSync_();
                    apply(write(Regs::BAUD::baud, s.baud));
                    // RX on even without MISO: the RX DMA completion marks the end of a frame.
                    apply(set(Regs::CTRLB::rxen));
                    using Cpol = typename decltype(Regs::CTRLA::cpol)::DataType;
                    using Cpha = typename decltype(Regs::CTRLA::cpha)::DataType;
                    apply(write(Regs::CTRLA::cpol, static_cast<Cpol>(s.cpol)),
                          write(Regs::CTRLA::cpha, static_cast<Cpha>(s.cpha)));
                    apply(set(Regs::CTRLA::enable));
                    waitEnableSync_();
                }

                static void start(Kvasir::SPI::Transfer const& t,
                                  std::uint32_t                gen,
                                  void (*)(std::uint32_t,
                                           bool)) {
                    // Drop leftovers of an aborted transfer.
                    for(int i = 0; i < 4 && apply(read(Regs::INTFLAG::rxc)); ++i) {
                        apply(read(Regs::DATA8::data));
                    }
                    apply(set(Regs::STATUS::bufovf));   // write '1' to clear (27.8.7)
                    static_cast<void>(Dma::template takeEvent<RxChannel>());
                    static_cast<void>(Dma::template takeEvent<TxChannel>());

                    auto const n = static_cast<std::uint16_t>(t.frames);
                    // Incrementing addresses point past the end (20.10.3, 20.10.4).
                    auto const rxAddr
                      = reinterpret_cast<std::uint32_t>(t.rx) + (t.rxIncrement ? n : 0U);
                    auto const txAddr
                      = reinterpret_cast<std::uint32_t>(t.tx) + (t.txIncrement ? n : 0U);
                    Dma::template rd<RxChannel>() = DMAC::DmacDescriptor{
                      {.dstInc = t.rxIncrement ? DMAC::DmacDescriptor::Dstinc::increment
                                               : DMAC::DmacDescriptor::Dstinc::noIncrement,
                       .count  = n,
                       .src    = Regs::DATA8::Addr::value,
                       .dst    = rxAddr}
                    };
                    Dma::template rd<TxChannel>() = DMAC::DmacDescriptor{
                      {.stepSel = DMAC::DmacDescriptor::Stepsel::src,
                       .srcInc  = t.txIncrement ? DMAC::DmacDescriptor::Srcinc::increment
                                                : DMAC::DmacDescriptor::Srcinc::noIncrement,
                       .count   = n,
                       .src     = txAddr,
                       .dst     = Regs::DATA8::Addr::value}
                    };
                    gen_    = gen;
                    active_ = true;
                    // the descriptors and the TX bytes are the DMAC's from here (Kvasir/Hal/DmaHandoff.hpp); RX first.
                    Hal::handToHardware();
                    apply(Dma::template start<RxChannel, Priority, base::rxDmaTrigger>());
                    apply(Dma::template start<TxChannel, Priority, base::txDmaTrigger>());
                }

                static void poll() {
                    if(!active_) { return; }
                    auto const rx = Dma::template takeEvent<RxChannel>();
                    auto const tx = Dma::template takeEvent<TxChannel>();
                    if(rx == Dma::ChannelEvent::none && tx != Dma::ChannelEvent::error) { return; }
                    // the received bytes are the core's again
                    Hal::takeFromHardware();
                    active_            = false;
                    bool const overrun = apply(read(Regs::STATUS::bufovf)) != 0;
                    bool const failed
                      = overrun || rx == Dma::ChannelEvent::error || tx == Dma::ChannelEvent::error;
                    if(rx == Dma::ChannelEvent::error || tx == Dma::ChannelEvent::error) {
                        ++dmaErrors_;
                        apply(Dma::template stop<TxChannel>());
                        apply(Dma::template stop<RxChannel>());
                    }
                    SPIQueuedHalf::Core::complete(gen_, failed);
                }

                static void abort() {
                    bool const txOff = Dma::template stopAndWait<TxChannel>();
                    bool const rxOff = Dma::template stopAndWait<RxChannel>();
                    if(!txOff || !rxOff) { ++dmaStopsExhausted_; }
                    active_ = false;
                    // Let queued characters finish (TXC, 27.6.2.6.1), bounded whatever policy the
                    // application injects: a shifter that never empties is counted, the caller resets.
                    if(!Kvasir::Register::waitUntil<
                         Kvasir::Chip::Sam::SpiShifterBound<SPIConfig::clockSpeed>,
                         Kvasir::Register::OnTimeout::Report>(
                         Kvasir::Register::isSet(Regs::INTFLAG::txc)))
                    {
                        ++shifterWaitsExhausted_;
                    }
                    for(int i = 0; i < 4 && apply(read(Regs::INTFLAG::rxc)); ++i) {
                        apply(read(Regs::DATA8::data));
                    }
                }

                static void reinit() {
                    abort();
                    apply(set(Regs::CTRLA::swrst));
                    Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound>(
                      Kvasir::Register::isClear(Regs::SYNCBUSY::swrst));
                    apply(base::initStepPeripheryConfig);
                    apply(base::initStepPeripheryEnable);
                    waitEnableSync_();
                }

                static Snapshot snapshot() {
                    return Snapshot{
                      .intflag               = get<0>(apply(read(Regs::INTFLAG::FULLREGISTER))),
                      .status                = get<0>(apply(read(Regs::STATUS::FULLREGISTER))),
                      .ctrla                 = get<0>(apply(read(Regs::CTRLA::FULLREGISTER))),
                      .active                = active_,
                      .shifterWaitsExhausted = shifterWaitsExhausted_,
                      .dmaStopsExhausted     = dmaStopsExhausted_,
                    };
                }

                static void log([[maybe_unused]] Snapshot const& s) {
                    UC_LOG_W(
                      "sercom{} spi at the timeout: INTFLAG {:#04x}, STATUS {:#06x}, CTRLA "
                      "{:#010x}, {}, {} DMA error(s) so far, {} abort(s) with the transmitter "
                      "never done, {} with a DMA channel that never stopped",
                      instance,
                      s.intflag,
                      s.status,
                      s.ctrla,
                      std::string_view{s.active ? "DMA armed" : "nothing armed"},
                      dmaErrors_,
                      s.shifterWaitsExhausted,
                      s.dmaStopsExhausted);
                }

                static std::uint32_t dmaErrors() { return dmaErrors_; }

            private:
                inline static std::uint32_t          gen_{};
                inline static bool                   active_{};
                inline static Kvasir::Core::IrqState irqs_{};   ///< PRIMASK as mask() found it
                inline static std::uint32_t          dmaErrors_{};
                inline static std::uint32_t          shifterWaitsExhausted_{};
                inline static std::uint32_t          dmaStopsExhausted_{};

                static void waitEnableSync_() {
                    Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound>(
                      Kvasir::Register::isClear(Regs::SYNCBUSY::enable));
                }
            };

            using Core = Kvasir::SPI::QueueCore<Hw, Clock, QueueDepth, CallbackSize, Timing>;
        };
    }   // namespace Detail

    template<typename SPIConfig,
             typename Clock,
             typename Dma,
             DMAC::DMAChannel  TxChannel,
             DMAC::DMAChannel  RxChannel,
             DMAC::DMAPriority Priority,
             std::size_t       QueueDepth   = 8,
             std::size_t       CallbackSize = 16,
             typename Timing                = Kvasir::SPI::QueueCoreDefaults>
    struct SPIQueued
      : Detail::SPIQueuedHalf<SPIConfig,
                              Clock,
                              Dma,
                              TxChannel,
                              RxChannel,
                              Priority,
                              QueueDepth,
                              CallbackSize,
                              Timing>
      , Kvasir::SPI::QueuedFront<typename Detail::SPIQueuedHalf<SPIConfig,
                                                                Clock,
                                                                Dma,
                                                                TxChannel,
                                                                RxChannel,
                                                                Priority,
                                                                QueueDepth,
                                                                CallbackSize,
                                                                Timing>::Core> {};

}}}   // namespace Kvasir::Sercom::SPI
