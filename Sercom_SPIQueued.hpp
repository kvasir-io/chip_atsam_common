#pragma once
// Queued SPI master for the SERCOM with DMAC on TX and RX: the SAM policy for kvasir_devices'
// SPI/QueueCore.hpp. Completion is polled (no DMAC interrupt): call handler() every loop turn.
// Section numbers refer to DS40001882L (SAM D21).
#include "chip/atsam_common/DMAC.hpp"
#include "chip/atsam_common/Sercom_SPI.hpp"
#include "chip/atsam_common/WaitBounds.hpp"
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Devices/Quantities.hpp"
#include "kvasir/Devices/SPI/QueueCore.hpp"
#include "kvasir/StartUp/Hooks.hpp"
#include "kvasir/Util/Prescaler.hpp"

#include <cstdint>
#include <string_view>

namespace Kvasir { namespace Sercom { namespace SPI {

    template<typename SPIConfig,
             typename Clock,
             typename Dma,
             DMAC::DMAChannel  TxChannel,
             DMAC::DMAChannel  RxChannel,
             DMAC::DMAPriority Priority,
             std::size_t       QueueDepth   = 8,
             std::size_t       CallbackSize = 16,
             typename Timing                = Kvasir::SPI::QueueCoreDefaults>
    struct SPIQueued : SPIBase<SPIConfig> {
        using base = SPIBase<SPIConfig>;
        using Regs = typename base::Regs;

        /// f_sck = f_ref / (2 (BAUD + 1)) (27.6.2.3), rounded down to the device's ceiling.
        struct Setup {
            std::uint8_t  baud{};
            std::uint8_t  cpol{};
            std::uint8_t  cpha{};
            std::uint32_t hz{};
            std::uint32_t usPerBitQ10{};

            constexpr bool operator==(Setup const&) const = default;
        };

        static consteval Setup setup(Kvasir::SPI::ClockMode mode,
                                     Units::Hertz           maxClock) {
            auto const          max = std::min(maxClock.numerical_value_in(Units::si::hertz),
                                               static_cast<std::uint32_t>(SPIConfig::baudRate));
            std::uint32_t const ref = SPIConfig::clockSpeed;
            std::uint32_t const div = (ref + 2U * max - 1U) / (2U * max);   // BAUD + 1
            if(div == 0U || div > 256U) {
                // the note "in call to 'rateOutOfReach(wanted Hz, slowest mHz, fastest mHz)'"
                Prescaler::rateOutOfReach(max,
                                          std::uint64_t{ref} * 1000U / 512U,
                                          std::uint64_t{ref} * 1000U / 2U);
                return Setup{};
            }
            auto const m = static_cast<std::uint8_t>(mode);
            return Setup{.baud        = static_cast<std::uint8_t>(div - 1U),
                         .cpol        = static_cast<std::uint8_t>((m >> 1U) & 1U),
                         .cpha        = static_cast<std::uint8_t>(m & 1U),
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
            static constexpr unsigned      Instance     = base::Instance;
            static constexpr bool          SupportsWide = false;
            static constexpr std::uint32_t MaxFrames    = 65535;
            using Setup                                 = SPIQueued::Setup;
            using Snapshot                              = SPIQueued::Snapshot;

            // submit() may come from any interrupt.
            static void mask() { enabled_ = Kvasir::Nvic::disable_all_and_get_old_state(); }

            static void unmask() {
                if(enabled_) { Kvasir::Nvic::enable_all(); }
            }

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
                Dma::template rd<RxChannel>()
                  = DMAC::DmacDescriptor(true,
                                         DMAC::DmacDescriptor::stepsize::x1,
                                         DMAC::DmacDescriptor::stepsel::dst,
                                         t.rxIncrement ? DMAC::DmacDescriptor::dstinc::increment
                                                       : DMAC::DmacDescriptor::dstinc::no_increment,
                                         DMAC::DmacDescriptor::srcinc::no_increment,
                                         DMAC::DmacDescriptor::beatsize::byte,
                                         DMAC::DmacDescriptor::blockact::noact,
                                         DMAC::DmacDescriptor::evosel::disabled,
                                         n,
                                         Regs::DATA8::Addr::value,
                                         rxAddr);
                Dma::template rd<TxChannel>()
                  = DMAC::DmacDescriptor(true,
                                         DMAC::DmacDescriptor::stepsize::x1,
                                         DMAC::DmacDescriptor::stepsel::src,
                                         DMAC::DmacDescriptor::dstinc::no_increment,
                                         t.txIncrement ? DMAC::DmacDescriptor::srcinc::increment
                                                       : DMAC::DmacDescriptor::srcinc::no_increment,
                                         DMAC::DmacDescriptor::beatsize::byte,
                                         DMAC::DmacDescriptor::blockact::noact,
                                         DMAC::DmacDescriptor::evosel::disabled,
                                         n,
                                         txAddr,
                                         Regs::DATA8::Addr::value);
                gen_    = gen;
                active_ = true;
                // RX first.
                apply(Dma::template start<RxChannel, Priority, base::RxDmaTrigger>());
                apply(Dma::template start<TxChannel, Priority, base::TxDmaTrigger>());
            }

            static void poll() {
                if(!active_) { return; }
                auto const rx = Dma::template takeEvent<RxChannel>();
                auto const tx = Dma::template takeEvent<TxChannel>();
                if(rx == Dma::ChannelEvent::none && tx != Dma::ChannelEvent::error) { return; }
                active_            = false;
                bool const overrun = apply(read(Regs::STATUS::bufovf)) != 0;
                bool const failed
                  = overrun || rx == Dma::ChannelEvent::error || tx == Dma::ChannelEvent::error;
                if(rx == Dma::ChannelEvent::error || tx == Dma::ChannelEvent::error) {
                    ++dmaErrors_;
                    apply(Dma::template stop<TxChannel>());
                    apply(Dma::template stop<RxChannel>());
                }
                SPIQueued::Core::complete(gen_, failed);
            }

            static void abort() {
                bool const txOff = Dma::template stopAndWait<TxChannel>();
                bool const rxOff = Dma::template stopAndWait<RxChannel>();
                if(!txOff || !rxOff) { ++dmaStopsExhausted_; }
                active_ = false;
                // Let queued characters finish (TXC, 27.6.2.6.1), bounded.
                bool idle = false;
                for(std::uint32_t spins = 0; spins < ShifterWaitSpins; ++spins) {
                    if(apply(read(Regs::INTFLAG::txc)) != 0) {
                        idle = true;
                        break;
                    }
                }
                if(!idle) { ++shifterWaitsExhausted_; }
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
                  Instance,
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
            static constexpr std::uint32_t ShifterWaitSpins = 20'000;

            inline static std::uint32_t gen_{};
            inline static bool          active_{};
            inline static bool          enabled_{};
            inline static std::uint32_t dmaErrors_{};
            inline static std::uint32_t shifterWaitsExhausted_{};
            inline static std::uint32_t dmaStopsExhausted_{};

            static void waitEnableSync_() {
                Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound>(
                  Kvasir::Register::isClear(Regs::SYNCBUSY::enable));
            }
        };

        using Core    = Kvasir::SPI::QueueCore<Hw, Clock, QueueDepth, CallbackSize, Timing>;
        using Request = typename Core::RequestT;

        static bool submit(Request const& r) { return Core::submit(r); }

        /// QueueCoreFeatures::cancel / ::deadlines (kvasir_devices BusTypes.hpp): a ticket, and cancel(ticket).
        using Result = typename Core::Result;

        static Bus::Ticket submitTracked(Request const& r)
            requires(Core::Tracked)
        {
            return Core::submitTracked(r);
        }

        static Bus::Cancel cancel(Bus::Ticket t)
            requires(Core::Features.cancel)
        {
            return Core::cancel(t);
        }

        static void releaseHold(Kvasir::SPI::Lines const& l) { Core::releaseHold(l); }

        static void handler() { Core::handler(); }

        // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() calls it (StartUp/Hooks.hpp);
        // a firmware that runs the hook must not also call handler() by hand
        using Extends = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &handler>;

        static void reset() { Core::reset(); }
    };

}}}   // namespace Kvasir::Sercom::SPI
