#pragma once
// Builds on kvasir_devices' bus tier: link kvasir::bus (or kvasir::devices).

#include "Sercom_I2C.hpp"
#include "chip/WaitBounds.hpp"
#include "kvasir/Devices/I2C/Controller/LineRecovery.hpp"
#include "kvasir/Devices/I2C/Controller/QueuedFront.hpp"
#include "kvasir/Log/LogLimited.hpp"
#include "kvasir/Register/Apply.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ratio>
#include <string_view>

namespace Kvasir { namespace Sercom { namespace I2C {

    namespace Detail {
        /// SercomBaud.hpp's per-device BAUD as kvasir_devices' QueuedShape takes it.
        struct SercomClockTiming {
            using ClockTiming = Detail::ClockTiming;

            template<std::intmax_t Num,
                     std::intmax_t Denom>
            static consteval ClockTiming clockTiming(std::uint32_t     fClockSpeed,
                                                     std::uint32_t     fBaud,
                                                     std::ratio<Num,
                                                                Denom> maxError) {
                return Detail::clockTiming(fClockSpeed, fBaud, maxError);
            }
        };

        /// The SERCOM's half of the queued master: kvasir_devices' QueueCore over `Hw`, the register work.
        /// I2CBehaviorQueued is this and the front (kvasir_devices I2C/Controller/QueuedFront.hpp says why the two are apart).
        template<typename I2CConfig,
                 typename Clock,
                 std::size_t QueueDepth,
                 std::size_t CallbackSize>
        struct I2CQueuedHalf
          : I2CBase<I2CConfig>
          , Kvasir::I2C::QueuedShape<typename I2CBase<I2CConfig>::I2CConfig,
                                     Clock,
                                     CallbackSize,
                                     SercomClockTiming> {
            using base  = I2CBase<I2CConfig>;
            using Shape = Kvasir::I2C::
              QueuedShape<typename base::I2CConfig, Clock, CallbackSize, SercomClockTiming>;
            using Regs     = typename base::Regs;
            using tp       = typename Clock::time_point;
            using Recovery = Kvasir::I2C::LineRecovery<base, Clock>;

            /// The bus state is UNKNOWN after the enable and nothing starts from there; with one
            /// master it is idle (28.6.2.3). Startup calls this after initStepPeripheryEnable.
            static void runtimeInit() {
                apply(write(Regs::STATUS::BUSSTATEValC::idle));
                Hw::waitSync_();
            }

            /// The block's registers when a transaction timed out, read before the reset (the core's
            /// TimeoutSnapshot holds them as `hw`, with the request's progress and the lines: SCL low
            /// with STATUS.CLKHOLD set is the master waiting for its interrupt to be served, SCL low
            /// without it is a part holding the clock).
            struct Snapshot {
                std::uint16_t status{};      ///< STATUS: CLKHOLD, BUSSTATE, RXNACK, ARBLOST, BUSERR
                std::uint8_t  intFlag{};     ///< INTFLAG: ERROR, SB, MB
                std::uint8_t  intEnable{};   ///< INTENSET
                std::uint32_t ctrlA{};
                std::uint32_t syncBusy{};
            };

            /// The SERCOM's half of the queued master (kvasir_devices I2C/Controller/QueueHw.hpp).
            struct Hw {
                static constexpr auto instance       = base::instance;
                static constexpr bool cancellable    = Shape::Cancellable;
                static constexpr bool deadlines      = Shape::Deadlines;
                static constexpr bool perDeviceClock = Shape::PerDeviceClock;
                static constexpr auto busFeatures    = Shape::BusFeatures;
                using Recovery                       = I2CQueuedHalf::Recovery;
                using Request                        = typename Shape::Request;
                using Snapshot                       = I2CQueuedHalf::Snapshot;
                using ClockTiming                    = typename Shape::ClockTiming;
                using State                          = Kvasir::I2C::QueueState;

                // The bus interrupt line(s); the core counts the depth (Nvic::LineMask: ICER, dsb; isb - ISER).
                [[gnu::always_inline]] static void mask() {
                    Kvasir::Nvic::LineMask<typename base::InterruptIndexs>::mask();
                }

                [[gnu::always_inline]] static void unmask() {
                    Kvasir::Nvic::LineMask<typename base::InterruptIndexs>::unmask();
                }

                /// The block through a software reset and set up again, bus state idle. The interrupt is
                /// masked by the caller (the core), and its unmask enables it again: the timeout
                /// path keeps it masked until its request is completed.
                static void reinit() {
                    apply(set(Regs::CTRLA::swrst));
                    waitSync_();
                    apply(base::initStepPeripheryConfig);
                    // The block is back at baudRate's BAUD: the next request writes its own.
                    if constexpr(perDeviceClock) { timingCache_.invalidate(); }
                    apply(base::initStepInterruptConfig);
                    apply(set(Regs::CTRLA::enable));
                    waitSync_();
                    runtimeInit();
                }

                [[gnu::always_inline]] static auto transferTimeout(std::size_t    totalBytes,
                                                                   Request const& r) {
                    if constexpr(perDeviceClock) {
                        return base::calcTransferTimeout(totalBytes, r.timing.usPerByte);
                    } else {
                        static_cast<void>(r);
                        return base::calcTransferTimeout(totalBytes);
                    }
                }

                static void start(Request const& r) {
                    if constexpr(perDeviceClock) { applyTiming_(r.timing); }
                    if(r.sendBytes() != 0) {
                        Core::state_ = State::sending;
                        apply(write(Regs::ADDR::addr, unsigned(r.address) << 1U));
                    } else {
                        Core::state_ = State::receiving;
                        apply(write(Regs::ADDR::addr, (unsigned(r.address) << 1U) | 0x01U));
                    }
                }

                /// Nothing to undo: the command that ended the request (STOP, or the SWRST of a timeout)
                /// left the block ready for the next ADDR write.
                [[gnu::always_inline]] static void finish() {}

                /// The SERCOM takes a command only while MB or SB is set (D21 md l.23999, C21 md l.28087): the
                /// interrupt at the next byte boundary issues NACK + STOP (Core::cancelPending_). A byte boundary
                /// that never comes is the transfer timeout's (SWRST).
                [[gnu::always_inline]] static void requestStop() {}

                /// The block may be anywhere in a byte with the clock held: only the software reset is
                /// sure to let go of the lines (see the header).
                [[gnu::always_inline]] static void timeoutAbort() { reinit(); }

                /// The STOP of a NAKed or failed transfer is still going out while the block owns the bus.
                [[gnu::always_inline]] static bool busActive() {
                    return !fieldEquals(Regs::STATUS::BUSSTATEValC::idle);
                }

                [[gnu::always_inline]] static void logTimeout([[maybe_unused]] tp now) {
                    KVASIR_LOG_LIMITED(Core::faultLog_.note(Core::Fault::timeout,
                                                            now,
                                                            Core::currentRequest_.address,
                                                            std::uint16_t{0}),
                                       UC_LOG_W,
                                       "i2c{} timeout addr={:#04x} STATUS {:#06x}",
                                       base::instance,
                                       Core::currentRequest_.address,
                                       status_());
                }

                /// STATUS as the timeout found it: the snapshot's, or read now on a bus without one.
                static std::uint16_t status_() {
                    if constexpr(busFeatures.timeoutSnapshot) {
                        return Core::lastTimeout().hw.status;
                    } else {
                        return static_cast<std::uint16_t>(
                          Kvasir::Register::get<0>(apply(read(Regs::STATUS::FULLREGISTER))));
                    }
                }

                static Snapshot snapshot() {
                    return Snapshot{
                      .status  = Kvasir::Register::get<0>(apply(read(Regs::STATUS::FULLREGISTER))),
                      .intFlag = Kvasir::Register::get<0>(apply(read(Regs::INTFLAG::FULLREGISTER))),
                      .intEnable
                      = Kvasir::Register::get<0>(apply(read(Regs::INTENSET::FULLREGISTER))),
                      .ctrlA = Kvasir::Register::get<0>(apply(read(Regs::CTRLA::FULLREGISTER))),
                      .syncBusy
                      = Kvasir::Register::get<0>(apply(read(Regs::SYNCBUSY::FULLREGISTER))),
                    };
                }

                /// lastTimeout() as a log line (state 1 is sending, 2 receiving; sent/received are
                /// bytes done of bytes asked).
                static void
                logLastTimeout([[maybe_unused]] Kvasir::I2C::TimeoutSnapshot<Snapshot> const& t) {
                    UC_LOG_W(
                      "i2c{} last timeout: addr {:#04x}, state {}, sent {}/{}, received {}/{}, "
                      "STATUS {:#06x}, INTFLAG {:#04x}, INTENSET {:#04x}, CTRLA {:#010x}, SYNCBUSY "
                      "{:#x}, SDA {}, SCL {}, {} interrupt(s) in the request, last {} us before, "
                      "request {} us old",
                      base::instance,
                      t.address,
                      t.state,
                      t.sent,
                      t.toSend,
                      t.received,
                      t.toReceive,
                      t.hw.status,
                      t.hw.intFlag,
                      t.hw.intEnable,
                      t.hw.ctrlA,
                      t.hw.syncBusy,
                      std::string_view{t.sdaHigh ? "high" : "LOW"},
                      std::string_view{t.sclHigh ? "high" : "LOW"},
                      t.isrEntries,
                      t.usSinceIsr,
                      t.usAge);
                }

                // INTFLAG.MB and SB are cleared by the next operation itself -- a write of ADDR or
                // DATA, or a command in CTRLB.CMD (28.10.6) -- and are NOT written here after it: a
                // write of '1' that came late (this interrupt preempted for longer than a byte takes,
                // 22 us at 400 kHz) would clear the flag of the byte that has just gone out, and the
                // transfer would stand until its timeout. Only ERROR needs the write.
                static void onIsr() {
                    auto&      req = Core::currentRequest_;
                    auto const flags
                      = apply(read(Regs::INTFLAG::error, Regs::INTFLAG::mb, Regs::INTFLAG::sb));
                    // STATUS whole: it goes into the fault key and the log as it is
                    auto const status
                      = Kvasir::Register::get<0>(apply(read(Regs::STATUS::FULLREGISTER)));
                    bool const error = Kvasir::Register::get<0>(flags) != 0;
                    bool const mb    = Kvasir::Register::get<1>(flags) != 0;
                    bool const sb    = Kvasir::Register::get<2>(flags) != 0;
                    bool const rxnack
                      = Kvasir::Register::extract(Regs::STATUS::rxnack, status) != 0;
                    if(error) { apply(set(Regs::INTFLAG::error)); }

                    if(!Core::active_) {
                        // Nothing running: whatever raised it is over. The flags are written here,
                        // since no operation follows that would clear them.
                        ++Core::spuriousIsr_;
                        apply(set(Regs::INTFLAG::mb, Regs::INTFLAG::sb));
                        return;
                    }

                    // our own stop (cancel, deadline), taken at this byte boundary: NACK + STOP, not a fault. A NAK or
                    // a bus error that came first keeps its own result (below).
                    if constexpr(Shape::Tracked) {
                        if(Core::cancelPending_ && !error && !rxnack && (mb || sb)) {
                            apply(nackStop);
                            Core::complete(Core::stopAs_);
                            return;
                        }
                    }

                    if(error || (mb && rxnack)) {
                        // rxnack alone is the device not answering, which is what tells a driver the
                        // part is absent; anything else is a bus fault and says nothing about it.
                        bool const isNak = rxnack && !error;
                        // The address NAKed (before the first data byte, or the read phase's address: a
                        // host never receives a NAK for data) is an answer - nobody there, every probe of a
                        // scan - not a fault: debug, and not on the faults' budget. A known part that stops
                        // answering is Presence's warning; a NAK of a data byte stays a warning.
                        bool const sending    = Core::state_ == State::sending;
                        bool const addressNak = isNak && (!sending || Core::sendIndex_ == 0);
                        auto const fault
                          = sending ? Core::Fault::abortSend : Core::Fault::abortRecv;
                        if(addressNak) {
                            KVASIR_LOG_LIMITED(Core::nakLog_.allow(Core::faultKey(fault, status)),
                                               UC_LOG_D,
                                               "i2c{} {} addr={:#04x}: no ACK, nobody there",
                                               base::instance,
                                               std::string_view{sending ? "send" : "recv"},
                                               req.address);
                        } else {
                            KVASIR_LOG_LIMITED(
                              Core::faultLog_.note(fault,
                                                   req.address,
                                                   static_cast<std::uint16_t>(status)),
                              UC_LOG_W,
                              "i2c{} abort {} addr={:#04x} STATUS {:#06x}{}",
                              base::instance,
                              std::string_view{sending ? "send" : "recv"},
                              req.address,
                              status,
                              std::string_view{isNak ? " (NAK)" : ""});
                        }
                        // A STOP, which also clears MB/SB and lets go of the clock. Without the bus
                        // (arbitration lost) the command does nothing, so the flags are written too.
                        apply(nackStop);
                        if(Kvasir::Register::extract(Regs::STATUS::arblost, status) != 0) {
                            apply(set(Regs::INTFLAG::mb, Regs::INTFLAG::sb));
                        }
                        Core::complete(isNak ? Core::Result::nack : Core::Result::failed);
                        return;
                    }

                    if(Core::state_ == State::sending) {
                        if(!mb) {
                            ++Core::spuriousIsr_;
                            return;
                        }
                        if(Core::sendIndex_ < req.sendBytes()) {
                            apply(write(Regs::DATA::data,
                                        static_cast<std::uint8_t>(req.sendByte(Core::sendIndex_))));
                            ++Core::sendIndex_;
                        } else if(!req.receiveData.empty()) {
                            // repeated START into the read phase
                            Core::state_ = State::receiving;
                            apply(write(Regs::ADDR::addr, (unsigned(req.address) << 1U) | 0x01U));
                        } else {
                            apply(ackStop);
                            Core::complete(Core::Result::ok);
                        }
                    } else {   // receiving
                        if(!sb) {
                            ++Core::spuriousIsr_;
                            return;
                        }
                        req.receiveData[Core::receivedCount_] = static_cast<std::byte>(
                          Kvasir::Register::get<0>(apply(read(Regs::DATA::data))));
                        ++Core::receivedCount_;
                        if(Core::receivedCount_ < req.receiveData.size()) {
                            apply(ackByteRead);
                        } else {
                            apply(nackStop);
                            Core::complete(Core::Result::ok);
                        }
                    }
                }

                // -- the register work's own state and helpers ------------------------------------------

                static constexpr auto ackByteRead
                  = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_ack),
                                                       write(Regs::CTRLB::CMDValC::ack_byte_read)));
                static constexpr auto nackStop
                  = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_nack),
                                                       write(Regs::CTRLB::CMDValC::ack_stop)));
                static constexpr auto ackStop
                  = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_ack),
                                                       write(Regs::CTRLB::CMDValC::ack_stop)));

                inline static std::uint32_t waitsExhausted_{};

                // perDeviceClock: the BAUD in the block, and whether it is known (not after a
                // reset). A member of a class template: never instantiated on a bus without it.
                inline static Kvasir::I2C::TimingCache<ClockTiming> timingCache_{};

                /// Times a wait for the block's register synchronisation (SYNCBUSY) ran out of spins (the
                /// bus's disableWaitsExhausted(): the RP driver's name, where the wait is for IC_ENABLE_STATUS -
                /// the same kind of event, a block that does not follow its own handshake).
                static std::uint32_t waitsExhausted() { return waitsExhausted_; }

                /// Times a start rewrote BAUD for a device at another clock (perDeviceClock).
                static std::uint32_t clockSwitches() { return timingCache_.switches(); }

                /// Spin until the block has taken what was written to it (SYNCBUSY: SWRST, ENABLE,
                /// SYSOP). Bounded (Sam::SyncBound, whatever policy the application injects: the caller
                /// recovers): a block that never gets there is the dead-bus watchdog's.
                static void waitSync_() {
                    if(!Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound,
                                                    Kvasir::Register::OnTimeout::Report>(
                         Kvasir::Register::isClear(Regs::SYNCBUSY::FULLREGISTER)))
                    {
                        ++waitsExhausted_;
                    }
                }

                /// The BAUD (and CTRLA.SPEED) of the request about to start, written only when it
                /// differs from the last. Both are enable-protected: "writing to these registers will
                /// be discarded" while CTRLA.ENABLE=1 (SAM D21 DS40001882 28.6.2.1, SAM C21
                /// DS60001479 33.6.2.1). So the block is disabled for it, after the bus has gone idle
                /// (the previous transfer's STOP is still on its way out when a success starts the
                /// next request from its interrupt), and the bus state forced to idle again after the
                /// enable, which leaves it UNKNOWN (D21 28.6.2.3, C21 33.6.2.3). Interrupt masked or
                /// in the ISR.
                static void applyTiming_(ClockTiming const& t) {
                    if(!timingCache_.needs(t)) { return; }
                    // bounded (Sam::I2cStopBound); a bus that stays busy is switched anyway, as before
                    static_cast<void>(
                      Kvasir::Register::waitUntil<Kvasir::Chip::Sam::I2cStopBound,
                                                  Kvasir::Register::OnTimeout::Report>(
                        Regs::STATUS::BUSSTATEValC::idle));
                    apply(clear(Regs::CTRLA::enable));
                    waitSync_();
                    apply(write(Regs::BAUD::baud, std::uint32_t{t.baud}),
                          write(Regs::BAUD::baudlow, std::uint32_t{t.baudlow}),
                          write(Regs::BAUD::hsbaud, std::uint32_t{0}),
                          write(Regs::BAUD::hsbaudlow, std::uint32_t{0}));
                    if(t.speed == 0) {
                        apply(write(Regs::CTRLA::SPEEDValC::standard_and_fast_mode));
                    } else {
                        apply(write(Regs::CTRLA::SPEEDValC::fastplus_mode));
                    }
                    apply(set(Regs::CTRLA::enable));
                    waitSync_();
                    runtimeInit();
                    timingCache_.took(t);
                }
            };

            using Core = Kvasir::I2C::QueueCore<Hw, Clock, QueueDepth, CallbackSize>;
        };
    }   // namespace Detail

    /// The queued I2C master for the SAMD SERCOM, with the same interface as the RP2040 and
    /// RP2350 `Kvasir::I2C::I2CBehaviorQueued`: a request carries its own buffers and a
    /// callback, the bus owns the sequencing, and completion arrives in the ISR. That is the
    /// interface `Kvasir::I2C::Device` -- and so every chip
    /// description in kvasir_devices -- is written against, so this is what lets a SAMD board
    /// use them. Both are kvasir_devices' `Kvasir::I2C::QueueCore` (the queue, cancel, deadlines,
    /// the timeout, the dead-bus watchdog, the recovery's clock, the counters, the request and
    /// result types of I2C/Controller/Request.hpp) over their chip's register work, `Hw` here.
    ///
    /// Sercom_I2C.hpp keeps the SERCOM's register configuration (Detail::I2CBase) this is
    /// built on. What this one guarantees the engine:
    ///
    ///  * a NAK is reported as `nack` and a bus error as `failed`. The engine
    ///    counts only NAKs towards declaring a device absent, so this is what makes
    ///    parking and probing work;
    ///  * the payload is read straight out of `prefix` + `sendData` and written straight into
    ///    `receiveData`, so a transfer is not limited by an internal buffer;
    ///  * requests queue, so a device can submit from inside another's callback; what a callback
    ///    submits starts after the callback has returned (QueueCore).
    ///
    /// One transfer is on the wire at a time, as the peripheral allows.
    ///
    /// It also looks after the bus the way the RP driver does, with the same
    /// calls (a bus is judged by them on either chip):
    ///
    ///  * a transfer gets calcTransferTimeout() for its length, not a flat 100 ms, and one that
    ///    runs out is ended by a software reset of the block - the data sheet's own way out of
    ///    a hang (DS40001882, 28.6.2.3: "Violating the protocol may cause the I2C to hang. If
    ///    this happens it is possible to recover from this state by a software Reset"), and the
    ///    only one: STATUS.BUSSTATE can be forced to idle from UNKNOWN and from nowhere else
    ///    (28.10.7), which the write the timeout used to make here ignored;
    ///  * `Recovery` (kvasir_devices' kvasir/Devices/I2C/Controller/LineRecovery.hpp): SDA held low on an
    ///    idle bus is noticed and clocked free, with a forced STOP as the last resort;
    ///  * the dead-bus watchdog: Core::DeadBusFailures bus faults in a row re-initialise the block,
    ///    then run a recovery;
    ///  * the fault counters, lastTimeout() / logLastTimeout() and takeLatency().
    ///
    /// The SERCOM's hardware time-outs (CTRLA.LOWTOUTEN, SEXTTOEN, MEXTTOEN, INACTOUT) are
    /// left off: they run from GCLK_SERCOM_SLOW, which "must be configured to use a 32KHz
    /// oscillator" (28.6.3.1) - a clock this driver cannot ask every firmware for.
    template<typename I2CConfig,
             typename Clock,
             std::size_t QueueDepth_   = 8,
             std::size_t CallbackSize_ = 16>
    struct I2CBehaviorQueued
      : Detail::I2CQueuedHalf<I2CConfig, Clock, QueueDepth_, CallbackSize_>
      , Kvasir::I2C::QueuedFront<
          typename Detail::I2CQueuedHalf<I2CConfig, Clock, QueueDepth_, CallbackSize_>::Core> {
        using Isr = Nvic::IsrsFor<std::addressof(I2CBehaviorQueued::onIsr),
                                  typename I2CBehaviorQueued::InterruptIndexs>;
    };

}}}   // namespace Kvasir::Sercom::I2C
