#pragma once

#include "Sercom_I2C.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Devices/BusTypes.hpp"
#include "kvasir/Devices/I2C/LineRecovery.hpp"
#include "kvasir/Register/Apply.hpp"
#include "kvasir/StartUp/Hooks.hpp"
#include "kvasir/Util/RateLimiter.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace Sercom { namespace I2C {

    /// The queued I2C master for the SAMD SERCOM, with the same interface as the RP2040 and
    /// RP2350 `Kvasir::I2C::I2CBehaviorQueued`: a request carries its own buffers and a
    /// callback, the bus owns the sequencing, and completion arrives in the ISR. That is the
    /// interface `Kvasir::I2C::Device` -- and so every chip
    /// description in kvasir_devices -- is written against, so this is what lets a SAMD board
    /// use them.
    ///
    /// Sercom_I2C.hpp keeps the SERCOM's register configuration (Detail::I2CBase) this is
    /// built on; the older non-queued `I2CBehavior` that lived there went on 2026-09-13
    /// with the last driver written against its acquire/operationState style. What this
    /// one guarantees the engine:
    ///
    ///  * a NAK is reported as `notAcknowledged` and a bus error as `failed`. The engine
    ///    counts only NAKs towards declaring a device absent, so this is what makes
    ///    parking and probing work;
    ///  * the payload is read straight out of `prefix` + `sendData` and written straight into
    ///    `receiveData`, so a transfer is not limited by an internal buffer;
    ///  * requests queue, so a device can submit from inside another's callback.
    ///
    /// One transfer is on the wire at a time, as the peripheral allows.
    ///
    /// Since 2026-09-20 it also looks after the bus the way the RP driver does, with the same
    /// calls (i2c_testing judges a bus by them on either chip):
    ///
    ///  * a transfer gets calcTransferTimeout() for its length, not a flat 100 ms, and one that
    ///    runs out is ended by a software reset of the block - the data sheet's own way out of
    ///    a hang (DS40001882, 28.6.2.3: "Violating the protocol may cause the I2C to hang. If
    ///    this happens it is possible to recover from this state by a software Reset"), and the
    ///    only one: STATUS.BUSSTATE can be forced to idle from UNKNOWN and from nowhere else
    ///    (28.10.7), which the write the timeout used to make here ignored;
    ///  * `Recovery` (kvasir_devices' kvasir/Devices/I2C/LineRecovery.hpp): SDA held low on an
    ///    idle bus is noticed and clocked free, with a forced STOP as the last resort;
    ///  * the dead-bus watchdog: kDeadBusFailures bus faults in a row re-initialise the block,
    ///    then run a recovery;
    ///  * the fault counters, lastTimeout() / logLastTimeout() and takeLatency().
    ///
    /// The SERCOM's hardware time-outs (CTRLA.LOWTOUTEN, SEXTTOEN, MEXTTOEN, INACTOUT) are
    /// left off: they run from GCLK_SERCOM_SLOW, which "must be configured to use a 32KHz
    /// oscillator" (28.6.3.1) - a clock this driver cannot ask every firmware for.
    enum class I2CRequestResult : std::uint8_t { failed, notAcknowledged, succeeded };

    /// The result on a bus with `cancellable` or `requestDeadlines`: the first three as I2CRequestResult.
    enum class I2CRequestResultTracked : std::uint8_t {
        failed,
        notAcknowledged,
        succeeded,
        cancelled,
        timedOut
    };

    /// The request of a bus with `cancellable` / `requestDeadlines` (kvasir_devices BusTypes.hpp): I2CRequest's
    /// fields with the tracked result, a ticket, the tombstone flag and an optional deadline. A type of its own, so a
    /// bus without the features keeps I2CRequest and its names.
    template<std::size_t CallbackSize, typename TimePoint, bool Deadlines>
    struct I2CTrackedRequest {
        static constexpr std::size_t MaxPrefix = 2;

        std::uint8_t                                                address{};
        std::uint8_t                                                prefixBytes{};
        std::array<std::byte, MaxPrefix>                            prefix{};
        std::span<std::byte const>                                  sendData{};
        std::span<std::byte>                                        receiveData{};
        StaticFunction<void(I2CRequestResultTracked), CallbackSize> callback{};
        std::uint16_t                                               ticket{};
        bool                                                        cancelled{};
        [[no_unique_address]] Kvasir::I2C::detail::IfFeature<Deadlines, TimePoint, 30> deadline{
          TimePoint::max()};

        template<std::integral... B>
            requires(sizeof...(B) <= MaxPrefix)
        constexpr void setPrefix(B... b) {
            prefix      = {std::byte{static_cast<std::uint8_t>(b)}...};
            prefixBytes = sizeof...(B);
        }

        [[nodiscard]] constexpr std::size_t sendBytes() const {
            return prefixBytes + sendData.size();
        }

        [[nodiscard]] constexpr std::byte sendByte(std::size_t i) const {
            return i < prefixBytes ? prefix[i] : sendData[i - prefixBytes];
        }
    };

    template<std::size_t CallbackSize,
             typename TimePoint,
             bool Deadlines,
             typename Timing,
             Timing BusDefault>
    struct I2CTimedTrackedRequest : I2CTrackedRequest<CallbackSize, TimePoint, Deadlines> {
        Timing timing{BusDefault};
    };

    template<std::size_t CallbackSize>
    struct I2CRequest {
        static constexpr std::size_t MaxPrefix = 2;

        std::uint8_t address{};
        /// Sent ahead of sendData in the same write, with no START or STOP between them: a
        /// register address, an EEPROM memory address, a display's control byte. Up to two
        /// bytes, kept in the padding behind `address`, so the request stays the size it was
        /// (the RP driver's request is the same struct).
        std::uint8_t                                         prefixBytes{};
        std::array<std::byte, MaxPrefix>                     prefix{};
        std::span<std::byte const>                           sendData{};
        std::span<std::byte>                                 receiveData{};
        StaticFunction<void(I2CRequestResult), CallbackSize> callback{};

        template<std::integral... B>
            requires(sizeof...(B) <= MaxPrefix)
        constexpr void setPrefix(B... b) {
            prefix      = {std::byte{static_cast<std::uint8_t>(b)}...};
            prefixBytes = sizeof...(B);
        }

        /// Everything that goes out after the address, prefix first.
        [[nodiscard]] constexpr std::size_t sendBytes() const {
            return prefixBytes + sendData.size();
        }

        [[nodiscard]] constexpr std::byte sendByte(std::size_t i) const {
            return i < prefixBytes ? prefix[i] : sendData[i - prefixBytes];
        }
    };

    static_assert(sizeof(void*) != 4 || sizeof(I2CRequest<16>) == 40,
                  "the prefix must stay in the padding behind address");

    /// The request of a bus with I2CConfig::perDeviceClock: the device's BAUD ride along.
    /// A type of its own, not a parameter of I2CRequest, so that a bus without the feature
    /// keeps the very same type (and names: a sanitize image hashes them).
    ///
    /// `BusDefault` is the timing of the bus's own rate: a request nobody gave a timing - the bus
    /// scan's probes, a raw request of the application's - goes out at baudRate, never at the
    /// all-zero counts a value-initialised member would be.
    template<std::size_t CallbackSize, typename Timing, Timing BusDefault>
    struct I2CTimedRequest : I2CRequest<CallbackSize> {
        Timing timing{BusDefault};
    };

    namespace Detail {
        /// Not constexpr: reaching one in timing() is the compile error that says why.
        inline void i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus() {}

        inline void i2cDeviceClockBelowMinBaudRateOfTheBus() {}
    }   // namespace Detail

    template<typename I2CConfig,
             typename Clock,
             std::size_t QueueDepth_   = 8,
             std::size_t CallbackSize_ = 16>
    struct I2CBehaviorQueued : Detail::I2CBase<I2CConfig> {
        static constexpr std::size_t QueueDepth   = QueueDepth_;
        static constexpr std::size_t CallbackSize = CallbackSize_;
        /// What the bus is clocked at, re-exported from the config so a Kvasir::I2C::Bus can
        /// report its periodic traffic as a fraction of the bandwidth rather than a bit rate.
        static constexpr auto BaudRate = I2CConfig::baudRate;

        using base     = Detail::I2CBase<I2CConfig>;
        using Regs     = typename base::Regs;
        using tp       = typename Clock::time_point;
        using Recovery = Kvasir::I2C::LineRecovery<base, Clock>;

        /// Tickets / cancel() and per-request deadlines (I2CConfig::cancellable, ::requestDeadlines).
        static constexpr bool Cancellable = base::I2CConfig::cancellable;
        static constexpr bool Deadlines   = base::I2CConfig::requestDeadlines;
        static constexpr bool Tracked     = Cancellable || Deadlines;
        using Result = std::conditional_t<Tracked, I2CRequestResultTracked, I2CRequestResult>;

        /// Each device at its own clock (I2CConfig::perDeviceClock). Off, a request has no
        /// timing member and startNext_() never looks at BAUD: nothing of this is in the image.
        static constexpr bool PerDeviceClock = base::I2CConfig::perDeviceClock;
        /// Count the requests accepted for the wire (I2CConfig::countTransfers, transfers()).
        static constexpr bool CountTransfers = base::I2CConfig::countTransfers;
        using ClockTiming                    = Detail::ClockTiming;
        /// The timing of baudRate itself, what a request without one of its own carries.
        static constexpr ClockTiming DefaultTiming = [] {
            if constexpr(PerDeviceClock) {
                return Detail::clockTiming(I2CConfig::clockSpeed,
                                           static_cast<std::uint32_t>(BaudRate),
                                           base::I2CConfig::maxBaudRateError);
            } else {
                return ClockTiming{};
            }
        }();
        using Request = std::conditional_t<
          Tracked,
          std::conditional_t<
            PerDeviceClock,
            I2CTimedTrackedRequest<CallbackSize, tp, Deadlines, ClockTiming, DefaultTiming>,
            I2CTrackedRequest<CallbackSize, tp, Deadlines>>,
          std::conditional_t<PerDeviceClock,
                             I2CTimedRequest<CallbackSize, ClockTiming, DefaultTiming>,
                             I2CRequest<CallbackSize>>>;

        /// What a request carries for a device clocked at most at `hz` (kvasir_devices'
        /// Device fills it in from Config::BusClock / Chip::I2cMaxClock): BAUD for
        /// min(hz, baudRate). On a bus without perDeviceClock a device slower than the bus is
        /// a compile error, and the answer is nothing.
        static consteval auto timing(std::uint32_t hz) {
            if constexpr(PerDeviceClock) {
                auto const f = std::min(hz, static_cast<std::uint32_t>(BaudRate));
                if(f < base::I2CConfig::minBaudRate) {
                    Detail::i2cDeviceClockBelowMinBaudRateOfTheBus();
                }
                return Detail::clockTiming(I2CConfig::clockSpeed,
                                           f,
                                           base::I2CConfig::maxBaudRateError);
            } else {
                if(hz < BaudRate) {
                    Detail::i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus();
                }
                return std::false_type{};
            }
        }

        /// Bus faults in an unbroken row that mean the bus is dead. A success resets it, and
        /// so does a NAK: the address went out and nobody took it, which is a working wire
        /// with nothing at that address -- an empty header must never look like a dead bus.
        static constexpr std::uint32_t kDeadBusFailures = 40;

        /// The bus state is UNKNOWN after the enable and nothing starts from there; with one
        /// master it is idle (28.6.2.3). Startup calls this after initStepPeripheryEnable.
        static void runtimeInit() {
            apply(write(Regs::STATUS::BUSSTATEValC::idle));
            waitSync_();
        }

        /// Every submitted request gets exactly one callback: the one on the wire and the
        /// ones queued behind it are failed here, before the block is reset, so nothing waits
        /// on a request the reset threw away. A callback that submits from here only queues:
        /// nothing starts on the block before it is back.
        static void reset() {
            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            auto const outer = resetting_;
            resetting_       = true;
            failActive_();
            Recovery::resetState();
            drainQueueWithFailure_();
            resetting_ = outer;
            reinit_();
        }

        /// False when the queue is full; the caller tries again next turn.
        static bool submit(Request const& req) {
            if(requestQueue_.size() >= requestQueue_.max_size()) { return false; }
            requestQueue_.push(req);

            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            // inside the masked window: submit() also runs from completion callbacks in the ISR
            if constexpr(CountTransfers) { ++transfers_; }
            tryStart_(Clock::now());
            // From a callback reset() or requestRecovery() runs, the interrupt stays masked:
            // they unmask it when they are done.
            if(!resetting_) { apply(Nvic::makeEnable(typename base::InterruptIndexs{})); }
            return true;
        }

        /// submit(), with a ticket for cancel(); an invalid ticket (refused, no callback) when submit() would be false.
        static Bus::Ticket submitTracked(Request req)
            requires(Tracked)
        {
            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            req.ticket = Bus::nextTicket(ticketCounter_);
            if(!resetting_) { apply(Nvic::makeEnable(typename base::InterruptIndexs{})); }
            return submit(req) ? Bus::Ticket{req.ticket} : Bus::Ticket{};
        }

        /// removed: queued, never started - its callback (cancelled) ran before this returns. stopping: on the
        /// wire - the SERCOM takes a command only while MB or SB is set (D21 md l.23999, C21 md l.28087), so the
        /// interrupt at the next byte boundary issues NACK + STOP and completes it (cancelled); its buffers belong
        /// to the bus until then. A byte boundary that never comes is the transfer timeout's (SWRST). tooLate:
        /// completed already, or an unknown ticket.
        static Bus::Cancel cancel(Bus::Ticket t)
            requires(Cancellable)
        {
            if(!t.valid()) { return Bus::Cancel::tooLate; }
            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            auto r = Bus::Cancel::tooLate;
            if(active_ && currentRequest_.ticket == t.id) {
                if(!cancelPending_) {
                    cancelPending_ = true;
                    stopAs_        = Result::cancelled;
                }
                r = Bus::Cancel::stopping;
            } else {
                requestQueue_.forEachQueued([&](Request& q) {
                    if(q.ticket == t.id && !q.cancelled) {
                        q.cancelled = true;
                        if(q.callback) { q.callback(Result::cancelled); }
                        r = Bus::Cancel::removed;
                    }
                });
            }
            if(!resetting_) { apply(Nvic::makeEnable(typename base::InterruptIndexs{})); }
            return r;
        }

        /// Once per main-loop turn per bus. Owns the timeout, the recovery's clock, the
        /// dead-bus watchdog, and restarts the queue if a submit happened to lose the race
        /// with a completing transfer.
        static void handler() {
            auto const now = Clock::now();

            // Recovery state machine -- highest priority, blocks normal operation
            auto const rv = Recovery::tick(now);
            if(rv == Recovery::TickResult::needsReinit) {
                reset();
                apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
                [[maybe_unused]] auto const recoveredLine
                  = faultLog_.allow(faultKey_(Fault::recovered), now);
                apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
                // The line states right after the sequence are the diagnosis: both high
                // and the bus is back, SDA low means a slave still holds data, SCL low
                // means no master can help.
                KVASIR_LOG_LIMITED(recoveredLine,
                                   UC_LOG_W,
                                   "i2c{} recovery #{} complete -- SDA {}, SCL {}, {} of 9 "
                                   "clocks unused, {} forced STOP(s) so far",
                                   base::Instance,
                                   Recovery::recoveries(),
                                   std::string_view{Recovery::sdaIsHigh() ? "high" : "LOW"},
                                   std::string_view{Recovery::sclIsHigh() ? "high" : "LOW"},
                                   Recovery::clocksLeft(),
                                   Recovery::forcedStops());
                return;
            }
            if(rv == Recovery::TickResult::busy) { return; }

            // What the ISR writes too, taken in one go with its interrupt masked.
            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            auto const droppedFaults = faultLog_.takeSummary(now);
            auto const droppedNaks   = nakLog_.takeSummary(now);
            bool const deadBus       = consecutiveFailures_ >= kDeadBusFailures;
            if(deadBus) { consecutiveFailures_ = 0; }
            bool const settled = Recovery::isPastSettle(now);
            apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
            if(droppedFaults != 0) {
                UC_LOG_W("i2c{} +{} faults not logged", base::Instance, droppedFaults);
            }
            if(droppedNaks != 0) {
                UC_LOG_D("i2c{} +{} address NAKs not logged", base::Instance, droppedNaks);
            }

            // Dead-bus watchdog: line-state checks miss failures on a healthy wire, so this
            // asks instead whether anything works at all. A NAK counts as a completed
            // transfer, so absent devices cannot trip it.
            if(deadBus) {
                ++resuscitations_;
                // Alternate the cheap fix and the expensive one.
                if((resuscitations_ % 2U) == 1U) {
                    UC_LOG_W(
                      "i2c{} {} transfers failed in a row with no success -- "
                      "re-initialising the peripheral (SDA {}, SCL {})",
                      base::Instance,
                      kDeadBusFailures,
                      std::string_view{Recovery::sdaIsHigh() ? "high" : "LOW"},
                      std::string_view{Recovery::sclIsHigh() ? "high" : "LOW"});
                    reset();
                } else {
                    UC_LOG_W("i2c{} still dead after a re-initialise -- full bus recovery",
                             base::Instance);
                    requestRecovery();
                }
                return;
            }

            // Post-abort settle: the bus was sick, wait before starting the next transaction
            if(!settled) { return; }

            if constexpr(Deadlines) {
                apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
                expireDeadlines_(now);
                apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
            }

            // Nothing active: a stuck SDA is looked for on every idle turn, not only when a
            // request waits, because a stuck bus parks every device as absent and that
            // leaves the queue empty for seconds. Then whatever waits may start.
            if(!active_) {
                if(Recovery::checkBusStuck(now)) {
                    // The recovery takes the pins; whatever is queued is failed like on the RP.
                    apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
                    auto const outer = resetting_;
                    resetting_       = true;
                    drainQueueWithFailure_();
                    resetting_ = outer;
                    apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
                    return;
                }
                apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
                tryStart_(now);
                apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
                return;
            }

            // Active transaction: check for timeout
            if(now > timeoutTime_) {
                apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
                if(active_ && now > timeoutTime_) {
                    ++timeouts_;
                    lastTimeout_ = snapshot_();
                    KVASIR_LOG_LIMITED(faultLog_.allow(faultKey_(Fault::timeout), now),
                                       UC_LOG_W,
                                       "i2c{} timeout addr={:#04x} STATUS {:#06x}",
                                       base::Instance,
                                       currentRequest_.address,
                                       lastTimeout_.status);
                    // The block may be anywhere in a byte with the clock held: only the
                    // software reset is sure to let go of the lines (see the header).
                    reinit_(false);
                    // a stop the caller asked for (cancel, deadline) whose byte boundary never came: that outcome
                    complete_(stoppingResult_(Result::failed));
                }
                apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
            }
        }

        // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() calls it (StartUp/Hooks.hpp);
        // a firmware that runs the hook must not also call handler() by hand
        using Extends
          = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &I2CBehaviorQueued::handler>;

        /// A full bus recovery sequence, non-blocking. Safe to call at any time. Any active
        /// transaction is failed at once, and so is everything queued.
        static void requestRecovery() {
            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            auto const outer = resetting_;
            resetting_       = true;
            failActive_();
            drainQueueWithFailure_();
            Recovery::begin();
            resetting_ = outer;
            if(!outer) { apply(Nvic::makeEnable(typename base::InterruptIndexs{})); }
        }

        static bool isRecovering() { return Recovery::isActive(); }

        /// Failures with no success between them, and how often the watchdog stepped in.
        static std::uint32_t consecutiveFailures() { return consecutiveFailures_; }

        static std::uint32_t resuscitations() { return resuscitations_; }

        // -- fault counters: numbers, because a rate-limited log cannot say how often -------

        /// What the block looked like when a transaction timed out, read before the reset.
        struct TimeoutSnapshot {
            std::uint16_t status{};      ///< STATUS: CLKHOLD, BUSSTATE, RXNACK, ARBLOST, BUSERR
            std::uint8_t  intFlag{};     ///< INTFLAG: ERROR, SB, MB
            std::uint8_t  intEnable{};   ///< INTENSET
            std::uint32_t ctrlA{};
            std::uint32_t syncBusy{};
            std::uint16_t sent{};
            std::uint16_t toSend{};
            std::uint16_t received{};
            std::uint16_t toReceive{};
            std::uint8_t  address{};
            std::uint8_t  state{};   ///< 0 idle, 1 sending, 2 receiving
            /// The lines themselves: SCL low with STATUS.CLKHOLD set is the master waiting for
            /// its interrupt to be served, SCL low without it is a part holding the clock.
            bool          sdaHigh{};
            bool          sclHigh{};
            std::uint32_t isrEntries{};   ///< interrupt entries since this request started
            std::uint32_t usSinceIsr{};   ///< since the last interrupt entry, of any request
            std::uint32_t usAge{};        ///< since the request started
        };

        /// Transactions the handler gave up on after calcTransferTimeout().
        static std::uint32_t timeouts() { return timeouts_; }

        /// Failed transfers after which SDA read low with the bus state idle. Each one defers
        /// the next start; whether the line is really held is the idle watchdog's to decide,
        /// and Recovery::idleStuck() counts the times it did.
        static std::uint32_t sdaLowAfterAbort() { return sdaLowAfterAbort_; }

        /// Queued requests failed without going on the wire (a recovery emptying the queue).
        static std::uint32_t drainedRequests() { return drainedRequests_; }

        /// Times a wait for the block's register synchronisation (SYNCBUSY) ran out of spins.
        /// The name is the RP driver's, where the wait is for IC_ENABLE_STATUS: the same
        /// kind of event, a block that does not follow its own handshake.
        static std::uint32_t disableWaitsExhausted() { return syncWaitsExhausted_; }

        /// Requests accepted for the wire since boot: what tells "nothing talks on this bus"
        /// from "nothing went wrong on it" (I2CConfig::countTransfers only, 0 without it).
        static std::uint32_t transfers() {
            if constexpr(CountTransfers) {
                return transfers_;
            } else {
                return 0;
            }
        }

        /// Times startNext_() rewrote BAUD for a device at another clock (perDeviceClock only, 0
        /// without it).
        static std::uint32_t clockSwitches() {
            if constexpr(PerDeviceClock) {
                return clockSwitches_;
            } else {
                return 0;
            }
        }

        static TimeoutSnapshot const& lastTimeout() { return lastTimeout_; }

        /// lastTimeout() as a log line (state 1 is sending, 2 receiving; sent/received are
        /// bytes done of bytes asked).
        static void logLastTimeout() {
            [[maybe_unused]] auto const& t = lastTimeout_;
            UC_LOG_W(
              "i2c{} last timeout: addr {:#04x}, state {}, sent {}/{}, received {}/{}, "
              "STATUS {:#06x}, INTFLAG {:#04x}, INTENSET {:#04x}, CTRLA {:#010x}, SYNCBUSY "
              "{:#x}, SDA {}, SCL {}, {} interrupt(s) in the request, last {} us before, "
              "request {} us old",
              base::Instance,
              t.address,
              t.state,
              t.sent,
              t.toSend,
              t.received,
              t.toReceive,
              t.status,
              t.intFlag,
              t.intEnable,
              t.ctrlA,
              t.syncBusy,
              std::string_view{t.sdaHigh ? "high" : "LOW"},
              std::string_view{t.sclHigh ? "high" : "LOW"},
              t.isrEntries,
              t.usSinceIsr,
              t.usAge);
        }

        /// Interrupt entries that found nothing to do for the state the request was in: no
        /// request at all, a send without INTFLAG.MB, a receive without INTFLAG.SB.
        static std::uint32_t spuriousIsr() { return spuriousIsr_; }

        /// The longest a request waited for an interrupt since the last call: from its start to
        /// its first entry, and between two entries of one request. Taken and cleared.
        struct Latency {
            std::uint32_t firstIsrUs{};
            std::uint32_t isrGapUs{};
        };

        // The interrupt keeps its waits in the clock's own ticks, 32 bits, saturated (2^32 ticks
        // are 28 s or more: "very long" for a latency). They become microseconds where they
        // are read: in the handler that was a 64-bit division per interrupt.
        static constexpr std::uint32_t ticks32_(typename Clock::duration d) {
            auto const n = static_cast<std::uint64_t>(d.count());
            return n > 0xFFFF'FFFFU ? 0xFFFF'FFFFU : static_cast<std::uint32_t>(n);
        }

        static constexpr std::uint32_t usOfTicks_(std::uint32_t ticks) {
            return static_cast<std::uint32_t>(
              std::chrono::duration_cast<std::chrono::microseconds>(typename Clock::duration{ticks})
                .count());
        }

        static Latency takeLatency() {
            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            std::uint32_t const first = longestFirstIsrTicks_;
            std::uint32_t const gap   = longestIsrGapTicks_;
            longestFirstIsrTicks_     = 0;
            longestIsrGapTicks_       = 0;
            apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
            return Latency{usOfTicks_(first), usOfTicks_(gap)};
        }

        // INTFLAG.MB and SB are cleared by the next operation itself -- a write of ADDR or
        // DATA, or a command in CTRLB.CMD (28.10.6) -- and are NOT written here after it: a
        // write of '1' that came late (this interrupt preempted for longer than a byte takes,
        // 22 us at 400 kHz) would clear the flag of the byte that has just gone out, and the
        // transfer would stand until its timeout. Only ERROR needs the write.
        static void onIsr() {
            {
                auto const now     = Clock::now();
                auto const waited  = ticks32_(now - (isrEntries_ == 0 ? requestStart_ : lastIsr_));
                auto&      longest = isrEntries_ == 0 ? longestFirstIsrTicks_ : longestIsrGapTicks_;
                if(active_ && waited > longest) { longest = waited; }
                ++isrEntries_;
                lastIsr_ = now;
            }
            auto const flags  = Kvasir::Register::get<0>(apply(read(Regs::INTFLAG::FULLREGISTER)));
            auto const status = Kvasir::Register::get<0>(apply(read(Regs::STATUS::FULLREGISTER)));
            bool const error  = (flags & ErrorFlag) != 0;
            bool const mb     = (flags & MbFlag) != 0;
            bool const sb     = (flags & SbFlag) != 0;
            bool const rxnack = (status & RxNackBit) != 0;
            if(error) { apply(set(Regs::INTFLAG::error)); }

            if(!active_) {
                // Nothing running: whatever raised it is over. The flags are written here,
                // since no operation follows that would clear them.
                ++spuriousIsr_;
                apply(set(Regs::INTFLAG::mb, Regs::INTFLAG::sb));
                return;
            }

            // our own stop (cancel, deadline), taken at this byte boundary: NACK + STOP, not a fault. A NAK or a
            // bus error that came first keeps its own result (below).
            if constexpr(Tracked) {
                if(cancelPending_ && !error && !rxnack && (mb || sb)) {
                    apply(nack_stop);
                    complete_(stopAs_);
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
                bool const addressNak = isNak && (state_ == State::receiving || sendIndex_ == 0);
                auto const fault = state_ == State::sending ? Fault::abortSend : Fault::abortRecv;
                if(addressNak) {
                    KVASIR_LOG_LIMITED(nakLog_.allow(faultKey_(fault, status)),
                                       UC_LOG_D,
                                       "i2c{} {} addr={:#04x}: no ACK, nobody there",
                                       base::Instance,
                                       std::string_view{state_ == State::sending ? "send" : "recv"},
                                       currentRequest_.address);
                } else {
                    KVASIR_LOG_LIMITED(faultLog_.allow(faultKey_(fault, status)),
                                       UC_LOG_W,
                                       "i2c{} abort {} addr={:#04x} STATUS {:#06x}{}",
                                       base::Instance,
                                       std::string_view{state_ == State::sending ? "send" : "recv"},
                                       currentRequest_.address,
                                       status,
                                       std::string_view{isNak ? " (NAK)" : ""});
                }
                // A STOP, which also clears MB/SB and lets go of the clock. Without the bus
                // (arbitration lost) the command does nothing, so the flags are written too.
                apply(nack_stop);
                if((status & ArbLostBit) != 0) { apply(set(Regs::INTFLAG::mb, Regs::INTFLAG::sb)); }
                complete_(isNak ? Result::notAcknowledged : Result::failed);
                return;
            }

            if(state_ == State::sending) {
                if(!mb) {
                    ++spuriousIsr_;
                    return;
                }
                if(sendIndex_ < currentRequest_.sendBytes()) {
                    apply(write(Regs::DATA::data,
                                static_cast<std::uint8_t>(currentRequest_.sendByte(sendIndex_))));
                    ++sendIndex_;
                } else if(!currentRequest_.receiveData.empty()) {
                    // repeated START into the read phase
                    state_ = State::receiving;
                    apply(
                      write(Regs::ADDR::addr, (unsigned(currentRequest_.address) << 1U) | 0x01U));
                } else {
                    apply(ack_stop);
                    complete_(Result::succeeded);
                }
            } else {   // receiving
                if(!sb) {
                    ++spuriousIsr_;
                    return;
                }
                currentRequest_.receiveData[receivedCount_]
                  = static_cast<std::byte>(Kvasir::Register::get<0>(apply(read(Regs::DATA::data))));
                ++receivedCount_;
                if(receivedCount_ < currentRequest_.receiveData.size()) {
                    apply(ack_byte_read);
                } else {
                    apply(nack_stop);
                    complete_(Result::succeeded);
                }
            }
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));

    private:
        enum class State : std::uint8_t { idle, sending, receiving };

        enum class Fault : std::uint8_t {
            abortSend = 1,
            abortRecv,
            timeout,
            recovered,
        };

        // INTFLAG and STATUS bit positions (DS40001882, 28.10.6 and 28.10.7).
        static constexpr std::uint8_t  MbFlag     = 1U << 0U;
        static constexpr std::uint8_t  SbFlag     = 1U << 1U;
        static constexpr std::uint8_t  ErrorFlag  = 1U << 7U;
        static constexpr std::uint16_t ArbLostBit = 1U << 1U;
        static constexpr std::uint16_t RxNackBit  = 1U << 2U;
        static_assert(Regs::INTFLAG::mb.Mask == MbFlag && Regs::INTFLAG::sb.Mask == SbFlag
                      && Regs::INTFLAG::error.Mask == ErrorFlag);
        static_assert(Regs::STATUS::arblost.Mask == ArbLostBit
                      && Regs::STATUS::rxnack.Mask == RxNackBit);

        static constexpr auto ack_byte_read
          = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_ack),
                                               write(Regs::CTRLB::CMDValC::ack_byte_read)));
        static constexpr auto nack_stop
          = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_nack),
                                               write(Regs::CTRLB::CMDValC::ack_stop)));
        static constexpr auto ack_stop
          = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_ack),
                                               write(Regs::CTRLB::CMDValC::ack_stop)));

        // One kind of fault at one address with one cause, for the log rate limiter.
        static std::uint32_t faultKey_(Fault         kind,
                                       std::uint16_t cause = 0) {
            return Kvasir::rateLimitKey(kind, currentRequest_.address, cause);
        }

        /// Spin until the block has taken what was written to it (SYNCBUSY: SWRST, ENABLE,
        /// SYSOP). Bounded: a block that never gets there is the dead-bus watchdog's.
        static void waitSync_() {
            for(std::uint32_t spins = 0; spins < 100'000U; ++spins) {
                if(Kvasir::Register::get<0>(apply(read(Regs::SYNCBUSY::FULLREGISTER))) == 0) {
                    return;
                }
            }
            ++syncWaitsExhausted_;
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
            if(timingValid_ && t == timing_) { return; }
            for(std::uint32_t spins = 0; spins < 100'000U; ++spins) {
                if(fieldEquals(Regs::STATUS::BUSSTATEValC::idle)) { break; }
            }
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
            timing_      = t;
            timingValid_ = true;
            ++clockSwitches_;
        }

        /// The block through a software reset and set up again, bus state idle. With
        /// `unmask` the interrupt is enabled at the end (initStepPeripheryEnable does it);
        /// the timeout path keeps it masked until its request is completed.
        static void reinit_(bool unmask = true) {
            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            apply(set(Regs::CTRLA::swrst));
            waitSync_();
            apply(base::initStepPeripheryConfig);
            // The block is back at baudRate's BAUD: the next request writes its own.
            if constexpr(PerDeviceClock) { timingValid_ = false; }
            apply(base::initStepInterruptConfig);
            apply(set(Regs::CTRLA::enable));
            waitSync_();
            runtimeInit();
            if(unmask && !resetting_) { apply(Nvic::makeEnable(typename base::InterruptIndexs{})); }
        }

        static TimeoutSnapshot snapshot_() {
            auto const now = Clock::now();
            return TimeoutSnapshot{
              .status     = Kvasir::Register::get<0>(apply(read(Regs::STATUS::FULLREGISTER))),
              .intFlag    = Kvasir::Register::get<0>(apply(read(Regs::INTFLAG::FULLREGISTER))),
              .intEnable  = Kvasir::Register::get<0>(apply(read(Regs::INTENSET::FULLREGISTER))),
              .ctrlA      = Kvasir::Register::get<0>(apply(read(Regs::CTRLA::FULLREGISTER))),
              .syncBusy   = Kvasir::Register::get<0>(apply(read(Regs::SYNCBUSY::FULLREGISTER))),
              .sent       = static_cast<std::uint16_t>(sendIndex_),
              .toSend     = static_cast<std::uint16_t>(currentRequest_.sendBytes()),
              .received   = static_cast<std::uint16_t>(receivedCount_),
              .toReceive  = static_cast<std::uint16_t>(currentRequest_.receiveData.size()),
              .address    = currentRequest_.address,
              .state      = static_cast<std::uint8_t>(state_),
              .sdaHigh    = Recovery::sdaIsHigh(),
              .sclHigh    = Recovery::sclIsHigh(),
              .isrEntries = isrEntries_,
              .usSinceIsr = static_cast<std::uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - lastIsr_).count()),
              .usAge = static_cast<std::uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - requestStart_).count()),
            };
        }

        /// Finish the running request, hand the result to its callback, then start whatever
        /// the callback (or anyone else) queued behind it. Interrupt masked, or in the ISR.
        static void complete_(Result r) {
            if constexpr(Tracked) { cancelPending_ = false; }
            active_ = false;
            state_  = State::idle;

            // A NAK is a transfer that completed: the address went out and nobody took it,
            // which says the wire works. Only a bus fault counts towards a dead bus.
            if(r == Result::failed) {
                if(consecutiveFailures_ != std::numeric_limits<std::uint32_t>::max()) {
                    ++consecutiveFailures_;
                }
            } else {
                consecutiveFailures_ = 0;
            }

            auto cb = currentRequest_.callback;
            if(cb) { cb(r); }
            if(active_) { return; }   // the callback's submit started the next one

            if(r != Result::succeeded) {
                // The STOP of a NAKed or failed transfer is still going out while the block
                // owns the bus: the normal path, not logged, the next start waits a moment.
                auto const now = Clock::now();
                if(!fieldEquals(Regs::STATUS::BUSSTATEValC::idle)) {
                    Recovery::deferSettle(now + std::chrono::milliseconds{1});
                    return;
                }
                // Idle and SDA low: one look is not a stuck bus (slow pull-ups on a long
                // wire), so it only defers; the idle watchdog decides after kStuckThreshold.
                if(!Recovery::sdaIsHigh()) {
                    ++sdaLowAfterAbort_;
                    Recovery::deferSettle(now + std::chrono::milliseconds{1});
                    return;
                }
            }
            startNext_();
        }

        /// The request on the wire, if there is one, is over: its callback runs once, as
        /// failed. The block is left for the caller to reset, and the dead-bus counter is not
        /// touched -- this is the bus giving up on a request, not a request reporting on the
        /// bus. Interrupts disabled.
        static void failActive_() {
            if(!active_) { return; }
            active_ = false;
            state_  = State::idle;
            if(currentRequest_.callback) { currentRequest_.callback(Result::failed); }
        }

        /// Start the next queued request if the bus may take one: no reset() or
        /// requestRecovery() failing requests, nothing on the wire, no recovery running, the
        /// post-abort settle over, SDA released, and something waiting. Interrupts disabled.
        static bool tryStart_(tp now) {
            if(resetting_ || active_ || Recovery::isActive() || !Recovery::isPastSettle(now)) {
                return false;
            }
            if(requestQueue_.empty() || !Recovery::sdaIsHigh()) { return false; }
            startNext_();
            return true;
        }

        static void startNext_() {
            if constexpr(!Tracked) {
                if(requestQueue_.empty()) { return; }
                currentRequest_ = requestQueue_.front();
                requestQueue_.pop();
            } else {
                while(true) {
                    if(requestQueue_.empty()) { return; }
                    currentRequest_ = requestQueue_.front();
                    requestQueue_.pop();
                    if(currentRequest_.cancelled) { continue; }   // a tombstone: dropped
                    if constexpr(Deadlines) {
                        if(Clock::now()
                           > currentRequest_.deadline) {   // out of time before it started
                            ++drainedRequests_;
                            if(currentRequest_.callback) {
                                currentRequest_.callback(Result::timedOut);
                            }
                            continue;
                        }
                    }
                    break;
                }
            }

            sendIndex_     = 0;
            receivedCount_ = 0;
            isrEntries_    = 0;
            // Every start goes through here -- submit(), handler(), and the ISR chaining the
            // next request after a success -- and each one is the bus seen free (LineRecovery
            // noteBusFree(): the stuck timer must not outlive the transfers, as on the RP).
            Recovery::noteBusFree();
            requestStart_ = Clock::now();
            if constexpr(PerDeviceClock) {
                timeoutTime_ = requestStart_
                             + base::calcTransferTimeout(currentRequest_.sendBytes()
                                                           + currentRequest_.receiveData.size(),
                                                         currentRequest_.timing.usPerByte);
                applyTiming_(currentRequest_.timing);
            } else {
                timeoutTime_ = requestStart_
                             + base::calcTransferTimeout(currentRequest_.sendBytes()
                                                         + currentRequest_.receiveData.size());
            }
            active_ = true;

            if(currentRequest_.sendBytes() != 0) {
                state_ = State::sending;
                apply(write(Regs::ADDR::addr, unsigned(currentRequest_.address) << 1U));
            } else if(!currentRequest_.receiveData.empty()) {
                state_ = State::receiving;
                apply(write(Regs::ADDR::addr, (unsigned(currentRequest_.address) << 1U) | 0x01U));
            } else {
                // Nothing to transfer: not something a driver asks for, but do not wedge.
                complete_(Result::failed);
            }
        }

        static void drainQueueWithFailure_() {
            while(!requestQueue_.empty()) {
                auto req = requestQueue_.front();
                requestQueue_.pop();
                if constexpr(Tracked) {
                    if(req.cancelled) { continue; }   // a tombstone: its callback has run
                }
                ++drainedRequests_;
                if(req.callback) { req.callback(Result::failed); }
            }
        }

        // only odr-used (so only defined) on a tracked bus
        inline static std::uint16_t ticketCounter_{};
        inline static bool          cancelPending_{};   // stop at the next byte boundary
        inline static Result        stopAs_{};   // cancelled or timedOut, for that completion

        static Result stoppingResult_(Result otherwise) {
            if constexpr(Tracked) {
                if(cancelPending_) { return stopAs_; }
            }
            return otherwise;
        }

        /// Interrupt masked: the request on the wire past its deadline stops at the next byte boundary
        /// (timedOut); queued ones past theirs become tombstones with their callback (timedOut).
        static void expireDeadlines_(tp now)
            requires(Deadlines)
        {
            if(active_ && !cancelPending_ && now > currentRequest_.deadline) {
                cancelPending_ = true;
                stopAs_        = Result::timedOut;
            }
            requestQueue_.forEachQueued([&](Request& q) {
                if(!q.cancelled && now > q.deadline) {
                    q.cancelled = true;
                    ++drainedRequests_;
                    if(q.callback) { q.callback(Result::timedOut); }
                }
            });
        }

        inline static Kvasir::Atomic::Queue<Request, QueueDepth> requestQueue_{};
        inline static Request                                    currentRequest_{};
        inline static bool                                       active_{false};
        inline static State                                      state_{State::idle};
        inline static std::size_t                                sendIndex_{0};
        inline static std::size_t                                receivedCount_{0};
        inline static tp                                         timeoutTime_{};
        // Set while reset() / requestRecovery() run the failure callbacks: a callback that
        // submits only queues, and leaves the interrupt masked.
        inline static bool resetting_{};
        // Dead-bus watchdog state.
        inline static std::uint32_t consecutiveFailures_{};
        inline static std::uint32_t resuscitations_{};
        // Fault logging goes through this: a bad bus faults on every transaction.
        inline static Kvasir::RateLimiter<Clock> faultLog_{};
        // address NAKs (debug): apart, so a scan does not spend the faults' budget and summary
        inline static Kvasir::LogRateLimiter<Clock> nakLog_{};

        inline static std::uint32_t   spuriousIsr_{};
        inline static std::uint32_t   isrEntries_{};
        inline static tp              lastIsr_{};
        inline static tp              requestStart_{};
        inline static std::uint32_t   longestFirstIsrTicks_{};
        inline static std::uint32_t   longestIsrGapTicks_{};
        inline static std::uint32_t   timeouts_{};
        inline static std::uint32_t   transfers_{};   // countTransfers only: never used without it
        inline static std::uint32_t   sdaLowAfterAbort_{};
        inline static std::uint32_t   drainedRequests_{};
        inline static std::uint32_t   syncWaitsExhausted_{};
        inline static TimeoutSnapshot lastTimeout_{};

        // perDeviceClock: the BAUD in the block, and whether it is known (not after a
        // reset). Members of a class template: never instantiated on a bus without it.
        inline static ClockTiming   timing_{};
        inline static bool          timingValid_{};
        inline static std::uint32_t clockSwitches_{};
    };

}}}   // namespace Kvasir::Sercom::I2C
