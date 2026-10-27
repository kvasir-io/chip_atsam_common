#pragma once

// The SAM USB controller (SAM D21, and the families that share the module: L21, D51, E5x) in
// device mode, as a backend of kvasir_devices' USB device. kvasir/Devices/USB/Backend.hpp has the
// contract; what differs from chip to chip is in "chip/Usb_Traits.hpp" (register names, pins,
// interrupt, clock gates, pad calibration).
//
//     namespace HW {
//     template<typename Clock, typename Config>
//     using UsbBackend = Kvasir::USB::Sam::Backend<Clock, Config>;
//     }
//     using Usb = Kvasir::USB::CdcAcm<HW::UsbBackend, Clock, UsbConfig>;
//
// THE CLOCK IS THE APPLICATION'S: GCLK_USB has to be 48 MHz +-0.25 % (SAM D21 datasheet
// DS40001882, 32.5.3), which on a board without a crystal means the DFLL48M in USB clock recovery
// mode (chip/DFLL.hpp, enableUsbRecovery()), and the clock settings have to route a generator to
// Traits::gclkChannel - and say so: `using Provides = brigand::list<Kvasir::Hal::Clk<
// Kvasir::Clocks::ClkUsb, 48'000'000>>;` (../Clocks.hpp), or Startup refuses the USB. Without that
// clock the module never finishes its reset; prepare() gives up after a while and logs it.
//
// How this controller differs from the RP one, all of it within the contract:
//   - An endpoint has one transfer with the controller at a time (queueDepth 1), in ordinary RAM
//     which the module reads and writes by itself (32.6.2.2): a packet in a buffer of this
//     driver's, or - bulk IN - up to MaxTransfer bytes where the caller keeps them, which the
//     module splits into packets itself (32.6.2.10), with one interrupt at the end.
//   - The data toggle is the hardware's (EPSTATUS.DTGLIN / DTGLOUT).
//   - Taking a packet back is done on return (asyncCancel = false).
//   - A bus reset disables every endpoint but 0 and every endpoint interrupt (32.6.2.4): the
//     device sets every endpoint up again on a bus reset, which is what puts them back.
//
// Besides what the device reads, the config struct may carry
//
//     isrPriority      1
//     usbMaxTransfer   512: how much one arm of a bulk IN endpoint carries (a multiple of 64; 64
//                      turns the multi-packet transfers off). The bulk endpoint's staging buffer
//                      is this large.
//     usbMaxReceive    512: the same for a bulk OUT endpoint, which the module fills by itself
//                      (32.6.2.8); 64 turns it off.
//
// Run on hardware (ATSAMD21G18A).

#include "../Clocks.hpp"
#include "chip/Usb_Traits.hpp"
#include "chip/WaitBounds.hpp"
#include "kvasir/Hal/DmaHandoff.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Util/Config.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <kvasir/Devices/Log.hpp>
#include <kvasir/Devices/USB/Backend.hpp>
#include <kvasir/Devices/USB/Config.hpp>
#include <kvasir/Devices/USB/Descriptors.hpp>
#include <optional>
#include <span>
#include <string_view>

namespace Kvasir::USB::Sam {
namespace detail {
    // One bank of an endpoint descriptor, as the module reads it out of RAM (32.13 "Endpoint
    // Descriptor Structure", 32.14 register summary): ADDR at 0x00, PCKSIZE at 0x04, EXTREG at
    // 0x08, STATUS_BK at 0x0A; 16 bytes per bank, bank 0 (OUT, SETUP) then bank 1 (IN), endpoint
    // n at n * 0x20.
    struct BankDescriptor {
        std::uint32_t addr;
        std::uint32_t pcksize;
        std::uint16_t extreg;
        std::uint8_t  statusBk;
        std::uint8_t  reserved[5];
    };

    static_assert(sizeof(BankDescriptor) == 16);

    struct EndpointDescriptor {
        BankDescriptor bank[2];
    };

    static_assert(sizeof(EndpointDescriptor) == 32);

    // PCKSIZE (32.15.2): BYTE_COUNT is bits 13:0, MULTI_PACKET_SIZE 27:14, SIZE 30:28 (3 = 64
    // bytes), AUTO_ZLP bit 31.
    inline constexpr std::uint32_t byteCountMask  = 0x3FFFU;
    inline constexpr unsigned      multiPacketPos = 14;
    inline constexpr std::uint32_t size64         = 3U << 28U;

    // Plain memory the module reads and writes (BYTE_COUNT, MULTI_PACKET_SIZE, STATUS_BK) while an
    // endpoint is armed, as it does the endpoint buffers: each bank and buffer is the core's until
    // the register write that arms it (Hal::handToHardware() before it) and again once its
    // TRCPT/RXSTP flag is seen (Hal::takeFromHardware() in dispatchEndpoint, before the device
    // hears of it; Kvasir/Hal/DmaHandoff.hpp). A count read while the module may still move it is
    // a Hal::loadHardwareField(). One table: there is one controller, whatever Clock and Config the
    // backend is instantiated with.
    alignas(4) inline EndpointDescriptor descriptorTable[Traits::endpointCount]{};
}   // namespace detail

template<typename Clock, typename ConfigT>
struct Backend {
private:
    using Regs      = typename Traits::Regs;
    using CfgTraits = Kvasir::USB::ConfigTraits<ConfigT>;

    static constexpr auto isrPriority = KVASIR_CONFIG_OR(ConfigT, isrPriority, 1);

    // SYNCBUSY clears once GCLK_USB has carried the write across (32.5.3: the generic clock is
    // asynchronous to the bus clock). With no GCLK_USB it never does, so the wait is bounded
    // (Sam::UsbSyncBound, whatever policy the application injects: prepare() reports it).
    static bool waitSync() {
        return static_cast<bool>(Kvasir::Register::waitUntil<Kvasir::Chip::Sam::UsbSyncBound,
                                                             Kvasir::Register::OnTimeout::Report>(
          Kvasir::Register::isClear(Regs::SYNCBUSY::FULLREGISTER)));
    }

    static inline bool clockMissing{false};

public:
    static constexpr std::size_t maxPacketSize = 64;
    static constexpr std::size_t endpointCount = Traits::endpointCount;

    static constexpr std::size_t configuredMaxTransfer
      = static_cast<std::size_t>(KVASIR_CONFIG_OR(ConfigT, usbMaxTransfer, std::size_t{512}));
    // BYTE_COUNT and MULTI_PACKET_SIZE are 14 bits (32.15.2).
    static_assert(configuredMaxTransfer % maxPacketSize == 0 && configuredMaxTransfer != 0
                    && configuredMaxTransfer < (1U << 14U),
                  "usbMaxTransfer: a multiple of 64 below 16384");
    static constexpr bool asyncCancel = false;

    static constexpr std::size_t configuredMaxReceive
      = static_cast<std::size_t>(KVASIR_CONFIG_OR(ConfigT, usbMaxReceive, std::size_t{512}));
    static_assert(configuredMaxReceive % maxPacketSize == 0 && configuredMaxReceive != 0
                    && configuredMaxReceive < (1U << 14U),
                  "usbMaxReceive: a multiple of 64 below 16384");

    template<std::size_t N, EndpointDirection Dir, EndpointTransferType Type>
    struct Endpoint {
    private:
        friend Backend;

        using E = typename Traits::template Endpoint<N>;

        static constexpr bool     isIn = Dir == EndpointDirection::In;
        static constexpr unsigned bank = isIn ? 1 : 0;

        static_assert(N < Traits::endpointCount,
                      "the controller has no such endpoint");

        // The packet, where the module reads or writes it: "Endpoint data can be placed anywhere
        // in the device RAM" (32.6.2.2). Word aligned, because the module is a bus master of its
        // own. A SETUP packet arrives in endpoint 0's OUT buffer.
        alignas(4) static inline std::array<std::byte,
                                            maxPacketSize> buffer{};

        static auto& bankDescriptor() { return detail::descriptorTable[N].bank[bank]; }

        static inline std::size_t sentOfCancelled_{};

        // This direction's bits: bank 1 / DTGLIN / STALLRQ1 / TRCPT1 for IN, bank 0 / DTGLOUT /
        // STALLRQ0 / TRCPT0 for OUT. EPSTATUSSET / EPSTATUSCLR are strobes (a '1' sets or clears
        // that bit, a '0' does nothing): write-only registers, so a set() of them is one store.
        template<typename InField, typename OutField>
        using PerDir = std::remove_cvref_t<std::conditional_t<isIn, InField, OutField>>;

        using Ready    = PerDir<decltype(E::EPSTATUS::bk1rdy), decltype(E::EPSTATUS::bk0rdy)>;
        using ReadySet = PerDir<decltype(E::EPSTATUSSET::bk1rdy), decltype(E::EPSTATUSSET::bk0rdy)>;
        using ReadyClear
          = PerDir<decltype(E::EPSTATUSCLR::bk1rdy), decltype(E::EPSTATUSCLR::bk0rdy)>;
        using ToggleClear
          = PerDir<decltype(E::EPSTATUSCLR::dtglin), decltype(E::EPSTATUSCLR::dtglout)>;
        using StallSet
          = PerDir<decltype(E::EPSTATUSSET::stallrq1), decltype(E::EPSTATUSSET::stallrq0)>;
        using StallClear
          = PerDir<decltype(E::EPSTATUSCLR::stallrq1), decltype(E::EPSTATUSCLR::stallrq0)>;
        using Complete = PerDir<decltype(E::EPINTFLAG::trcpt1), decltype(E::EPINTFLAG::trcpt0)>;
        using CompleteEnable
          = PerDir<decltype(E::EPINTENSET::trcpt1), decltype(E::EPINTENSET::trcpt0)>;

        // Bank 1 ready: a packet waits to be sent. Bank 0 ready: a packet was received, or
        // nothing is wanted - the module NAKs an OUT while it is set (32.6.2.7, 32.6.2.9). So an
        // IN endpoint is with the controller while BK1RDY is set, an OUT one while BK0RDY is clear.
        static bool withController() {
            bool const ready = apply(read(Ready{})) != 0;
            return isIn ? ready : !ready;
        }

        static bool completionPending() { return apply(read(Complete{})) != 0; }

        // EPCFG.EPTYPEn (32.12.1): 1 control, 2 isochronous, 3 bulk, 4 interrupt - one more than
        // the transfer type's code in a descriptor.
        static constexpr unsigned epType = static_cast<unsigned>(Type) + 1;

        static void configure() {
            auto& b = bankDescriptor();
            // Where the data is, is said when an endpoint is armed - but a SETUP comes unasked,
            // so endpoint 0's OUT bank always points at its buffer.
            if constexpr(Type == EndpointTransferType::Control && !isIn) {
                b.addr = reinterpret_cast<std::uint32_t>(buffer.data());
            } else {
                b.addr = 0;
            }
            b.pcksize  = detail::size64;
            b.extreg   = 0;
            b.statusBk = 0;
            Kvasir::Hal::handToHardware();

            // nothing to send, nothing wanted, DATA0 next, not stalled
            if constexpr(isIn) {
                apply(set(ReadyClear{}, ToggleClear{}, StallClear{}));
                apply(write(E::EPCFG::eptype1, epType));
            } else {
                apply(set(ReadySet{}));
                apply(set(ToggleClear{}, StallClear{}));
                apply(write(E::EPCFG::eptype0, epType));
            }
            // The flag is a strobe too (write '1' to clear); then the interrupt. Transfer
            // failures (every NAK is one) and sent STALLs stay off.
            apply(Kvasir::Register::reset(Complete{}));
            if constexpr(Type == EndpointTransferType::Control && !isIn) {
                apply(set(CompleteEnable{}, E::EPINTENSET::rxstp));
            } else {
                apply(set(CompleteEnable{}));
            }
        }

    public:
        static constexpr std::size_t queueDepth  = 1;
        static constexpr bool        asyncCancel = false;

        using FreeBuffers = std::array<bool, 1>;

        // IN: a bank is free only once its completion has been reported. Armed between TRCPT1
        // and the interrupt that clears it, a new transfer would sit next to the old one's flag
        // and cancel() would take it for "the new one went out". OUT checks the same in
        // armReceive().
        static FreeBuffers freeBuffers() {
            if constexpr(isIn) {
                return {!withController() && !completionPending()};
            } else {
                return {!withController()};
            }
        }

        static std::size_t armedBuffers() { return withController() ? 1 : 0; }

        static void setupEndpoint() { configure(); }

        template<bool Last>
        static bool tryTransfer(std::span<std::byte const> data) {
            return tryTransfer<Last>(data, freeBuffers());
        }

        // One packet. BYTE_COUNT is what goes out, and MULTI_PACKET_SIZE - the module's count
        // of what it has sent - starts at zero (32.6.2.10).
        template<bool Last>
        static bool tryTransfer(std::span<std::byte const> data,
                                FreeBuffers const&         free) {
            static_assert(isIn, "tryTransfer is the IN side");
            if(!free[0] || data.size() > maxPacketSize) { return false; }
            std::ranges::copy(data, buffer.begin());
            bankDescriptor().addr    = reinterpret_cast<std::uint32_t>(buffer.data());
            bankDescriptor().pcksize = detail::size64 | static_cast<std::uint32_t>(data.size());
            Kvasir::Hal::handToHardware();
            apply(set(ReadySet{}));
            return true;
        }

        // How much one arm carries: bulk IN only. Control transfers stay at a packet (the device
        // runs their stages packet by packet), interrupt ones are a packet by nature.
        static constexpr std::size_t maxTransfer
          = isIn && Type == EndpointTransferType::Bulk ? configuredMaxTransfer : maxPacketSize;

        // A whole transfer, read by the module where it is: BYTE_COUNT is its length and
        // MULTI_PACKET_SIZE, the module's count of what has gone out, starts at zero; it sends
        // full packets and then the rest, and raises TRCPT1 once, at the end (32.6.2.10). The
        // memory has to be RAM - the module is a bus master of its own - and word aligned.
        template<bool Last>
        static bool tryTransferInPlace(std::span<std::byte const> data,
                                       FreeBuffers const&         free)
            requires(isIn && Type == EndpointTransferType::Bulk)
        {
            if(!free[0] || data.size() > maxTransfer
               || reinterpret_cast<std::uintptr_t>(data.data()) % 4 != 0)
            {
                return false;
            }
            bankDescriptor().addr    = reinterpret_cast<std::uint32_t>(data.data());
            bankDescriptor().pcksize = detail::size64 | static_cast<std::uint32_t>(data.size());
            Kvasir::Hal::handToHardware();
            apply(set(ReadySet{}));
            return true;
        }

        // Of the transfer cancel() took back: what MULTI_PACKET_SIZE said had gone out.
        static std::size_t sentOfCancelled()
            requires(isIn && Type == EndpointTransferType::Bulk)
        {
            return sentOfCancelled_;
        }

        // One packet. MULTI_PACKET_SIZE is what the transfer may take in all and has to be a
        // multiple of the endpoint size; BYTE_COUNT, the module's count of what came, starts at
        // zero (32.6.2.8). A packet that came and was not yet reported still owns the buffer.
        template<bool Last = false>
        static bool armReceive(std::size_t) {
            static_assert(!isIn, "armReceive is the OUT side");
            if(withController() || completionPending()) { return false; }
            bankDescriptor().addr = reinterpret_cast<std::uint32_t>(buffer.data());
            bankDescriptor().pcksize
              = detail::size64
              | (static_cast<std::uint32_t>(maxPacketSize) << detail::multiPacketPos);
            Kvasir::Hal::handToHardware();
            apply(set(ReadyClear{}));
            return true;
        }

        // A whole transfer, written by the module where the caller wants it (32.6.2.8):
        // MULTI_PACKET_SIZE is what it may take in all - "must be a multiple of PCKSIZE.SIZE,
        // otherwise excess data may be written to SRAM locations used by other parts of the
        // application" - and BYTE_COUNT, the module's count of what came, starts at zero. ADDR's
        // two low bits must be zero (32.15.1). What the data sheet leaves open was asked of the
        // silicon: full packets are acknowledged and only add to BYTE_COUNT, without
        // TRCPT0; a short packet ends the transfer with one; and behind a short packet's payload
        // the module writes its CRC, up to two bytes, still inside that packet's 64 (32.6.2.7).
        static constexpr std::size_t maxReceive
          = !isIn && Type == EndpointTransferType::Bulk ? configuredMaxReceive : maxPacketSize;

        static bool armReceiveInto(std::span<std::byte> dest)
            requires(!isIn && Type == EndpointTransferType::Bulk)
        {
            if(withController() || completionPending() || dest.empty() || dest.size() > maxReceive
               || dest.size() % maxPacketSize != 0
               || reinterpret_cast<std::uintptr_t>(dest.data()) % 4 != 0)
            {
                return false;
            }
            bankDescriptor().addr = reinterpret_cast<std::uint32_t>(dest.data());
            bankDescriptor().pcksize
              = detail::size64
              | (static_cast<std::uint32_t>(dest.size()) << detail::multiPacketPos);
            Kvasir::Hal::handToHardware();
            apply(set(ReadyClear{}));
            return true;
        }

        // in transferComplete (after its takeFromHardware), or polled by takeBackReceive(): one
        // load of the module's count each call
        static std::size_t received()
            requires(!isIn && Type == EndpointTransferType::Bulk)
        {
            return Kvasir::Hal::loadHardwareField(bankDescriptor().pcksize) & detail::byteCountMask;
        }

        // while armed: the module moves it under the caller
        static std::size_t receivedSoFar()
            requires(!isIn && Type == EndpointTransferType::Bulk)
        {
            return Kvasir::Hal::loadHardwareField(bankDescriptor().pcksize) & detail::byteCountMask;
        }

        // Ends the transfer: BK0RDY set makes the module NAK from the next token on. A packet
        // that is on the wire at that moment is still taken and acknowledged, so BYTE_COUNT is
        // only believed once it has held still for longer than a packet lasts (a full one with
        // its token and handshake is under 60 us at full speed). If that packet ended the
        // transfer, its completion is flagged and reported as usual: nothing is taken back here.
        // The cost: the caller holds the USB interrupt (or is in it, for a SET_FEATURE halt) for
        // those 120 us - two packets' time, once per taken-back transfer, not per packet.
        static std::optional<std::size_t> takeBackReceive()
            requires(!isIn && Type == EndpointTransferType::Bulk)
        {
            static constexpr auto settle = std::chrono::microseconds{120};
            apply(set(ReadySet{}));
            std::size_t count = received();
            auto        since = Clock::now();
            while(Clock::now() - since < settle) {
                if(std::size_t const now = received(); now != count) {
                    count = now;
                    since = Clock::now();
                }
            }
            if(completionPending()) { return std::nullopt; }
            // BK0RDY stops the module: what it wrote into the caller's memory is the core's again
            Kvasir::Hal::takeFromHardware();
            return count;
        }

        static std::size_t readCurrentBuffer(std::span<std::byte> dest) {
            static_assert(!isIn, "read only on an OUT endpoint");
            // in transferComplete: dispatchEndpoint took the bank and the buffer back first
            std::size_t const received = bankDescriptor().pcksize & detail::byteCountMask;
            std::size_t const n        = std::min(received, dest.size());
            std::memcpy(dest.data(), buffer.data(), n);
            return n;
        }

        static void stall() { apply(set(StallSet{})); }

        static void clearStall() { apply(set(StallClear{})); }

        static void resetDataToggle() { apply(set(ToggleClear{})); }

        static void reset() { resetDataToggle(); }

        // Takes the packet back; done on return. IN: whether it had not gone out yet - if its
        // completion is flagged it has, and that report is dropped with it. OUT: a packet that
        // already came stays reported (it was acknowledged), only the wait for one ends.
        static std::size_t cancel() {
            if constexpr(isIn) {
                bool const armed = withController();
                apply(set(ReadyClear{}));
                bool const sent = completionPending();
                apply(Kvasir::Register::reset(Complete{}));
                // BK1RDY clear: the bank is the core's again
                Kvasir::Hal::takeFromHardware();
                // A transfer of several packets may have gone out in part: the module counts
                // what it sent in MULTI_PACKET_SIZE, a packet at a time (32.6.2.10).
                sentOfCancelled_
                  = armed && !sent
                    ? (bankDescriptor().pcksize >> detail::multiPacketPos) & detail::byteCountMask
                    : 0;
                return armed && !sent ? 1 : 0;
            } else {
                apply(set(ReadySet{}));
                return 0;
            }
        }
    };

private:
    using Ep0Out = Endpoint<0, EndpointDirection::Out, EndpointTransferType::Control>;

    template<std::size_t N,
             typename Sink>
    static void dispatchEndpoint() {
        using E = typename Traits::template Endpoint<N>;
        // one read of EPINTFLAG
        auto const flags
          = apply(read(E::EPINTFLAG::trcpt1, E::EPINTFLAG::trcpt0, E::EPINTFLAG::rxstp));
        // what completed - its bank, its buffer, a caller's memory the module wrote - is the core's
        // again before anyone reads it (Kvasir/Hal/DmaHandoff.hpp)
        Kvasir::Hal::takeFromHardware();

        // Each flag is cleared before its endpoint is told, so that the handler can arm the
        // endpoint again. What completed first is reported first: a SETUP last.
        if(get<0>(flags) != 0) {
            apply(reset(E::EPINTFLAG::trcpt1));
            Sink::transferComplete(N, true);
        }
        if(get<1>(flags) != 0) {
            apply(reset(E::EPINTFLAG::trcpt0));
            Sink::transferComplete(N, false);
        }
        if constexpr(N == 0) {
            if(get<2>(flags) != 0) {
                // The 8 bytes are in endpoint 0's OUT buffer, and the next SETUP is only taken
                // once this flag is clear (32.6.2.6) - so the packet is read first.
                SetupPacket pkt;
                std::memcpy(std::addressof(pkt), Ep0Out::buffer.data(), sizeof(pkt));
                apply(reset(E::EPINTFLAG::rxstp));
                Sink::setup(pkt);
            }
        }
    }

    template<typename Sink,
             std::size_t... Ns>
    static void dispatchEndpoints(std::uint32_t summary,
                                  std::index_sequence<Ns...>) {
        ((((summary >> Ns) & 1U) != 0 ? dispatchEndpoint<Ns, Sink>() : void()), ...);
    }

public:
    template<typename Sink>
    static void dispatchEvents() {
        // one read of INTFLAG
        auto const flags = apply(read(Regs::INTFLAG::sof,
                                      Regs::INTFLAG::eorst,
                                      Regs::INTFLAG::suspend,
                                      Regs::INTFLAG::wakeup));

        if constexpr(CfgTraits::UseSof) {
            if(get<0>(flags) != 0) {
                apply(reset(Regs::INTFLAG::sof));
                std::uint16_t const frame = apply(read(Regs::FNUM::fnum));
                Sink::startOfFrame(frame);
            }
        }

        if(get<1>(flags) != 0) {
            apply(reset(Regs::INTFLAG::eorst));
            // The reset disabled every endpoint but 0 and every endpoint interrupt, 0's too, and
            // cleared the address (32.6.2.4): the device sets the endpoints up again in there.
            Sink::busReset();
            return;
        }

        // INTFLAG.SUSPEND "is set when a USB Suspend state has been detected", puts the pad into
        // its idle state by being set, and is "cleared on wakeup" by the module (32.6.2.13,
        // figures 32-7 and 32-8) - so it is not acknowledged here but masked, until WAKEUP (which
        // software does clear) says the bus is active again.
        auto const enabled = apply(read(Regs::INTENSET::suspend, Regs::INTENSET::wakeup));
        if((get<2>(flags) & get<0>(enabled)) != 0) {
            apply(set(Regs::INTENCLR::suspend));
            apply(reset(Regs::INTFLAG::wakeup));
            apply(set(Regs::INTENSET::wakeup));
            Sink::suspend();
        } else if((get<3>(flags) & get<1>(enabled)) != 0) {
            apply(set(Regs::INTENCLR::wakeup));
            // SUSPEND goes with it. Figure 32-8 says the module clears it on wakeup, the register
            // description says "this flag is cleared by writing a one to the flag" - and on the
            // SAM D21 it was still set here: enabling its interrupt again fired it
            // at once, that enabled WAKEUP, the next token on the bus set that, and so on - two
            // interrupts per frame on an idle bus, 36 000 to 119 000 a second under traffic,
            // a third to nine tenths of the core. Found by counting the interrupt's entries.
            apply(reset(Regs::INTFLAG::wakeup, Regs::INTFLAG::suspend));
            apply(set(Regs::INTENSET::suspend));
            Sink::resume();
        }

        std::uint16_t const summary = apply(read(Regs::EPINTSMRY::FULLREGISTER));
        dispatchEndpoints<Sink>(summary, std::make_index_sequence<Traits::endpointCount>{});
    }

    // DADD.ADDEN: "should be written to one to accept communications directed to this address"
    // (32.6.2.1); address 0, after a reset, is ADDEN clear.
    static void setAddress(std::uint8_t address) {
        if(address == 0) {
            apply(write(Regs::DADD::dadd, Kvasir::Register::value<std::uint8_t, 0>()),
                  clear(Regs::DADD::adden));
        } else {
            apply(write(Regs::DADD::dadd, address), set(Regs::DADD::adden));
        }
    }

    // The module has done it all by the time RXSTP is set: "DTGLOUT, DTGLIN, CURRBK and BK0RDY
    // are set. BK1RDY and STALLRQ0/1 are cleared on receiving the SETUP request" (32.6.2.6) -
    // nothing armed either way, no stall, DATA1 next in both directions.
    static void beginControlTransfer() {}

    static void maskInterrupt() { apply(Kvasir::Nvic::makeDisable(Traits::interruptIndexes)); }

    static void unmaskInterrupt() { apply(Kvasir::Nvic::makeEnable(Traits::interruptIndexes)); }

    // A software reset, the pad calibration "before enabling the USB" (32.5.10), device mode at
    // full speed, the descriptor table, and the module enabled - still detached (CTRLB.DETACH is
    // set after a reset), so that the endpoints can be set up before the host sees the device.
    static void prepare() {
        apply(Regs::CTRLA::overrideDefaults(set(Regs::CTRLA::swrst)));
        if(!waitSync()) {
            clockMissing = true;
            UC_LOG_E(
              "USB: the module does not come out of its reset - no GCLK_USB? The clock "
              "settings have to route a 48 MHz generator to the USB's GCLK channel");
            return;
        }

        auto const cal = Traits::padCalibration();
        apply(write(Regs::PADCAL::transn, cal.transn),
              write(Regs::PADCAL::transp, cal.transp),
              write(Regs::PADCAL::trim, cal.trim));

        for(auto& descriptor : detail::descriptorTable) {
            for(auto& b : descriptor.bank) {
                b.addr     = 0;
                b.pcksize  = 0;
                b.extreg   = 0;
                b.statusBk = 0;
            }
        }
        Kvasir::Hal::handToHardware();
        apply(write(Regs::DESCADD::descadd,
                    reinterpret_cast<std::uint32_t>(std::addressof(detail::descriptorTable[0]))));

        apply(write(Regs::CTRLB::SPDCONFValC::fs));
        if constexpr(CfgTraits::UseSof) {
            apply(set(Regs::INTENSET::eorst, Regs::INTENSET::suspend, Regs::INTENSET::sof));
        } else {
            apply(set(Regs::INTENSET::eorst, Regs::INTENSET::suspend));
        }

        apply(write(Regs::CTRLA::MODEValC::device), set(Regs::CTRLA::enable));
        static_cast<void>(waitSync());
    }

    static void connect() {
        if(clockMissing) { return; }
        apply(Kvasir::Nvic::makeEnable(Traits::interruptIndexes));
        apply(clear(Regs::CTRLB::detach));
    }

    /// What this controller can check about itself, one log line each, for the first contact
    /// with a new board: true = nothing here explains a device that does not enumerate. Call it
    /// from main(), after Startup.
    static bool selfTest() {
        bool       ok    = true;
        auto const check = [&ok](bool pass, std::string_view what, std::string_view hint) {
            if(pass) {
                UC_LOG_D("USB self-test: ok   {}", what);
            } else {
                UC_LOG_E("USB self-test: FAIL {} - {}", what, hint);
                ok = false;
            }
        };
        check(Traits::busClocksEnabled(),
              "the bus clocks are on",
              "the AHB/APB mask bits of the USB");
        check(!clockMissing,
              "the module came out of its software reset",
              "no GCLK_USB: the clock settings have to route 48 MHz to the USB's channel");
        if(clockMissing) { return false; }
        std::uint8_t const busy = apply(read(Regs::SYNCBUSY::FULLREGISTER));
        check(busy == 0, "nothing waits for synchronisation", "GCLK_USB stopped after the start?");
        check(Traits::padCalibrationProgrammed(),
              "the pad calibration is programmed",
              "the NVM calibration area reads all ones; typical values are in use");
        std::uint32_t const enabled = apply(read(Regs::CTRLA::enable));
        check(enabled != 0, "the module is enabled", "Usb is not in the Startup list?");
        std::uint32_t const table = apply(read(Regs::DESCADD::descadd));
        check(table == reinterpret_cast<std::uint32_t>(std::addressof(detail::descriptorTable[0])),
              "DESCADD points at the endpoint descriptor table",
              "prepare() did not get that far");
        std::uint32_t const detached = apply(read(Regs::CTRLB::detach));
        check(detached == 0, "attached to the bus (DETACH clear)", "connect() did not run");
        return ok;
    }

    // Off the bus: CTRLB.DETACH, "to detach the device from the USB host" (32.6.2.1).
    static void disconnect() { apply(set(Regs::CTRLB::detach)); }

    //Kvasir Callbacks
    template<auto Handler>
    using Isr
      = Kvasir::Nvic::IsrsFor<Handler, std::remove_cvref_t<decltype(Traits::interruptIndexes)>>;

    using Provides = brigand::list<Kvasir::Hal::Owns<Regs>>;
    // GCLK_USB at 48 MHz, which only the application's clock settings can set up.
    using Claims = Kvasir::Hal::Claim<Kvasir::Clocks::ClkUsb, 48'000'000>;

    static constexpr auto powerClockEnable = Traits::powerClockEnable;

    static constexpr auto initStepPinConfig = Traits::pinConfig;

    static constexpr auto initStepInterruptConfig
      = list(Kvasir::Nvic::makeSetPriority<isrPriority>(Traits::interruptIndexes),
             Kvasir::Nvic::makeClearPending(Traits::interruptIndexes));
};
}   // namespace Kvasir::USB::Sam
