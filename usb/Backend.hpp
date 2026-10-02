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
// Traits::gclkChannel - and say so: `using Provides = brigand::list<Kvasir::Clocks::Clk<
// Kvasir::Clocks::ClkUsb, 48'000'000>>;` (../Clocks.hpp), or Startup refuses the USB. Without that
// clock the module never finishes its reset; prepare() gives up after a while and logs it.
//
// How this controller differs from the RP one, all of it within the contract:
//   - An endpoint has one transfer with the controller at a time (QueueDepth 1), in ordinary RAM
//     which the module reads and writes by itself (32.6.2.2): a packet in a buffer of this
//     driver's, or - bulk IN - up to MaxTransfer bytes where the caller keeps them, which the
//     module splits into packets itself (32.6.2.10), with one interrupt at the end.
//   - The data toggle is the hardware's (EPSTATUS.DTGLIN / DTGLOUT).
//   - Taking a packet back is done on return (AsyncCancel = false).
//   - A bus reset disables every endpoint but 0 and every endpoint interrupt (32.6.2.4): the
//     device sets every endpoint up again on a bus reset, which is what puts them back.
//
// Besides what the device reads, the config struct may carry
//
//     isrPriority      1
//     UsbMaxTransfer   512: how much one arm of a bulk IN endpoint carries (a multiple of 64; 64
//                      turns the multi-packet transfers off). The bulk endpoint's staging buffer
//                      is this large.
//     UsbMaxReceive    512: the same for a bulk OUT endpoint, which the module fills by itself
//                      (32.6.2.8); 64 turns it off.
//
// On hardware since 2026-09-20 (ATSAMD21G18A, usb_playground's suites).

#include "../Clocks.hpp"
#include "chip/Usb_Traits.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/Log.hpp>
#include <kvasir/Devices/USB/Backend.hpp>
#include <kvasir/Devices/USB/Config.hpp>
#include <kvasir/Devices/USB/Descriptors.hpp>
#include <optional>
#include <span>
#include <string_view>

namespace Kvasir::USB::Sam {
// Startup resource: the one USB controller (kvasir/StartUp/Resources.hpp).
struct InstanceTag {};

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
    inline constexpr std::uint32_t ByteCountMask  = 0x3FFFU;
    inline constexpr unsigned      MultiPacketPos = 14;
    inline constexpr std::uint32_t Size64         = 3U << 28U;

    // The module writes it (BYTE_COUNT, STATUS_BK) behind the compiler's back. One table: there
    // is one controller, whatever Clock and Config the backend is instantiated with.
    alignas(4) inline EndpointDescriptor volatile descriptorTable[Traits::EndpointCount]{};

    // An endpoint buffer the module wrote, read without the compiler assuming it knows what is
    // in there.
    inline void copyFromModule(std::span<std::byte> dest,
                               std::byte const*     src) {
        auto const* const in = static_cast<std::byte const volatile*>(src);
        for(std::size_t i = 0; i != dest.size(); ++i) { dest[i] = in[i]; }
    }
}   // namespace detail

template<typename Clock, typename ConfigT>
struct Backend {
private:
    using Regs      = typename Traits::Regs;
    using CfgTraits = Kvasir::USB::ConfigTraits<ConfigT>;

    static constexpr auto isrPriority = [] {
        if constexpr(requires { ConfigT::isrPriority; }) {
            return ConfigT::isrPriority;
        } else {
            return 1;
        }
    }();

    // SYNCBUSY clears once GCLK_USB has carried the write across (32.5.3: the generic clock is
    // asynchronous to the bus clock). With no GCLK_USB it never does, so the wait is bounded: a
    // synchronisation takes a few clock cycles, this is tens of milliseconds.
    static bool waitSync() {
        for(std::uint32_t i = 0; i != 1'000'000U; ++i) {
            std::uint8_t const busy = apply(read(Regs::SYNCBUSY::FULLREGISTER));
            if(busy == 0) { return true; }
        }
        return false;
    }

    static inline bool clockMissing{false};

public:
    static constexpr std::size_t MaxPacketSize = 64;
    static constexpr std::size_t EndpointCount = Traits::EndpointCount;

    static constexpr std::size_t ConfiguredMaxTransfer = [] {
        if constexpr(requires { ConfigT::UsbMaxTransfer; }) {
            return static_cast<std::size_t>(ConfigT::UsbMaxTransfer);
        } else {
            return std::size_t{512};
        }
    }();
    // BYTE_COUNT and MULTI_PACKET_SIZE are 14 bits (32.15.2).
    static_assert(ConfiguredMaxTransfer % MaxPacketSize == 0 && ConfiguredMaxTransfer != 0
                    && ConfiguredMaxTransfer < (1U << 14U),
                  "UsbMaxTransfer: a multiple of 64 below 16384");
    static constexpr bool AsyncCancel = false;

    static constexpr std::size_t ConfiguredMaxReceive = [] {
        if constexpr(requires { ConfigT::UsbMaxReceive; }) {
            return static_cast<std::size_t>(ConfigT::UsbMaxReceive);
        } else {
            return std::size_t{512};
        }
    }();
    static_assert(ConfiguredMaxReceive % MaxPacketSize == 0 && ConfiguredMaxReceive != 0
                    && ConfiguredMaxReceive < (1U << 14U),
                  "UsbMaxReceive: a multiple of 64 below 16384");

    template<std::size_t N, EndpointDirection Dir, EndpointTransferType Type>
    struct Endpoint {
    private:
        friend Backend;

        using E = typename Traits::template Endpoint<N>;

        static constexpr bool     IsIn = Dir == EndpointDirection::In;
        static constexpr unsigned Bank = IsIn ? 1 : 0;

        static_assert(N < Traits::EndpointCount,
                      "the controller has no such endpoint");

        // The packet, where the module reads or writes it: "Endpoint data can be placed anywhere
        // in the device RAM" (32.6.2.2). Word aligned, because the module is a bus master of its
        // own. A SETUP packet arrives in endpoint 0's OUT buffer.
        alignas(4) static inline std::array<std::byte,
                                            MaxPacketSize> buffer{};

        static auto& bank() { return detail::descriptorTable[N].bank[Bank]; }

        // EPSTATUSSET / EPSTATUSCLR are strobes: a '1' sets or clears that bit, a '0' does
        // nothing, so they are written whole and never read-modify-written.
        static void statusSet(std::uint8_t mask) {
            apply(write(E::EPSTATUSSET::FULLREGISTER, mask));
        }

        static void statusClear(std::uint8_t mask) {
            apply(write(E::EPSTATUSCLR::FULLREGISTER, mask));
        }

        static inline std::size_t sentOfCancelled_{};

        static constexpr std::uint8_t ReadyMask
          = IsIn ? E::EPSTATUSSET::bk1rdy.Mask : E::EPSTATUSSET::bk0rdy.Mask;
        static constexpr std::uint8_t ToggleMask
          = IsIn ? E::EPSTATUSSET::dtglin.Mask : E::EPSTATUSSET::dtglout.Mask;
        static constexpr std::uint8_t StallMask
          = IsIn ? E::EPSTATUSSET::stallrq1.Mask : E::EPSTATUSSET::stallrq0.Mask;
        static constexpr std::uint8_t CompleteMask
          = IsIn ? E::EPINTFLAG::trcpt1.Mask : E::EPINTFLAG::trcpt0.Mask;

        // Bank 1 ready: a packet waits to be sent. Bank 0 ready: a packet was received, or
        // nothing is wanted - the module NAKs an OUT while it is set (32.6.2.7, 32.6.2.9). So an
        // IN endpoint is with the controller while BK1RDY is set, an OUT one while BK0RDY is clear.
        static bool withController() {
            std::uint8_t const status = apply(read(E::EPSTATUS::FULLREGISTER));
            return IsIn ? (status & ReadyMask) != 0 : (status & ReadyMask) == 0;
        }

        static bool completionPending() {
            std::uint8_t const flags = apply(read(E::EPINTFLAG::FULLREGISTER));
            return (flags & CompleteMask) != 0;
        }

        // EPCFG.EPTYPEn (32.12.1): 1 control, 2 isochronous, 3 bulk, 4 interrupt - one more than
        // the transfer type's code in a descriptor.
        static constexpr unsigned EpType = static_cast<unsigned>(Type) + 1;

        static void configure() {
            auto& b = bank();
            // Where the data is, is said when an endpoint is armed - but a SETUP comes unasked,
            // so endpoint 0's OUT bank always points at its buffer.
            if constexpr(Type == EndpointTransferType::Control && !IsIn) {
                b.addr = reinterpret_cast<std::uint32_t>(buffer.data());
            } else {
                b.addr = 0;
            }
            b.pcksize  = detail::Size64;
            b.extreg   = 0;
            b.statusBk = 0;

            // nothing to send, nothing wanted, DATA0 next, not stalled
            if constexpr(IsIn) {
                statusClear(ReadyMask | ToggleMask | StallMask);
                apply(write(E::EPCFG::eptype1, EpType));
            } else {
                statusSet(ReadyMask);
                statusClear(ToggleMask | StallMask);
                apply(write(E::EPCFG::eptype0, EpType));
            }
            // The flag is a strobe too (write '1' to clear); then the interrupt. Transfer
            // failures (every NAK is one) and sent STALLs stay off.
            apply(write(E::EPINTFLAG::FULLREGISTER, CompleteMask));
            if constexpr(Type == EndpointTransferType::Control && !IsIn) {
                apply(write(E::EPINTENSET::FULLREGISTER,
                            static_cast<std::uint8_t>(CompleteMask | E::EPINTENSET::rxstp.Mask)));
            } else {
                apply(write(E::EPINTENSET::FULLREGISTER, CompleteMask));
            }
        }

    public:
        static constexpr std::size_t QueueDepth  = 1;
        static constexpr bool        AsyncCancel = false;

        using FreeBuffers = std::array<bool, 1>;

        // IN: a bank is free only once its completion has been reported. Armed between TRCPT1
        // and the interrupt that clears it, a new transfer would sit next to the old one's flag
        // and cancel() would take it for "the new one went out". OUT checks the same in
        // armReceive().
        static FreeBuffers freeBuffers() {
            if constexpr(IsIn) {
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
            static_assert(IsIn, "tryTransfer is the IN side");
            if(!free[0] || data.size() > MaxPacketSize) { return false; }
            std::copy(data.begin(), data.end(), buffer.begin());
            bank().addr    = reinterpret_cast<std::uint32_t>(buffer.data());
            bank().pcksize = detail::Size64 | static_cast<std::uint32_t>(data.size());
            statusSet(ReadyMask);
            return true;
        }

        // How much one arm carries: bulk IN only. Control transfers stay at a packet (the device
        // runs their stages packet by packet), interrupt ones are a packet by nature.
        static constexpr std::size_t MaxTransfer
          = IsIn && Type == EndpointTransferType::Bulk ? ConfiguredMaxTransfer : MaxPacketSize;

        // A whole transfer, read by the module where it is: BYTE_COUNT is its length and
        // MULTI_PACKET_SIZE, the module's count of what has gone out, starts at zero; it sends
        // full packets and then the rest, and raises TRCPT1 once, at the end (32.6.2.10). The
        // memory has to be RAM - the module is a bus master of its own - and word aligned.
        template<bool Last>
        static bool tryTransferInPlace(std::span<std::byte const> data,
                                       FreeBuffers const&         free)
            requires(IsIn && Type == EndpointTransferType::Bulk)
        {
            if(!free[0] || data.size() > MaxTransfer
               || reinterpret_cast<std::uintptr_t>(data.data()) % 4 != 0)
            {
                return false;
            }
            bank().addr    = reinterpret_cast<std::uint32_t>(data.data());
            bank().pcksize = detail::Size64 | static_cast<std::uint32_t>(data.size());
            statusSet(ReadyMask);
            return true;
        }

        // Of the transfer cancel() took back: what MULTI_PACKET_SIZE said had gone out.
        static std::size_t sentOfCancelled()
            requires(IsIn && Type == EndpointTransferType::Bulk)
        {
            return sentOfCancelled_;
        }

        // One packet. MULTI_PACKET_SIZE is what the transfer may take in all and has to be a
        // multiple of the endpoint size; BYTE_COUNT, the module's count of what came, starts at
        // zero (32.6.2.8). A packet that came and was not yet reported still owns the buffer.
        template<bool Last = false>
        static bool armReceive(std::size_t) {
            static_assert(!IsIn, "armReceive is the OUT side");
            if(withController() || completionPending()) { return false; }
            bank().addr    = reinterpret_cast<std::uint32_t>(buffer.data());
            bank().pcksize = detail::Size64
                           | (static_cast<std::uint32_t>(MaxPacketSize) << detail::MultiPacketPos);
            statusClear(ReadyMask);
            return true;
        }

        // A whole transfer, written by the module where the caller wants it (32.6.2.8):
        // MULTI_PACKET_SIZE is what it may take in all - "must be a multiple of PCKSIZE.SIZE,
        // otherwise excess data may be written to SRAM locations used by other parts of the
        // application" - and BYTE_COUNT, the module's count of what came, starts at zero. ADDR's
        // two low bits must be zero (32.15.1). What the data sheet leaves open was asked of the
        // silicon (2026-09-20): full packets are acknowledged and only add to BYTE_COUNT, without
        // TRCPT0; a short packet ends the transfer with one; and behind a short packet's payload
        // the module writes its CRC, up to two bytes, still inside that packet's 64 (32.6.2.7).
        static constexpr std::size_t MaxReceive
          = !IsIn && Type == EndpointTransferType::Bulk ? ConfiguredMaxReceive : MaxPacketSize;

        static bool armReceiveInto(std::span<std::byte> dest)
            requires(!IsIn && Type == EndpointTransferType::Bulk)
        {
            if(withController() || completionPending() || dest.empty() || dest.size() > MaxReceive
               || dest.size() % MaxPacketSize != 0
               || reinterpret_cast<std::uintptr_t>(dest.data()) % 4 != 0)
            {
                return false;
            }
            bank().addr    = reinterpret_cast<std::uint32_t>(dest.data());
            bank().pcksize = detail::Size64
                           | (static_cast<std::uint32_t>(dest.size()) << detail::MultiPacketPos);
            statusClear(ReadyMask);
            return true;
        }

        static std::size_t received()
            requires(!IsIn && Type == EndpointTransferType::Bulk)
        {
            return bank().pcksize & detail::ByteCountMask;
        }

        static std::size_t receivedSoFar()
            requires(!IsIn && Type == EndpointTransferType::Bulk)
        {
            return bank().pcksize & detail::ByteCountMask;
        }

        // Ends the transfer: BK0RDY set makes the module NAK from the next token on. A packet
        // that is on the wire at that moment is still taken and acknowledged, so BYTE_COUNT is
        // only believed once it has held still for longer than a packet lasts (a full one with
        // its token and handshake is under 60 us at full speed). If that packet ended the
        // transfer, its completion is flagged and reported as usual: nothing is taken back here.
        // The cost: the caller holds the USB interrupt (or is in it, for a SET_FEATURE halt) for
        // those 120 us - two packets' time, once per taken-back transfer, not per packet.
        static std::optional<std::size_t> takeBackReceive()
            requires(!IsIn && Type == EndpointTransferType::Bulk)
        {
            static constexpr auto Settle = std::chrono::microseconds{120};
            statusSet(ReadyMask);
            std::size_t count = received();
            auto        since = Clock::now();
            while(Clock::now() - since < Settle) {
                if(std::size_t const now = received(); now != count) {
                    count = now;
                    since = Clock::now();
                }
            }
            if(completionPending()) { return std::nullopt; }
            return count;
        }

        static std::size_t readCurrentBuffer(std::span<std::byte> dest) {
            static_assert(!IsIn, "read only on an OUT endpoint");
            std::size_t const received = bank().pcksize & detail::ByteCountMask;
            std::size_t const n        = std::min(received, dest.size());
            detail::copyFromModule(dest.first(n), buffer.data());
            return n;
        }

        static void stall() { statusSet(StallMask); }

        static void clearStall() { statusClear(StallMask); }

        static void resetDataToggle() { statusClear(ToggleMask); }

        static void reset() { resetDataToggle(); }

        // Takes the packet back; done on return. IN: whether it had not gone out yet - if its
        // completion is flagged it has, and that report is dropped with it. OUT: a packet that
        // already came stays reported (it was acknowledged), only the wait for one ends.
        static std::size_t cancel() {
            if constexpr(IsIn) {
                bool const armed = withController();
                statusClear(ReadyMask);
                bool const sent = completionPending();
                apply(write(E::EPINTFLAG::FULLREGISTER, CompleteMask));
                // A transfer of several packets may have gone out in part: the module counts
                // what it sent in MULTI_PACKET_SIZE, a packet at a time (32.6.2.10).
                sentOfCancelled_ = armed && !sent ? (bank().pcksize >> detail::MultiPacketPos)
                                                      & detail::ByteCountMask
                                                  : 0;
                return armed && !sent ? 1 : 0;
            } else {
                statusSet(ReadyMask);
                return 0;
            }
        }
    };

private:
    using EP0_OUT = Endpoint<0, EndpointDirection::Out, EndpointTransferType::Control>;

    template<std::size_t N,
             typename Sink>
    static void dispatchEndpoint() {
        using E                  = typename Traits::template Endpoint<N>;
        std::uint8_t const flags = apply(read(E::EPINTFLAG::FULLREGISTER));

        // Each flag is cleared before its endpoint is told, so that the handler can arm the
        // endpoint again. What completed first is reported first: a SETUP last.
        if(flags & E::EPINTFLAG::trcpt1.Mask) {
            apply(write(E::EPINTFLAG::FULLREGISTER, E::EPINTFLAG::trcpt1.Mask));
            Sink::transferComplete(N, true);
        }
        if(flags & E::EPINTFLAG::trcpt0.Mask) {
            apply(write(E::EPINTFLAG::FULLREGISTER, E::EPINTFLAG::trcpt0.Mask));
            Sink::transferComplete(N, false);
        }
        if constexpr(N == 0) {
            if(flags & E::EPINTFLAG::rxstp.Mask) {
                // The 8 bytes are in endpoint 0's OUT buffer, and the next SETUP is only taken
                // once this flag is clear (32.6.2.6) - so the packet is read first.
                SetupPacket pkt;
                detail::copyFromModule(
                  std::span{reinterpret_cast<std::byte*>(std::addressof(pkt)), sizeof(pkt)},
                  EP0_OUT::buffer.data());
                apply(write(E::EPINTFLAG::FULLREGISTER, E::EPINTFLAG::rxstp.Mask));
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

    template<auto Handler,
             typename... Ts>
    static constexpr auto makeIsr(brigand::list<Ts...>)
      -> brigand::list<Kvasir::Nvic::Isr<Handler,
                                         Kvasir::Nvic::Index<Ts::value>>...>;

public:
    template<typename Sink>
    static void dispatchEvents() {
        std::uint16_t const flags = apply(read(Regs::INTFLAG::FULLREGISTER));

        if constexpr(CfgTraits::UseSof) {
            if(flags & Regs::INTFLAG::sof.Mask) {
                apply(write(Regs::INTFLAG::FULLREGISTER, Regs::INTFLAG::sof.Mask));
                std::uint16_t const frame = apply(read(Regs::FNUM::fnum));
                Sink::startOfFrame(frame);
            }
        }

        if(flags & Regs::INTFLAG::eorst.Mask) {
            apply(write(Regs::INTFLAG::FULLREGISTER, Regs::INTFLAG::eorst.Mask));
            // The reset disabled every endpoint but 0 and every endpoint interrupt, 0's too, and
            // cleared the address (32.6.2.4): the device sets the endpoints up again in there.
            Sink::busReset();
            return;
        }

        // INTFLAG.SUSPEND "is set when a USB Suspend state has been detected", puts the pad into
        // its idle state by being set, and is "cleared on wakeup" by the module (32.6.2.13,
        // figures 32-7 and 32-8) - so it is not acknowledged here but masked, until WAKEUP (which
        // software does clear) says the bus is active again.
        std::uint16_t const enabled = apply(read(Regs::INTENSET::FULLREGISTER));
        if(flags & enabled & Regs::INTFLAG::suspend.Mask) {
            apply(write(Regs::INTENCLR::FULLREGISTER, Regs::INTENCLR::suspend.Mask));
            apply(write(Regs::INTFLAG::FULLREGISTER, Regs::INTFLAG::wakeup.Mask));
            apply(write(Regs::INTENSET::FULLREGISTER, Regs::INTENSET::wakeup.Mask));
            Sink::suspend();
        } else if(flags & enabled & Regs::INTFLAG::wakeup.Mask) {
            apply(write(Regs::INTENCLR::FULLREGISTER, Regs::INTENCLR::wakeup.Mask));
            // SUSPEND goes with it. Figure 32-8 says the module clears it on wakeup, the register
            // description says "this flag is cleared by writing a one to the flag" - and on the
            // SAM D21 it was still set here (2026-09-20): enabling its interrupt again fired it
            // at once, that enabled WAKEUP, the next token on the bus set that, and so on - two
            // interrupts per frame on an idle bus, 36 000 to 119 000 a second under traffic,
            // a third to nine tenths of the core. Found by counting the interrupt's entries.
            apply(write(Regs::INTFLAG::FULLREGISTER,
                        static_cast<std::uint16_t>(Regs::INTFLAG::wakeup.Mask
                                                   | Regs::INTFLAG::suspend.Mask)));
            apply(write(Regs::INTENSET::FULLREGISTER, Regs::INTENSET::suspend.Mask));
            Sink::resume();
        }

        std::uint16_t const summary = apply(read(Regs::EPINTSMRY::FULLREGISTER));
        dispatchEndpoints<Sink>(summary, std::make_index_sequence<Traits::EndpointCount>{});
    }

    // DADD.ADDEN: "should be written to one to accept communications directed to this address"
    // (32.6.2.1); address 0, after a reset, is ADDEN clear.
    static void setAddress(std::uint8_t address) {
        apply(
          write(Regs::DADD::FULLREGISTER,
                static_cast<std::uint8_t>(address == 0 ? 0 : (address | Regs::DADD::adden.Mask))));
    }

    // The module has done it all by the time RXSTP is set: "DTGLOUT, DTGLIN, CURRBK and BK0RDY
    // are set. BK1RDY and STALLRQ0/1 are cleared on receiving the SETUP request" (32.6.2.6) -
    // nothing armed either way, no stall, DATA1 next in both directions.
    static void beginControlTransfer() {}

    static void maskInterrupt() { apply(Kvasir::Nvic::makeDisable(Traits::InterruptIndexes)); }

    static void unmaskInterrupt() { apply(Kvasir::Nvic::makeEnable(Traits::InterruptIndexes)); }

    // A software reset, the pad calibration "before enabling the USB" (32.5.10), device mode at
    // full speed, the descriptor table, and the module enabled - still detached (CTRLB.DETACH is
    // set after a reset), so that the endpoints can be set up before the host sees the device.
    static void prepare() {
        apply(write(Regs::CTRLA::FULLREGISTER, Regs::CTRLA::swrst.Mask));
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
        apply(write(Regs::DESCADD::descadd,
                    reinterpret_cast<std::uint32_t>(std::addressof(detail::descriptorTable[0]))));

        apply(write(Regs::CTRLB::SPDCONFValC::fs));
        static constexpr std::uint16_t Interrupts
          = Regs::INTENSET::eorst.Mask | Regs::INTENSET::suspend.Mask
          | (CfgTraits::UseSof ? Regs::INTENSET::sof.Mask : 0);
        apply(write(Regs::INTENSET::FULLREGISTER, Interrupts));

        apply(write(Regs::CTRLA::MODEValC::device), set(Regs::CTRLA::enable));
        static_cast<void>(waitSync());
    }

    static void connect() {
        if(clockMissing) { return; }
        apply(Kvasir::Nvic::makeEnable(Traits::InterruptIndexes));
        apply(clear(Regs::CTRLB::detach));
    }

    /// What this controller can check about itself, one log line each, for the first contact
    /// with a new board: true = nothing here explains a device that does not enumerate. Call it
    /// from main(), after Startup.
    static bool selfTest() {
        bool       ok    = true;
        auto const check = [&ok](bool pass, std::string_view what, std::string_view hint) {
            if(pass) {
                UC_LOG_I("USB self-test: ok   {}", what);
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
    using Isr = decltype(makeIsr<Handler>(Traits::InterruptIndexes));

    using Provides = brigand::list<Kvasir::Startup::Resource<InstanceTag, 0>>;
    // GCLK_USB at 48 MHz, which only the application's clock settings can set up.
    using Claims = Kvasir::Clocks::Claim<Kvasir::Clocks::ClkUsb, 48'000'000>;

    static constexpr auto powerClockEnable = Traits::powerClockEnable;

    static constexpr auto initStepPinConfig = Traits::pinConfig;

    static constexpr auto initStepInterruptConfig
      = list(Kvasir::Nvic::makeSetPriority<isrPriority>(Traits::InterruptIndexes),
             Kvasir::Nvic::makeClearPending(Traits::InterruptIndexes));
};
}   // namespace Kvasir::USB::Sam
