#pragma once
#include "chip/Dmac_Traits.hpp"
#include "chip/PM.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/DMAC.hpp"

#include <cstdint>

namespace Kvasir {
namespace DMAC {
    struct alignas(16) DmacDescriptor {
        enum class stepsize : unsigned char {
            x1   = 0x00,
            x2   = 0x01,
            x4   = 0x02,
            x8   = 0x03,
            x16  = 0x04,
            x32  = 0x05,
            x64  = 0x06,
            x128 = 0x07
        };

        enum class stepsel : unsigned char { dst = 0x00, src = 0x01 };

        enum class dstinc : unsigned char { no_increment = 0x00, increment = 0x01 };

        enum class srcinc : unsigned char { no_increment = 0x00, increment = 0x01 };

        enum class beatsize : unsigned char {
            byte  = 0x00,
            hword = 0x01,
            word  = 0x02,
        };

        enum class blockact : unsigned char {
            noact     = 0x00,
            interrupt = 0x01,
            suspend   = 0x02,
            both      = 0x03,
        };

        enum class evosel : unsigned char {
            disabled = 0x00,
            block    = 0x01,
            beat     = 0x03,
        };

        constexpr DmacDescriptor()
          : BTCTRL{}
          , BTCNT{}
          , SRCADDR{}
          , DSTADDR{}
          , DESCADDR{} {}

        constexpr DmacDescriptor(bool          valid,
                                 stepsize      stsize,
                                 stepsel       stsel,
                                 dstinc        dinc,
                                 srcinc        sinc,
                                 beatsize      bsize,
                                 blockact      bact,
                                 evosel        esel,
                                 std::uint16_t beatcount,
                                 std::uint32_t srcaddr,
                                 std::uint32_t dstaddr)
          : BTCTRL(std::uint16_t((valid ? 1U : 0U) | (unsigned(esel) << 1U) | (unsigned(bact) << 3U)
                                 | (unsigned(bsize) << 8U) | (unsigned(sinc) << 10U)
                                 | (unsigned(dinc) << 11U) | (unsigned(stsel) << 12U)
                                 | (unsigned(stsize) << 13U)))
          , BTCNT(beatcount)
          , SRCADDR(srcaddr)
          , DSTADDR(dstaddr)
          , DESCADDR(0) {}

        bool isValid() const {
            return ((*reinterpret_cast<std::uint16_t const volatile*>(&BTCTRL)) & 0x0001U) != 0;
        }

        void setisValid(bool b) { BTCTRL = b ? BTCTRL | 1U : BTCTRL & 0xFFFEU; }

        std::uint16_t BTCTRL;
        std::uint16_t BTCNT;
        std::uint32_t SRCADDR;
        std::uint32_t DSTADDR;
        std::uint32_t DESCADDR;
    };

    static_assert(sizeof(DmacDescriptor) == 16);

    enum class DMAChannel {
        ch0  = 0,
        ch1  = 1,
        ch2  = 2,
        ch3  = 3,
        ch4  = 4,
        ch5  = 5,
        ch6  = 6,
        ch7  = 7,
        ch8  = 8,
        ch9  = 9,
        ch10 = 10,
        ch11 = 11,
    };

    enum class DMAPriority {
        p0 = 0,
        p1,
        p2,
        p3,
    };

    using TriggerSource = Traits::DmacTraits::TriggerSource;

    template<typename DMAConfig>
    struct DmaBase {
        // needed config
        // numberOfChannels
        static constexpr auto numberOfChannels = DMAConfig::numberOfChannels;

        static_assert(Traits::DmacTraits::Channels >= numberOfChannels);

        static inline std::array<DmacDescriptor, numberOfChannels> rdescriptors{};
        static inline std::array<DmacDescriptor, numberOfChannels> wdescriptors{};

        using Regs                   = Peripheral::DMAC::Registers<>;
        static constexpr unsigned ba = Regs::baseAddr;

        static constexpr auto powerClockEnable = list(typename PM::enable<ba>::action{});

        template<DMAChannel Channel>
        static constexpr DmacDescriptor& rd() {
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));
            return rdescriptors[static_cast<std::size_t>(Channel)];
        }

        template<DMAChannel Channel>
        static constexpr DmacDescriptor& wd() {
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));
            return wdescriptors[static_cast<std::size_t>(Channel)];
        }

        [[gnu::always_inline]] static void runtimeInit() {
            apply(
              write(Regs::BASEADDR::baseaddr, reinterpret_cast<std::uint32_t>(rdescriptors.data())),
              write(Regs::WRBADDR::wrbaddr, reinterpret_cast<std::uint32_t>(wdescriptors.data())));
            apply(Regs::CTRL::overrideDefaults(set(Regs::CTRL::dmaenable),
                                               set(Regs::CTRL::lvlen0),
                                               set(Regs::CTRL::lvlen1),
                                               set(Regs::CTRL::lvlen2),
                                               set(Regs::CTRL::lvlen3)));
        }

        template<DMAChannel    Channel,
                 DMAPriority   Priority,
                 TriggerSource Trigger,
                 typename Regs_t>
        static constexpr auto start() {
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));

            if constexpr(Traits::DmacTraits::OldImpl) {
                return list(
                  write(
                    Regs_t::CHID::id,
                    Kvasir::Register::value<std::uint8_t, static_cast<std::uint8_t>(Channel)>()),
                  Kvasir::Register::sequencePoint,
                  write(Regs_t::CHCTRLB::CMDValC::noact),
                  write(Regs_t::CHCTRLB::TRIGACTValC::beat),
                  write(Regs_t::CHCTRLB::trigsrc,
                        Register::value<typename Regs_t::CHCTRLB::TRIGSRCVal, Trigger>()),
                  write(Regs_t::CHCTRLB::lvl,
                        Register::value<typename Regs_t::CHCTRLB::LVLVal,
                                        static_cast<typename Regs_t::CHCTRLB::LVLVal>(
                                          static_cast<int>(Regs_t::CHCTRLB::LVLVal::lvl0)
                                          + static_cast<int>(Priority))>()),
                  clear(Regs_t::CHCTRLB::evoe),
                  clear(Regs_t::CHCTRLB::evie),
                  write(Regs_t::CHCTRLB::EVACTValC::noact),
                  Kvasir::Register::sequencePoint,
                  set(Regs_t::CHCTRLA::enable),
                  clear(Regs_t::CHCTRLA::swrst));
            } else {
                using CHRegs = typename Regs_t::template CHANNEL<static_cast<std::size_t>(Channel)>;
                return list(
                  CHRegs::CHCTRLA::overrideDefaults(),

                  Kvasir::Register::sequencePoint,
                  CHRegs::CHCTRLA::overrideDefaults(set(CHRegs::CHCTRLA::swrst)),

                  Kvasir::Register::sequencePoint,
                  write(CHRegs::CHPRILVL::prilvl,
                        Register::value<typename CHRegs::CHPRILVL::PRILVLVal,
                                        static_cast<typename CHRegs::CHPRILVL::PRILVLVal>(
                                          static_cast<int>(CHRegs::CHPRILVL::PRILVLVal::lvl0)
                                          + static_cast<int>(Priority))>()),

                  Kvasir::Register::sequencePoint,
                  CHRegs::CHCTRLA::overrideDefaults(
                    write(CHRegs::CHCTRLA::TRIGACTValC::burst),
                    write(CHRegs::CHCTRLA::trigsrc,
                          Register::value<typename CHRegs::CHCTRLA::TRIGSRCVal,
                                          static_cast<typename CHRegs::CHCTRLA::TRIGSRCVal>(
                                            Trigger)>())),
                  Kvasir::Register::sequencePoint,
                  set(CHRegs::CHCTRLA::enable));
            }
        }

        template<DMAChannel    Channel,
                 DMAPriority   Priority,
                 TriggerSource Trigger>
        static constexpr auto start() {
            return start<Channel, Priority, Trigger, Regs>();
        }

        /// Disable one channel: what an aborted transfer needs, so the channel stops moving
        /// bytes and the next start() finds it idle.
        template<DMAChannel Channel,
                 typename Regs_t>
        static constexpr auto stop() {
            static_assert(numberOfChannels > static_cast<std::size_t>(Channel));

            if constexpr(Traits::DmacTraits::OldImpl) {
                return list(
                  write(
                    Regs_t::CHID::id,
                    Kvasir::Register::value<std::uint8_t, static_cast<std::uint8_t>(Channel)>()),
                  Kvasir::Register::sequencePoint,
                  clear(Regs_t::CHCTRLA::enable));
            } else {
                using CHRegs = typename Regs_t::template CHANNEL<static_cast<std::size_t>(Channel)>;
                return list(clear(CHRegs::CHCTRLA::enable));
            }
        }

        template<DMAChannel Channel>
        static constexpr auto stop() {
            return stop<Channel, Regs>();
        }
    };

}   // namespace DMAC

namespace CRC {

    namespace detail {
        template<typename CRC>
        void reset_crc() {
            if constexpr(requires {
                             { CRC::CTRL::crcenable };
                         })
            {
                if(1 == apply(read(CRC::CTRL::crcenable))) { apply(set(CRC::CRCSTATUS::crcbusy)); }
                apply(clear(CRC::CTRL::crcenable));
            } else {
                apply(CRC::CRCCTRL::overrideDefaults());
            }
        }

        template<typename CRC>
        void enable_crc() {
            if constexpr(requires {
                             { CRC::CTRL::crcenable };
                         })
            {
                apply(set(CRC::CTRL::crcenable));
            }
        }
    }   // namespace detail

    enum class CRC_Type { crc16, crc32 };

    template<CRC_Type Crc,
             typename It>
    auto calcCrc(It start,
                 It end) {
        using CRC = Peripheral::DMAC::Registers<>;

        static constexpr std::size_t dataSize = sizeof(decltype(*start));
        static_assert(dataSize == 1 || dataSize == 2 || dataSize == 4, "wrong data size");

        detail::reset_crc<CRC>();

        auto crcTypes = []() {
            if constexpr(Crc == CRC_Type::crc16) {
                return std::make_tuple(write(CRC::CRCCTRL::CRCPOLYValC::crc16),
                                       read(CRC::CRCCHKSUM::crcchksum16));
            } else {
                return std::make_tuple(write(CRC::CRCCTRL::CRCPOLYValC::crc32),
                                       read(CRC::CRCCHKSUM::crcchksum32));
            }
        }();

        auto beatsize = []() {
            if constexpr(dataSize == 1) {
                return write(CRC::CRCCTRL::CRCBEATSIZEValC::byte);
            } else if constexpr(dataSize == 2) {
                return write(CRC::CRCCTRL::CRCBEATSIZEValC::hword);
            } else {
                return write(CRC::CRCCTRL::CRCBEATSIZEValC::word);
            }
        }();

        apply(write(CRC::CRCCHKSUM::crcchksum32, Kvasir::Register::value<0xFFFFFFFF>()));

        apply(std::get<0>(crcTypes), beatsize, write(CRC::CRCCTRL::CRCSRCValC::io));

        detail::enable_crc<CRC>();
        while(start != end) {
            if constexpr(dataSize == 1) {
                apply(write(CRC::CRCDATAIN::crcdatain8, static_cast<std::uint8_t>(*start)));
            } else if constexpr(dataSize == 2) {
                apply(write(CRC::CRCDATAIN::crcdatain16, static_cast<std::uint16_t>(*start)));
            } else {
                apply(write(CRC::CRCDATAIN::crcdatain, static_cast<std::uint32_t>(*start)));
            }
            ++start;
        }
        apply(set(CRC::CRCSTATUS::crcbusy));
        return static_cast<
          std::conditional_t<Crc == CRC_Type::crc16, std::uint16_t, std::uint32_t>>(
          Kvasir::Register::get<0>(apply(std::get<1>(crcTypes))));
    }

    template<CRC_Type Crc,
             typename T>
    auto calcCrc(T const& v) {
        return calcCrc<Crc>(reinterpret_cast<std::uint8_t const*>(&v),
                            reinterpret_cast<std::uint8_t const*>(&v + 1));
    }

}   // namespace CRC

}   // namespace Kvasir
