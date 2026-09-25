#pragma once
#include "kvasir/Register/Utility.hpp"
#include "peripherals/EVSYS.hpp"

namespace Kvasir { namespace EVSYS {
    template<typename EVSYSConfig>
    struct EvsysBase {
        // needed config
        // numberOfChannels
        using Regs      = Kvasir::Peripheral::EVSYS::Registers<>;
        using Path      = Regs::CHANNEL::PATHVal;
        using Edge      = Regs::CHANNEL::EDGSELVal;
        using User      = Regs::USER::USERVal;
        using Generator = Regs::CHANNEL::EVGENVal;

        enum class Channel {
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
            ch11 = 11
        };

        static constexpr auto powerClockEnable
          = list(typename PM::enable<Regs::baseAddr>::action{});
        static constexpr auto initStepPeripheryConfig
          = list(set(Regs::CTRL::gclkreq), clear(Regs::CTRL::swrst));

        //TODO
        template<Channel channel>
        static auto triggerEventChannel()
          -> decltype(list(write(Regs::CHANNEL_TRIG::channel,
                                 Register::value<std::uint16_t,
                                                 static_cast<std::uint16_t>(channel)>()),
                           set(Regs::CHANNEL_TRIG::swevt))) {
            return {};
        }

        template<Channel   channel,
                 Generator generator,
                 User      user,
                 Path      path = Path::synchronous,
                 Edge      edge = Edge::rising_edge>
        static auto setupEventChannel() -> decltype(list(
          write(Regs::USER::channel,
                Register::value<static_cast<unsigned>(channel) + 1>()),
          write(Register::FieldValue<typename decltype(Regs::USER::user)::type,
                                     user>{}),
          Register::sequencePoint,
          write(Regs::CHANNEL::channel,
                Register::value<static_cast<unsigned>(channel)>()),
          clear(Regs::CHANNEL::swevt),
          write(Register::FieldValue<typename decltype(Regs::CHANNEL::evgen)::type,
                                     generator>{}),
          write(Register::FieldValue<typename decltype(Regs::CHANNEL::path)::type,
                                     path>{}),
          write(Register::FieldValue<typename decltype(Regs::CHANNEL::edgsel)::type,
                                     edge>{}))) {
            return {};
        }
    };

}}   // namespace Kvasir::EVSYS
