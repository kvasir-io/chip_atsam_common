#pragma once
#include "chip/atsam_common/Io.hpp"
#include "chip/atsam_common/PinMux.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"

#include <limits>

namespace Kvasir { namespace CAN { namespace Traits {

    template<typename Child>
    struct CanTraitsBase {
        struct PinInfo {
            enum Dir { rx, tx };

            int instance;   // CANn
            int port;
            int pin;
            int pinFunction;
            Dir dir;
        };

        // the row of a bonded pin for CanInstance (PinMux.hpp), or none
        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr std::optional<PinInfo> rowOf() {
            if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) { return std::nullopt; }
            return Sam::findMux(Child::pinMuxInfos, CanInstance, Port, Pin);
        }

        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr int getPinFunction(Kvasir::Io::PinLocation<Port,
                                                                    Pin>) {
            auto const row = rowOf<CanInstance, Port, Pin>();
            return row ? row->pinFunction : std::numeric_limits<int>::max();
        }

        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr bool isDir(Kvasir::Io::PinLocation<Port,
                                                            Pin>,
                                    typename PinInfo::Dir dir) {
            auto const row = rowOf<CanInstance, Port, Pin>();
            return row && row->dir == dir;
        }

        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr bool isRX(Kvasir::Io::PinLocation<Port,
                                                           Pin> p) {
            return isDir<CanInstance>(p, PinInfo::rx);
        }

        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr bool isTX(Kvasir::Io::PinLocation<Port,
                                                           Pin> p) {
            return isDir<CanInstance>(p, PinInfo::tx);
        }
    };

}}}   // namespace Kvasir::CAN::Traits
