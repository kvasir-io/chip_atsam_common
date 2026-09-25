#pragma once
#include "chip/atsam_common/Io.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"

#include <limits>

namespace Kvasir { namespace CAN { namespace Traits {

    template<typename Child>
    struct CanTraitsBase {
        struct PinInfo {
            enum Dir { RX, TX };

            int canInstance;
            int port;
            int pin;
            int pinFunction;
            Dir dir;
        };

        template<unsigned CanInstance>
        static constexpr auto getCanIsrIndexs() {
            return typename Child::template IsrIndex<CanInstance>::Type{};
        }

        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr int GetPinFunction(Kvasir::Register::PinLocation<Port,
                                                                          Pin>) {
            if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) {
                return std::numeric_limits<int>::max();
            }
            for(auto mi : Child::pinMuxInfos) {
                if(mi.canInstance != CanInstance) { continue; }
                if(mi.port != Port) { continue; }
                if(mi.pin != Pin) { continue; }
                return mi.pinFunction;
            }
            return std::numeric_limits<int>::max();
        }

        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr bool isDir(Kvasir::Register::PinLocation<Port,
                                                                  Pin>,
                                    typename PinInfo::Dir dir) {
            if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) { return false; }
            for(auto mi : Child::pinMuxInfos) {
                if(mi.canInstance != CanInstance) { continue; }
                if(mi.port != Port) { continue; }
                if(mi.pin != Pin) { continue; }
                if(mi.dir != dir) { continue; }
                return true;
            }
            return false;
        }

        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr bool isRX(Kvasir::Register::PinLocation<Port,
                                                                 Pin> p) {
            return isDir<CanInstance>(p, PinInfo::RX);
        }

        template<unsigned CanInstance,
                 int      Port,
                 int      Pin>
        static constexpr bool isTX(Kvasir::Register::PinLocation<Port,
                                                                 Pin> p) {
            return isDir<CanInstance>(p, PinInfo::TX);
        }
    };

}}}   // namespace Kvasir::CAN::Traits
