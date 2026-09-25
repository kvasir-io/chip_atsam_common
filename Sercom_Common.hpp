#pragma once
#include "chip/atsam_common/Io.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"

#include <limits>

namespace Kvasir { namespace Sercom { namespace Traits {

    template<typename Child>
    struct SercomTraitsBase {
        struct PinInfo {
            int sercomInstance;
            int port;
            int pin;
            int pinFunction;
            int POVal;
        };

        template<unsigned SercomInstance>
        static constexpr auto getSercomIsrIndexs() {
            return typename Child::template IsrIndex<SercomInstance>::Type{};
        }

        template<unsigned SercomInstance,
                 int      Port,
                 int      Pin>
        static constexpr int GetPinFunction(Kvasir::Register::PinLocation<Port,
                                                                          Pin>) {
            if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) {
                return std::numeric_limits<int>::max();
            }
            for(auto mi : Child::pinMuxInfos) {
                if(mi.sercomInstance != SercomInstance) { continue; }
                if(mi.port != Port) { continue; }
                if(mi.pin != Pin) { continue; }
                return mi.pinFunction;
            }
            return std::numeric_limits<int>::max();
        }

        template<unsigned SercomInstance,
                 int      Port,
                 int      Pin>
        static constexpr int GetPOVal(Kvasir::Register::PinLocation<Port,
                                                                    Pin>) {
            if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) {
                return std::numeric_limits<int>::max();
            }
            for(auto mi : Child::pinMuxInfos) {
                if(mi.sercomInstance != SercomInstance) { continue; }
                if(mi.port != Port) { continue; }
                if(mi.pin != Pin) { continue; }
                return mi.POVal;
            }
            return std::numeric_limits<int>::max();
        }

        template<unsigned SercomInstance,
                 int      Port,
                 int      Pin,
                 typename... Ts>
        static constexpr bool ValidIfPOVal(Ts... POs) {
            if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) { return false; }
            for(auto mi : Child::pinMuxInfos) {
                if(mi.sercomInstance != SercomInstance) { continue; }
                if(mi.port != Port) { continue; }
                if(mi.pin != Pin) { continue; }
                auto found = ((mi.POVal == POs) || ...);
                if(found) { return true; }
            }
            return false;
        }
    };

}}}   // namespace Kvasir::Sercom::Traits
