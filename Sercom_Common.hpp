#pragma once
#include "chip/atsam_common/Io.hpp"
#include "chip/atsam_common/PinMux.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"

#include <limits>
#include <type_traits>

namespace Kvasir { namespace Sercom { namespace Traits {

    template<typename Child>
    struct SercomTraitsBase {
        struct PinInfo {
            int instance;   // SERCOMn
            int port;
            int pin;
            int pinFunction;
            int poVal;   // the pad
        };

        // the row of a bonded pin for SercomInstance (PinMux.hpp), or none
        template<unsigned SercomInstance,
                 int      Port,
                 int      Pin>
        static constexpr std::optional<PinInfo> rowOf() {
            if(!Kvasir::Io::isValidPinLocation<Port, Pin>()) { return std::nullopt; }
            return Sam::findMux(Child::pinMuxInfos, SercomInstance, Port, Pin);
        }

        template<unsigned SercomInstance,
                 int      Port,
                 int      Pin>
        static constexpr int getPinFunction(Kvasir::Io::PinLocation<Port,
                                                                    Pin>) {
            auto const row = rowOf<SercomInstance, Port, Pin>();
            return row ? row->pinFunction : std::numeric_limits<int>::max();
        }

        template<unsigned SercomInstance,
                 int      Port,
                 int      Pin>
        static constexpr int getPOVal(Kvasir::Io::PinLocation<Port,
                                                              Pin>) {
            auto const row = rowOf<SercomInstance, Port, Pin>();
            return row ? row->poVal : std::numeric_limits<int>::max();
        }

        template<unsigned SercomInstance,
                 int      Port,
                 int      Pin,
                 typename... Ts>
        static constexpr bool validIfPOVal(Ts... pOs) {
            auto const row = rowOf<SercomInstance, Port, Pin>();
            return row && ((row->poVal == pOs) || ...);
        }
    };

}}}   // namespace Kvasir::Sercom::Traits

namespace Kvasir { namespace Sercom {
    /// A compile-time value in the field's own type: the C21's SERCOM_SPIM / SERCOM_USART_INT give the pad, form and
    /// clock-polarity fields enumerations (DOPOVal, TXPOVal, CPOLVal, ...), the D21's and E51's plain integers - the
    /// register layer takes a value of exactly the field's type (Register/Factories.hpp, ValueTypeMismatch).
    template<typename Field,
             auto V>
    constexpr auto fieldValue() {
        using T = typename std::remove_cvref_t<Field>::DataType;
        return Register::value<T, static_cast<T>(V)>();
    }
}}   // namespace Kvasir::Sercom
