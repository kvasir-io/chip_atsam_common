#pragma once
// The SAM pin-multiplexing tables, looked up once: a chip lists one row per (instance, port, pin) a SERCOM or CAN
// pad reaches (chip/Sercom_Traits.hpp, chip/CAN_Traits.hpp), `findMux` finds a pin's row, `PinFunctionOf` turns a pin
// location into its PORT function action - or into nothing for an unused pin (`Kvasir::Io::NotUsed<>`).
//
// Register-free: `PinFunctionOf` goes through `Io::action()`, which resolves to the chip's `Io::MakeAction`
// (chip_atsam_common Io.hpp) - a host test specialises MakeAction for a fake. Whether a pin is bonded on this part
// (`Io::isValidPinLocation`) is the chip-side caller's check (Sercom_Common.hpp, CAN_Common.hpp).

#include "kvasir/Io/PinFactories.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Mpl/brigand.hpp"

#include <concepts>
#include <iterator>
#include <optional>
#include <type_traits>
#include <utility>

namespace Kvasir { namespace Sam {

    /// A row of a pin-multiplexing table: the instance (SERCOMn, CANn) a pad belongs to and the pin it is on.
    template<typename Row>
    concept MuxRow = requires(Row const& r) {
        { r.instance } -> std::convertible_to<int>;
        { r.port } -> std::convertible_to<int>;
        { r.pin } -> std::convertible_to<int>;
    };

    /// The first row of `table` for `instance` on (port, pin), or none: a table lists a pad once, and where a chip
    /// repeats one the first row is the one taken.
    template<typename Table>
        requires MuxRow<std::remove_cvref_t<decltype(*std::begin(std::declval<Table const&>()))>>
    constexpr auto findMux(Table const& table,
                           int          instance,
                           int          port,
                           int          pin)
      -> std::optional<std::remove_cvref_t<decltype(*std::begin(table))>> {
        for(auto const& row : table) {
            if(row.instance == instance && row.port == port && row.pin == pin) { return row; }
        }
        return std::nullopt;
    }

    namespace Detail {
        template<typename Traits, unsigned Instance, typename Location>
        struct
          PinFunctionOf;   // neither a PinLocation nor Io::NotUsed<>: no pin configuration for it

        template<typename Traits, unsigned Instance>
        struct PinFunctionOf<Traits, Instance, Io::NotUsed<>> {
            using type = brigand::list<>;
        };

        template<typename Traits, unsigned Instance, int Port, int Pin>
        struct PinFunctionOf<Traits, Instance, Io::PinLocation<Port, Pin>> {
            using type = decltype(Io::action(
              Io::Action::PinFunction<Traits::template getPinFunction<Instance>(
                Io::PinLocation<Port, Pin>{})>{},
              Io::PinLocation<Port, Pin>{}));
        };
    }   // namespace Detail

    /// The PORT function action that puts `Location` on its function for `Instance` (`Traits::GetPinFunction`), or
    /// an empty list for `Io::NotUsed<>`.
    template<typename Traits, unsigned Instance, typename Location>
    using PinFunctionOf =
      typename Detail::PinFunctionOf<Traits, Instance, std::remove_cvref_t<Location>>::type;

}}   // namespace Kvasir::Sam
