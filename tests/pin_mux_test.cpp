// PinMux.hpp: a pin's row in a pin-multiplexing table and its PORT function action, over chip-free fake tables. Every
// check is a static_assert: building is the test.
#include "../PinMux.hpp"

#include <array>
#include <type_traits>

namespace {
struct Row {
    int instance;
    int port;
    int pin;
    int pinFunction;
    int pad;
};

// SERCOM-like: instance 0 on PA04/PA05 (function D), instance 2 on PA08 (D) and instance 0 on PA08 (C); PA05 repeated
// with another pad to show which row a repeated one gives
constexpr std::array table{
  Row{0, 0, 4, 3, 0},
  Row{0, 0, 5, 3, 1},
  Row{2, 0, 8, 3, 0},
  Row{0, 0, 8, 2, 0},
  Row{0, 0, 5, 3, 3},
};

static_assert(Kvasir::Sam::findMux(table,
                                   0,
                                   0,
                                   4)
                ->pad
              == 0);
static_assert(Kvasir::Sam::findMux(table,
                                   0,
                                   0,
                                   5)
                  ->pad
                == 1,
              "the first matching row wins");
static_assert(!Kvasir::Sam::findMux(table,
                                    0,
                                    0,
                                    6),
              "no row: none");
static_assert(!Kvasir::Sam::findMux(table,
                                    0,
                                    1,
                                    4),
              "another port: none");
static_assert(!Kvasir::Sam::findMux(table,
                                    1,
                                    0,
                                    4),
              "another instance's row is not found");
static_assert(Kvasir::Sam::findMux(table,
                                   2,
                                   0,
                                   8)
                ->pinFunction
              == 3);
static_assert(Kvasir::Sam::findMux(table,
                                   0,
                                   0,
                                   8)
                  ->pinFunction
                == 2,
              "one pin, two instances: each its own row");
static_assert(!Kvasir::Sam::findMux(std::array<Row,
                                               0>{},
                                    0,
                                    0,
                                    0),
              "an empty table");

// the traits shape the drivers hand to PinFunctionOf
struct FakeTraits {
    template<unsigned Instance,
             int      Port,
             int      Pin>
    static constexpr int getPinFunction(Kvasir::Io::PinLocation<Port,
                                                                Pin>) {
        auto const row = Kvasir::Sam::findMux(table, static_cast<int>(Instance), Port, Pin);
        return row ? row->pinFunction : -1;
    }
};

template<int Function, int Port, int Pin>
struct FakePinAction {};
}   // namespace

// what the chip's Io.hpp does for a real part: a PinFunction action on a pin location is an action type
template<int Function, int Port, int Pin>
struct Kvasir::Io::MakeAction<Kvasir::Io::Action::PinFunction<Function>,
                              Kvasir::Io::PinLocation<Port, Pin>> {
    using type = FakePinAction<Function, Port, Pin>;
};

namespace {
using Kvasir::Io::PinLocation;
using Kvasir::Sam::PinFunctionOf;

static_assert(std::is_same_v<PinFunctionOf<FakeTraits,
                                           0,
                                           PinLocation<0,
                                                       4>>,
                             FakePinAction<3,
                                           0,
                                           4>>);
static_assert(std::is_same_v<PinFunctionOf<FakeTraits,
                                           0,
                                           PinLocation<0,
                                                       8>>,
                             FakePinAction<2,
                                           0,
                                           8>>);
static_assert(std::is_same_v<PinFunctionOf<FakeTraits,
                                           2,
                                           PinLocation<0,
                                                       8>>,
                             FakePinAction<3,
                                           0,
                                           8>>);
static_assert(std::is_same_v<PinFunctionOf<FakeTraits,
                                           0,
                                           Kvasir::Io::NotUsed<>>,
                             brigand::list<>>,
              "an unused pin: no pin configuration");
// the drivers name a config member's type: const-qualified
static_assert(std::is_same_v<PinFunctionOf<FakeTraits,
                                           0,
                                           PinLocation<0,
                                                       4> const>,
                             FakePinAction<3,
                                           0,
                                           4>>);
static_assert(std::is_same_v<PinFunctionOf<FakeTraits,
                                           0,
                                           Kvasir::Io::NotUsed<> const>,
                             brigand::list<>>);
}   // namespace

int main() { return 0; }
