#pragma once

#include "chip/Io.hpp"
#include "kvasir/Hal/Pins.hpp"
#include "kvasir/Io/Claims.hpp"
#include "kvasir/Io/Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"

#include <cstdint>
#include <type_traits>

// The SAM pin actions on the generated PORT registers, through the chip's `Sam::PortGroup<Port>` (chip/Io.hpp). DIRCLR,
// DIRSET, OUTCLR, OUTSET and OUTTGL are marked one-to-clear/-set/-toggle in the chip's SVD ("Writing a '0' to a bit has
// no effect"), so a pin's bit of them is one store, and two pins of one group in one apply() are one store too. A pin's
// PINCFG is written whole by every configuring action (PMUXEN, INEN, PULLEN, DRVSTR; the reserved bits 7 and 5:3 go out
// 0 - DS40001882L Table 46-4, md l.51430: "always write reserved bits to zero"): one store, no read. PMUX is a nibble of
// a byte register shared by a pin pair: a read-modify-write.
//
// Pin ownership: the PMUXEN write of a pin's configuration in an entry's initStepPinConfig provides
// `Io::PinResource<Port, Pin>` (every configuring action below writes it exactly once; Startup derives provides from
// that phase only, Kvasir_SDK StartUp/Resources.hpp), so two entries that configure one pin there are a compile error
// naming the pin (chip_atsamd21 tests/conformance pins_must_fail), and a driver may claim a pin it does not configure
// (`Io::PinClaims`).
namespace Kvasir { namespace Io {
    template<int Port,
             int Pin>
    constexpr bool isValidPinLocation() {
        return Hal::Pins::isBonded<PinLocationTraits<void>>(Port, Pin);   // the chip's bonded masks
    }

    namespace Detail {
        template<int Port>
        using Group = Sam::PortGroup<Port>;

        template<typename F>
        using FieldOf = std::remove_cvref_t<F>;

        // the pin's bit of a group register's one 32-bit field
        template<int Pin, typename F, typename T = std::uint32_t>
        using PinBit = decltype(Register::bit<static_cast<unsigned>(Pin), T>(F{}));

        template<int Port, int Pin>
        using DirClr
          = decltype(Register::reset(PinBit<Pin, FieldOf<decltype(Group<Port>::dirclr)>>{}));
        template<int Port, int Pin>
        using DirSet
          = decltype(Register::set(PinBit<Pin, FieldOf<decltype(Group<Port>::dirset)>>{}));
        template<int Port, int Pin>
        using OutClr
          = decltype(Register::reset(PinBit<Pin, FieldOf<decltype(Group<Port>::outclr)>>{}));
        template<int Port, int Pin>
        using OutSet
          = decltype(Register::set(PinBit<Pin, FieldOf<decltype(Group<Port>::outset)>>{}));
        template<int Port, int Pin>
        using OutTgl
          = decltype(Register::toggle(PinBit<Pin, FieldOf<decltype(Group<Port>::outtgl)>>{}));
        template<int Port, int Pin>
        using In
          = decltype(Register::read(PinBit<Pin, FieldOf<decltype(Group<Port>::in)>, bool>{}));

        // a pulled input's pull direction is its OUT bit (DS40001882L Table 23-2); pullNone clears DIR once more
        template<PullConfiguration PC, int Port, int Pin>
        using PullDirection
          = std::conditional_t<PC == PullConfiguration::pullNone,
                               DirClr<Port, Pin>,
                               std::conditional_t<PC == PullConfiguration::pullDown,
                                                  OutClr<Port, Pin>,
                                                  OutSet<Port, Pin>>>;

        template<bool On, typename F>
        using SetOrClear
          = std::conditional_t<On, decltype(Register::set(F{})), decltype(Register::clear(F{}))>;

        template<int Port, int Pin>
        using PincfgReg = typename Group<Port>::template Pincfg<Pin>;

        // the whole PINCFG of the pin: PMUXEN, INEN, PULLEN, DRVSTR
        template<int Port, int Pin, bool PmuxEn, bool InEn, bool PullEn, bool DrvStr>
        using Pincfg
          = brigand::list<SetOrClear<PmuxEn, FieldOf<decltype(PincfgReg<Port, Pin>::pmuxen)>>,
                          SetOrClear<InEn, FieldOf<decltype(PincfgReg<Port, Pin>::inen)>>,
                          SetOrClear<PullEn, FieldOf<decltype(PincfgReg<Port, Pin>::pullen)>>,
                          SetOrClear<DrvStr, FieldOf<decltype(PincfgReg<Port, Pin>::drvstr)>>>;

        // the pin's nibble of its pair's PMUX: PMUXE for an even pin, PMUXO for an odd one (the D21's fields are
        // enumerations, the C21's and E51's integers: the value goes in as the field's own type)
        template<int Port, int Pin, int Function>
        struct PmuxOf {
            using Reg = typename Group<Port>::template Pmux<Pin>;
            using F   = FieldOf<
              std::conditional_t<(Pin % 2) != 0, decltype(Reg::pmuxo), decltype(Reg::pmuxe)>>;
            using T = typename F::DataType;
            using type
              = decltype(Register::write(F{}, Register::value<T, static_cast<T>(Function)>()));
        };

        template<int Port, int Pin, int Function>
        using Pmux = typename PmuxOf<Port, Pin, Function>::type;

        // the port and pin of a PINCFG address (PINCFGy at 0x40 + y of each 0x80 group; DS40001882L 23.7, DS60001479M
        // 28.7, DS60001507N 32.7), -1 for any other address
        inline constexpr unsigned groupStride = 0x80;
        inline constexpr unsigned pincfgBase  = 0x40;
        inline constexpr unsigned pmuxenMask  = 0x01;   // PINCFGy.PMUXEN, bit 0

        consteval int pincfgPort(unsigned address) {
            constexpr unsigned base = PinLocationTraits<void>::baseAddress;
            constexpr auto     end  = static_cast<unsigned>(PinLocationTraits<void>::portEnd);
            if(address < base) { return -1; }
            unsigned const offset = address - base;
            unsigned const inside = offset % groupStride;
            if(offset / groupStride >= end || inside < pincfgBase || inside >= pincfgBase + 32U) {
                return -1;
            }
            return static_cast<int>(offset / groupStride);
        }

        consteval int pincfgPin(unsigned address) {
            return static_cast<int>((address - PinLocationTraits<void>::baseAddress) % groupStride
                                    - pincfgBase);
        }
    }   // namespace Detail

    template<Io::PullConfiguration PC, int Port, int Pin>
    struct MakeAction<Action::Input<PC>, PinLocation<Port, Pin>>
      : brigand::flatten<brigand::list<
          Detail::DirClr<Port, Pin>,
          Detail::Pincfg<Port, Pin, false, true, PC != PullConfiguration::pullNone, false>,
          Detail::PullDirection<PC, Port, Pin>>> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<Io::OutputType OT, Io::OutputSpeed OS, Io::OutputInit OI, int Port, int Pin>
    struct MakeAction<Action::Output<OT, OS, OI>, PinLocation<Port, Pin>>
      : brigand::flatten<
          brigand::list<std::conditional_t<OI == Io::OutputInit::high,
                                           Detail::OutSet<Port, Pin>,
                                           Detail::OutClr<Port, Pin>>,
                        Detail::DirSet<Port, Pin>,
                        Detail::Pincfg<Port, Pin, false, true, false, OS != OutputSpeed::low>>> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
        static_assert(OT == Io::OutputType::pushPull,
                      "only push pull supported");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Clear, PinLocation<Port, Pin>> : Detail::OutClr<Port, Pin> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Set, PinLocation<Port, Pin>> : Detail::OutSet<Port, Pin> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Toggle, PinLocation<Port, Pin>> : Detail::OutTgl<Port, Pin> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Read, PinLocation<Port, Pin>> : Detail::In<Port, Pin> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<Io::OutputType    OT,
             Io::OutputSpeed   OS,
             Io::OutputInit    OI,
             PullConfiguration PC,
             int               Port,
             int               Pin,
             int               Function>
    struct MakeAction<Action::PinFunction<Function, OT, OS, OI, PC>, PinLocation<Port, Pin>>
      : brigand::flatten<brigand::list<Detail::Pmux<Port, Pin, Function>,
                                       Detail::Pincfg<Port,
                                                      Pin,
                                                      true,
                                                      true,
                                                      PC != PullConfiguration::pullNone,
                                                      OS != OutputSpeed::low>,
                                       Detail::PullDirection<PC, Port, Pin>>> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
        static_assert(OT == Io::OutputType::pushPull,
                      "only push pull supported");
    };
}}   // namespace Kvasir::Io

namespace Kvasir { namespace Startup {
    // The PMUXEN write of a pin's PINCFG in initStepPinConfig provides the pin (see the top of this file).
    template<unsigned Addr,
             unsigned Z,
             unsigned O,
             typename RegType,
             typename Mode,
             typename Access,
             typename FieldType,
             typename TAction>
        requires(Io::Detail::pincfgPort(Addr) >= 0)
    struct ResourceOfAction<
      Register::Action<Register::FieldLocation<Register::Address<Addr, Z, O, RegType, Mode>,
                                               Io::Detail::pmuxenMask,
                                               Access,
                                               FieldType>,
                       TAction>> {
        using type = brigand::list<
          Io::PinResource<Io::Detail::pincfgPort(Addr), Io::Detail::pincfgPin(Addr)>>;
    };
}}   // namespace Kvasir::Startup
