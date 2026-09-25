#pragma once

#include "chip/Io.hpp"
#include "kvasir/Io/Io.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"

namespace Kvasir { namespace Io {
    template<int Port,
             int Pin>
    constexpr bool isValidPinLocation() {
        if(!(Port < PinLocationTraits<void>::portEnd && Port >= PinLocationTraits<void>::portBegin))
        {
            return false;
        }

        if(!(Pin < PinLocationTraits<void>::pinEnd && Pin >= PinLocationTraits<void>::pinBegin)) {
            return false;
        }

        for(auto dp : PinLocationTraits<void>::PinsDisabled[Port]) {
            if(dp == PinLocationTraits<void>::ListEndIndicator) { break; }
            if(dp == Pin) { return false; }
        }

        return true;
    }

    namespace Detail {
        using namespace Register;

        template<unsigned A, int BitPos>
        using BlindSet
          = Register::Action<WOBitLocT<Register::Address<A, maskFromRange(31, 0)>, BitPos>,
                             WriteLiteralAction<(1U << unsigned(BitPos))>>;

        template<unsigned A, int BitPos>
        using Read = Register::Action<RWBitLocT<Register::Address<A, maskFromRange(31, 0)>, BitPos>,
                                      ReadAction>;

        static constexpr unsigned baseAddress = PinLocationTraits<void>::baseAddress;
        static constexpr unsigned portOffset  = 0x00000080;

        template<typename Address, int Position, typename TFieldType = bool>
        using RWBitLocT
          = FieldLocation<Address, (1U << unsigned(Position)), ReadWriteAccess, TFieldType>;

        template<int Port, int Pin>
        using CFG_BaseAddress = Register::
          Address<baseAddress + 0x40 + Port * portOffset + Pin, 0b10111000, 0, unsigned char>;

        template<int Port, int Pin, int Position, bool SetClr>
        using CFG_BitPos
          = Register::Action<RWBitLocT<CFG_BaseAddress<Port, Pin>, Position>,
                             WriteLiteralAction<((SetClr ? 1U : 0U) << unsigned(Position))>>;

        template<int Port, int Pin, bool SetClr>
        using CFG_PMUXEN = CFG_BitPos<Port, Pin, 0, SetClr>;
        template<int Port, int Pin, bool SetClr>
        using CFG_INEN = CFG_BitPos<Port, Pin, 1, SetClr>;

        template<int Port, int Pin, bool SetClr>
        using CFG_PULLEN = CFG_BitPos<Port, Pin, 2, SetClr>;

        template<int Port, int Pin, bool SetClr>
        using CFG_DRVSTR = CFG_BitPos<Port, Pin, 6, SetClr>;

        template<int Port, int Pin, int Function>
        using PMUX = Register::Action<
          FieldLocation<
            Register::
              Address<baseAddress + 0x30 + Port * portOffset + Pin / 2, 0, 0, unsigned char>,
            (Pin % 2) != 0 ? 0xF0U : 0x0FU,
            ReadWriteAccess,
            unsigned char>,
          WriteLiteralAction<unsigned(Function) << ((Pin % 2) != 0 ? 4U : 0U)>>;
    }   // namespace Detail

    template<Io::PullConfiguration PC, int Port, int Pin>
    struct MakeAction<Action::Input<PC>, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::BlindSet<Detail::baseAddress + 0x04 + Port * Detail::portOffset,
                                            Pin>{},   // DIRCLR
                           Detail::CFG_PMUXEN<Port, Pin, false>{},
                           Detail::CFG_INEN<Port, Pin, true>{},
                           Detail::CFG_PULLEN<Port, Pin, PC != Io::PullConfiguration::PullNone>{},
                           Detail::CFG_DRVSTR<Port, Pin, false>{},
                           Detail::BlindSet<Detail::baseAddress
                                              + (PC == PullConfiguration::PullNone   ? 0x04
                                                 : PC == PullConfiguration::PullDown ? 0x14
                                                                                     : 0x18)
                                              + Port * Detail::portOffset,
                                            Pin>{}   // DIRCLR || OUTSET || OUTCLR
                           )) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<Io::OutputType OT, Io::OutputSpeed OS, Io::OutputInit OI, int Port, int Pin>
    struct MakeAction<Action::Output<OT, OS, OI>, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(
          Detail::BlindSet<Detail::baseAddress + (OI == Io::OutputInit::High ? 0x18 : 0x14)
                             + Port * Detail::portOffset,
                           Pin>{},   // OUTSET || OUTCLR
          Detail::BlindSet<Detail::baseAddress + 0x08 + Port * Detail::portOffset,
                           Pin>{},   // DIRSET
          Detail::CFG_PMUXEN<Port, Pin, false>{},
          Detail::CFG_INEN<Port, Pin, true>{},
          Detail::CFG_PULLEN<Port, Pin, false>{},
          Detail::CFG_DRVSTR<Port, Pin, OS != OutputSpeed::Low>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
        static_assert(OT == Io::OutputType::PushPull, "only push pull supported");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Clear, Register::PinLocation<Port, Pin>>
      : Detail::BlindSet<Detail::baseAddress + 0x14 + Port * Detail::portOffset, Pin>   // OUTCLR
    {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Set, Register::PinLocation<Port, Pin>>
      : Detail::BlindSet<Detail::baseAddress + 0x18 + Port * Detail::portOffset, Pin>   // OUTSET
    {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Toggle, Register::PinLocation<Port, Pin>>
      : Detail::BlindSet<Detail::baseAddress + 0x1C + Port * Detail::portOffset, Pin>   // OUTTGL
    {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Read, Register::PinLocation<Port, Pin>>
      : Detail::Read<Detail::baseAddress + 0x20 + Port * Detail::portOffset, Pin>   // IN
    {
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
    struct MakeAction<Action::PinFunction<Function, OT, OS, OI, PC>,
                      Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::PMUX<Port, Pin, Function>{},
                           Detail::CFG_PMUXEN<Port, Pin, true>{},
                           Detail::CFG_INEN<Port, Pin, true>{},
                           Detail::CFG_PULLEN<Port, Pin, PC != Io::PullConfiguration::PullNone>{},
                           Detail::CFG_DRVSTR<Port, Pin, OS != OutputSpeed::Low>{},
                           Detail::BlindSet<Detail::baseAddress
                                              + (PC == PullConfiguration::PullNone   ? 0x04
                                                 : PC == PullConfiguration::PullDown ? 0x14
                                                                                     : 0x18)
                                              + Port * Detail::portOffset,
                                            Pin>{}   // DIRCLR || OUTSET || OUTCLR
                           )) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
        static_assert(OT == Io::OutputType::PushPull, "only push pull supported");
    };
}}   // namespace Kvasir::Io
