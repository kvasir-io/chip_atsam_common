#pragma once
// The SAM D2x/C2x DMAC's channel layout for the shared driver (DMAC.hpp), over the chip's generated DMAC registers:
// "Up to 12 Channels" (D21 DS40001882L 20.2, md l.12296; C21 DS60001479M 25.2, md l.15080), and one register window
// for all of them, the channel picked by "the channel number ... written to the Channel ID (CHID) register" (D21
// 20.6.2.1, md l.12560). The chip's Dmac_Traits.hpp derives its DmacTraits from it:
//
//     struct DmacTraits : Sam::DmacChannelWindow<Kvasir::Peripheral::DMAC::Registers<>> {};
//
// The D5x/E5x DMAC has 32 channels with a register block each: its chip writes its own DmacTraits with
// `channelSelectedByChid = false`.
#include <cstddef>

namespace Kvasir { namespace Sam {
    template<typename Dmac>
    struct DmacChannelWindow {
        static constexpr std::size_t channels              = 12;
        static constexpr bool        channelSelectedByChid = true;
        // "If data is written to CRCDATAIN in two consecutive instructions, the CRC computation may be incorrect":
        // a NOP between two writes (D21 errata DS80000760M 1.7.1, revisions A-F; C21 DS80000740T 1.10.1, E/G/J
        // revision B - the revision tables read on the PDF pages, 14 and 39)
        static constexpr bool crcDataInWritesNeedAGap = true;

        using TriggerSource = typename Dmac::CHCTRLB::TRIGSRCVal;
    };
}}   // namespace Kvasir::Sam
