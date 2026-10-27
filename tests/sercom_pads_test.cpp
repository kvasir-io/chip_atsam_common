// SercomPads.hpp: the SERCOM pad tables (DOPO, TXPO) of the SAM D21/C21 and D5x/E5x, and the loop-back pin rule.
// Every check is a static_assert: building is the test. The refusals are must_fail cases (CMakeLists.txt).
#include "../SercomPads.hpp"
#include "kvasir/Io/Types.hpp"

namespace {
namespace P = Kvasir::Sam::Pads;
using Kvasir::Io::PinLocation;

// D21 DS40001882L 27.8.1 / C21 DS60001479M 32.8.1: DOPO 0x0..0x3 = (DO, SCK, SS) (0,1,2) (2,3,1) (3,1,2) (0,3,1)
static_assert(P::dopo<P::SamD2xC2x,
                      0,
                      1>()
                  .dopo
                == 0
              && P::dopo<P::SamD2xC2x,
                         0,
                         1>()
                     .ss
                   == 2);
static_assert(P::dopo<P::SamD2xC2x,
                      2,
                      3>()
                  .dopo
                == 1
              && P::dopo<P::SamD2xC2x,
                         2,
                         3>()
                     .ss
                   == 1);
static_assert(P::dopo<P::SamD2xC2x,
                      3,
                      1>()
                  .dopo
                == 2
              && P::dopo<P::SamD2xC2x,
                         3,
                         1>()
                     .ss
                   == 2);
static_assert(P::dopo<P::SamD2xC2x,
                      0,
                      3>()
                  .dopo
                == 3
              && P::dopo<P::SamD2xC2x,
                         0,
                         3>()
                     .ss
                   == 1);
static_assert(!P::findDopo<P::SamD2xC2x>(1,
                                         0),
              "SCK is never on PAD[0]");
static_assert(!P::findDopo<P::SamD2xC2x>(2,
                                         1),
              "MOSI on PAD[2] takes SCK on PAD[3]");

// E5x DS60001507N 35.8.1: DOPO 0x1 and 0x3 Reserved
static_assert(P::dopo<P::SamE5x,
                      0,
                      1>()
                  .dopo
                == 0
              && P::dopo<P::SamE5x,
                         0,
                         1>()
                     .ss
                   == 2);
static_assert(P::dopo<P::SamE5x,
                      3,
                      1>()
                  .dopo
                == 2
              && P::dopo<P::SamE5x,
                         3,
                         1>()
                     .ss
                   == 2);
static_assert(!P::findDopo<P::SamE5x>(2,
                                      3),
              "DOPO 0x1 is Reserved on the E5x");
static_assert(!P::findDopo<P::SamE5x>(0,
                                      3),
              "DOPO 0x3 is Reserved on the E5x");

// the hardware SS where the row puts it
static_assert((P::checkSs<P::dopo<P::SamD2xC2x,
                                  0,
                                  1>(),
                          2>(),
               true));
static_assert((P::checkSs<P::dopo<P::SamD2xC2x,
                                  2,
                                  3>(),
                          1>(),
               true));

// TXPO: D21 26.8.1 / C21 31.8.1 0x0 TxD PAD[0], 0x1 TxD PAD[2]; E5x 34.8.1 0x1 Reserved
static_assert(P::txpo<P::SamD2xC2x,
                      0>()
              == 0);
static_assert(P::txpo<P::SamD2xC2x,
                      2>()
              == 1);
static_assert(!P::findTxpo<P::SamD2xC2x>(1) && !P::findTxpo<P::SamD2xC2x>(3),
              "TxD is on PAD[0] or PAD[2]");
static_assert(P::txpo<P::SamE5x,
                      0>()
              == 0);
static_assert(!P::findTxpo<P::SamE5x>(2),
              "TXPO 0x1 is Reserved on the E5x");

static_assert(P::PadTable<P::SamD2xC2x> && P::PadTable<P::SamE5x>);

// loop-back: the receive pin is the transmit pin; without it the two differ, an unused receive pin always fits
static_assert(P::receivePinFits<true,
                                PinLocation<0,
                                            16>,
                                PinLocation<0,
                                            16>>());
static_assert(!P::receivePinFits<true,
                                 PinLocation<0,
                                             19>,
                                 PinLocation<0,
                                             16>>());
static_assert(!P::receivePinFits<true,
                                 Kvasir::Io::NotUsed<>,
                                 PinLocation<0,
                                             16>>());
static_assert(P::receivePinFits<false,
                                PinLocation<0,
                                            19>,
                                PinLocation<0,
                                            16>>());
static_assert(!P::receivePinFits<false,
                                 PinLocation<0,
                                             16>,
                                 PinLocation<0,
                                             16>>());
static_assert(P::receivePinFits<false,
                                Kvasir::Io::NotUsed<>,
                                PinLocation<0,
                                            16>>());
static_assert(P::checkReceivePin<true,
                                 PinLocation<1,
                                             22>,
                                 PinLocation<1,
                                             22>,
                                 "USART rx/tx">());
}   // namespace

#ifdef MUST_FAIL
    #if MUST_FAIL == 1
// E5x DOPO 0x1 (MOSI PAD[2], SCK PAD[3]): Reserved
constexpr auto row = P::dopo<P::SamE5x, 2, 3>();
    #elif MUST_FAIL == 2
// E5x DOPO 0x3 (MOSI PAD[0], SCK PAD[3]): Reserved
constexpr auto row = P::dopo<P::SamE5x, 0, 3>();
    #elif MUST_FAIL == 3
// E5x TXPO 0x1 (TxD PAD[2]): Reserved
constexpr auto txpo = P::txpo<P::SamE5x, 2>();
    #elif MUST_FAIL == 4
// loopBack with a MISO pin that is not the MOSI pin
static_assert(P::checkReceivePin<true,
                                 PinLocation<0,
                                             19>,
                                 PinLocation<0,
                                             16>,
                                 "SPI misoPinLocation/mosiPinLocation">());
    #elif MUST_FAIL == 5
// a hardware SS on another pad than the DOPO row's
static_assert((P::checkSs<P::dopo<P::SamD2xC2x,
                                  0,
                                  1>(),
                          1>(),
               true));
    #endif
#endif

int main() { return 0; }
