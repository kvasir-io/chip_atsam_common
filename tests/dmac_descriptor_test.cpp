// DmacDescriptor.hpp: the DMAC transfer descriptor's builder against BTCTRL's bit table (D21 DS40001882L 20.10.1 md
// l.14588-14681; the C21 and E5x tables are the same bits, DS60001479M 25.10.1, DS60001507N 22.10.1): every field
// alone at its bits, the word the 11-argument constructor it replaced built (that formula kept here as the oracle,
// over every value of every field), the defaults, setValid, and the descriptor's 16 bytes in their order.
#include "../DmacDescriptor.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace {
using D = Kvasir::DMAC::DmacDescriptor;

// the replaced constructor's BTCTRL expression, verbatim
constexpr std::uint16_t oldWord(bool        valid,
                                D::Stepsize stsize,
                                D::Stepsel  stsel,
                                D::Dstinc   dinc,
                                D::Srcinc   sinc,
                                D::Beatsize bsize,
                                D::Blockact bact,
                                D::Evosel   esel) {
    return std::uint16_t((valid ? 1U : 0U) | (unsigned(esel) << 1U) | (unsigned(bact) << 3U)
                         | (unsigned(bsize) << 8U) | (unsigned(sinc) << 10U)
                         | (unsigned(dinc) << 11U) | (unsigned(stsel) << 12U)
                         | (unsigned(stsize) << 13U));
}

constexpr bool everyCombination() {
    constexpr D::Evosel   evs[]{D::Evosel::disabled, D::Evosel::block, D::Evosel::beat};
    constexpr D::Beatsize bss[]{D::Beatsize::byte, D::Beatsize::hword, D::Beatsize::word};
    for(unsigned valid = 0; valid != 2; ++valid) {
        for(unsigned st = 0; st != 8; ++st) {
            for(unsigned sel = 0; sel != 2; ++sel) {
                for(unsigned di = 0; di != 2; ++di) {
                    for(unsigned si = 0; si != 2; ++si) {
                        for(auto const bs : bss) {
                            for(unsigned ba = 0; ba != 4; ++ba) {
                                for(auto const ev : evs) {
                                    D::Transfer const t{.valid    = valid != 0,
                                                        .stepSize = D::Stepsize(st),
                                                        .stepSel  = D::Stepsel(sel),
                                                        .dstInc   = D::Dstinc(di),
                                                        .srcInc   = D::Srcinc(si),
                                                        .beatSize = bs,
                                                        .blockAct = D::Blockact(ba),
                                                        .evOutput = ev};
                                    if(D::btctrlOf(t)
                                       != oldWord(valid != 0,
                                                  t.stepSize,
                                                  t.stepSel,
                                                  t.dstInc,
                                                  t.srcInc,
                                                  bs,
                                                  t.blockAct,
                                                  ev))
                                    {
                                        return false;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}

static_assert(everyCombination());

// each field at the bits the table gives it
static_assert(D::btctrlOf({.valid = false}) == 0x0000);
static_assert(D::btctrlOf({}) == 0x0001);   // the defaults: valid, everything else 0
static_assert(D::btctrlOf({.valid    = false,
                           .evOutput = D::Evosel::beat})
              == 0x0006);   // EVOSEL 2:1
static_assert(D::btctrlOf({.valid    = false,
                           .blockAct = D::Blockact::both})
              == 0x0018);   // BLOCKACT 4:3
static_assert(D::btctrlOf({.valid    = false,
                           .beatSize = D::Beatsize::word})
              == 0x0200);   // BEATSIZE 9:8
static_assert(D::btctrlOf({.valid  = false,
                           .srcInc = D::Srcinc::increment})
              == 0x0400);   // SRCINC 10
static_assert(D::btctrlOf({.valid  = false,
                           .dstInc = D::Dstinc::increment})
              == 0x0800);   // DSTINC 11
static_assert(D::btctrlOf({.valid   = false,
                           .stepSel = D::Stepsel::src})
              == 0x1000);   // STEPSEL 12
static_assert(D::btctrlOf({.valid    = false,
                           .stepSize = D::Stepsize::x128})
              == 0xE000);   // STEPSIZE 15:13

constexpr D sample{
  {.srcInc = D::Srcinc::increment, .count = 7, .src = 0x2000'0010, .dst = 0x4200'0828, .next = 0}
};
static_assert(sample.btctrl == 0x0401 && sample.btcnt == 7 && sample.srcaddr == 0x2000'0010
              && sample.dstaddr == 0x4200'0828 && sample.descaddr == 0);

constexpr bool validToggles() {
    D d{
      {.valid = false, .beatSize = D::Beatsize::hword}
    };
    d.setValid(true);
    bool const on = d.btctrl == 0x0101;
    d.setValid(false);
    return on && d.btctrl == 0x0100;
}

static_assert(validToggles());

// 20.10: BTCTRL 0x00, BTCNT 0x02, SRCADDR 0x04, DSTADDR 0x08, DESCADDR 0x0C; 128-bit aligned (BASEADDR, 20.8.13)
static_assert(offsetof(D,
                       btctrl)
                == 0
              && offsetof(D,
                          btcnt)
                   == 2
              && offsetof(D,
                          srcaddr)
                   == 4
              && offsetof(D,
                          dstaddr)
                   == 8
              && offsetof(D,
                          descaddr)
                   == 12);
static_assert(alignof(D) == 16 && sizeof(D) == 16);
}   // namespace

int main() {
    D const d{{.count = 3}};
    std::printf("dmac_descriptor: BTCTRL %#06x, isValid %d\n",
                unsigned{d.btctrl},
                d.isValid() ? 1 : 0);
    return d.isValid() ? 0 : 1;
}
