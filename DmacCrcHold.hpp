#pragma once
// Who holds the DMAC's CRC engine (DmacCrc.hpp, Kvasir::DmaCrc) right now, for a debug check. There is one engine per
// DMAC (CRCCTRL, CRCCHKSUM), every start resets it, and a DmacEngine keeps its CRC in CRCCHKSUM from its constructor
// to its destructor: a second user that starts it in between - an interrupt's calcCrc or engine - corrupts both CRCs
// without a word, and no Startup list rule can see it (the users of the image's engine are library calls, not list
// entries). With debug checks on (KVASIR_DCHECK_LEVEL > 0: the debug and sanitize variants, kvasir/Runtime/Check.hpp)
// such a start is a programming error and panics (Panic::Cause::checkFailed, the line names the engine); with them off
// nothing of this is in the image. Register-free, so it is host-tested (tests/dmac_crc_hold_test.cpp).
#include "kvasir/Runtime/Check.hpp"

#include <atomic>

namespace Kvasir::DmaCrc::detail {

struct Hold {
#if KVASIR_DCHECK_LEVEL > 0
    // A load and a store, no read-modify-write: inline on the Cortex-M0+ too, where an atomic exchange is a call into
    // the lock shim. An interrupt between the two is a window of a few instructions; the check is a debug aid, not
    // a lock.
    static inline std::atomic<bool> held{};
#endif

    static void take() {
#if KVASIR_DCHECK_LEVEL > 0
        KVASIR_DCHECK(!held.load(std::memory_order_relaxed),
                      "DMAC CRC engine started while another user holds it: one context at a time");
        held.store(true, std::memory_order_relaxed);
#endif
    }

    static void give() {
#if KVASIR_DCHECK_LEVEL > 0
        held.store(false, std::memory_order_relaxed);
#endif
    }

    /// Whether a user holds the engine (always false with debug checks off).
    [[nodiscard]] static bool taken() {
#if KVASIR_DCHECK_LEVEL > 0
        return held.load(std::memory_order_relaxed);
#else
        return false;
#endif
    }
};

/// The engine held for this object's lifetime.
struct Held {
    Held() { Hold::take(); }

    ~Held() { Hold::give(); }

    Held(Held const&)            = delete;
    Held& operator=(Held const&) = delete;
};

}   // namespace Kvasir::DmaCrc::detail
