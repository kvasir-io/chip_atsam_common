#pragma once

// Kvasir::Chip::debuggerPresent(), declared, and the functor for uc_log's RTT backends (their
// DebuggerPresentFunction). No includes, so a firmware can name the functor in its ComBackend before uc_log and the
// chip: the generated DSU header the definition needs (atsam_common/Debug.hpp, from chip/chip.hpp) includes
// remote_fmt's catalog when it is there, which must come after the ComBackend.
//
//     #include "chip/atsam_common/DebuggerPresent.hpp"
//     template<> struct uc_log::ComBackend<uc_log::Tag::User>
//       : uc_log::RttLog<Kvasir::Chip::DebuggerPresent, ...> {};
namespace Kvasir::Chip {
/// A debugger probe was seen since the last power-on or external reset (DSU STATUSB.DBGPRES). Declared here, defined
/// inline in atsam_common/Debug.hpp (chip/chip.hpp includes it): a TU that calls it without the chip fails to link.
bool debuggerPresent();

struct DebuggerPresent {
    static bool operator()() { return debuggerPresent(); }
};
}   // namespace Kvasir::Chip
