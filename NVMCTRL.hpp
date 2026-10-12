#pragma once
// The SAM NVM controller's row erase and page write, and the stores built on them (SimpleEeprom, EepromEmulator,
// FlashJournal, Flash::NvmRegion for CellStore).
//
// Every command waits for READY (a datasheet bound, never issued while busy: "Any commands written while
// INTFLAG.READY is low will be ignored", D21 DS40001882L 22.6.4), clears the error flags, runs, waits again and
// returns an Nvm::Result: lock (LOCKE - a locked region or the BOOTPROT section: "the erase will not be performed",
// 22.6.4.5), program (PROGE - bad key, invalid command, busy: 22.8.1), nvm (NVME), timeout. Every page write is read
// back word by word (verify). A store that fails keeps its value pending and writes it by itself later, with a
// pause that doubles up to 10 min (Kvasir::Retry), as soon as it is listed in the Startup list (its Hook::MainLoop
// function maintain() does the retry): `pending()`, `retry()` say how it stands.
#include "chip/NVMCTRL_Traits.hpp"
#include "chip/atsam_common/DMAC.hpp"
#include "chip/atsam_common/NvmResult.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "kvasir/Register/Wait.hpp"
#include "kvasir/StartUp/Hooks.hpp"
#include "kvasir/Util/Retry.hpp"
#include "kvasir/Util/attributes.hpp"
#include "peripherals/NVMCTRL.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#ifdef USE_UC_LOG
    #include "uc_log/uc_log.hpp"
#endif

namespace Kvasir {
namespace Nvm {
    /// A store's pending write: Kvasir::Retry with the NVM error.
    template<typename Clock>
    using Retry = Kvasir::Retry<Clock, Error>;

    namespace Detail {
        /// Error flags found set BEFORE a command (left by an earlier one, or by code that does not go through here):
        /// counted, not blamed on the next command. `kvasir_bench.py peek` reads it.
        inline constinit std::uint32_t volatile staleFlags{};

        template<typename Bound>
        [[nodiscard]] inline bool waitReady() {
            return static_cast<bool>(
              Kvasir::Register::waitUntil<Bound, Kvasir::Register::OnTimeout::Report>(
                [] { return NVMCTRL::Traits::ready(); }));
        }

        /// One command: ready, flags cleared, `issue()` (writes ADDR and CTRLA), ready, the flags it raised.
        template<typename Bound,
                 typename Issue>
        [[nodiscard]] Result<> command(Issue issue) {
            if(!waitReady<Bound>()) { return std::unexpected{Error::timeout}; }
            if(NVMCTRL::Traits::takeErrors() != 0) { staleFlags = staleFlags + 1; }
            issue();
            if(!waitReady<Bound>()) { return std::unexpected{Error::timeout}; }
            return fromBits(NVMCTRL::Traits::takeErrors());
        }

        /// The address range of a command inside its array (D21, C21: the controller wraps an address past the end
        /// and erases a row of the image instead; E5x reports ADDRE itself). A range outside is refused before the
        /// controller sees it, as a program error (what E5x's ADDRE is too).
        template<bool isMainFlash>
        [[nodiscard]] inline bool inArray(std::uint32_t address,
                                          std::uint32_t bytes) {
            return NVMCTRL::Traits::inArray(address, bytes, isMainFlash);
        }

        /// The cache is off while a command runs (D21 errata 1.11.4; a no-op where the chip needs none).
        struct CacheOff {
            CacheOff() { NVMCTRL::Traits::disable_cache(); }

            ~CacheOff() { NVMCTRL::Traits::enable_cache(); }

            CacheOff(CacheOff const&)            = delete;
            CacheOff& operator=(CacheOff const&) = delete;
        };

        /// A store's bookkeeping after an attempt: the retry, and one log line when it fails and one when it is back.
        template<typename Clock>
        void note(Retry<Clock>&   retry,
                  Result<> const& r,
                  std::uintptr_t  where) {
            if(r) {
                if(retry.pending) {
#ifdef USE_UC_LOG
                    UC_LOG_I("nvm: store at {:#x} written after {} failures",
                             where,
                             retry.failures);
#endif
                }
                retry.succeeded();
                return;
            }
            auto const now = Clock::now();
            retry.failed(r.error(), now);
#ifdef USE_UC_LOG
            UC_LOG_E("nvm: store at {:#x} failed: {} (failure {}, retry in {})",
                     where,
                     r.error(),
                     retry.failures,
                     std::chrono::duration_cast<std::chrono::milliseconds>(retry.next - now));
#else
            static_cast<void>(where);
#endif
        }
    }   // namespace Detail
}   // namespace Nvm

namespace detail {
    // A 32-bit word that may alias any object: the emulation's flash pages and their images in RAM are read and
    // written word by word, and gcc's -Wstrict-aliasing rejects a plain std::uint32_t lvalue over them.
    using AliasedWord [[gnu::may_alias]] = std::uint32_t;

    template<typename Clock,
             bool isMainFlash>
    [[nodiscard]] Nvm::Result<> clearRow(std::uint32_t address) {
        if(!Nvm::Detail::inArray<isMainFlash>(address,
                                              NVMCTRL::Traits::PageSize
                                                * NVMCTRL::Traits::PagesPerRow))
        {
            return std::unexpected{Nvm::Error::program};
        }
        Nvm::Detail::CacheOff const cacheOff{};
        return Nvm::Detail::command<NVMCTRL::Traits::EraseBound>([address] {
            NVMCTRL::Traits::set_addr(address);
            if constexpr(isMainFlash) {
                NVMCTRL::Traits::command_erase_row();
            } else {
                NVMCTRL::Traits::prepare_erase_rww(address);
                NVMCTRL::Traits::command_erase_rww();
            }
        });
    }

    /// `n` rows of `rowBytes` from `first`, stopping at the first that fails (one function for every store).
    template<typename Clock,
             bool isMainFlash>
    [[nodiscard]] Nvm::Result<> clearRows(std::uint32_t first,
                                          std::size_t   n,
                                          std::size_t   rowBytes) {
        for(std::size_t i{}; i != n; ++i) {
            if(auto const r
               = clearRow<Clock, isMainFlash>(first + static_cast<std::uint32_t>(i * rowBytes));
               !r)
            {
                return r;
            }
        }
        return {};
    }

    template<typename Clock, std::size_t pageSize, std::size_t pagesPerRow>
    struct Flash {
        static constexpr std::size_t RowSize  = pageSize * pagesPerRow;
        static constexpr std::size_t PageSize = pageSize;

        /// One page: page buffer cleared, filled (words past `end` 0xFFFFFFFF), written, read back.
        template<bool isMainFlash,
                 typename I>
        [[nodiscard]] static Nvm::Result<> writePage(std::uint32_t address,
                                                     I             start,
                                                     I             end) {
            static_assert(sizeof(decltype(*start)) == 4, "needs to be 4 byte type");
            assert(PageSize / 4 >= static_cast<std::size_t>(std::distance(start, end)));
            if(!Nvm::Detail::inArray<isMainFlash>(address, PageSize)) {
                return std::unexpected{Nvm::Error::program};
            }
            Nvm::Detail::CacheOff const cacheOff{};
            auto* const                 page = reinterpret_cast<std::uint32_t volatile*>(address);

            if(auto const r = Nvm::Detail::command<NVMCTRL::Traits::WriteBound>([address] {
                   NVMCTRL::Traits::set_addr(address);
                   NVMCTRL::Traits::command_page_buffer_clear();
               });
               !r)
            {
                return r;
            }

            auto from = start;
            for(std::size_t i{}; i < PageSize / 4; ++i) {
                if(from != end) {
                    page[i] = *from;
                    ++from;
                } else {
                    page[i] = 0xffffffff;
                }
            }

            if(auto const r = Nvm::Detail::command<NVMCTRL::Traits::WriteBound>([] {
                   if constexpr(isMainFlash) {
                       NVMCTRL::Traits::command_write_page();
                   } else {
                       NVMCTRL::Traits::command_write_rww();
                   }
               });
               !r)
            {
                return r;
            }

            // the read-back: what STATUS cannot see (a page written without its erase, a worn cell)
            for(std::size_t i{}; i < PageSize / 4; ++i) {
                std::uint32_t const want = start != end ? std::uint32_t(*start) : 0xffffffffU;
                if(start != end) { ++start; }
                if(page[i] != want) { return std::unexpected{Nvm::Error::verify}; }
            }
            return {};
        }

        template<bool isMainFlash,
                 typename I>
        [[nodiscard]] static Nvm::Result<> writeRow(std::uint32_t address,
                                                    I             start,
                                                    I             end) {
            static_assert(sizeof(decltype(*start)) == 4, "needs to be 4 byte type");
            assert(RowSize / 4 >= static_cast<std::size_t>(std::distance(start, end)));

            for(std::size_t subPage{}; subPage < pagesPerRow; ++subPage) {
                auto const n
                  = std::min(static_cast<std::ptrdiff_t>(PageSize / 4), std::distance(start, end));
                if(auto const r = writePage<isMainFlash>(address, start, std::next(start, n)); !r) {
                    return r;
                }
                if(n < static_cast<std::ptrdiff_t>(PageSize / 4)) { return {}; }
                std::advance(start, n);
                address += PageSize;
            }
            return {};
        }

        template<bool isMainFlash,
                 typename I>
        [[nodiscard]] static Nvm::Result<> clearAndWriteRow(std::uint32_t address,
                                                            I             start,
                                                            I             end) {
            if(auto const r = clearRow<Clock, isMainFlash>(address); !r) { return r; }
            return writeRow<isMainFlash>(address, start, end);
        }
    };

}   // namespace detail

namespace Nvm {
    /// Row erase and page writes of this chip's NVM. (Not `Kvasir::Flash<Clock>` any more: that name is the namespace
    /// of CellStore, which a SAM firmware may want next to it.)
    template<typename Clock>
    using Flash = detail::Flash<Clock, NVMCTRL::Traits::PageSize, NVMCTRL::Traits::PagesPerRow>;
}   // namespace Nvm

namespace detail {

    template<typename Clock, typename T, std::size_t pageSize, bool isMainFlash>
    struct SimpleEeprom {
        static std::uint16_t calcCrc(T const& v) {
            return Kvasir::CRC::calcCrc<Kvasir::CRC::CRC_Type::crc16>(v);
        }

        struct alignas(std::uint32_t) ValueStruct {
            T             v{};
            std::uint16_t crc{};
        };

        [[KVASIR_SECTION_MEMBER(".eeprom")]] static inline ValueStruct flashValue{};

        static inline T    ramCopy{};
        static inline bool valueRead{false};

        static T readFlashValue() {
            if(flashValue.crc != calcCrc(flashValue.v)) { return T{}; }
            return flashValue.v;
        }

        static T& value() {
            if(!valueRead) {
                ramCopy   = readFlashValue();
                valueRead = true;
            }
            return ramCopy;
        }

        static Nvm::Result<> internalWrite() {
            ValueStruct newV;
            newV.v   = ramCopy;
            newV.crc = calcCrc(ramCopy);

            asm("" : "=m"(newV)::);

            return Kvasir::Nvm::Flash<Clock>::template clearAndWriteRow<isMainFlash>(
              reinterpret_cast<std::uint32_t>(&flashValue),
              reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&newV),
              reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&newV + 1));
        }

        static Nvm::Result<> attempt() {
            if(ramCopy == readFlashValue()) { return {}; }
            if(auto const r = internalWrite(); !r) { return r; }
            if(ramCopy != readFlashValue()) { return std::unexpected{Nvm::Error::verify}; }
            return {};
        }

        /// The RAM copy into the flash if it differs; true when the flash holds it. A failure stays pending and is
        /// written again by maintain(); retry().last says why. (A bool, not the Nvm::Result: libstdc++'s
        /// std::expected is [[nodiscard]], and a caller may leave the store to it.)
        static bool writeValue() {
            auto const r = attempt();
            Nvm::Detail::note(retry_, r, reinterpret_cast<std::uintptr_t>(&flashValue));
            return r.has_value();
        }

        [[nodiscard]] static bool pending() { return retry_.pending; }

        [[nodiscard]] static Nvm::Retry<Clock> const& retry() { return retry_; }

        static void maintain() {
            if(retry_.due(Clock::now())) { static_cast<void>(writeValue()); }
        }

        using Extends = brigand::list<Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &maintain>>;

    private:
        static inline Nvm::Retry<Clock> retry_{};
    };

    template<typename Clock,
             typename T,
             std::size_t pageSize,
             std::size_t pagesPerRow,
             std::size_t rowToUse,
             bool        isMainFlash>
    struct EepromEmulator {
        static std::uint16_t calcCrc(T const& v) {
            return Kvasir::CRC::calcCrc<Kvasir::CRC::CRC_Type::crc16>(v);
        }

        struct ValueStruct {
            T             v{};
            std::uint16_t crc{};
        };

        template<typename Block_t>
        struct SizeConfig {
            using blockType                       = Block_t;
            static constexpr auto blockResetValue = std::numeric_limits<blockType>::max();

            static constexpr auto typeSize    = sizeof(ValueStruct);
            static constexpr auto numRawTypes = pageSize / typeSize;
            static constexpr auto numTypes = (typeSize * numRawTypes > pageSize - sizeof(blockType))
                                             ? numRawTypes - 1
                                             : numRawTypes;
        };

        using SConfig8  = SizeConfig<std::uint8_t>;
        using SConfig16 = SizeConfig<std::uint16_t>;
        using SConfig32 = SizeConfig<std::uint32_t>;
        using SConfig64 = SizeConfig<std::uint64_t>;

        template<typename Config>
        static constexpr bool configValid() {
            return Config::numTypes
                   < static_cast<std::size_t>(std::popcount(Config::blockResetValue))
                && Config::numTypes != 0;
        }

        using SConfig = std::conditional_t<
          configValid<SConfig8>(),
          SConfig8,
          std::conditional_t<configValid<SConfig16>(),
                             SConfig16,
                             std::conditional_t<configValid<SConfig32>(), SConfig32, SConfig64>>>;

        using blockType                       = typename SConfig::blockType;
        static constexpr auto blockResetValue = SConfig::blockResetValue;
        static constexpr auto numTypes        = SConfig::numTypes;

        static_assert(configValid<SConfig>(),
                      "bla");

        static ValueStruct calcVS(T const& v) { return ValueStruct{v, calcCrc(v)}; }

        struct alignas(pageSize) PageStruct {
            std::array<ValueStruct, numTypes> array{};
            blockType                         validBlock{};
        };

        struct alignas(pageSize* pagesPerRow) RowStruct {
            std::array<PageStruct, pagesPerRow> pages;
        };

        static_assert(sizeof(PageStruct) == pageSize,
                      "bla");
        static_assert(std::is_standard_layout_v<PageStruct>,
                      "bla");
        static_assert(std::alignment_of_v<PageStruct> == pageSize,
                      "bla");

        static_assert(sizeof(RowStruct) == pageSize * pagesPerRow,
                      "bla");
        static_assert(std::is_standard_layout_v<RowStruct>,
                      "bla");
        static_assert(std::alignment_of_v<RowStruct> == pageSize * pagesPerRow,
                      "bla");

        [[KVASIR_SECTION_MEMBER(".eeprom")]] static inline std::array<RowStruct, rowToUse> rows{};

        static constexpr blockType blockFromIndex(std::size_t index) {
            assert(index == std::clamp<decltype(index)>(index, 0, numTypes));
            blockType v = blockResetValue;

            for(std::size_t i = 0; i < index + 1; ++i) { v = v & (~(blockType{1} << i)); }
            return v;
        }

        static constexpr bool validIndex(blockType v) {
            auto const vv = (std::popcount(blockResetValue) - 1) - std::popcount(v);
            if(vv != std::clamp<decltype(vv)>(vv, 0, numTypes - 1)) { return false; }

            return blockFromIndex(static_cast<std::size_t>(vv)) == v;
        }

        static constexpr std::size_t getIndex(blockType v) {
            assert(validIndex(v));
            auto const vv = (std::popcount(blockResetValue) - 1) - std::popcount(v);
            return static_cast<std::size_t>(vv);
        }

        static constexpr blockType nextBlockValue(blockType v) {
            if(validIndex(v)) {
                auto newIndex = getIndex(v) + 1;
                if(newIndex >= numTypes) { return 0; }
                return blockFromIndex(newIndex);
            }
            if(v == blockResetValue) { return blockFromIndex(0); }
            return 0;
        }

        struct ValidPages {
            PageStruct* page{nullptr};
            std::size_t nPages{};
        };

        static ValidPages getValidPage() {
            ValidPages ret{};
            for(auto& row : rows) {
                for(auto& page : row.pages) {
                    if(validIndex(page.validBlock)) {
                        ++ret.nPages;
                        ret.page = &page;
                    }
                }
            }
            return ret;
        }

        static T readFlashValue() {
            auto const vp = getValidPage();

            if(vp.nPages != 1 || vp.page == nullptr) { return T{}; }

            auto const         index = getIndex(vp.page->validBlock);
            ValueStruct const& cpy{vp.page->array[index]};
            if(cpy.crc == calcCrc(cpy.v)) { return cpy.v; }
            return T{};
        }

        static T& value() {
            if(!valueRead) {
                ramCopy   = readFlashValue();
                valueRead = true;
            }
            return ramCopy;
        }

        static Nvm::Result<> attempt() {
            if(ramCopy == readFlashValue()) { return {}; }
            if(auto const r = internalWrite(); !r) { return r; }
            if(ramCopy != readFlashValue()) { return std::unexpected{Nvm::Error::verify}; }
            return {};
        }

        /// The RAM copy into the flash if it differs; true when the flash holds it. A failure stays pending and is
        /// written again by maintain(); retry().last says why. (A bool, not the Nvm::Result: libstdc++'s
        /// std::expected is [[nodiscard]], and a caller may leave the store to it.)
        static bool writeValue() {
            auto const r = attempt();
            Nvm::Detail::note(retry_, r, reinterpret_cast<std::uintptr_t>(&rows));
            return r.has_value();
        }

        [[nodiscard]] static bool pending() { return retry_.pending; }

        [[nodiscard]] static Nvm::Retry<Clock> const& retry() { return retry_; }

        static void maintain() {
            if(retry_.due(Clock::now())) { static_cast<void>(writeValue()); }
        }

        using Extends = brigand::list<Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &maintain>>;

        static inline Nvm::Retry<Clock> retry_{};
        static inline T                 ramCopy{};
        static inline bool              valueRead{false};

        static bool pageFull(PageStruct const* p) {
            if(!validIndex(p->validBlock)) { return true; }

            return getIndex(p->validBlock) == numTypes - 1;
        }

        static bool pageCleared(PageStruct const* p) {
            for(std::size_t i = 0; i < pageSize / 4; ++i) {
                if(*(reinterpret_cast<::Kvasir::detail::AliasedWord const*>(p) + i) != 0xffffffff) {
                    return false;
                }
            }
            return true;
        }

        static PageStruct* nextPage(PageStruct* p) {
            std::size_t index{};
            for(auto& row : rows) {
                if(p == &row.pages.back()) {
                    if(index + 1 == rows.size()) { return &(*rows.front().pages.begin()); }
                    return &(*rows[index + 1].pages.begin());
                }
                ++index;
            }
            return ++p;
        }

        static Nvm::Result<> internalWrite() {
            using F       = Flash<Clock, pageSize, pagesPerRow>;
            auto zeroPage = [](PageStruct* p) {
                std::array<std::uint32_t, pageSize / 4> const zeros{};
                return F::template writePage<isMainFlash>(reinterpret_cast<std::uint32_t>(p),
                                                          zeros.data(),
                                                          zeros.data() + zeros.size());
            };

            auto writeNewV = [](PageStruct* p, blockType blockV, ValueStruct const& v) {
                PageStruct newp{*p};

                newp.validBlock              = blockV;
                newp.array[getIndex(blockV)] = v;

                asm("" : "=m"(newp)::);

                return F::template writePage<isMainFlash>(
                  reinterpret_cast<std::uint32_t>(p),
                  reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&newp),
                  reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&newp + 1));
            };

            auto clearAll = [] {
                return clearRows<Clock, isMainFlash>(reinterpret_cast<std::uint32_t>(rows.data()),
                                                     rows.size(),
                                                     sizeof(RowStruct));
            };

            auto const newV{calcVS(ramCopy)};

            auto vp = getValidPage();

            PageStruct* pageToZero{nullptr};
            PageStruct* pageToWrite{nullptr};
            RowStruct*  rowToClear{nullptr};

            if(vp.nPages != 1 || vp.page == nullptr) {
                if(auto const r = clearAll(); !r) { return r; }
                pageToWrite = &(*rows.front().pages.begin());
            } else {
                if(pageFull(vp.page)) {
                    pageToWrite = nextPage(vp.page);
                    if(!pageCleared(pageToWrite)) {
                        if(auto const r = clearAll(); !r) { return r; }
                        pageToWrite = &(*rows.front().pages.begin());
                    } else {
                        if(pageToWrite == &rows[0].pages[0]) {
                            rowToClear = &rows[1];
                        } else if(pageToWrite == &rows[1].pages[0]) {
                            rowToClear = &rows[0];
                        } else {
                            pageToZero = vp.page;
                        }
                    }
                } else {
                    pageToWrite = vp.page;
                }
            }

            if(auto const r = writeNewV(pageToWrite, nextBlockValue(pageToWrite->validBlock), newV);
               !r)
            {
                return r;
            }

            if(rowToClear != nullptr) {
                if(auto const r
                   = clearRow<Clock, isMainFlash>(reinterpret_cast<std::uint32_t>(rowToClear));
                   !r)
                {
                    return r;
                }
            }

            if(pageToZero != nullptr) { return zeroPage(pageToZero); }
            return {};
        }
    };

    /// A value that is written often (a position, a counter): every write() is ONE page write
    /// into the next page of a ring of rows, and a row is erased when the ring comes back to
    /// its first page. That keeps to the flash's rule (SAM D21 data sheet DS40001882, Flash
    /// Endurance and Data Retention, the note above table 37-43: "a max number of 8
    /// consecutive write is allowed per row. Once this number is reached, a row erase is
    /// mandatory" - four in the 105 degC table) with pagesPerRow writes per erase, which
    /// EepromEmulator does not: it rewrites a page for every value. One erase cycle of a row
    /// (25k min., 150k typ., table 37-43) is rowsToUse * pagesPerRow writes.
    ///
    /// The newest record with a good CRC wins, so a write or an erase cut short by a power
    /// loss costs that one value, never the one before it: needs two rows at least.
    template<typename Clock,
             typename T,
             std::size_t pageSize,
             std::size_t pagesPerRow,
             std::size_t rowsToUse,
             bool        isMainFlash>
    struct FlashJournal {
        static_assert(rowsToUse >= 2,
                      "the newest record must survive the erase of a row");
        static_assert(std::is_trivially_copyable_v<T>);

        struct Payload {
            std::uint32_t sequence{};   ///< counts the writes; erased flash reads 0xffffffff
            T             v{};
        };

        struct alignas(pageSize) Record {
            Payload       payload{};
            std::uint16_t crc{};
        };

        static_assert(sizeof(Record) == pageSize,
                      "a record is a page");

        static constexpr std::size_t   RowSize  = pageSize * pagesPerRow;
        static constexpr std::size_t   NumPages = pagesPerRow * rowsToUse;
        static constexpr std::uint32_t Erased   = 0xffffffff;

        struct alignas(RowSize) Storage {
            std::array<Record, NumPages> pages;
        };

        [[KVASIR_SECTION_MEMBER(".eeprom")]] static inline Storage storage{};

        static inline bool          scanned{false};
        static inline std::size_t   newest{NumPages};   ///< NumPages: none
        static inline std::uint32_t sequence{};

        static std::uint16_t calcCrc(Payload const& p) {
            return Kvasir::CRC::calcCrc<Kvasir::CRC::CRC_Type::crc16>(p);
        }

        static bool valid(Record const& r) {
            return r.payload.sequence != Erased && r.crc == calcCrc(r.payload);
        }

        static void scan() {
            if(scanned) { return; }
            scanned = true;
            for(std::size_t i{}; i < NumPages; ++i) {
                Record const& r{storage.pages[i]};
                if(valid(r) && (newest == NumPages || r.payload.sequence > sequence)) {
                    newest   = i;
                    sequence = r.payload.sequence;
                }
            }
        }

        /// The value written last; empty when there is none.
        static std::optional<T> read() {
            scan();
            if(newest == NumPages) { return std::nullopt; }
            Record const& r{storage.pages[newest]};
            return r.payload.v;
        }

        static bool erased(Record const& r) {
            auto const* words = reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&r);
            for(std::size_t i{}; i < pageSize / 4; ++i) {
                if(words[i] != Erased) { return false; }
            }
            return true;
        }

        /// Blocks for a page write (2.5 ms max.), every pagesPerRow-th time for a row erase
        /// as well (6 ms max.; data sheet table 37-45, tFPP and tFRE). A failure keeps `v` pending: maintain()
        /// writes it later unless a newer write() came first.
        /// True when the flash holds `v`; retry().last says why not (a bool: see EepromEmulator::writeValue).
        static bool write(T const& v) {
            auto const r = attempt(v);
            if(!r) { pendingValue_ = v; }
            Nvm::Detail::note(retry_, r, reinterpret_cast<std::uintptr_t>(&storage));
            return r.has_value();
        }

        [[nodiscard]] static bool pending() { return retry_.pending; }

        [[nodiscard]] static Nvm::Retry<Clock> const& retry() { return retry_; }

        static void maintain() {
            if(retry_.due(Clock::now())) { static_cast<void>(write(pendingValue_)); }
        }

        using Extends = brigand::list<Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &maintain>>;

    private:
        static inline Nvm::Retry<Clock> retry_{};
        static inline T                 pendingValue_{};

        static Nvm::Result<> attempt(T const& v) {
            scan();
            std::size_t slot = newest == NumPages ? 0 : (newest + 1) % NumPages;
            // A page that is not blank where one should be (a write cut short): on to the
            // next row, which is erased before its first page is written.
            if(slot % pagesPerRow != 0 && !erased(storage.pages[slot])) {
                slot = (slot / pagesPerRow + 1) * pagesPerRow % NumPages;
            }
            if(slot % pagesPerRow == 0) {
                if(auto const r = clearRow<Clock, isMainFlash>(
                     reinterpret_cast<std::uint32_t>(&storage.pages[slot]));
                   !r)
                {
                    return r;
                }
            }

            Record r{};
            r.payload.sequence = sequence + 1;
            r.payload.v        = v;
            r.crc              = calcCrc(r.payload);

            asm("" : "=m"(r)::);

            // the sequence is used up either way: a failed page may hold it, and is never written over
            sequence = r.payload.sequence;
            if(auto const w = Flash<Clock, pageSize, pagesPerRow>::template writePage<isMainFlash>(
                 reinterpret_cast<std::uint32_t>(&storage.pages[slot]),
                 reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&r),
                 reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&r + 1));
               !w)
            {
                return w;
            }

            newest = slot;
            return {};
        }
    };
}   // namespace detail

template<typename Clock, typename ValueType, std::size_t rowsToUse, bool isMainFlash>
using EepromEmulator = detail::EepromEmulator<Clock,
                                              ValueType,
                                              NVMCTRL::Traits::PageSize,
                                              NVMCTRL::Traits::PagesPerRow,
                                              rowsToUse,
                                              isMainFlash>;

template<typename Clock, typename ValueType, std::size_t rowsToUse, bool isMainFlash>
using FlashJournal = detail::FlashJournal<Clock,
                                          ValueType,
                                          NVMCTRL::Traits::PageSize,
                                          NVMCTRL::Traits::PagesPerRow,
                                          rowsToUse,
                                          isMainFlash>;

template<typename Clock, typename ValueType, bool isMainFlash>
using SimpleEeprom = detail::SimpleEeprom<Clock,
                                          ValueType,
                                          NVMCTRL::Traits::PageSize * NVMCTRL::Traits::PagesPerRow,
                                          isMainFlash>;

namespace Nvm {
    /// A SAM flash range as a Kvasir::Flash::NorRegion (kvasir/Util/CellStore.hpp): `Rows` rows from the absolute
    /// address `Base` (a row boundary), in the main array or (isMainFlash = false) in the RWWEE array. A sector is a
    /// row (the erase unit), a page a page. The range is the firmware's to choose: it must not overlap the image or
    /// a .eeprom store, nothing checks that. A program or erase that the controller refuses (Nvm::Error) is a false
    /// to CellStore, which reports CellError::flash; CellEeprom then keeps the value pending and retries.
    ///
    ///     using Region   = Kvasir::Nvm::Region<Clock, 0x3'F000, 4, true>;            // the last 4 rows of 256 KB
    ///     using Settings = Kvasir::Flash::CellEeprom<Kvasir::Flash::CellStore<Region, Cfg>, Clock>;
    template<typename Clock, std::uint32_t Base, std::uint32_t Rows, bool isMainFlash>
    struct Region {
        using F = Flash<Clock>;
        static_assert(Base % F::RowSize == 0,
                      "a region starts on a row");
        static_assert(Rows >= 2,
                      "CellStore needs two erase units at least");

        static constexpr std::uint32_t SectorBytes = F::RowSize;
        static constexpr std::uint32_t PageBytes   = F::PageSize;

        static constexpr std::uint32_t sectorCount() { return Rows; }

        static bool read(std::uint32_t        addr,
                         std::span<std::byte> out) {
            if(std::uint64_t{addr} + out.size() > std::uint64_t{Rows} * SectorBytes) {
                return false;
            }
            std::memcpy(out.data(), reinterpret_cast<void const*>(Base + addr), out.size());
            return true;
        }

        /// Bytes inside one page; the rest of the page is written as 0xFF (programs nothing).
        static bool program(std::uint32_t              addr,
                            std::span<std::byte const> in) {
            if(in.empty() || std::uint64_t{addr} + in.size() > std::uint64_t{Rows} * SectorBytes
               || addr / PageBytes != (addr + in.size() - 1) / PageBytes)
            {
                return false;
            }
            std::array<std::uint32_t, PageBytes / 4> words{};
            words.fill(0xFFFF'FFFFU);
            std::memcpy(reinterpret_cast<std::byte*>(words.data()) + addr % PageBytes,
                        in.data(),
                        in.size());
            return F::template writePage<isMainFlash>(Base + addr / PageBytes * PageBytes,
                                                      words.begin(),
                                                      words.end())
              .has_value();
        }

        static bool erase(std::uint32_t sector) {
            if(sector >= Rows) { return false; }
            return detail::clearRow<Clock, isMainFlash>(Base + sector * SectorBytes).has_value();
        }
    };
}   // namespace Nvm

}   // namespace Kvasir
