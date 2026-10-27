#pragma once
// The SAM NVM controller's row erase and page write, and the stores built on them (EepromEmulator, FlashJournal,
// Nvm::Region for CellStore). The two `.eeprom` stores' layouts (rows, array, record format) are this file's to
// change: an image finds only what an image of the same layout wrote, anything else reads as no value.
//
// Every command waits for READY (a datasheet bound, never issued while busy: "Any commands written while
// INTFLAG.READY is low will be ignored", D21 DS40001882L 22.6.4), clears the error flags, runs, waits again and
// returns an Nvm::Result: lock (LOCKE - a locked region or the BOOTPROT section: "the erase will not be performed",
// 22.6.4.5), program (PROGE - bad key, invalid command, busy: 22.8.1), nvm (NVME), timeout. Every page write is read
// back word by word (verify). A store that fails keeps its value pending and writes it by itself later, with a
// pause that doubles up to 10 min (Kvasir::Flash::RetryingStore, Runtime/Store.hpp), as soon as it is listed in the
// Startup list (its Hook::MainLoop function turn() does the retry): `pending()`, `retry()`, `link()` say how it
// stands. Raw access: Nvm::eraseRow<Array>(addr), Nvm::eraseRows<Array>(first, n), Nvm::Flash::writePage<Array>.
#include "chip/Package.hpp"
#include "chip/atsam_common/DmacCrc.hpp"   // the record CRC: the DMAC's CRC engine, no DMA channel
#include "chip/atsam_common/NvmLayout.hpp"
#include "chip/atsam_common/NvmResult.hpp"
#include "kvasir/Hal/Flash.hpp"
#include "kvasir/Port/Extends.hpp"
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "kvasir/Register/Wait.hpp"
#include "kvasir/Runtime/Store.hpp"
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
    /// The NVM array a command goes to: the main array, or the RWWEE array (read-while-write EEPROM emulation area,
    /// D21 DS40001882L 22.6.5; parts with `hasRwweeArray = false` have none).
    enum class Array : bool { rwwee = false, main = true };

    /// Error flags found set BEFORE a command (left by an earlier one, or by code that does not go through here):
    /// counted, not blamed on the next command. `kvasir_bench.py peek` reads it.
    inline constinit std::uint32_t volatile staleFlags{};

    namespace Detail {
        // A 32-bit word that may alias any object: the emulation's flash pages and their images in RAM are read and
        // written word by word, and gcc's -Wstrict-aliasing rejects a plain std::uint32_t lvalue over them.
        using AliasedWord [[gnu::may_alias]] = std::uint32_t;

        template<typename Bound>
        [[nodiscard]] inline bool waitReady() {
            return static_cast<bool>(
              Kvasir::Register::waitUntil<Bound, Kvasir::Register::OnTimeout::Report>(
                [] { return Sam::Package::Nvm::ready(); }));
        }

        /// The address range of a command inside its array (D21, C21: the controller wraps an address past the end
        /// and erases a row of the image instead; E5x reports ADDRE itself). A range outside is refused before the
        /// controller sees it, as a program error (what E5x's ADDRE is too).
        template<Array A>
        [[nodiscard]] inline bool inArray(std::uint32_t address,
                                          std::uint32_t bytes) {
            return Sam::Package::Nvm::inArray(address, bytes, A == Array::main);
        }

        /// The chip's traits as a dependent name, for the RWWEE branches: they name commands a part without an RWWEE
        /// array (E5x, `hasRwweeArray = false`) does not have, and a discarded branch is only skipped when dependent.
        template<Array A>
        struct TraitsFor {
            using type = Sam::Package::Nvm;
            static_assert(A == Array::main || type::hasRwweeArray,
                          "this SAM part has no RWWEE array");
        };

        /// The stores' log lines (Kvasir::Flash::RetryingStore's hooks): one when a write fails, one when it is back.
        /// `Derived::where()` is the store's flash address.
        template<typename Derived>
        struct NvmLog {
            template<typename R>
            static void logRecovered([[maybe_unused]] R const& retry) {
#ifdef USE_UC_LOG
                UC_LOG_I("nvm: store at {:#x} written after {} failures",
                         Derived::where(),
                         retry.failures);
#endif
            }

            template<typename R,
                     typename Duration>
            static void logFailed([[maybe_unused]] R const& retry,
                                  [[maybe_unused]] Error    e,
                                  [[maybe_unused]] Duration untilNext) {
#ifdef USE_UC_LOG
                UC_LOG_E("nvm: store at {:#x} failed: {} (failure {}, retry in {})",
                         Derived::where(),
                         e,
                         retry.failures,
                         std::chrono::duration_cast<std::chrono::milliseconds>(untilNext));
#endif
            }
        };

        /// The CRC-16 of a store's record (the DMAC CRC engine, DmacCrc.hpp): the stores' record format.
        template<typename T>
        std::uint16_t crc16(T const& v) {
            return Kvasir::DmaCrc::calcCrc<Kvasir::DmaCrc::CrcType::crc16>(v);
        }
    }   // namespace Detail

    /// One command: ready, flags cleared, `issue()` (writes ADDR and CTRLA), ready, the flags it raised.
    template<typename Bound,
             typename Issue>
    [[nodiscard]] Result<> command(Issue issue) {
        if(!Detail::waitReady<Bound>()) { return std::unexpected{Error::timeout}; }
        if(Sam::Package::Nvm::takeErrors() != 0) { staleFlags = staleFlags + 1; }
        issue();
        if(!Detail::waitReady<Bound>()) { return std::unexpected{Error::timeout}; }
        return ErrorBits::fromBits(Sam::Package::Nvm::takeErrors());
    }

    /// The cache is off while a command runs (D21 errata 1.11.4; a no-op where the chip needs none).
    struct CacheOff {
        CacheOff() { Sam::Package::Nvm::disableCache(); }

        ~CacheOff() { Sam::Package::Nvm::enableCache(); }

        CacheOff(CacheOff const&)            = delete;
        CacheOff& operator=(CacheOff const&) = delete;
    };

    /// One row erase at `address` (a row boundary) in array A.
    template<Array A>
    [[nodiscard]] Result<> eraseRow(std::uint32_t address) {
        if(!Detail::inArray<A>(address,
                               Sam::Package::Nvm::pageSize * Sam::Package::Nvm::pagesPerRow))
        {
            return std::unexpected{Error::program};
        }
        CacheOff const cacheOff{};
        return command<Sam::Package::Nvm::EraseBound>([address] {
            Sam::Package::Nvm::setAddr(address);
            if constexpr(A == Array::main) {
                Sam::Package::Nvm::commandEraseRow();
            } else {
                using T = typename Detail::TraitsFor<A>::type;
                T::prepareEraseRww(address);
                T::commandEraseRww();
            }
        });
    }

    /// `n` rows from `first`, stopping at the first that fails.
    template<Array A>
    [[nodiscard]] Result<> eraseRows(std::uint32_t first,
                                     std::size_t   n) {
        constexpr std::size_t rowBytes
          = Sam::Package::Nvm::pageSize * Sam::Package::Nvm::pagesPerRow;
        for(std::size_t i{}; i != n; ++i) {
            if(auto const r = eraseRow<A>(first + static_cast<std::uint32_t>(i * rowBytes)); !r) {
                return r;
            }
        }
        return {};
    }

    namespace Detail {
        template<std::size_t PageSize, std::size_t PagesPerRow>
        struct Flash {
            static constexpr std::size_t rowSize  = PageSize * PagesPerRow;
            static constexpr std::size_t pageSize = PageSize;

            /// One page: page buffer cleared, filled (words past `end` 0xFFFFFFFF), written, read back.
            template<Array A,
                     typename I>
            [[nodiscard]] static Result<> writePage(std::uint32_t address,
                                                    I             start,
                                                    I             end) {
                static_assert(sizeof(decltype(*start)) == 4, "needs to be 4 byte type");
                assert(pageSize / 4 >= static_cast<std::size_t>(std::distance(start, end)));
                if(!inArray<A>(address, pageSize)) { return std::unexpected{Error::program}; }
                CacheOff const                             cacheOff{};
                Kvasir::Memory::Words<std::uint32_t> const page{address};

                if(auto const r = command<Sam::Package::Nvm::WriteBound>([address] {
                       Sam::Package::Nvm::setAddr(address);
                       Sam::Package::Nvm::commandPageBufferClear();
                   });
                   !r)
                {
                    return r;
                }

                auto from = start;
                for(std::size_t i{}; i < pageSize / 4; ++i) {
                    if(from != end) {
                        page.store(i, *from);
                        ++from;
                    } else {
                        page.store(i, 0xffffffffU);
                    }
                }

                if(auto const r = command<Sam::Package::Nvm::WriteBound>([] {
                       if constexpr(A == Array::main) {
                           Sam::Package::Nvm::commandWritePage();
                       } else {
                           TraitsFor<A>::type::commandWriteRww();
                       }
                   });
                   !r)
                {
                    return r;
                }

                // the read-back: what STATUS cannot see (a page written without its erase, a worn cell)
                for(std::size_t i{}; i < pageSize / 4; ++i) {
                    std::uint32_t const want = start != end ? std::uint32_t(*start) : 0xffffffffU;
                    if(start != end) { ++start; }
                    if(page.load(i) != want) { return std::unexpected{Error::verify}; }
                }
                return {};
            }
        };
    }   // namespace Detail

    /// Row erase and page writes of this chip's NVM. (Not `Kvasir::Flash`: that name is the namespace of CellStore,
    /// which a SAM firmware may want next to it.)
    using Flash = Detail::Flash<Sam::Package::Nvm::pageSize, Sam::Package::Nvm::pagesPerRow>;

    namespace Detail {

        /// Kvasir::EepromEmulator (below): a value in a ring of the pages of `rowCount` rows of array A. Each
        /// write puts the value with its CRC-16 into the next slot of the valid page; a full page moves on to the
        /// next page, and the first page of a row erases the row before it once the value is in: one page is valid.
        template<typename Clock,
                 typename T,
                 std::size_t PageSize,
                 std::size_t PagesPerRow,
                 std::size_t RowCount,
                 Array       A>
        struct EepromEmulator
          : Kvasir::Flash::CachedValue<EepromEmulator<Clock, T, PageSize, PagesPerRow, RowCount, A>,
                                       T,
                                       Clock,
                                       Error>
          , NvmLog<EepromEmulator<Clock, T, PageSize, PagesPerRow, RowCount, A>> {
            static_assert(RowCount >= 2,
                          "the valid page must survive the erase of a row: two rows at least");
            static_assert(A == Array::main || Sam::Package::Nvm::hasRwweeArray,
                          "this SAM part has no RWWEE array: put the store on Nvm::Array::main");

            struct ValueStruct {
                T             v{};
                std::uint16_t crc{};
            };

            template<typename BlockT>
            struct SizeConfig {
                using BlockType                       = BlockT;
                static constexpr auto blockResetValue = std::numeric_limits<BlockType>::max();

                static constexpr auto typeSize    = sizeof(ValueStruct);
                static constexpr auto numRawTypes = PageSize / typeSize;
                static constexpr auto numTypes
                  = (typeSize * numRawTypes > PageSize - sizeof(BlockType)) ? numRawTypes - 1
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
              std::conditional_t<
                configValid<SConfig16>(),
                SConfig16,
                std::conditional_t<configValid<SConfig32>(), SConfig32, SConfig64>>>;

            using BlockType                       = typename SConfig::BlockType;
            static constexpr auto blockResetValue = SConfig::blockResetValue;
            static constexpr auto numTypes        = SConfig::numTypes;

            static_assert(
              configValid<SConfig>(),
              "the page's validity word has too few bits for its slots (or T fills the page)");

            static ValueStruct calcVS(T const& v) { return ValueStruct{v, crc16(v)}; }

            struct alignas(PageSize) PageStruct {
                std::array<ValueStruct, numTypes> array{};
                BlockType                         validBlock{};
            };

            struct alignas(PageSize* PagesPerRow) RowStruct {
                std::array<PageStruct, PagesPerRow> pages;
            };

            static_assert(sizeof(PageStruct) == PageSize,
                          "a page struct is exactly one page");
            static_assert(std::is_standard_layout_v<PageStruct>,
                          "a page struct is standard layout (written word by word)");
            static_assert(std::alignment_of_v<PageStruct> == PageSize,
                          "a page struct is aligned to a page");

            static_assert(sizeof(RowStruct) == PageSize * PagesPerRow,
                          "a row struct is exactly one row");
            static_assert(std::is_standard_layout_v<RowStruct>,
                          "a row struct is standard layout (written word by word)");
            static_assert(std::alignment_of_v<RowStruct> == PageSize * PagesPerRow,
                          "a row struct is aligned to a row");

            [[KVASIR_SECTION_MEMBER(
              ".eeprom")]] static inline std::array<RowStruct, RowCount> rows{};

            static std::uintptr_t where() { return reinterpret_cast<std::uintptr_t>(&rows); }

            using Index = BlockIndex<BlockType, numTypes>;

            struct ValidPages {
                PageStruct* page{nullptr};
                std::size_t nPages{};
            };

            static ValidPages getValidPage() {
                ValidPages ret{};
                for(auto& row : rows) {
                    for(auto& page : row.pages) {
                        if(Index::validIndex(page.validBlock)) {
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

                auto const         index = Index::getIndex(vp.page->validBlock);
                ValueStruct const& cpy{vp.page->array[index]};
                if(cpy.crc == crc16(cpy.v)) { return cpy.v; }
                return T{};
            }

            static bool pageFull(PageStruct const* p) {
                if(!Index::validIndex(p->validBlock)) { return true; }

                return Index::getIndex(p->validBlock) == numTypes - 1;
            }

            static bool pageCleared(PageStruct const* p) {
                for(std::size_t i = 0; i < PageSize / 4; ++i) {
                    if(*(reinterpret_cast<AliasedWord const*>(p) + i) != 0xffffffff) {
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

            /// `next` follows the full valid page: the first page of row I erases the row before it once the value is
            /// in (that row holds only the full page); any other page zeroes the full page. An if/else-if chain over
            /// the rows.
            template<std::size_t I = 0>
            [[gnu::always_inline]] static void afterFullPage(PageStruct const* next,
                                                             ValidPages const& valid,
                                                             RowStruct*&       rowToClear,
                                                             PageStruct*&      pageToZero) {
                if constexpr(I == RowCount) {
                    pageToZero = valid.page;
                } else if(next == &rows[I].pages[0]) {
                    rowToClear = &rows[(I + RowCount - 1) % RowCount];
                } else {
                    afterFullPage<I + 1>(next, valid, rowToClear, pageToZero);
                }
            }

            static Result<> internalWrite() {
                using F             = Flash<PageSize, PagesPerRow>;
                auto const zeroPage = [](PageStruct* p) {
                    std::array<std::uint32_t, PageSize / 4> const zeros{};
                    return F::template writePage<A>(reinterpret_cast<std::uint32_t>(p),
                                                    zeros.data(),
                                                    zeros.data() + zeros.size());
                };

                auto const writeNewV = [](PageStruct* p, BlockType blockV, ValueStruct const& v) {
                    PageStruct newp{*p};

                    newp.validBlock                     = blockV;
                    newp.array[Index::getIndex(blockV)] = v;

                    asm("" : "=m"(newp)::);

                    return F::template writePage<A>(
                      reinterpret_cast<std::uint32_t>(p),
                      reinterpret_cast<AliasedWord const*>(&newp),
                      reinterpret_cast<AliasedWord const*>(&newp + 1));
                };

                auto const clearAll = [] {
                    return eraseRows<A>(reinterpret_cast<std::uint32_t>(rows.data()), rows.size());
                };

                auto const newV{calcVS(EepromEmulator::ramCopy)};

                auto const vp = getValidPage();

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
                            afterFullPage(pageToWrite, vp, rowToClear, pageToZero);
                        }
                    } else {
                        pageToWrite = vp.page;
                    }
                }

                if(auto const r
                   = writeNewV(pageToWrite, Index::nextBlockValue(pageToWrite->validBlock), newV);
                   !r)
                {
                    return r;
                }

                if(rowToClear != nullptr) {
                    if(auto const r = eraseRow<A>(reinterpret_cast<std::uint32_t>(rowToClear)); !r)
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
        /// (25k min., 150k typ., table 37-43) is rowCount * pagesPerRow writes.
        ///
        /// The newest record with a good CRC wins, so a write or an erase cut short by a power
        /// loss costs that one value, never the one before it: needs two rows at least.
        template<typename Clock,
                 typename T,
                 std::size_t PageSize,
                 std::size_t PagesPerRow,
                 std::size_t RowCount,
                 Array       A>
        struct FlashJournal
          : Kvasir::Flash::RetryingStore<FlashJournal<Clock, T, PageSize, PagesPerRow, RowCount, A>,
                                         Clock,
                                         Error>
          , NvmLog<FlashJournal<Clock, T, PageSize, PagesPerRow, RowCount, A>> {
            static_assert(RowCount >= 2,
                          "the newest record must survive the erase of a row");
            static_assert(std::is_trivially_copyable_v<T>);
            static_assert(A == Array::main || Sam::Package::Nvm::hasRwweeArray,
                          "this SAM part has no RWWEE array: put the store on Nvm::Array::main");

            struct Payload {
                std::uint32_t sequence{};   ///< counts the writes; erased flash reads 0xffffffff
                T             v{};
            };

            struct alignas(PageSize) Record {
                Payload       payload{};
                std::uint16_t crc{};
            };

            static_assert(sizeof(Record) == PageSize,
                          "a record is a page");

            static constexpr std::size_t   rowSize  = PageSize * PagesPerRow;
            static constexpr std::size_t   numPages = PagesPerRow * RowCount;
            static constexpr std::uint32_t erased   = 0xffffffff;

            struct alignas(rowSize) Storage {
                std::array<Record, numPages> pages;
            };

            [[KVASIR_SECTION_MEMBER(".eeprom")]] static inline Storage storage{};

            static std::uintptr_t where() { return reinterpret_cast<std::uintptr_t>(&storage); }

            static inline bool          scanned{false};
            static inline std::size_t   newest{numPages};   ///< NumPages: none
            static inline std::uint32_t sequence{};

            static bool valid(Record const& r) {
                return r.payload.sequence != erased && r.crc == crc16(r.payload);
            }

            static void scan() {
                if(scanned) { return; }
                scanned = true;
                for(std::size_t i{}; i < numPages; ++i) {
                    Record const& r{storage.pages[i]};
                    if(valid(r) && (newest == numPages || r.payload.sequence > sequence)) {
                        newest   = i;
                        sequence = r.payload.sequence;
                    }
                }
            }

            /// The value written last; empty when there is none.
            static std::optional<T> read() {
                scan();
                if(newest == numPages) { return std::nullopt; }
                Record const& r{storage.pages[newest]};
                return r.payload.v;
            }

            static bool isErased(Record const& r) {
                auto const* words = reinterpret_cast<AliasedWord const*>(&r);
                for(std::size_t i{}; i < PageSize / 4; ++i) {
                    if(words[i] != erased) { return false; }
                }
                return true;
            }

            /// Blocks for a page write (2.5 ms max.), every pagesPerRow-th time for a row erase
            /// as well (6 ms max.; data sheet table 37-45, tFPP and tFRE). A failure keeps `v` pending: turn()
            /// writes it later unless a newer write() came first.
            /// True when the flash holds `v`; retry().last says why not (a bool: see CachedValue::writeValue).
            static bool write(T const& v) {
                auto const r = attempt(v);
                if(!r) { pendingValue_ = v; }
                return FlashJournal::settle(r);
            }

            static bool retryNow() { return write(pendingValue_); }

        private:
            static inline T pendingValue_{};

            static Result<> attempt(T const& v) {
                scan();
                std::size_t slot = newest == numPages ? 0 : (newest + 1) % numPages;
                // A page that is not blank where one should be (a write cut short): on to the
                // next row, which is erased before its first page is written.
                if(slot % PagesPerRow != 0 && !isErased(storage.pages[slot])) {
                    slot = (slot / PagesPerRow + 1) * PagesPerRow % numPages;
                }
                if(slot % PagesPerRow == 0) {
                    if(auto const r
                       = eraseRow<A>(reinterpret_cast<std::uint32_t>(&storage.pages[slot]));
                       !r)
                    {
                        return r;
                    }
                }

                Record r{};
                r.payload.sequence = sequence + 1;
                r.payload.v        = v;
                r.crc              = crc16(r.payload);

                asm("" : "=m"(r)::);

                // the sequence is used up either way: a failed page may hold it, and is never written over
                sequence = r.payload.sequence;
                if(auto const w = Flash<PageSize, PagesPerRow>::template writePage<A>(
                     reinterpret_cast<std::uint32_t>(&storage.pages[slot]),
                     reinterpret_cast<AliasedWord const*>(&r),
                     reinterpret_cast<AliasedWord const*>(&r + 1));
                   !w)
                {
                    return w;
                }

                newest = slot;
                return {};
            }
        };
    }   // namespace Detail
}   // namespace Nvm

/// A value in a page ring over `rowCount` rows (two at least) of NVM array A, placed in `.eeprom`: A is the array the
/// image's linker script puts that section in (D21/C21 chip.ld: the RWWEE array, `Nvm::Array::rwwee`).
///
///     using Settings = Kvasir::EepromEmulator<Clock, Stored, 2, Kvasir::Nvm::Array::rwwee>;
template<typename Clock, typename ValueType, std::size_t RowCount, Nvm::Array A>
using EepromEmulator = Nvm::Detail::EepromEmulator<Clock,
                                                   ValueType,
                                                   Sam::Package::Nvm::pageSize,
                                                   Sam::Package::Nvm::pagesPerRow,
                                                   RowCount,
                                                   A>;

/// A value written often: one page write per value into a ring of `rowCount` rows of array A (`.eeprom`, as above).
template<typename Clock, typename ValueType, std::size_t RowCount, Nvm::Array A>
using FlashJournal = Nvm::Detail::FlashJournal<Clock,
                                               ValueType,
                                               Sam::Package::Nvm::pageSize,
                                               Sam::Package::Nvm::pagesPerRow,
                                               RowCount,
                                               A>;

namespace Nvm {
    /// A SAM flash range as a Kvasir::Hal::NorRegion (kvasir/Hal/Flash.hpp, for CellStore): `Rows` rows from the
    /// absolute address `Base` (a row boundary) in array A. A sector is a row (the erase unit), a page a page; a
    /// program across a page is refused. The range is the firmware's to choose and must not overlap a .eeprom store;
    /// a main-array region that overlaps the image is refused (outsideImage(), FlashRegionBase: CellStore answers
    /// overlapsImage and touches nothing). A program or erase that the controller refuses (Nvm::Error) is a false
    /// to CellStore, which reports CellError::flash; CellEeprom then keeps the value pending and retries.
    ///
    ///     using Region   = Kvasir::Nvm::Region<0x3'F000, 4>;            // the last 4 rows of 256 KB
    ///     using Settings = Kvasir::Flash::CellEeprom<Kvasir::Flash::CellStore<Region, Cfg>, Clock>;
    template<std::uint32_t Base, std::uint32_t Rows, Array A = Array::main>
    struct Region
      : Kvasir::Hal::FlashRegionBase<Region<Base, Rows, A>,
                                     Kvasir::Hal::FlashGeometry{
                                       Base,
                                       static_cast<std::uint32_t>(Flash::rowSize),
                                       static_cast<std::uint32_t>(Flash::pageSize),
                                       Rows},
                                     Kvasir::Hal::PageSplit::refuse> {
        static_assert(Base % Flash::rowSize == 0,
                      "a region starts on a row");
        static_assert(Rows >= 2,
                      "CellStore needs two erase units at least");

        static constexpr std::uint32_t pageBytes = static_cast<std::uint32_t>(Flash::pageSize);

        static void readAt(std::uint32_t        addr,
                           std::span<std::byte> out) {
            Kvasir::Memory::loadBytes(Base + addr, out.data(), out.size());
        }

        static bool programPage(std::uint32_t        page,
                                std::span<std::byte const,
                                          pageBytes> bytes) {
            // word-aligned: FlashRegionBase's page buffer is alignas(4) (Hal/Flash.hpp); through void*
            // for gcc's -Wcast-align
            auto const* words
              = static_cast<Detail::AliasedWord const*>(static_cast<void const*>(bytes.data()));
            return Flash::writePage<A>(Base + page, words, words + pageBytes / 4).has_value();
        }

        static bool eraseSector(std::uint32_t sector) {
            return eraseRow<A>(Base + sector * static_cast<std::uint32_t>(Flash::rowSize))
              .has_value();
        }

        /// An RWWEE region is another address range than the image's; a main-array one: FlashRegionBase's check.
        [[nodiscard]] static bool outsideImage() {
            if constexpr(A == Array::rwwee) {
                return true;
            } else {
                return Region::FlashRegionBase::outsideImage();
            }
        }
    };
}   // namespace Nvm

}   // namespace Kvasir
