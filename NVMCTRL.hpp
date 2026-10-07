#pragma once

#include "chip/NVMCTRL_Traits.hpp"
#include "chip/atsam_common/DMAC.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "kvasir/Util/attributes.hpp"
#include "peripherals/NVMCTRL.hpp"

#include <cassert>

namespace Kvasir {
namespace detail {
    // A 32-bit word that may alias any object: the emulation's flash pages and their images in RAM are read and
    // written word by word, and gcc's -Wstrict-aliasing rejects a plain std::uint32_t lvalue over them.
    using AliasedWord [[gnu::may_alias]] = std::uint32_t;

    template<typename Clock>
    static constexpr void waitForReady() {
        auto const timeout = Clock::now() + 100ms;
        while(!NVMCTRL::Traits::ready()) {
            if(Clock::now() > timeout) { assert(false); }
        }
    }

    template<typename Clock,
             bool isMainFlash>
    static constexpr void clearRow(std::uint32_t address) {
        NVMCTRL::Traits::set_addr(address);
        if constexpr(isMainFlash) {
            NVMCTRL::Traits::command_erase_row();
        } else {
            NVMCTRL::Traits::command_erase_rww();
        }
        waitForReady<Clock>();
    }

    template<typename Clock, std::size_t pageSize, std::size_t pagesPerRow>
    struct Flash {
        static constexpr std::size_t RowSize  = pageSize * pagesPerRow;
        static constexpr std::size_t PageSize = pageSize;

        template<std::size_t WriteSize, bool isMainFlash>
        struct Writer {
            std::uint32_t currentAddress;
            std::uint32_t remainingSize;

            Writer(std::uint32_t start,
                   std::uint32_t size)
              : currentAddress{start}
              , remainingSize{size} {
                assert(start % RowSize == 0);
            }

            template<typename I>
            void write(I start,
                       I end) {
                static_assert(sizeof(decltype(*start)) == 1, "needs to be 1 byte type");
                std::uint32_t const size{static_cast<std::uint32_t>(std::distance(start, end))};

                assert(reinterpret_cast<std::uint32_t>(&(*start)) % 4 == 0);
                assert(remainingSize >= size && (size == WriteSize || size == remainingSize));

                if(currentAddress % RowSize == 0) { clearRow<Clock, isMainFlash>(currentAddress); }

                if constexpr(WriteSize == RowSize) {
                    writeRow<isMainFlash>(
                      currentAddress,
                      reinterpret_cast<::Kvasir::detail::AliasedWord const*>(start),
                      reinterpret_cast<::Kvasir::detail::AliasedWord const*>(end));
                } else {
                    writePage<isMainFlash>(
                      currentAddress,
                      reinterpret_cast<::Kvasir::detail::AliasedWord const*>(start),
                      reinterpret_cast<::Kvasir::detail::AliasedWord const*>(end));
                }

                currentAddress += size;
                remainingSize -= size;
            }
        };

        template<bool isMainFlash,
                 typename I>
        static void writePage(std::uint32_t address,
                              I             start,
                              I             end) {
            static_assert(sizeof(decltype(*start)) == 4, "needs to be 4 byte type");
            NVMCTRL::Traits::disable_cache();
            std::size_t const size = static_cast<std::size_t>(std::distance(start, end));
            assert(PageSize / 4 >= size);
            NVMCTRL::Traits::set_addr(address);

            NVMCTRL::Traits::command_page_buffer_clear();
            waitForReady<Clock>();

            for(std::size_t i{}; i < PageSize / 4; ++i) {
                if(start != end) {
                    *(reinterpret_cast<std::uint32_t volatile*>(address) + i) = *start;
                    ++start;
                } else {
                    *(reinterpret_cast<std::uint32_t volatile*>(address) + i) = 0xffffffff;
                }
            }

            waitForReady<Clock>();

            if constexpr(isMainFlash) {
                NVMCTRL::Traits::command_write_page();
            } else {
                NVMCTRL::Traits::command_write_rww();
            }

            waitForReady<Clock>();
            NVMCTRL::Traits::enable_cache();
        }

        template<bool isMainFlash,
                 typename I>
        static void writeRow(std::uint32_t address,
                             I             start,
                             I             end) {
            static_assert(sizeof(decltype(*start)) == 4, "needs to be 4 byte type");
            std::size_t const size = static_cast<std::size_t>(std::distance(start, end));
            assert(RowSize / 4 >= size);

            for(std::size_t subPage{}; subPage < pagesPerRow; ++subPage) {
                writePage<isMainFlash>(
                  address,
                  start,
                  std::next(start,
                            std::min(static_cast<int>(PageSize / 4), std::distance(start, end))));
                if(std::distance(start, end) < static_cast<int>(PageSize / 4)) { return; }
                std::advance(start, PageSize / 4);
                address += PageSize;
            }
        }

        template<bool isMainFlash,
                 typename I>
        static void clearAndWriteRow(std::uint32_t address,
                                     I             start,
                                     I             end) {
            clearRow<Clock, isMainFlash>(address);
            writeRow<isMainFlash>(address, start, end);
        }

        template<typename I>
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void
        clearAndWriteMultibleRowsRam(std::uint32_t address,
                                     I             start,
                                     I             end) {
            KVASIR_RAM_FUNC_MARK();
            static_assert(sizeof(decltype(*start)) == 4, "needs to be 4 byte type");

            NVMCTRL::Traits::disable_cache();
            std::size_t size = static_cast<std::size_t>(end - start) * 4;
            while(size != 0) {
                NVMCTRL::Traits::set_addr(address);

                NVMCTRL::Traits::command_erase_row();
                while(!NVMCTRL::Traits::ready()) {}

                for(std::size_t subPage{}; subPage < pagesPerRow; ++subPage) {
                    NVMCTRL::Traits::set_addr(address);

                    NVMCTRL::Traits::command_page_buffer_clear();
                    while(!NVMCTRL::Traits::ready()) {}

                    for(std::size_t i{}; i < PageSize / 4; ++i) {
                        if(start != end) {
                            *(reinterpret_cast<std::uint32_t volatile*>(address) + i) = *start;
                            ++start;
                        } else {
                            *(reinterpret_cast<std::uint32_t volatile*>(address) + i) = 0xffffffff;
                        }
                    }
                    while(!NVMCTRL::Traits::ready()) {}

                    NVMCTRL::Traits::command_write_page();
                    while(!NVMCTRL::Traits::ready()) {}
                    address += PageSize;
                }
                if(size > RowSize) {
                    size -= RowSize;
                } else {
                    size = 0;
                }
            }
            NVMCTRL::Traits::enable_cache();
        }
    };

}   // namespace detail

template<typename Clock>
using Flash = detail::Flash<Clock, NVMCTRL::Traits::PageSize, NVMCTRL::Traits::PagesPerRow>;

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

        static void internalWrite() {
            ValueStruct newV;
            newV.v   = ramCopy;
            newV.crc = calcCrc(ramCopy);

            asm("" : "=m"(newV)::);

            Kvasir::Flash<Clock>::template clearAndWriteRow<isMainFlash>(
              reinterpret_cast<std::uint32_t>(&flashValue),
              reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&newV),
              reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&newV + 1));
        }

        static void writeValue() {
            auto const currentFlashValue = readFlashValue();
            if(ramCopy != currentFlashValue) { internalWrite(); }
        }
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

        static void writeValue() {
            auto const currentFlashValue = readFlashValue();
            if(ramCopy != currentFlashValue) { internalWrite(); }
        }

        static inline T    ramCopy{};
        static inline bool valueRead{false};

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

        static void internalWrite() {
            auto zeroPage = [&](PageStruct* p) {
                NVMCTRL::Traits::disable_cache();
                NVMCTRL::Traits::set_addr(p);
                NVMCTRL::Traits::command_page_buffer_clear();
                waitForReady<Clock>();

                for(std::size_t i = 0; i < pageSize / 4; ++i) {
                    *(reinterpret_cast<::Kvasir::detail::AliasedWord*>(p) + i) = 0;
                }

                waitForReady<Clock>();

                if constexpr(isMainFlash) {
                    NVMCTRL::Traits::command_write_page();
                } else {
                    NVMCTRL::Traits::command_write_rww();
                }
                waitForReady<Clock>();
                NVMCTRL::Traits::enable_cache();
            };

            auto writeNewV = [&](PageStruct* p, blockType blockV, ValueStruct const& v) {
                PageStruct newp{*p};

                newp.validBlock              = blockV;
                newp.array[getIndex(blockV)] = v;

                asm("" : "=m"(newp)::);

                NVMCTRL::Traits::set_addr(p);

                NVMCTRL::Traits::command_page_buffer_clear();
                waitForReady<Clock>();

                for(std::size_t i = 0; i < pageSize / 4; ++i) {
                    *(reinterpret_cast<::Kvasir::detail::AliasedWord*>(p) + i)
                      = *(reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&newp) + i);
                }

                if constexpr(isMainFlash) {
                    NVMCTRL::Traits::command_write_page();
                } else {
                    NVMCTRL::Traits::command_write_rww();
                }
                waitForReady<Clock>();
            };

            auto const newV{calcVS(ramCopy)};

            auto vp = getValidPage();

            PageStruct* pageToZero{nullptr};
            PageStruct* pageToWrite{nullptr};
            RowStruct*  rowToClear{nullptr};

            if(vp.nPages != 1 || vp.page == nullptr) {
                for(auto& row : rows) {
                    clearRow<Clock, isMainFlash>(reinterpret_cast<std::uint32_t>(&row));
                }
                pageToWrite = &(*rows.front().pages.begin());
            } else {
                if(pageFull(vp.page)) {
                    pageToWrite = nextPage(vp.page);
                    if(!pageCleared(pageToWrite)) {
                        for(auto& row : rows) {
                            clearRow<Clock, isMainFlash>(reinterpret_cast<std::uint32_t>(&row));
                        }
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

            writeNewV(pageToWrite, nextBlockValue(pageToWrite->validBlock), newV);

            if(rowToClear != nullptr) {
                clearRow<Clock, isMainFlash>(reinterpret_cast<std::uint32_t>(rowToClear));
            }

            if(pageToZero != nullptr) { zeroPage(pageToZero); }
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
        /// as well (6 ms max.; data sheet table 37-45, tFPP and tFRE).
        static void write(T const& v) {
            scan();
            std::size_t slot = newest == NumPages ? 0 : (newest + 1) % NumPages;
            // A page that is not blank where one should be (a write cut short): on to the
            // next row, which is erased before its first page is written.
            if(slot % pagesPerRow != 0 && !erased(storage.pages[slot])) {
                slot = (slot / pagesPerRow + 1) * pagesPerRow % NumPages;
            }
            if(slot % pagesPerRow == 0) {
                clearRow<Clock, isMainFlash>(reinterpret_cast<std::uint32_t>(&storage.pages[slot]));
            }

            Record r{};
            r.payload.sequence = sequence + 1;
            r.payload.v        = v;
            r.crc              = calcCrc(r.payload);

            asm("" : "=m"(r)::);

            Flash<Clock, pageSize, pagesPerRow>::template writePage<isMainFlash>(
              reinterpret_cast<std::uint32_t>(&storage.pages[slot]),
              reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&r),
              reinterpret_cast<::Kvasir::detail::AliasedWord const*>(&r + 1));

            newest   = slot;
            sequence = r.payload.sequence;
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

}   // namespace Kvasir
