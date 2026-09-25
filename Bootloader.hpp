#pragma once

#include "NVMCTRL.hpp"
#include "core/SystemControl.hpp"
#include "kvasir/Util/Can.hpp"
#include "kvasir/Util/StaticString.hpp"
#include "kvasir/Util/StaticVector.hpp"
#include "kvasir/Util/attributes.hpp"

#include <cassert>

#if __has_include("aglio/serializer.hpp")

    #include "aglio/packager.hpp"
    #include "aglio/serializer.hpp"
    #include "kvasir/Util/Bootloader_td.hpp"

namespace Kvasir { namespace Bootloader {

    template<typename Clock, std::size_t BootLoaderSize, typename WDReset>
    struct Flash {
        using base                            = Kvasir::Flash<Clock>;
        static constexpr std::size_t RowSize  = base::RowSize;
        static constexpr std::size_t PageSize = base::PageSize;
        template<std::size_t WriteSize>
        using Writer = typename base::template Writer<WriteSize, true>;

        [[gnu::section(".noInit")]] alignas(std::uint32_t) static inline Kvasir::StaticVector<
          std::byte,
          BootLoaderSize - RowSize> bootloaderFlashBuffer;

        static void overrideSelf() {
            Kvasir::Nvic::disable_all();
            WDReset{}();
            overrideSelfRam();
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void overrideSelfRam() {
            KVASIR_RAM_FUNC_MARK();
            base::clearAndWriteMultibleRowsRam(
              0,
              reinterpret_cast<std::uint32_t const*>(&(*bootloaderFlashBuffer.begin())),
              reinterpret_cast<std::uint32_t const*>(&(*bootloaderFlashBuffer.end())));
            apply(Kvasir::SystemControl::SystemReset{});
        }
    };

    using Packager = Kvasir::CAN::Packager;

    struct Crc {
        using type = std::uint32_t;

        template<typename I>
        static type calc(I begin,
                         I end) {
            return Kvasir::CRC::calcCrc<Kvasir::CRC::CRC_Type::crc32>(begin, end);
        }
    };

    template<typename T,
             typename MsgBuffer,
             typename Buffer>
    std::optional<T> parse(MsgBuffer const& newMsg,
                           Buffer&          recvBuffer) {
        return Kvasir::CAN::parse<T, Packager>(newMsg, recvBuffer);
    }

    namespace CAN {

        template<typename Can,
                 typename T>
        void packAndSend(T const&     v,
                         std::uint8_t channel) {
            Kvasir::CAN::packAndSend<Can, Packager, 192>(v, channel + NodeId);
        }

        template<typename Clock,
                 typename Can,
                 typename RequestSet,
                 typename ResponseSet,
                 typename WDReset>
        struct Com {
            template<typename Rep,
                     typename Periode>
            static std::optional<RequestSet> recv(std::chrono::duration<Rep,
                                                                        Periode> timeout) {
                Kvasir::StaticVector<std::byte, 640> recvBuffer{};
                auto const                           endTime = Clock::now() + timeout;
                while(true) {
                    std::optional<typename Can::CanMessage> newMsg{};
                    WDReset{}();
                    while(!newMsg || newMsg->id() != CoordinatorId) {
                        newMsg = Can::recv();
                        if(Clock::now() > endTime) { return {}; }
                        WDReset{}();
                    }
                    auto ret = parse<RequestSet>(*newMsg, recvBuffer);
                    if(ret) { return ret; }
                }
                return {};
            }

            static void wait_for_all_send() {
                WDReset{}();
                while(!Can::transmissonComplete()) { WDReset{}(); }
            }

            static void send(ResponseSet const& response,
                             std::uint8_t       channel) {
                WDReset{}();
                packAndSend<Can>(response, channel);
            }
        };
    }   // namespace CAN

    namespace Uart {

        template<typename Clock,
                 typename Uart,
                 typename RequestSet,
                 typename ResponseSet,
                 typename WDReset,
                 typename DirectionPin>
        struct Com {
            template<typename Rep,
                     typename Periode>
            static std::optional<RequestSet> recv(std::chrono::duration<Rep,
                                                                        Periode> timeout) {
                Kvasir::StaticVector<std::byte, 640> recvBuffer;
                auto                                 endTime = Clock::now() + timeout;
                while(true) {
                    std::optional<std::byte> newByte;
                    WDReset{}();
                    while(!Uart::rxbuffer_.pop_into(newByte) || !newByte) {
                        if(Clock::now() > endTime) { return {}; }
                        WDReset{}();
                    }
                    recvBuffer.push_back(*newByte);
                    auto ret = Packager::unpack<RequestSet>(recvBuffer);
                    if(ret) { return ret; }
                }
            }

            static void send(ResponseSet const& response,
                             std::uint8_t) {
                Kvasir::StaticVector<std::byte, 128> buffer;

                if(!Packager::pack(buffer, response)) { assert(false); }
                apply(set(DirectionPin{}));
                Uart::send_nocopy(buffer.begin(), buffer.end());

                wait_for_all_send();
                apply(clear(DirectionPin{}));
            }

            static void wait_for_all_send() {
                while(true) {
                    auto const ostate = Uart::operationState();

                    WDReset{}();
                    if(ostate != Uart::OperationState::ongoing) { break; }
                }
            }
        };
    }   // namespace Uart

}}   // namespace Kvasir::Bootloader
#endif
