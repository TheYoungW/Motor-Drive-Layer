#include <array>
#include <iostream>
#include <stdexcept>

#include "damiao/socketcan_bus.hpp"

#if defined(__linux__)
#include <linux/can.h>
#endif

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  damiao::CanFrame frame{0x123, {1, 2, 3, 4, 5, 6, 7, 8}};
  frame.dlc = 8;
  const auto raw = damiao::SocketCanCodec::encode_classic(frame);
  require(raw.can_id == 0x123, "classic standard can id");
  require(raw.can_dlc == 8, "classic dlc");
  require(raw.data == (std::array<uint8_t, 8>{1, 2, 3, 4, 5, 6, 7, 8}), "classic payload");

  damiao::CanFrame ext{0x1ABCDE, {8, 7, 6, 5, 4, 3, 2, 1}};
  ext.dlc = 8;
  ext.is_extended = true;
  const auto raw_ext = damiao::SocketCanCodec::encode_classic(ext);
  require((raw_ext.can_id & damiao::SocketCanCodec::kCanEffFlag) != 0, "extended flag");
  require((raw_ext.can_id & damiao::SocketCanCodec::kCanEffMask) == 0x1ABCDE, "extended id");

  const auto decoded = damiao::SocketCanCodec::decode_classic(raw_ext);
  require(decoded.id == 0x1ABCDE, "decoded id");
  require(decoded.is_extended, "decoded extended flag");
  require(decoded.dlc == 8, "decoded dlc");
  require(decoded.data == ext.data, "decoded payload");

  const auto fd_raw = damiao::SocketCanCodec::encode_fd(frame);
  require(fd_raw.len == 8, "fd len");
  require((fd_raw.flags & damiao::SocketCanCodec::kCanFdBrs) != 0,
          "fd default includes CANFD_BRS");
#if defined(__linux__)
  static_assert(damiao::SocketCanCodec::kCanFdBrs == CANFD_BRS,
                "portable BRS flag must match Linux canfd_frame");
  canfd_frame kernel_fd_frame{};
  kernel_fd_frame.flags = fd_raw.flags;
  require((kernel_fd_frame.flags & CANFD_BRS) != 0,
          "Linux canfd_frame.flags includes CANFD_BRS");
#endif
  const auto fd_raw_without_brs = damiao::SocketCanCodec::encode_fd(frame, false);
  require((fd_raw_without_brs.flags & damiao::SocketCanCodec::kCanFdBrs) == 0,
          "fd explicit BRS disable clears CANFD_BRS");
  const auto fd_decoded = damiao::SocketCanCodec::decode_fd(fd_raw);
  require(fd_decoded.id == 0x123, "fd decoded id");
  require(fd_decoded.data == frame.data, "fd decoded payload");

  std::cout << "socketcan codec tests passed\n";
  return 0;
}
