#include <array>
#include <atomic>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "damiao/runtime.hpp"
#include "damiao/socketcan_bus.hpp"

#if defined(__linux__)
#include <cerrno>
#include <linux/can.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace damiao {

class SocketCanBusTestPeer {
 public:
  static std::shared_ptr<SocketCanFdBus> adopt_fd(
      int fd, std::chrono::milliseconds send_timeout) {
    return std::shared_ptr<SocketCanFdBus>(
        new SocketCanFdBus(fd, "blocked-test", true, send_timeout));
  }
};

}  // namespace damiao

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

#if defined(__linux__)
class ImmediateBus final : public damiao::CanBus {
 public:
  void send(const damiao::CanFrame&) override { ++send_count; }
  std::optional<damiao::CanFrame> receive_for(std::chrono::milliseconds) override {
    return std::nullopt;
  }

  std::atomic<uint64_t> send_count{0};
};

void verify_blocked_fd_send_is_bounded() {
  int sockets[2] = {-1, -1};
  require(::socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0, sockets) == 0,
          "create non-blocking socket pair");

  int send_buffer = 4096;
  require(::setsockopt(sockets[0], SOL_SOCKET, SO_SNDBUF, &send_buffer,
                       sizeof(send_buffer)) == 0,
          "set test send buffer");
  std::array<uint8_t, 1024> filler{};
  while (::write(sockets[0], filler.data(), filler.size()) ==
         static_cast<ssize_t>(filler.size())) {
  }
  require(errno == EAGAIN || errno == EWOULDBLOCK,
          "fill the test transmit queue");

  constexpr auto send_timeout = std::chrono::milliseconds(15);
  auto bus = damiao::SocketCanBusTestPeer::adopt_fd(sockets[0], send_timeout);
  sockets[0] = -1;
  damiao::Controller controller(bus, "blocked SocketCAN-FD endpoint");
  auto motor = controller.add_damiao_motor(0x01, 0x11, "4340P");

  const auto started = std::chrono::steady_clock::now();
  std::string send_error;
  try {
    motor->send_mit(0.0f, 0.0f, 1.0f, 0.1f, 0.0f);
  } catch (const std::runtime_error& error) {
    send_error = error.what();
  }
  const auto elapsed = std::chrono::steady_clock::now() - started;
  require(send_error.find("socketcanfd blocked-test send timed out after 15 ms") !=
              std::string::npos &&
              send_error.find("transmit queue remained full") != std::string::npos,
          "blocked SocketCAN-FD send reports a stable queue timeout");
  require(elapsed >= std::chrono::milliseconds(10) &&
              elapsed < std::chrono::milliseconds(250),
          "blocked SocketCAN-FD send returns within a bounded interval");

  const auto health = controller.transport_health();
  require(!health.healthy && health.send_errors == 1 &&
              health.last_error.find("transmit queue remained full") != std::string::npos,
          "SocketCAN-FD timeout becomes structured transport health");

  auto healthy_bus = std::make_shared<ImmediateBus>();
  damiao::Controller healthy_controller(healthy_bus, "can-left");
  auto healthy_motor =
      healthy_controller.add_damiao_motor(0x02, 0x12, "4340P");
  std::string group_error;
  const auto group_started = std::chrono::steady_clock::now();
  {
    damiao::ControllerGroup group({&healthy_controller, &controller});
    try {
      group.send_mit({{healthy_motor, 0.0f, 0.0f, 1.0f, 0.1f, 0.0f},
                      {motor, 0.0f, 0.0f, 1.0f, 0.1f, 0.0f}});
    } catch (const std::runtime_error& error) {
      group_error = error.what();
    }
  }
  const auto group_elapsed = std::chrono::steady_clock::now() - group_started;
  require(healthy_bus->send_count.load() == 1,
          "healthy controller still completes its parallel dispatch");
  require(group_error.find("controller index 1") != std::string::npos &&
              group_error.find("blocked SocketCAN-FD endpoint") != std::string::npos &&
              group_error.find("transmit queue remained full") != std::string::npos,
          "bounded queue timeout propagates through ControllerGroup context");
  require(group_elapsed < std::chrono::milliseconds(250),
          "dual-controller dispatch returns after the bounded send timeout");
  healthy_controller.close_bus();

  std::atomic<bool> sender_started{false};
  std::exception_ptr sender_error;
  std::thread sender([&] {
    sender_started.store(true, std::memory_order_release);
    try {
      motor->send_mit(0.0f, 0.0f, 1.0f, 0.1f, 0.0f);
    } catch (...) {
      sender_error = std::current_exception();
    }
  });
  while (!sender_started.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  const auto close_started = std::chrono::steady_clock::now();
  controller.close_bus();
  const auto close_elapsed = std::chrono::steady_clock::now() - close_started;
  sender.join();
  require(sender_error != nullptr,
          "concurrent blocked sender exits with a transport error");
  require(close_elapsed < std::chrono::milliseconds(250),
          "close waits only for the bounded SocketCAN-FD send");

  ::close(sockets[1]);
}
#endif

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

#if defined(__linux__)
  verify_blocked_fd_send_is_bounded();
#endif

  std::cout << "socketcan codec tests passed\n";
  return 0;
}
