#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "damiao/can_bus.hpp"

namespace damiao {

class SocketCanFdBusTestPeer;

struct SocketCanFdRawFrame {
  uint32_t can_id = 0;
  uint8_t len = 0;
  uint8_t flags = 0;
  std::array<uint8_t, 64> data{};
};

class SocketCanCodec {
 public:
  static constexpr uint32_t kCanEffFlag = 0x80000000U;
  static constexpr uint32_t kCanEffMask = 0x1FFFFFFFU;
  static constexpr uint32_t kCanSffMask = 0x000007FFU;
  static constexpr uint8_t kCanFdBrs = 0x01U;

  static SocketCanFdRawFrame encode_fd(const CanFrame& frame, bool enable_brs = true);
  static CanFrame decode_fd(const SocketCanFdRawFrame& raw);
};

class SocketCanFdBus final : public CanBus {
 public:
  static std::shared_ptr<SocketCanFdBus> open(const std::string& interface,
                                             bool enable_brs = true);
  ~SocketCanFdBus() override;

  SocketCanFdBus(const SocketCanFdBus&) = delete;
  SocketCanFdBus& operator=(const SocketCanFdBus&) = delete;

  void send(const CanFrame& frame) override;
  std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) override;
  void wake_receive() override;
  void shutdown() override;
  TransportCapabilities capabilities() const override;

 private:
  friend class SocketCanFdBusTestPeer;

  SocketCanFdBus(int fd, std::string interface, bool enable_brs,
                 std::chrono::milliseconds send_timeout);

  int fd_;
  int wake_fd_;
  std::string interface_;
  bool enable_brs_;
  std::chrono::milliseconds send_timeout_;
  std::mutex state_mutex_;
  std::mutex send_mutex_;
};

}  // namespace damiao
