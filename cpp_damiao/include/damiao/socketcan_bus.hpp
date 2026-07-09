#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include "damiao/can_bus.hpp"

namespace damiao {

struct SocketCanRawFrame {
  uint32_t can_id = 0;
  uint8_t can_dlc = 0;
  std::array<uint8_t, 8> data{};
};

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

  static SocketCanRawFrame encode_classic(const CanFrame& frame);
  static CanFrame decode_classic(const SocketCanRawFrame& raw);
  static SocketCanFdRawFrame encode_fd(const CanFrame& frame, bool enable_brs);
  static CanFrame decode_fd(const SocketCanFdRawFrame& raw);
};

class SocketCanBus final : public CanBus {
 public:
  static std::shared_ptr<SocketCanBus> open(const std::string& interface);
  ~SocketCanBus() override;

  SocketCanBus(const SocketCanBus&) = delete;
  SocketCanBus& operator=(const SocketCanBus&) = delete;

  void send(const CanFrame& frame) override;
  std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) override;
  void shutdown() override;

 private:
  SocketCanBus(int fd, std::string interface);

  int fd_;
  std::string interface_;
  std::mutex mutex_;
};

class SocketCanFdBus final : public CanBus {
 public:
  static std::shared_ptr<SocketCanFdBus> open(const std::string& interface,
                                             bool enable_brs = false);
  ~SocketCanFdBus() override;

  SocketCanFdBus(const SocketCanFdBus&) = delete;
  SocketCanFdBus& operator=(const SocketCanFdBus&) = delete;

  void send(const CanFrame& frame) override;
  std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) override;
  void shutdown() override;

 private:
  SocketCanFdBus(int fd, std::string interface, bool enable_brs);

  int fd_;
  std::string interface_;
  bool enable_brs_;
  std::mutex mutex_;
};

}  // namespace damiao
