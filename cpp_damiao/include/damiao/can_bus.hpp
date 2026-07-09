#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace damiao {

struct CanFrame {
  uint32_t id;
  std::array<uint8_t, 8> data;
  uint8_t dlc = 8;
  bool is_extended = false;
};

class CanBus {
 public:
  virtual ~CanBus() = default;

  virtual void send(const CanFrame& frame) = 0;
  virtual std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) = 0;
  virtual void shutdown() {}
};

}  // namespace damiao
