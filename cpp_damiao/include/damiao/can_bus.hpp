#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace damiao {

struct CanFrame {
  uint32_t id;
  std::array<uint8_t, 8> data;
  uint8_t dlc = 8;
  bool is_extended = false;
  // Physical receive channel when the transport exposes one. 0xFF means the
  // transport is already single-channel or does not report channel identity.
  uint8_t channel = 0xFF;
};

struct TransportCapabilities {
  std::string transport = "custom";
  uint32_t max_payload_bytes = 8;
  uint32_t channel_count = 1;
  bool can_fd = false;
  bool parallel_batches = true;
  bool hardware_rx_timestamps = false;
  bool reconnect = false;
  bool process_session_reuse = false;
};

struct TransportHealth {
  bool connected = true;
  bool healthy = true;
  uint64_t tx_frames = 0;
  uint64_t rx_frames = 0;
  uint64_t send_errors = 0;
  uint64_t receive_errors = 0;
  std::optional<std::chrono::nanoseconds> last_tx_age;
  std::optional<std::chrono::nanoseconds> last_rx_age;
  std::string last_error;
};

class CanBus {
 public:
  virtual ~CanBus() = default;

  virtual void send(const CanFrame& frame) = 0;
  virtual std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) = 0;
  virtual void shutdown() {}
  virtual TransportCapabilities capabilities() const { return {}; }
  virtual TransportHealth health() const { return {}; }
};

}  // namespace damiao
