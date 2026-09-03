#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "articore/dds/protocol.hpp"

namespace articore::dds {

enum class SequenceChannel : std::uint8_t {
  Control,
  Stream,
};

struct LeaseSnapshot {
  std::string client_id;
  std::uint64_t lease_id = 0;
  std::chrono::steady_clock::time_point expires_at{};
};

class LeaseManager {
 public:
  using Clock = std::chrono::steady_clock;
  using LostCallback = std::function<void(const std::string&)>;

  explicit LeaseManager(
      LostCallback on_lost,
      std::chrono::milliseconds timeout = kLeasePeriod);

  Result<LeaseSnapshot> acquire(const std::string& client_id,
                                Clock::time_point now = Clock::now());
  ProtocolError authorize(const std::string& client_id,
                          std::uint64_t lease_id,
                          std::uint64_t sequence,
                          SequenceChannel channel,
                          bool refresh,
                          Clock::time_point now = Clock::now());
  ProtocolError heartbeat(const std::string& client_id,
                          std::uint64_t lease_id,
                          std::uint64_t sequence,
                          Clock::time_point now = Clock::now());
  ProtocolError release(const std::string& client_id,
                        std::uint64_t lease_id,
                        const std::string& reason = "lease released");
  bool expire_if_needed(Clock::time_point now = Clock::now());
  void revoke(const std::string& reason);
  std::optional<LeaseSnapshot> snapshot() const;

 private:
  void notify_lost(const std::string& reason);

  mutable std::mutex mutex_;
  LostCallback on_lost_;
  std::chrono::milliseconds timeout_;
  std::optional<LeaseSnapshot> active_;
  std::optional<std::uint64_t> last_control_sequence_;
  std::optional<std::uint64_t> last_stream_sequence_;
  std::uint64_t next_lease_id_ = 1;
};

}  // namespace articore::dds
