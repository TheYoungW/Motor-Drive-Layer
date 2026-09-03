#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "articore/runtime.hpp"

namespace articore::dds {

inline constexpr std::uint16_t kProtocolMajor = 1;
inline constexpr std::uint16_t kProtocolMinor = 0;
inline constexpr auto kStreamLifespan = std::chrono::milliseconds(20);
inline constexpr auto kLeasePeriod = std::chrono::milliseconds(250);
inline constexpr auto kLeaseHeartbeatPeriod = std::chrono::milliseconds(50);

inline constexpr const char* kDiscoveryTopic = "articore.robot.discovery";
inline constexpr const char* kControlRequestTopic =
    "articore.robot.control.request";
inline constexpr const char* kControlReplyTopic =
    "articore.robot.control.reply";
inline constexpr const char* kStreamCommandTopic =
    "articore.robot.stream.command";
inline constexpr const char* kStateTopic = "articore.robot.state";
inline constexpr const char* kHealthTopic = "articore.robot.health";
inline constexpr const char* kMotionEventTopic =
    "articore.robot.motion.event";

enum class ProtocolError : std::uint8_t {
  Ok = 0,
  InvalidArgument,
  WrongState,
  Busy,
  NoLease,
  StaleSequence,
  Timeout,
  TransportError,
  VersionMismatch,
  InternalError,
};

enum class StreamKind : std::uint8_t { Pv = 1, Mit = 2, MitFast = 3 };

struct MessageIdentity {
  std::uint16_t protocol_major = kProtocolMajor;
  std::uint16_t protocol_minor = kProtocolMinor;
  std::string robot_id;
  std::string client_id;
  std::string security_identity;
  std::uint64_t sequence = 0;
};

struct StreamCommand {
  MessageIdentity identity;
  std::uint64_t lease_id = 0;
  StreamKind kind = StreamKind::Pv;
  JointArray positions{};
  JointArray velocities{};
  JointArray kp{};
  JointArray kd{};
  JointArray feedforward_torques{};
  float speed_percent = 100.0f;
  std::chrono::steady_clock::time_point received_at{};
};

ProtocolError validate_identity(const MessageIdentity& identity,
                                const std::string& robot_id) noexcept;
ProtocolError validate_stream(const StreamCommand& command,
                              const std::string& robot_id) noexcept;
ProtocolError map_runtime_error(RuntimeErrorCode error) noexcept;
const char* runtime_state_name(SafetyState state) noexcept;
std::string describe_runtime_rejection(
    const std::string& operation, const std::string& required_states,
    const Status& status, const RuntimeHealth& health);

class LatestCommandMailbox {
 public:
  void store(StreamCommand command);
  std::optional<StreamCommand> take(
      std::chrono::steady_clock::time_point now);
  void clear();

 private:
  std::mutex mutex_;
  std::optional<StreamCommand> latest_;
};

}  // namespace articore::dds
