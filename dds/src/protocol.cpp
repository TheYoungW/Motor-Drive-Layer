#include "articore/dds/protocol.hpp"

#include <algorithm>
#include <sstream>

namespace articore::dds {
namespace {
bool finite(const JointArray& values) {
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}
}  // namespace

ProtocolError validate_identity(const MessageIdentity& identity,
                                const std::string& robot_id) noexcept {
  if (identity.protocol_major != kProtocolMajor ||
      identity.protocol_minor > kProtocolMinor) {
    return ProtocolError::VersionMismatch;
  }
  if (identity.robot_id.empty() || identity.client_id.empty() ||
      identity.robot_id.size() > 63 || identity.client_id.size() > 63 ||
      identity.robot_id != robot_id) {
    return ProtocolError::InvalidArgument;
  }
  return ProtocolError::Ok;
}

ProtocolError validate_stream(const StreamCommand& command,
                              const std::string& robot_id) noexcept {
  const auto identity = validate_identity(command.identity, robot_id);
  if (identity != ProtocolError::Ok) return identity;
  if (command.lease_id == 0 || !finite(command.positions) ||
      !std::isfinite(command.speed_percent)) {
    return ProtocolError::InvalidArgument;
  }
  if (command.kind == StreamKind::Mit &&
      (!finite(command.velocities) || !finite(command.kp) ||
       !finite(command.kd) || !finite(command.feedforward_torques))) {
    return ProtocolError::InvalidArgument;
  }
  if (command.speed_percent < 0.0f || command.speed_percent > 100.0f ||
      (command.kind == StreamKind::Pv && command.speed_percent < 1.0f)) {
    return ProtocolError::InvalidArgument;
  }
  return ProtocolError::Ok;
}

ProtocolError map_runtime_error(RuntimeErrorCode error) noexcept {
  switch (error) {
    case RuntimeErrorCode::Ok: return ProtocolError::Ok;
    case RuntimeErrorCode::InvalidArgument: return ProtocolError::InvalidArgument;
    case RuntimeErrorCode::WrongState: return ProtocolError::WrongState;
    case RuntimeErrorCode::Busy: return ProtocolError::Busy;
    case RuntimeErrorCode::Timeout: return ProtocolError::Timeout;
    case RuntimeErrorCode::TransportError: return ProtocolError::TransportError;
    case RuntimeErrorCode::InternalError: return ProtocolError::InternalError;
  }
  return ProtocolError::InternalError;
}

const char* runtime_state_name(SafetyState state) noexcept {
  switch (state) {
    case SafetyState::Disconnected: return "DISCONNECTED";
    case SafetyState::Ready: return "READY";
    case SafetyState::Enabled: return "ENABLED";
    case SafetyState::Running: return "RUNNING";
    case SafetyState::SafeHold: return "SAFE_HOLD";
    case SafetyState::Fault: return "FAULT";
    case SafetyState::Degraded: return "DEGRADED";
    case SafetyState::SafeStop: return "SAFE_STOP";
    case SafetyState::PartiallyEnabled: return "PARTIALLY_ENABLED";
  }
  return "UNKNOWN";
}

std::string describe_runtime_rejection(
    const std::string& operation, const std::string& required_states,
    const Status& status, const RuntimeHealth& health) {
  std::vector<std::string> failed = health.operation_failed_motors;
  for (const auto& name : health.motor_faults) {
    if (std::find(failed.begin(), failed.end(), name) == failed.end()) {
      failed.push_back(name);
    }
  }
  std::ostringstream output;
  output << operation << " rejected: current_state="
         << runtime_state_name(health.state)
         << ", required_states=" << required_states
         << ", fault_reason="
         << (health.fault_reason.empty() ? "<none>" : health.fault_reason)
         << ", failed_motors=[";
  for (std::size_t index = 0; index < failed.size(); ++index) {
    if (index != 0) output << ", ";
    output << failed[index];
  }
  output << "]"
         << ", cause="
         << (status.message().empty() ? "<none>" : status.message());
  return output.str();
}

void LatestCommandMailbox::store(StreamCommand command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_ || command.identity.sequence > latest_->identity.sequence) {
    latest_ = std::move(command);
  }
}

std::optional<StreamCommand> LatestCommandMailbox::take(
    std::chrono::steady_clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_) return std::nullopt;
  if (now - latest_->received_at > kStreamLifespan) {
    latest_.reset();
    return std::nullopt;
  }
  auto output = std::move(latest_);
  latest_.reset();
  return output;
}

void LatestCommandMailbox::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
}

}  // namespace articore::dds
