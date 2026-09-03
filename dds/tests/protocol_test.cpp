#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "articore/dds/lease.hpp"

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main() {
  using namespace std::chrono_literals;
  using articore::dds::ProtocolError;
  require(articore::dds::kProtocolMajor == 1 &&
              articore::dds::kProtocolMinor == 2,
          "gripper state publication is DDS protocol 1.2");
  const auto epoch = articore::dds::LeaseManager::Clock::time_point{};
  std::string lost_reason;
  articore::dds::LeaseManager leases(
      [&](const std::string& reason) { lost_reason = reason; });

  auto first = leases.acquire("client-a", epoch);
  require(static_cast<bool>(first), "first client acquires the robot");
  auto competing = leases.acquire("client-b", epoch + 10ms);
  require(!competing &&
              competing.status().code() == articore::RuntimeErrorCode::Busy,
          "second client cannot acquire a live whole-robot lease");
  require(leases.authorize("client-a", first.value().lease_id, 1,
                           articore::dds::SequenceChannel::Stream, true,
                           epoch + 20ms) == ProtocolError::Ok,
          "valid stream command refreshes lease");
  require(leases.authorize("client-a", first.value().lease_id, 1,
                           articore::dds::SequenceChannel::Stream, true,
                           epoch + 21ms) == ProtocolError::StaleSequence,
          "duplicate sequence is rejected");
  require(leases.authorize("client-a", first.value().lease_id + 1, 2,
                           articore::dds::SequenceChannel::Stream, true,
                           epoch + 22ms) == ProtocolError::NoLease,
          "wrong lease id is rejected");
  require(leases.heartbeat("client-a", first.value().lease_id, 100,
                           epoch + 23ms) == ProtocolError::Ok,
          "control heartbeat advances only the control sequence domain");
  require(leases.authorize("client-a", first.value().lease_id, 2,
                           articore::dds::SequenceChannel::Stream, true,
                           epoch + 24ms) == ProtocolError::Ok,
          "a newer heartbeat cannot make an independently ordered stream stale");
  require(leases.authorize("client-a", first.value().lease_id, 2,
                           articore::dds::SequenceChannel::Control, false,
                           epoch + 25ms) == ProtocolError::StaleSequence,
          "control duplicate detection remains independent and monotonic");
  require(leases.refresh_after_control(
              "client-a", first.value().lease_id, epoch + 400ms) ==
              ProtocolError::Ok,
          "an accepted long-running control operation refreshes on completion");
  require(!leases.expire_if_needed(epoch + 640ms),
          "post-control refresh grants a complete lease period");
  require(leases.expire_if_needed(epoch + 651ms),
          "250 ms without heartbeat expires the lease");
  require(lost_reason == "control lease expired",
          "expiry invokes the Runtime safety callback");

  auto second = leases.acquire("client-b", epoch + 652ms);
  require(static_cast<bool>(second) &&
              second.value().lease_id != first.value().lease_id,
          "reconnect gets a new lease identity");

  articore::dds::StreamCommand command;
  command.identity.robot_id = "robot-1";
  command.identity.client_id = "client-b";
  command.identity.sequence = 9;
  command.lease_id = second.value().lease_id;
  command.received_at = epoch;
  require(articore::dds::validate_stream(command, "robot-1") ==
              ProtocolError::Ok,
          "fixed-size PV command validates");
  command.positions[4] = std::numeric_limits<float>::quiet_NaN();
  require(articore::dds::validate_stream(command, "robot-1") ==
              ProtocolError::InvalidArgument,
          "non-finite array member is rejected");

  articore::dds::LatestCommandMailbox mailbox;
  command.positions[4] = 0.0f;
  command.received_at = epoch;
  mailbox.store(command);
  require(!mailbox.take(epoch + 21ms), "expired stream sample is never run");

  articore::RuntimeHealth health;
  health.state = articore::SafetyState::Fault;
  health.fault_reason =
      "configure mode failed: right/r-joint7 register timeout";
  health.motor_faults = {"right/r-joint7", "right/r-gripper"};
  const auto rejection = articore::dds::describe_runtime_rejection(
      "CONFIGURE_MODE", "[READY]",
      articore::Status::failure(
          articore::RuntimeErrorCode::WrongState,
          "configure mode rejected while Runtime is faulted"),
      health);
  require(rejection.find("CONFIGURE_MODE rejected") != std::string::npos &&
              rejection.find("current_state=FAULT") != std::string::npos &&
              rejection.find("required_states=[READY]") != std::string::npos &&
              rejection.find("right/r-joint7 register timeout") !=
                  std::string::npos &&
              rejection.find(
                  "failed_motors=[right/r-joint7, right/r-gripper]") !=
                  std::string::npos,
          "state rejection retains state, root cause, and failed Motors");

  std::cout << "DDS protocol and lease tests passed\n";
}
