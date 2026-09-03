#include "articore/dds/service.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "articore/dds/lease.hpp"
#include "articore_protocol.h"
#include "dds/dds.h"

namespace articore::dds {
namespace {

using Clock = std::chrono::steady_clock;

void check(dds_return_t result, const char* operation) {
  if (result >= 0) return;
  throw std::runtime_error(std::string(operation) + ": " +
                           dds_strretcode(-result));
}

class Entity {
 public:
  Entity() = default;
  explicit Entity(dds_entity_t value) : value_(value) { check(value, "create DDS entity"); }
  ~Entity() { reset(); }
  Entity(const Entity&) = delete;
  Entity& operator=(const Entity&) = delete;
  Entity(Entity&& other) noexcept : value_(std::exchange(other.value_, 0)) {}
  Entity& operator=(Entity&& other) noexcept {
    if (this != &other) {
      reset();
      value_ = std::exchange(other.value_, 0);
    }
    return *this;
  }
  dds_entity_t get() const noexcept { return value_; }
  void reset() noexcept {
    if (value_ > 0) (void)dds_delete(value_);
    value_ = 0;
  }
 private:
  dds_entity_t value_ = 0;
};

class Qos {
 public:
  Qos() : value_(dds_create_qos()) {
    if (!value_) throw std::runtime_error("dds_create_qos failed");
  }
  ~Qos() { dds_delete_qos(value_); }
  dds_qos_t* get() const noexcept { return value_; }
 private:
  dds_qos_t* value_;
};

void reliable(dds_qos_t* qos, int depth, bool transient = false) {
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(20));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, depth);
  if (transient) dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
}

void best_effort(dds_qos_t* qos, int depth) {
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_MSECS(0));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, depth);
}

template <std::size_t N>
void copy_text(char (&destination)[N], const std::string& source) {
  std::snprintf(destination, N, "%s", source.c_str());
}

articore_wire_ProtocolError wire_error(ProtocolError error) {
  return static_cast<articore_wire_ProtocolError>(error);
}

MessageIdentity identity(const articore_wire_ControlRequest& request) {
  return {request.protocol_major, request.protocol_minor, request.robot_id,
          request.client_id, request.security_identity, request.sequence_id};
}

MessageIdentity identity(const articore_wire_StreamCommand& request) {
  return {request.protocol_major, request.protocol_minor, request.robot_id,
          request.client_id, request.security_identity, request.sequence_id};
}

template <std::size_t N>
std::array<float, N> copy_array(const float (&source)[N]) {
  std::array<float, N> output{};
  std::copy_n(source, N, output.begin());
  return output;
}

ProtocolError protocol_status(const Status& status) {
  return status ? ProtocolError::Ok : map_runtime_error(status.code());
}

const char* diagnostic_operation(articore_wire_ControlOperation operation) {
  switch (operation) {
    case articore_wire_CONFIGURE_MODE: return "CONFIGURE_MODE";
    case articore_wire_CLEAR_FAULTS: return "CLEAR_FAULTS";
    default: return nullptr;
  }
}

const char* required_states(articore_wire_ControlOperation operation) {
  return operation == articore_wire_CLEAR_FAULTS
      ? "[READY, FAULT(non_estop)]"
      : "[READY]";
}

}  // namespace

class CycloneService::Impl {
 public:
  explicit Impl(ServiceConfig selected)
      : config_(std::move(selected)),
        lease_([this](const std::string& reason) {
          mailbox_.clear();
          if (runtime_) (void)runtime_->on_control_lost(reason);
          if (pending_motion_request_ != 0) {
            publish_motion_event(pending_motion_client_, pending_motion_request_,
                                 articore_wire_MOTION_CANCELLED,
                                 ProtocolError::NoLease, reason);
            pending_motion_request_ = 0;
            pending_motion_client_.clear();
          }
        }) {
    const auto uri = cyclone_uri(config_);
    domain_ = Entity(dds_create_domain(config_.domain_id, uri.c_str()));
    participant_ = Entity(dds_create_participant(
        config_.domain_id, nullptr, nullptr));
    create_entities();
    next_runtime_retry_ = Clock::now();
  }

  Status run(const std::atomic<bool>& external_stop) {
    auto next_state = Clock::now();
    auto next_health = next_state;
    auto next_discovery = next_state;
    while (!external_stop.load(std::memory_order_relaxed) &&
           !stop_.load(std::memory_order_relaxed)) {
      const auto now = Clock::now();
      ensure_runtime(now);
      (void)lease_.expire_if_needed(now);

      dds_attach_t triggered[2]{};
      const auto until_state = next_state > now
          ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                next_state - now).count()
          : 0;
      // Anchor the state publisher to its absolute 2 ms deadline. A fixed
      // 2 ms WaitSet timeout would add request-processing overhead on every
      // iteration and slowly lower the observable state rate below 500 Hz.
      const auto wait_timeout = std::min<dds_duration_t>(
          static_cast<dds_duration_t>(until_state), DDS_MSECS(2));
      const auto count = dds_waitset_wait(
          waitset_.get(), triggered, 2, wait_timeout);
      if (count < 0) {
        return Status::failure(RuntimeErrorCode::TransportError,
                               dds_strretcode(-count));
      }
      if (count > 0) {
        take_control_requests();
        take_stream_commands();
      }
      pump_latest(Clock::now());

      const auto after_io = Clock::now();
      if (after_io >= next_state) {
        publish_state();
        advance(next_state, std::chrono::milliseconds(2), after_io);
      }
      publish_health_on_change();
      if (after_io >= next_health) {
        publish_health();
        advance(next_health, std::chrono::milliseconds(100), after_io);
      }
      if (after_io >= next_discovery) {
        publish_discovery();
        advance(next_discovery, std::chrono::seconds(1), after_io);
      }
    }

    // Fixed SIGTERM order: stop input, revoke lease/cancel, safe disable,
    // close CAN, then let RAII destroy DDS after run returns.
    lease_.revoke("runtime service is stopping");
    mailbox_.clear();
    if (runtime_) {
      (void)runtime_->stop_motion();
      (void)runtime_->disable();
      (void)runtime_->disconnect();
      runtime_.reset();
    }
    return Status::success();
  }

  void request_stop() noexcept { stop_.store(true, std::memory_order_relaxed); }

 private:
  static void advance(Clock::time_point& deadline,
                      Clock::duration period, Clock::time_point now) {
    do deadline += period; while (deadline <= now);
  }

  Entity topic(const dds_topic_descriptor_t* descriptor, const char* name) {
    return Entity(dds_create_topic(participant_.get(), descriptor, name,
                                   nullptr, nullptr));
  }

  Entity reader(dds_entity_t topic_entity, dds_qos_t* qos) {
    return Entity(dds_create_reader(participant_.get(), topic_entity,
                                    qos, nullptr));
  }
  Entity writer(dds_entity_t topic_entity, dds_qos_t* qos) {
    return Entity(dds_create_writer(participant_.get(), topic_entity,
                                    qos, nullptr));
  }

  void create_entities() {
    discovery_topic_ = topic(&articore_wire_Discovery_desc, kDiscoveryTopic);
    request_topic_ = topic(&articore_wire_ControlRequest_desc,
                           kControlRequestTopic);
    reply_topic_ = topic(&articore_wire_ControlReply_desc, kControlReplyTopic);
    stream_topic_ = topic(&articore_wire_StreamCommand_desc,
                          kStreamCommandTopic);
    state_topic_ = topic(&articore_wire_RobotState_desc, kStateTopic);
    health_topic_ = topic(&articore_wire_Health_desc, kHealthTopic);
    motion_topic_ = topic(&articore_wire_MotionEvent_desc, kMotionEventTopic);

    Qos discovery_qos;
    reliable(discovery_qos.get(), 1, true);
    discovery_writer_ = writer(discovery_topic_.get(), discovery_qos.get());
    Qos request_qos;
    reliable(request_qos.get(), 32);
    request_reader_ = reader(request_topic_.get(), request_qos.get());
    reply_writer_ = writer(reply_topic_.get(), request_qos.get());
    Qos stream_qos;
    best_effort(stream_qos.get(), 1);
    dds_qset_lifespan(stream_qos.get(), DDS_MSECS(20));
    stream_reader_ = reader(stream_topic_.get(), stream_qos.get());
    Qos state_qos;
    best_effort(state_qos.get(), 1);
    state_writer_ = writer(state_topic_.get(), state_qos.get());
    Qos health_qos;
    reliable(health_qos.get(), 1, true);
    health_writer_ = writer(health_topic_.get(), health_qos.get());
    Qos motion_qos;
    reliable(motion_qos.get(), 32);
    motion_writer_ = writer(motion_topic_.get(), motion_qos.get());

    check(dds_set_status_mask(request_reader_.get(), DDS_DATA_AVAILABLE_STATUS),
          "set request reader status mask");
    check(dds_set_status_mask(stream_reader_.get(), DDS_DATA_AVAILABLE_STATUS),
          "set stream reader status mask");
    waitset_ = Entity(dds_create_waitset(participant_.get()));
    check(dds_waitset_attach(waitset_.get(), request_reader_.get(),
                             request_reader_.get()),
          "attach request reader");
    check(dds_waitset_attach(waitset_.get(), stream_reader_.get(),
                             stream_reader_.get()),
          "attach stream reader");
  }

  void ensure_runtime(Clock::time_point now) {
    if (runtime_ || now < next_runtime_retry_) return;
    auto candidate = YunyiRuntime::create(config_.runtime);
    if (!candidate) {
      runtime_error_ = candidate.status().message();
      next_runtime_retry_ = now + retry_delay_;
      retry_delay_ = std::min(retry_delay_ * 2, config_.can_retry_max);
      return;
    }
    runtime_ = std::move(candidate).value();
    const auto connected = runtime_->connect();
    if (!connected) {
      runtime_error_ = connected.message();
      runtime_.reset();
      next_runtime_retry_ = now + retry_delay_;
      retry_delay_ = std::min(retry_delay_ * 2, config_.can_retry_max);
      return;
    }
    // connect verifies feedback and physical disable; it never enables.
    runtime_error_.clear();
    retry_delay_ = config_.can_retry_initial;
  }

  void take_control_requests() {
    for (std::size_t handled = 0; handled < 32; ++handled) {
      articore_wire_ControlRequest sample{};
      void* samples[1]{&sample};
      dds_sample_info_t info[1]{};
      const auto count = dds_take(request_reader_.get(), samples, info, 1, 1);
      if (count <= 0) break;
      if (info[0].valid_data) handle_request(sample);
    }
  }

  void take_stream_commands() {
    for (std::size_t handled = 0; handled < 64; ++handled) {
      articore_wire_StreamCommand wire{};
      void* samples[1]{&wire};
      dds_sample_info_t info[1]{};
      const auto count = dds_take(stream_reader_.get(), samples, info, 1, 1);
      if (count <= 0) break;
      if (!info[0].valid_data) continue;
      StreamCommand command;
      command.identity = identity(wire);
      command.lease_id = wire.lease_id;
      command.kind = static_cast<StreamKind>(wire.kind + 1);
      command.positions = copy_array(wire.positions);
      command.velocities = copy_array(wire.velocities);
      command.kp = copy_array(wire.kp);
      command.kd = copy_array(wire.kd);
      command.feedforward_torques = copy_array(wire.feedforward_torques);
      command.speed_percent = wire.speed_percent;
      command.received_at = Clock::now();
      auto error = validate_stream(command, config_.robot_id);
      if (error == ProtocolError::Ok) {
        error = lease_.authorize(command.identity.client_id,
                                 command.lease_id,
                                 command.identity.sequence,
                                 SequenceChannel::Stream, true,
                                 command.received_at);
      }
      if (error == ProtocolError::Ok) mailbox_.store(std::move(command));
    }
  }

  void pump_latest(Clock::time_point now) {
    auto command = mailbox_.take(now);
    if (!command || !runtime_) return;
    Status status;
    if (command->kind == StreamKind::Pv) {
      status = runtime_->set_joint_pv(command->positions,
                                      command->speed_percent);
    } else if (command->kind == StreamKind::MitFast) {
      status = runtime_->set_joint_mit_fast(command->positions,
                                            command->speed_percent);
    } else {
      MitCommand mit{command->positions, command->velocities, command->kp,
                     command->kd, command->feedforward_torques};
      status = runtime_->set_joint_mit(mit);
    }
    if (!status) runtime_error_ = status.message();
  }

  void initialize_reply(const articore_wire_ControlRequest& request,
                        articore_wire_ControlReply& reply) {
    reply.protocol_major = kProtocolMajor;
    reply.protocol_minor = kProtocolMinor;
    copy_text(reply.robot_id, config_.robot_id);
    copy_text(reply.client_id, request.client_id);
    copy_text(reply.security_identity, request.security_identity);
    reply.sequence_id = ++reply_sequence_;
    reply.request_id = request.request_id;
    reply.lease_id = request.lease_id;
  }

  void finish_reply(articore_wire_ControlReply& reply, ProtocolError error,
                    const std::string& message = {}) {
    reply.error = wire_error(error);
    copy_text(reply.message, message);
    check(dds_write(reply_writer_.get(), &reply), "write control reply");
  }

  void handle_request(const articore_wire_ControlRequest& request) {
    articore_wire_ControlReply reply{};
    initialize_reply(request, reply);
    auto error = validate_identity(identity(request), config_.robot_id);
    if (error != ProtocolError::Ok) {
      finish_reply(reply, error, "request identity or protocol version rejected");
      return;
    }
    const auto operation = request.operation;
    if (operation == articore_wire_ACQUIRE_LEASE) {
      auto acquired = lease_.acquire(request.client_id);
      if (!acquired) {
        finish_reply(reply, map_runtime_error(acquired.status().code()),
                     acquired.status().message());
      } else {
        reply.lease_id = acquired.value().lease_id;
        finish_reply(reply, ProtocolError::Ok);
      }
      return;
    }
    if (operation == articore_wire_HEARTBEAT) {
      error = lease_.heartbeat(request.client_id, request.lease_id,
                               request.sequence_id);
      finish_reply(reply, error);
      return;
    }
    if (operation == articore_wire_RELEASE_LEASE) {
      error = lease_.release(request.client_id, request.lease_id);
      finish_reply(reply, error);
      return;
    }
    const bool query_only = operation >= articore_wire_QUERY_STATE;
    if (!query_only) {
      error = lease_.authorize(request.client_id, request.lease_id,
                               request.sequence_id,
                               SequenceChannel::Control, false);
      if (error != ProtocolError::Ok) {
        finish_reply(reply, error);
        return;
      }
    }
    if (!runtime_) {
      finish_reply(reply, ProtocolError::TransportError,
                   runtime_error_.empty() ? "CAN is unavailable" : runtime_error_);
      return;
    }
    const auto status = execute(request, reply);
    const bool finite_motion =
        operation == articore_wire_MOVE_POSE ||
        operation == articore_wire_MOVE_LINEAR ||
        operation == articore_wire_MOVE_CIRCULAR;
    if (finite_motion) {
      if (status) {
        pending_motion_request_ = request.request_id;
        pending_motion_client_ = request.client_id;
        publish_motion_event(request.client_id, request.request_id,
                             articore_wire_MOTION_ACCEPTED,
                             ProtocolError::Ok, "motion accepted");
      } else {
        publish_motion_event(request.client_id, request.request_id,
                             articore_wire_MOTION_FAILED,
                             protocol_status(status), status.message());
      }
    } else if (operation == articore_wire_STOP_MOTION && status &&
               pending_motion_request_ != 0) {
      publish_motion_event(pending_motion_client_, pending_motion_request_,
                           articore_wire_MOTION_CANCELLED,
                           ProtocolError::Ok, "motion cancelled");
      pending_motion_request_ = 0;
      pending_motion_client_.clear();
    }
    std::string reply_message = status.message();
    if (!status) {
      const char* diagnostic = diagnostic_operation(operation);
      if (diagnostic) {
        const auto health = runtime_->health();
        if (health) {
          reply_message = describe_runtime_rejection(
              diagnostic, required_states(operation), status, health.value());
        }
      }
    }
    finish_reply(reply, protocol_status(status), reply_message);
  }

  Status execute(const articore_wire_ControlRequest& request,
                 articore_wire_ControlReply& reply) {
    const auto side = static_cast<RobotSide>(request.side);
    switch (request.operation) {
      case articore_wire_CONNECT: return runtime_->connect();
      case articore_wire_DISCONNECT: return runtime_->disconnect();
      case articore_wire_CONFIGURE_MODE:
        return runtime_->configure_mode(static_cast<ControlMode>(request.mode));
      case articore_wire_ENABLE: return runtime_->enable();
      case articore_wire_DISABLE: return runtime_->disable();
      case articore_wire_SET_ZERO: return runtime_->set_zero();
      case articore_wire_CLEAR_FAULTS: return runtime_->clear_faults();
      case articore_wire_ESTOP: return runtime_->estop();
      case articore_wire_RECOVER: return runtime_->recover();
      case articore_wire_SET_SPEED_PERCENT:
        return runtime_->set_speed_percent(request.scalar[0]);
      case articore_wire_SET_MAX_SPEED:
        return runtime_->set_max_speed(request.scalar[0]);
      case articore_wire_SET_MAX_ACCELERATION:
        return runtime_->set_max_acceleration(request.scalar[0]);
      case articore_wire_SOLVE_IK: {
        auto result = runtime_->solve_ik(copy_array(request.pose_a),
                                         copy_array(request.pose_b));
        if (!result) return result.status();
        std::copy(result.value().begin(), result.value().end(), reply.values);
        return Status::success();
      }
      case articore_wire_MOVE_POSE:
        return runtime_->move_pose(side, copy_array(request.pose_a));
      case articore_wire_MOVE_LINEAR:
        return runtime_->move_linear(side, copy_array(request.pose_b));
      case articore_wire_MOVE_CIRCULAR:
        return runtime_->move_circular(side, copy_array(request.pose_a),
                                       copy_array(request.pose_b),
                                       copy_array(request.pose_c));
      case articore_wire_STOP_MOTION: return runtime_->stop_motion();
      case articore_wire_SET_GRIPPERS:
        return runtime_->set_grippers(
            request.scalar[0], request.scalar[1],
            static_cast<int>(request.scalar[2]),
            static_cast<GripperMode>(static_cast<int>(request.scalar[3])));
      case articore_wire_SET_TCP_OFFSET:
        return runtime_->set_tcp_offset(side, copy_array(request.pose_a));
      case articore_wire_RESET_TCP_OFFSET:
        return runtime_->reset_tcp_offset(side);
      case articore_wire_START_GRAVITY_COMPENSATION:
        return runtime_->start_gravity_compensation(std::chrono::milliseconds(
            static_cast<std::uint32_t>(request.scalar[0])));
      case articore_wire_STOP_GRAVITY_COMPENSATION:
        return runtime_->stop_gravity_compensation();
      case articore_wire_START_BIMANUAL_FOLLOW:
        return runtime_->start_bimanual_follow(side);
      case articore_wire_STOP_BIMANUAL_FOLLOW:
        return runtime_->stop_bimanual_follow();
      case articore_wire_QUERY_STATE: {
        auto state = runtime_->state();
        if (!state) return state.status();
        std::copy(state.value().positions.begin(), state.value().positions.end(),
                  reply.values);
        return Status::success();
      }
      case articore_wire_QUERY_HEALTH: {
        auto health = runtime_->health();
        if (!health) return health.status();
        reply.values[0] = static_cast<float>(health.value().state);
        return Status::success();
      }
      case articore_wire_GET_POSE: {
        auto value = runtime_->pose(side);
        if (!value) return value.status();
        std::copy(value.value().begin(), value.value().end(), reply.values);
        return Status::success();
      }
      case articore_wire_GET_TCP_OFFSET: {
        auto value = runtime_->tcp_offset(side);
        if (!value) return value.status();
        std::copy(value.value().begin(), value.value().end(), reply.values);
        return Status::success();
      }
      case articore_wire_GET_JOINT_LIMITS: {
        auto value = runtime_->joint_limits();
        if (!value) return value.status();
        std::copy(value.value().lower_angles.begin(),
                  value.value().lower_angles.end(), reply.values);
        std::copy(value.value().upper_angles.begin(),
                  value.value().upper_angles.end(), reply.values + kRobotDof);
        std::copy(value.value().velocity_limits.begin(),
                  value.value().velocity_limits.end(),
                  reply.values + 2 * kRobotDof);
        return Status::success();
      }
      case articore_wire_GET_SPEED_PERCENT: {
        auto value = runtime_->speed_percent();
        if (!value) return value.status();
        reply.values[0] = value.value();
        return Status::success();
      }
      case articore_wire_GET_MAX_SPEED: {
        auto value = runtime_->max_speed();
        if (!value) return value.status();
        reply.values[0] = value.value();
        return Status::success();
      }
      case articore_wire_GET_MAX_ACCELERATION: {
        auto value = runtime_->max_acceleration();
        if (!value) return value.status();
        reply.values[0] = value.value();
        return Status::success();
      }
      case articore_wire_HAS_GRIPPERS: {
        auto value = runtime_->has_grippers();
        if (!value) return value.status();
        reply.values[0] = value.value() ? 1.0f : 0.0f;
        return Status::success();
      }
      case articore_wire_GET_GRAVITY_STATUS: {
        auto value = runtime_->gravity_compensation_status();
        if (!value) return value.status();
        reply.values[0] = static_cast<float>(value.value().phase);
        reply.values[1] = value.value().active ? 1.0f : 0.0f;
        reply.values[2] = value.value().transition_progress;
        std::copy(value.value().feedforward_torques.begin(),
                  value.value().feedforward_torques.end(), reply.values + 3);
        return Status::success();
      }
      case articore_wire_GET_BIMANUAL_STATUS: {
        auto value = runtime_->bimanual_follow_status();
        if (!value) return value.status();
        reply.values[0] = static_cast<float>(value.value().phase);
        reply.values[1] = value.value().active ? 1.0f : 0.0f;
        reply.values[2] = static_cast<float>(value.value().leader);
        reply.values[3] = value.value().transition_progress;
        reply.values[4] = value.value().maximum_tracking_error;
        std::copy(value.value().leader_positions.begin(),
                  value.value().leader_positions.end(), reply.values + 5);
        std::copy(value.value().follower_target_positions.begin(),
                  value.value().follower_target_positions.end(),
                  reply.values + 5 + kArmDof);
        return Status::success();
      }
      default:
        return Status::failure(RuntimeErrorCode::InvalidArgument,
                               "unsupported control operation");
    }
  }

  void publish_discovery() {
    articore_wire_Discovery sample{};
    sample.protocol_major = kProtocolMajor;
    sample.protocol_minor = kProtocolMinor;
    copy_text(sample.robot_id, config_.robot_id);
    copy_text(sample.client_id, "articore-runtime-service");
    sample.sequence_id = ++discovery_sequence_;
    copy_text(sample.service_version, ARTICORE_SERVICE_VERSION);
    sample.domain_id = config_.domain_id;
    sample.ready = runtime_ != nullptr;
    check(dds_write(discovery_writer_.get(), &sample), "write discovery");
  }

  void publish_state() {
    if (!runtime_) return;
    auto state = runtime_->state();
    if (!state) return;
    articore_wire_RobotState sample{};
    sample.protocol_major = kProtocolMajor;
    sample.protocol_minor = kProtocolMinor;
    copy_text(sample.robot_id, config_.robot_id);
    copy_text(sample.client_id, "articore-runtime-service");
    sample.sequence_id = ++state_sequence_;
    sample.source_timestamp_ns = state.value().timestamp.count();
    std::copy(state.value().positions.begin(), state.value().positions.end(),
              sample.positions);
    std::copy(state.value().velocities.begin(), state.value().velocities.end(),
              sample.velocities);
    std::copy(state.value().torques.begin(), state.value().torques.end(),
              sample.torques);
    std::copy(state.value().mos_temperatures.begin(),
              state.value().mos_temperatures.end(), sample.mos_temperatures);
    std::copy(state.value().rotor_temperatures.begin(),
              state.value().rotor_temperatures.end(), sample.rotor_temperatures);
    sample.enabled_mask = state.value().enabled_mask;
    sample.enabled_valid_mask = state.value().enabled_valid_mask;
    sample.temperature_valid_mask = state.value().temperature_valid_mask;
    sample.motion_arrived = state.value().motion_arrived;
    check(dds_write(state_writer_.get(), &sample), "write state");
    if (sample.motion_arrived && pending_motion_request_ != 0) {
      publish_motion_event(pending_motion_client_, pending_motion_request_,
                           articore_wire_MOTION_COMPLETED,
                           ProtocolError::Ok, "motion completed");
      pending_motion_request_ = 0;
      pending_motion_client_.clear();
    }
  }

  void publish_motion_event(const std::string& client_id,
                            std::uint64_t request_id,
                            articore_wire_MotionEventKind kind,
                            ProtocolError error,
                            const std::string& message) {
    articore_wire_MotionEvent sample{};
    sample.protocol_major = kProtocolMajor;
    sample.protocol_minor = kProtocolMinor;
    copy_text(sample.robot_id, config_.robot_id);
    copy_text(sample.client_id, client_id);
    sample.sequence_id = ++motion_sequence_;
    sample.request_id = request_id;
    sample.kind = kind;
    sample.error = wire_error(error);
    copy_text(sample.message, message);
    check(dds_write(motion_writer_.get(), &sample), "write motion event");
  }

  void publish_health() {
    articore_wire_Health sample{};
    sample.protocol_major = kProtocolMajor;
    sample.protocol_minor = kProtocolMinor;
    copy_text(sample.robot_id, config_.robot_id);
    copy_text(sample.client_id, "articore-runtime-service");
    sample.sequence_id = ++health_sequence_;
    if (!runtime_) {
      sample.state = static_cast<std::uint32_t>(SafetyState::Disconnected);
      sample.safe_stopped = true;
      copy_text(sample.safety_reason,
                runtime_error_.empty() ? "waiting for CAN" : runtime_error_);
    } else {
      auto health = runtime_->health();
      if (health) {
        last_health_signature_ = health_signature(health.value());
        sample.state = static_cast<std::uint32_t>(health.value().state);
        sample.safe_holding = health.value().safe_holding;
        sample.disable_confirmed = health.value().disable_confirmed;
        sample.degraded = health.value().degraded;
        sample.safe_stopped = health.value().safe_stopped;
        sample.requires_resynchronization =
            health.value().requires_resynchronization;
        sample.consecutive_send_failures =
            health.value().consecutive_send_failures;
        sample.consecutive_feedback_failures =
            health.value().consecutive_feedback_failures;
        copy_text(sample.fault_reason, health.value().fault_reason);
        copy_text(sample.safety_reason, health.value().safety_reason);
      }
    }
    check(dds_write(health_writer_.get(), &sample), "write health");
  }

  static std::string health_signature(const RuntimeHealth& health) {
    return std::to_string(static_cast<unsigned>(health.state)) + ":" +
        std::to_string(health.safe_holding) + ":" +
        std::to_string(health.disable_confirmed) + ":" +
        std::to_string(health.degraded) + ":" +
        std::to_string(health.safe_stopped) + ":" +
        std::to_string(health.requires_resynchronization) + ":" +
        std::to_string(health.consecutive_send_failures) + ":" +
        std::to_string(health.consecutive_feedback_failures) + ":" +
        health.fault_reason + ":" + health.safety_reason;
  }

  void publish_health_on_change() {
    if (!runtime_) return;
    auto health = runtime_->health();
    if (!health) return;
    if (health_signature(health.value()) != last_health_signature_) {
      publish_health();
    }
  }

  ServiceConfig config_;
  std::atomic<bool> stop_{false};
  LatestCommandMailbox mailbox_;
  LeaseManager lease_;
  std::unique_ptr<YunyiRuntime> runtime_;
  std::string runtime_error_;
  Clock::time_point next_runtime_retry_{};
  std::chrono::milliseconds retry_delay_{250};

  Entity domain_;
  Entity participant_;
  Entity discovery_topic_, request_topic_, reply_topic_, stream_topic_;
  Entity state_topic_, health_topic_, motion_topic_;
  Entity discovery_writer_, request_reader_, reply_writer_, stream_reader_;
  Entity state_writer_, health_writer_, motion_writer_, waitset_;
  std::uint64_t reply_sequence_ = 0;
  std::uint64_t discovery_sequence_ = 0;
  std::uint64_t state_sequence_ = 0;
  std::uint64_t health_sequence_ = 0;
  std::uint64_t motion_sequence_ = 0;
  std::uint64_t pending_motion_request_ = 0;
  std::string pending_motion_client_;
  std::string last_health_signature_;
};

CycloneService::CycloneService(ServiceConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
CycloneService::~CycloneService() = default;
Status CycloneService::run(const std::atomic<bool>& stop_requested) {
  try {
    return impl_->run(stop_requested);
  } catch (const std::exception& error) {
    return Status::failure(RuntimeErrorCode::TransportError, error.what());
  }
}
void CycloneService::request_stop() noexcept { impl_->request_stop(); }

}  // namespace articore::dds
