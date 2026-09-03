#include "articore/runtime.hpp"

#include <algorithm>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "articore/detail/yunyi_runtime_core.hpp"

namespace articore {
namespace {

RuntimeErrorCode classify_exception(const std::exception& error) {
  if (dynamic_cast<const std::invalid_argument*>(&error)) {
    return RuntimeErrorCode::InvalidArgument;
  }
  const std::string message = error.what();
  if (message.find("state") != std::string::npos ||
      message.find("enabled") != std::string::npos ||
      message.find("disabled") != std::string::npos) {
    return RuntimeErrorCode::WrongState;
  }
  if (message.find("busy") != std::string::npos ||
      message.find("active motion") != std::string::npos) {
    return RuntimeErrorCode::Busy;
  }
  if (message.find("timeout") != std::string::npos ||
      message.find("timed out") != std::string::npos) {
    return RuntimeErrorCode::Timeout;
  }
  if (message.find("CAN") != std::string::npos ||
      message.find("transport") != std::string::npos ||
      message.find("socket") != std::string::npos) {
    return RuntimeErrorCode::TransportError;
  }
  return RuntimeErrorCode::InternalError;
}

template <typename Function>
Status invoke(Function&& function) noexcept {
  try {
    function();
    return Status::success();
  } catch (const std::exception& error) {
    return Status::failure(classify_exception(error), error.what());
  } catch (...) {
    return Status::failure(RuntimeErrorCode::InternalError,
                           "unknown Runtime exception");
  }
}

template <typename T, typename Function>
Result<T> query(Function&& function) noexcept {
  try {
    return Result<T>(function());
  } catch (const std::exception& error) {
    return Result<T>(Status::failure(classify_exception(error), error.what()));
  } catch (...) {
    return Result<T>(Status::failure(RuntimeErrorCode::InternalError,
                                     "unknown Runtime exception"));
  }
}

std::vector<float> vector_from(const JointArray& values) {
  return {values.begin(), values.end()};
}

}  // namespace

class YunyiRuntime::Impl {
 public:
  explicit Impl(YunyiRuntimeConfig selected)
      : config(std::move(selected)),
        core(static_cast<ArticoreControlMode>(config.initial_control_mode),
               config.left_can_interface, config.right_can_interface,
               config.threads.realtime,
               config.threads.lock_memory, config.threads.control_cpu,
               config.threads.can_tx_cpu, config.threads.can_rx_cpu,
               config.threads.control_priority, config.threads.can_tx_priority,
               config.threads.can_rx_priority,
               static_cast<std::uint32_t>(config.feedback_max_age.count()),
               static_cast<std::uint32_t>(config.motor_watchdog.count()),
               static_cast<std::uint32_t>(
                   config.motor_discovery_timeout.count()),
               config.motor_discovery_retries) {}

  YunyiRuntimeConfig config;
  YunyiRuntimeCore core;
  mutable std::mutex terminal_mutex;
  bool terminally_disconnected = false;
  std::string control_lost_reason;
};

Result<std::unique_ptr<YunyiRuntime>> YunyiRuntime::create(
    YunyiRuntimeConfig config) {
  if (config.left_can_interface.empty() ||
      config.right_can_interface.empty() ||
      config.left_can_interface.size() > 15 ||
      config.right_can_interface.size() > 15) {
    return Status::failure(RuntimeErrorCode::InvalidArgument,
                           "CAN interface names must contain 1..15 bytes");
  }
  if (config.left_can_interface == config.right_can_interface) {
    return Status::failure(RuntimeErrorCode::InvalidArgument,
                           "left and right CAN interfaces must differ");
  }
  if (config.feedback_max_age <= std::chrono::milliseconds::zero() ||
      config.motor_watchdog <= std::chrono::milliseconds::zero() ||
      config.motor_discovery_timeout <= std::chrono::milliseconds::zero() ||
      config.motor_discovery_retries > 10) {
    return Status::failure(RuntimeErrorCode::InvalidArgument,
                           "Runtime timeouts must be positive and discovery "
                           "retries must be within 0..=10");
  }
  const int cpu_count = static_cast<int>(std::thread::hardware_concurrency());
  if (cpu_count > 0) {
    if (config.threads.control_cpu < 0) {
      config.threads.control_cpu = cpu_count - 1;
    }
    if (config.threads.can_tx_cpu < 0) {
      config.threads.can_tx_cpu = std::max(0, cpu_count - 2);
    }
    if (config.threads.can_rx_cpu < 0) {
      config.threads.can_rx_cpu = std::max(0, cpu_count - 3);
    }
    if (config.threads.control_cpu >= cpu_count ||
        config.threads.can_tx_cpu >= cpu_count ||
        config.threads.can_rx_cpu >= cpu_count) {
      return Status::failure(RuntimeErrorCode::InvalidArgument,
                             "configured CPU is not online");
    }
  }
  if (config.threads.realtime &&
      (config.threads.control_priority < 1 ||
       config.threads.control_priority > 99 ||
       config.threads.can_tx_priority < 1 ||
       config.threads.can_tx_priority > 99 ||
       config.threads.can_rx_priority < 1 ||
       config.threads.can_rx_priority > 99)) {
    return Status::failure(RuntimeErrorCode::InvalidArgument,
                           "SCHED_FIFO priorities must be within 1..99");
  }
  return query<std::unique_ptr<YunyiRuntime>>([&] {
    return std::unique_ptr<YunyiRuntime>(
        new YunyiRuntime(std::make_unique<Impl>(std::move(config))));
  });
}

YunyiRuntime::YunyiRuntime(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
YunyiRuntime::~YunyiRuntime() = default;

Status YunyiRuntime::connect() {
  return invoke([&] {
    std::lock_guard<std::mutex> lock(impl_->terminal_mutex);
    if (impl_->terminally_disconnected) {
      throw std::logic_error("Runtime cannot reconnect after disconnect");
    }
    impl_->core.connect();
  });
}

Status YunyiRuntime::disconnect() {
  return invoke([&] {
    std::lock_guard<std::mutex> lock(impl_->terminal_mutex);
    if (impl_->terminally_disconnected) return;
    impl_->core.disconnect();
    impl_->terminally_disconnected = true;
  });
}

Status YunyiRuntime::configure_mode(ControlMode mode) {
  return invoke([&] { impl_->core.configure_mode(
      static_cast<ArticoreControlMode>(mode)); });
}

Result<ControlMode> YunyiRuntime::control_mode() const {
  return query<ControlMode>([&] {
    return static_cast<ControlMode>(impl_->core.control_mode());
  });
}

Status YunyiRuntime::enable() { return invoke([&] { impl_->core.enable(); }); }
Status YunyiRuntime::disable() { return invoke([&] { impl_->core.disable(); }); }
Status YunyiRuntime::set_zero() { return invoke([&] { impl_->core.set_zero(); }); }
Status YunyiRuntime::clear_faults() {
  return invoke([&] { impl_->core.clear_faults(); });
}
Status YunyiRuntime::estop() { return invoke([&] { impl_->core.estop(); }); }
Status YunyiRuntime::recover() { return invoke([&] { impl_->core.recover(); }); }

Status YunyiRuntime::on_control_lost(const std::string& reason) {
  impl_->control_lost_reason = reason;
  // Run both actions even if cancellation reports an error: physical disable
  // is the final lease-loss barrier and must never be skipped.
  const auto stopped = invoke([&] { impl_->core.stop_motion(); });
  const auto disabled = invoke([&] { impl_->core.disable(); });
  if (!disabled) return disabled;
  return stopped;
}

Status YunyiRuntime::set_joint_pv(const JointArray& positions,
                                  float speed_percent) {
  return invoke([&] { impl_->core.set_joint_pv(
      vector_from(positions), speed_percent); });
}

Status YunyiRuntime::set_joint_mit(const MitCommand& command) {
  return invoke([&] {
    impl_->core.set_joint_mit(
        vector_from(command.positions), vector_from(command.velocities),
        vector_from(command.kp), vector_from(command.kd),
        vector_from(command.feedforward_torques));
  });
}

Status YunyiRuntime::set_joint_mit_fast(const JointArray& positions,
                                        float speed_percent) {
  return invoke([&] { impl_->core.set_joint_mit_fast(
      vector_from(positions), speed_percent); });
}

Status YunyiRuntime::set_speed_percent(float value) {
  return invoke([&] { impl_->core.set_speed_percent(value); });
}
Status YunyiRuntime::set_max_speed(float value) {
  return invoke([&] { impl_->core.set_max_speed(value); });
}
Status YunyiRuntime::set_max_acceleration(float value) {
  return invoke([&] { impl_->core.set_max_acceleration(value); });
}
Result<float> YunyiRuntime::speed_percent() const {
  return query<float>([&] { return impl_->core.speed_percent(); });
}
Result<float> YunyiRuntime::max_speed() const {
  return query<float>([&] { return impl_->core.max_speed(); });
}
Result<float> YunyiRuntime::max_acceleration() const {
  return query<float>([&] { return impl_->core.max_acceleration(); });
}
Result<bool> YunyiRuntime::has_grippers() const {
  return query<bool>([&] { return impl_->core.has_grippers(); });
}

Result<HardwareTopology> YunyiRuntime::hardware_topology() const {
  return query<HardwareTopology>([&] {
    HardwareTopology topology;
    const auto present = impl_->core.gripper_presence();
    for (std::size_t side = 0; side < present.size(); ++side) {
      topology.end_effectors[side] = present[side]
          ? EndEffectorType::DamiaoGripper : EndEffectorType::None;
    }
    topology.revision = 1U + static_cast<std::uint32_t>(present[0]) +
        2U * static_cast<std::uint32_t>(present[1]);
    return topology;
  });
}

Result<JointArray> YunyiRuntime::solve_ik(const Pose& left,
                                          const Pose& right) const {
  return query<JointArray>([&] { return impl_->core.solve_ik(left, right); });
}
Status YunyiRuntime::move_pose(RobotSide side, const Pose& target) {
  return invoke([&] { impl_->core.move_pose(
      static_cast<std::uint32_t>(side), target); });
}
Status YunyiRuntime::move_linear(RobotSide side, const Pose& target) {
  return invoke([&] { impl_->core.move_linear(
      static_cast<std::uint32_t>(side), target); });
}
Status YunyiRuntime::move_circular(RobotSide side, const Pose& start,
                                   const Pose& via, const Pose& end) {
  return invoke([&] { impl_->core.move_circular(
      static_cast<std::uint32_t>(side), start, via, end); });
}
Status YunyiRuntime::stop_motion() {
  return invoke([&] { impl_->core.stop_motion(); });
}

Status YunyiRuntime::set_grippers(float left, float right, int strength,
                                  GripperMode mode) {
  return invoke([&] { impl_->core.set_grippers(
      left, right, strength, static_cast<ArticoreGripperMode>(mode)); });
}
Status YunyiRuntime::set_tcp_offset(RobotSide side, const Pose& offset) {
  return invoke([&] { impl_->core.set_tcp_offset(
      static_cast<std::uint32_t>(side), offset); });
}
Status YunyiRuntime::reset_tcp_offset(RobotSide side) {
  return invoke([&] { impl_->core.reset_tcp_offset(
      static_cast<std::uint32_t>(side)); });
}
Result<Pose> YunyiRuntime::pose(RobotSide side) const {
  return query<Pose>([&] {
    const auto source = impl_->core.pose(static_cast<std::uint32_t>(side));
    Pose output{};
    std::copy_n(source.values, output.size(), output.begin());
    return output;
  });
}
Result<Pose> YunyiRuntime::tcp_offset(RobotSide side) const {
  return query<Pose>([&] {
    const auto source = impl_->core.tcp_offset(
        static_cast<std::uint32_t>(side));
    Pose output{};
    std::copy_n(source.values, output.size(), output.begin());
    return output;
  });
}
Result<JointLimits> YunyiRuntime::joint_limits() const {
  return query<JointLimits>([&] {
    const auto source = impl_->core.joint_angle_vel_limits();
    JointLimits output;
    std::copy_n(source.lower_angles, kRobotDof, output.lower_angles.begin());
    std::copy_n(source.upper_angles, kRobotDof, output.upper_angles.begin());
    std::copy_n(source.velocity_limits, kRobotDof,
                output.velocity_limits.begin());
    return output;
  });
}
Status YunyiRuntime::start_gravity_compensation(
    std::chrono::milliseconds transition) {
  return invoke([&] { impl_->core.start_gravity_compensation(
      static_cast<std::uint32_t>(transition.count())); });
}
Status YunyiRuntime::stop_gravity_compensation() {
  return invoke([&] { impl_->core.stop_gravity_compensation(); });
}
Status YunyiRuntime::start_bimanual_follow(RobotSide leader) {
  return invoke([&] { impl_->core.start_bimanual_follow(
      static_cast<std::uint32_t>(leader)); });
}
Status YunyiRuntime::stop_bimanual_follow() {
  return invoke([&] { impl_->core.stop_bimanual_follow(); });
}
Result<GravityCompensationStatus>
YunyiRuntime::gravity_compensation_status() const {
  return query<GravityCompensationStatus>([&] {
    const auto source = impl_->core.gravity_compensation_status();
    GravityCompensationStatus output;
    output.phase = source.phase;
    output.active = source.active != 0;
    output.transition_progress = source.transition_progress;
    output.control_cycles = source.control_cycles;
    std::copy_n(source.gravity_feedforward_torque, kRobotDof,
                output.feedforward_torques.begin());
    return output;
  });
}
Result<BimanualFollowStatus> YunyiRuntime::bimanual_follow_status() const {
  return query<BimanualFollowStatus>([&] {
    const auto source = impl_->core.bimanual_follow_status();
    BimanualFollowStatus output;
    output.phase = source.phase;
    output.active = source.active != 0;
    output.leader = static_cast<RobotSide>(source.leader_side);
    output.transition_progress = source.transition_progress;
    output.control_cycles = source.control_cycles;
    std::copy_n(source.leader_positions, kArmDof,
                output.leader_positions.begin());
    std::copy_n(source.follower_target_positions, kArmDof,
                output.follower_target_positions.begin());
    output.maximum_tracking_error = source.max_tracking_error;
    output.error = source.error;
    return output;
  });
}

Result<RuntimeState> YunyiRuntime::state() const {
  return query<RuntimeState>([&] {
    const auto source = impl_->core.state();
    RuntimeState output;
    for (std::size_t i = 0; i < kArmDof; ++i) {
      output.positions[i] = source.left.positions[i];
      output.positions[kArmDof + i] = source.right.positions[i];
      output.velocities[i] = source.left.velocities[i];
      output.velocities[kArmDof + i] = source.right.velocities[i];
      output.torques[i] = source.left.torques[i];
      output.torques[kArmDof + i] = source.right.torques[i];
      output.mos_temperatures[i] = source.left.mos_temperatures[i];
      output.mos_temperatures[kArmDof + i] = source.right.mos_temperatures[i];
      output.rotor_temperatures[i] = source.left.rotor_temperatures[i];
      output.rotor_temperatures[kArmDof + i] =
          source.right.rotor_temperatures[i];
    }
    output.enabled_mask = source.left.enabled_mask |
                          (source.right.enabled_mask << kArmDof);
    output.enabled_valid_mask = source.left.enabled_valid_mask |
      (source.right.enabled_valid_mask << kArmDof);
    output.temperature_valid_mask = source.left.temperature_valid_mask |
      (source.right.temperature_valid_mask << kArmDof);
    output.has_grippers = source.has_grippers != 0;
    output.gripper_openings = {source.left_gripper_opening,
                               source.right_gripper_opening};
    output.gripper_available = {source.left_gripper_available != 0,
                                source.right_gripper_available != 0};
    output.gripper_feedback_valid = {
        source.left_gripper_feedback_valid != 0,
        source.right_gripper_feedback_valid != 0};
    output.motion_arrived = source.motion_arrived != 0;
    output.timestamp = std::chrono::nanoseconds(source.timestamp_ns);
    output.sequence = source.sequence;
    return output;
  });
}

Result<RuntimeHealth> YunyiRuntime::health() const {
  return query<RuntimeHealth>([&] {
    const auto source = impl_->core.health();
    RuntimeHealth output;
    output.state = static_cast<SafetyState>(source.state);
    output.safe_holding = source.safe_holding != 0;
    output.disable_confirmed = source.disable_confirmed != 0;
    output.degraded = source.degraded != 0;
    output.safe_stopped = source.safe_stopped != 0;
    output.requires_resynchronization = source.requires_resynchronization != 0;
    output.consecutive_send_failures = source.consecutive_send_failures;
    output.consecutive_feedback_failures = source.consecutive_feedback_failures;
    output.fault_reason = source.fault_reason;
    output.safety_reason = source.safety_reason;
    output.last_operation_error = source.last_operation_error;
    output.motor_faults.reserve(source.motor_fault_count);
    for (std::uint32_t index = 0; index < source.motor_fault_count; ++index) {
      output.motor_faults.emplace_back(source.motor_faults[index]);
    }
    output.operation_failed_motors.reserve(
        source.operation_failed_motor_count);
    for (std::uint32_t index = 0;
         index < source.operation_failed_motor_count; ++index) {
      output.operation_failed_motors.emplace_back(
          source.operation_failed_motors[index]);
    }
    output.metrics.ticks = source.control_ticks;
    output.metrics.overruns = source.control_overruns;
    output.metrics.maximum_period =
        std::chrono::nanoseconds(source.maximum_control_period_ns);
    output.metrics.maximum_send_time =
        std::chrono::nanoseconds(source.maximum_send_time_ns);
    return output;
  });
}

const YunyiRuntimeConfig& YunyiRuntime::config() const noexcept {
  return impl_->config;
}

}  // namespace articore
