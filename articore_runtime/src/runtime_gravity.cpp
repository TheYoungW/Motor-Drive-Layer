#include "runtime.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <stdexcept>

namespace articore {
namespace {

constexpr std::size_t kArmDof = 7;

std::size_t model_joint_index(const char* name) {
  const std::string value(name ? name : "");
  const auto marker = value.rfind("joint");
  if (marker == std::string::npos) return kArmDof;
  const auto number = value.substr(marker + 5);
  if (number.size() != 1 || number[0] < '1' || number[0] > '7') {
    return kArmDof;
  }
  return static_cast<std::size_t>(number[0] - '1');
}

float transition_progress(std::chrono::steady_clock::time_point now,
                          std::chrono::steady_clock::time_point started,
                          std::chrono::milliseconds duration) {
  if (duration.count() <= 0) return 1.0f;
  return std::clamp(
      std::chrono::duration<float>(now - started).count() /
          std::chrono::duration<float>(duration).count(),
      0.0f, 1.0f);
}

}  // namespace

void SafetyRuntime::configure_gravity_products(
    const ArticoreGravityProductBinding* bindings, uint32_t count) {
  std::array<uint32_t, 2> arm_counts{};
  for (const auto& motor : motors_) {
    if (!motor.descriptor.is_gripper) ++arm_counts[motor.descriptor.side];
  }
  const uint32_t expected =
      (arm_counts[0] != 0 ? 1U : 0U) + (arm_counts[1] != 0 ? 1U : 0U);
  if (!bindings || count != expected) {
    throw std::invalid_argument(
        "gravity product bindings must cover every active arm side exactly once");
  }

  std::vector<GravityArm> pending;
  pending.reserve(count);
  std::set<uint32_t> unique_sides;
  for (uint32_t index = 0; index < count; ++index) {
    const auto& binding = bindings[index];
    const auto end = std::find(std::begin(binding.product_id),
                               std::end(binding.product_id), '\0');
    if (binding.struct_size < sizeof(binding) || binding.runtime_side > 1 ||
        binding.robot_side > 1 || end == std::end(binding.product_id) ||
        end == std::begin(binding.product_id) ||
        !unique_sides.insert(binding.runtime_side).second) {
      throw std::invalid_argument("invalid gravity product binding");
    }
    if (arm_counts[binding.runtime_side] != kArmDof) {
      throw std::invalid_argument(
          "gravity compensation requires exactly seven arm joints per bound side");
    }

    GravityArm arm;
    arm.runtime_side = binding.runtime_side;
    arm.robot_side = binding.robot_side;
    arm.product_id.assign(binding.product_id, end);
    arm.model = std::make_unique<RobotModel>(arm.product_id, arm.robot_side);
    for (const auto& motor : motors_) {
      if (!motor.descriptor.is_gripper &&
          motor.descriptor.side == binding.runtime_side) {
        const auto joint = model_joint_index(motor.descriptor.name);
        if (joint >= kArmDof || arm.joints[joint]) {
          throw std::invalid_argument(
              "gravity arm roles must end in unique joint1..joint7 names");
        }
        arm.joints[joint] = motor.descriptor.motor;
      }
    }
    pending.push_back(std::move(arm));
  }
  std::sort(pending.begin(), pending.end(),
            [](const GravityArm& lhs, const GravityArm& rhs) {
              return lhs.runtime_side < rhs.runtime_side;
            });

  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (state_ != ARTICORE_DISCONNECTED || hardware_transition_) {
    throw std::runtime_error(
        "gravity product bindings are fixed after connect");
  }
  gravity_arms_ = std::move(pending);
}

void SafetyRuntime::start_gravity_compensation(
    const ArticoreGravityCompensationConfig* config) {
  uint32_t transition_ms = 500;
  if (config) {
    if (config->struct_size < sizeof(*config)) {
      throw std::invalid_argument(
          "gravity compensation config struct_size is too small");
    }
    if (config->transition_ms != 0) transition_ms = config->transition_ms;
  }
  if (transition_ms > 60'000) {
    throw std::invalid_argument(
        "gravity compensation transition_ms exceeds 60000");
  }

  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (hardware_transition_ || fault_latched_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING)) {
      throw std::runtime_error(
          "gravity compensation requires an enabled Runtime");
    }
    if (mode_ != ARTICORE_MODE_MIT) {
      throw std::runtime_error(
          "gravity compensation requires MIT control mode");
    }
    if (gravity_control_.phase != ARTICORE_GRAVITY_INACTIVE) {
      throw std::runtime_error("gravity compensation is already active");
    }
  }
  if (gravity_arms_.empty()) {
    throw std::runtime_error("gravity product models are not configured");
  }

  std::vector<float> positions;
  positions.reserve(gravity_arms_.size() * kArmDof);
  for (const auto& arm : gravity_arms_) {
    for (void* joint : arm.joints) {
      if (joint_configs_.find(joint) == joint_configs_.end()) {
        throw std::runtime_error(
            "gravity compensation requires joint torque-limit configuration");
      }
      ArticoreMotorState state{};
      if (api_.motor_get_state(joint, &state) != 0 || !state.has_value ||
          state.status_code != 1 || !finite(state.pos) || !finite(state.vel)) {
        throw std::runtime_error(
            motor_roles_.at(joint) +
            ": gravity compensation requires finite enabled feedback");
      }
      positions.push_back(state.pos);
    }
  }

  const auto now = Clock::now();
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  clear_pending_arm_mailbox();
  arm_mailbox_ = ArmMailbox{};
  gravity_control_.phase = ARTICORE_GRAVITY_ENTERING;
  gravity_control_.transition_started = now;
  gravity_control_.transition_duration =
      std::chrono::milliseconds(transition_ms);
  gravity_control_.transition_start_gravity_scale = 0.0f;
  gravity_control_.hold_positions = std::move(positions);
  gravity_control_.control_cycles = 0;
  gravity_control_.status = {};
  gravity_control_.status.struct_size = sizeof(gravity_control_.status);
  gravity_control_.status.phase = ARTICORE_GRAVITY_ENTERING;
  gravity_control_.status.active = 1;
  gravity_control_.status.joint_count = static_cast<uint32_t>(
      gravity_arms_.size() * kArmDof);
  std::size_t output = 0;
  for (const auto& arm : gravity_arms_) {
    for (void* joint : arm.joints) {
      gravity_control_.status.joints[output++] = joint;
    }
  }
  has_successful_command_ = true;
  last_successful_command_ = now;
  state_ = ARTICORE_RUNNING;
  next_control_tick_ = now;
  wakeup_.notify_all();
}

void SafetyRuntime::stop_gravity_compensation() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  float current_gravity_scale = 1.0f;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (gravity_control_.phase == ARTICORE_GRAVITY_INACTIVE) return;
    if (hardware_transition_ || fault_latched_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING &&
         state_ != ARTICORE_DEGRADED)) {
      throw std::runtime_error(
          "gravity compensation cannot stop in the current Runtime state");
    }
    if (gravity_control_.phase == ARTICORE_GRAVITY_ENTERING) {
      const float progress = transition_progress(
          Clock::now(), gravity_control_.transition_started,
          gravity_control_.transition_duration);
      current_gravity_scale =
          gravity_control_.transition_start_gravity_scale +
          (1.0f - gravity_control_.transition_start_gravity_scale) * progress;
    }
  }

  std::vector<float> positions;
  positions.reserve(gravity_arms_.size() * kArmDof);
  for (const auto& arm : gravity_arms_) {
    for (void* joint : arm.joints) {
      ArticoreMotorState state{};
      if (api_.motor_get_state(joint, &state) != 0 || !state.has_value ||
          state.status_code != 1 || !finite(state.pos)) {
        throw std::runtime_error(
            motor_roles_.at(joint) +
            ": current position is unavailable while leaving gravity compensation");
      }
      positions.push_back(state.pos);
    }
  }

  const auto now = Clock::now();
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  gravity_control_.phase = ARTICORE_GRAVITY_EXITING;
  gravity_control_.transition_started = now;
  gravity_control_.transition_start_gravity_scale = current_gravity_scale;
  gravity_control_.hold_positions = std::move(positions);
  gravity_control_.status.phase = ARTICORE_GRAVITY_EXITING;
  gravity_control_.status.active = 1;
  gravity_control_.status.transition_progress = 0.0f;
  next_control_tick_ = now;
  wakeup_.notify_all();
}

ArticoreGravityCompensationStatus
SafetyRuntime::gravity_compensation_status() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return gravity_control_.status;
}

bool SafetyRuntime::run_gravity_control_cycle(Clock::time_point now,
                                              bool include_grippers,
                                              std::string& error) {
  ArticoreGravityCompensationPhase phase;
  Clock::time_point transition_started;
  std::chrono::milliseconds transition_duration;
  float transition_start_gravity_scale = 0.0f;
  std::vector<float> hold_positions;
  bool degraded = false;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    degraded = state_ == ARTICORE_DEGRADED;
    phase = gravity_control_.phase;
    transition_started = gravity_control_.transition_started;
    transition_duration = gravity_control_.transition_duration;
    transition_start_gravity_scale =
        gravity_control_.transition_start_gravity_scale;
    hold_positions = gravity_control_.hold_positions;
  }
  if (phase == ARTICORE_GRAVITY_INACTIVE) return true;

  const float progress = phase == ARTICORE_GRAVITY_ACTIVE
      ? 1.0f
      : transition_progress(now, transition_started, transition_duration);
  const float gravity_scale = phase == ARTICORE_GRAVITY_EXITING
      ? transition_start_gravity_scale * (1.0f - progress)
      : (phase == ARTICORE_GRAVITY_ACTIVE
             ? 1.0f
             : transition_start_gravity_scale +
                   (1.0f - transition_start_gravity_scale) * progress);
  const float hold_scale = 1.0f - gravity_scale;

  std::vector<ArticoreMitCommand> requested;
  requested.reserve(gravity_arms_.size() * kArmDof);
  std::size_t flat_index = 0;
  for (auto& arm : gravity_arms_) {
    std::array<double, kArmDof> q{};
    std::array<double, kArmDof> gravity{};
    std::array<ArticoreMotorState, kArmDof> feedback{};
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      if (api_.motor_get_state(arm.joints[joint], &feedback[joint]) != 0 ||
          !feedback[joint].has_value || feedback[joint].status_code != 1 ||
          !finite(feedback[joint].pos) || !finite(feedback[joint].vel)) {
        error = motor_roles_.at(arm.joints[joint]) +
            ": gravity compensation requires finite enabled feedback";
        return false;
      }
      q[joint] = feedback[joint].pos;
    }
    arm.model->gravity_realtime(q, gravity);
    for (std::size_t joint = 0; joint < kArmDof; ++joint, ++flat_index) {
      const auto& gains = joint_configs_.at(arm.joints[joint]);
      const float torque = static_cast<float>(gravity[joint]) * gravity_scale;
      if (!finite(torque)) {
        error = motor_roles_.at(arm.joints[joint]) +
            ": gravity feedforward is not finite";
        return false;
      }
      const float target = hold_scale > 0.0f
          ? hold_positions.at(flat_index) : feedback[joint].pos;
      requested.push_back(ArticoreMitCommand{
          arm.joints[joint], target, 0.0f,
          gains.mit_kp * hold_scale, gains.mit_kd * hold_scale, torque});
    }
  }

  ArticoreMitTorqueLimitStats cycle_stats{};
  if (!prepare_mit_torque_limited_commands(
          requested, mit_torque_limited_commands_, cycle_stats, error,
          degraded ? 0.25f : 1.0f)) {
    return false;
  }

  std::vector<ArticoreMitCommand> gripper_commands;
  std::vector<ArticoreMitCommand> combined;
  const ArticoreMitCommand* send_data = mit_torque_limited_commands_.data();
  uint32_t send_count =
      static_cast<uint32_t>(mit_torque_limited_commands_.size());
  if (include_grippers) {
    if (!prepare_gripper_commands_locked(now, gripper_commands, error)) {
      return false;
    }
    if (!gripper_commands.empty()) {
      combined.reserve(gripper_commands.size() + send_count);
      combined.insert(combined.end(), gripper_commands.begin(),
                      gripper_commands.end());
      combined.insert(combined.end(), mit_torque_limited_commands_.begin(),
                      mit_torque_limited_commands_.end());
      send_data = combined.data();
      send_count = static_cast<uint32_t>(combined.size());
    }
  }

  if (api_.group_send_mit(controller_group_, send_data, send_count) != 0) {
    error = motor_error("gravity compensation MIT send failed");
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    ++consecutive_send_failures_;
    return false;
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    consecutive_send_failures_ = 0;
    last_sent_mit_ = mit_torque_limited_commands_;
    last_sent_pv_.clear();
    const auto activation_count =
        mit_torque_limit_stats_.torque_limit_activation_count;
    mit_torque_limit_stats_ = cycle_stats;
    mit_torque_limit_stats_.torque_limit_activation_count =
        activation_count + (cycle_stats.torque_limited_joint_mask ? 1 : 0);
    ++gravity_control_.control_cycles;
    gravity_control_.status.control_cycles = gravity_control_.control_cycles;
    gravity_control_.status.transition_progress = progress;
    for (std::size_t index = 0;
         index < mit_torque_limited_commands_.size(); ++index) {
      gravity_control_.status.gravity_feedforward_torque[index] =
          mit_torque_limited_commands_[index].feedforward_torque;
    }

    if (phase == ARTICORE_GRAVITY_ENTERING && progress >= 1.0f) {
      gravity_control_.phase = ARTICORE_GRAVITY_ACTIVE;
      gravity_control_.status.phase = ARTICORE_GRAVITY_ACTIVE;
      gravity_control_.status.transition_progress = 1.0f;
    } else if (phase == ARTICORE_GRAVITY_EXITING && progress >= 1.0f) {
      ArmMailbox hold;
      hold.valid = true;
      hold.user_command = false;
      hold.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
      hold.mit = requested;
      arm_mailbox_ = std::move(hold);
      gravity_control_.phase = ARTICORE_GRAVITY_INACTIVE;
      gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
      gravity_control_.status.active = 0;
      gravity_control_.status.transition_progress = 0.0f;
      std::fill(
          std::begin(gravity_control_.status.gravity_feedforward_torque),
          std::end(gravity_control_.status.gravity_feedforward_torque), 0.0f);
    }
  }
  commit_gripper_commands_sent(gripper_commands, now);
  return true;
}

}  // namespace articore
