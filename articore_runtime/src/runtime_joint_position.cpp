#include "articore/detail/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace articore {
namespace {

const char* mode_name(ArticoreControlMode mode) {
  return mode == ARTICORE_MODE_MIT ? "MIT" : "PV";
}

template <typename Target>
std::vector<std::pair<void*, float>> collect_targets(
    const Target* targets, uint32_t count, const char* mode) {
  if (!targets || count == 0) {
    throw std::invalid_argument(
        std::string("ordinary ") + mode + " position target is empty");
  }

  std::vector<std::pair<void*, float>> collected;
  collected.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    if (targets[i].struct_size != sizeof(Target) || !targets[i].motor ||
        !std::isfinite(targets[i].target_position)) {
      throw std::invalid_argument(
          std::string("ordinary ") + mode +
          " position command contains invalid values");
    }
    collected.emplace_back(targets[i].motor, targets[i].target_position);
  }
  return collected;
}

}  // namespace

void SafetyRuntime::set_joint_mit(
    const ArticoreJointMitTarget* targets, uint32_t count,
    float max_reference_velocity) {
  install_joint_position(
      ARTICORE_MODE_MIT, collect_targets(targets, count, "MIT"),
      max_reference_velocity);
}

void SafetyRuntime::set_joint_mit_direct(
    const ArticoreJointMitTarget* targets, uint32_t count) {
  install_joint_position(
      ARTICORE_MODE_MIT, collect_targets(targets, count, "MIT"),
      0.0f, 0.0f, 0.0f, nullptr, 0, nullptr, nullptr, true);
}

void SafetyRuntime::set_joint_mit_planned(
    const ArticoreJointMitTarget* targets, uint32_t count,
    float max_reference_velocity,
    CommandTransaction& transaction, uint64_t planning_token) {
  if (planning_token == 0) {
    throw std::logic_error("planned MIT command requires a planning token");
  }
  install_joint_position(
      ARTICORE_MODE_MIT, collect_targets(targets, count, "MIT"),
      max_reference_velocity, 0.0f, 0.0f, &transaction, planning_token);
}

void SafetyRuntime::set_joint_mit_direct_planned(
    const ArticoreJointMitTarget* targets, uint32_t count,
    CommandTransaction& transaction, uint64_t planning_token) {
  if (planning_token == 0) {
    throw std::logic_error("planned direct MIT command requires a planning token");
  }
  install_joint_position(
      ARTICORE_MODE_MIT, collect_targets(targets, count, "MIT"),
      0.0f, 0.0f, 0.0f, &transaction, planning_token,
      nullptr, nullptr, true);
}

void SafetyRuntime::set_joint_pv(
    const ArticoreJointPvTarget* targets, uint32_t count,
    float max_reference_velocity) {
  set_joint_pv(targets, count, max_reference_velocity,
               kNativeOrdinaryPvDefaultAcceleration,
               max_reference_velocity);
}

void SafetyRuntime::set_joint_pv(
    const ArticoreJointPvTarget* targets, uint32_t count,
    float max_reference_velocity, float pv_velocity_limit) {
  set_joint_pv(targets, count, max_reference_velocity,
               kNativeOrdinaryPvDefaultAcceleration, pv_velocity_limit);
}

void SafetyRuntime::set_joint_pv(
    const ArticoreJointPvTarget* targets, uint32_t count,
    float max_reference_velocity, float max_reference_acceleration,
    float pv_velocity_limit) {
  install_joint_position(
      ARTICORE_MODE_PV, collect_targets(targets, count, "PV"),
      max_reference_velocity, max_reference_acceleration, pv_velocity_limit,
      nullptr, 0);
}

void SafetyRuntime::set_joint_pv_planned(
    const ArticoreJointPvTarget* targets, uint32_t count,
    float max_reference_velocity, float max_reference_acceleration,
    float pv_velocity_limit,
    CommandTransaction& transaction, uint64_t planning_token) {
  if (planning_token == 0) {
    throw std::logic_error("planned PV command requires a planning token");
  }
  install_joint_position(
      ARTICORE_MODE_PV, collect_targets(targets, count, "PV"),
      max_reference_velocity, max_reference_acceleration, pv_velocity_limit,
      &transaction, planning_token);
}

void SafetyRuntime::set_joint_pv_profile(
    const ArticoreJointPvTarget* targets, uint32_t count,
    const std::vector<float>& maximum_velocities,
    const std::vector<float>& maximum_accelerations) {
  const auto collected = collect_targets(targets, count, "PV");
  if (maximum_velocities.size() != collected.size() ||
      maximum_accelerations.size() != collected.size()) {
    throw std::invalid_argument(
        "ordinary PV per-joint limits must match the target count");
  }
  const float shared_velocity = *std::max_element(
      maximum_velocities.begin(), maximum_velocities.end());
  const float shared_acceleration = *std::max_element(
      maximum_accelerations.begin(), maximum_accelerations.end());
  install_joint_position(
      ARTICORE_MODE_PV, collected, shared_velocity, shared_acceleration,
      shared_velocity, nullptr, 0,
      &maximum_velocities, &maximum_accelerations);
}

void SafetyRuntime::set_joint_pv_profile_planned(
    const ArticoreJointPvTarget* targets, uint32_t count,
    const std::vector<float>& maximum_velocities,
    const std::vector<float>& maximum_accelerations,
    CommandTransaction& transaction, uint64_t planning_token) {
  if (planning_token == 0) {
    throw std::logic_error(
        "planned PV profile command requires a planning token");
  }
  const auto collected = collect_targets(targets, count, "PV");
  if (maximum_velocities.size() != collected.size() ||
      maximum_accelerations.size() != collected.size()) {
    throw std::invalid_argument(
        "ordinary PV per-joint limits must match the target count");
  }
  const float shared_velocity = *std::max_element(
      maximum_velocities.begin(), maximum_velocities.end());
  const float shared_acceleration = *std::max_element(
      maximum_accelerations.begin(), maximum_accelerations.end());
  install_joint_position(
      ARTICORE_MODE_PV, collected, shared_velocity, shared_acceleration,
      shared_velocity, &transaction, planning_token,
      &maximum_velocities, &maximum_accelerations);
}

float SafetyRuntime::ordinary_velocity_from_percent(
    ArticoreControlMode mode, float speed_percent) const {
  if (!finite(speed_percent) || speed_percent < 0.0f ||
      speed_percent > 100.0f) {
    throw std::invalid_argument(
        "ordinary speed must be finite and within 0..100");
  }
  if (joint_configs_.empty()) {
    throw std::runtime_error(
        "ordinary speed scaling requires joint velocity configuration");
  }
  float maximum = std::numeric_limits<float>::infinity();
  for (const auto& [motor, limits] : joint_configs_) {
    (void)motor;
    maximum = std::min(maximum, limits.velocity_limit);
  }
  if (mode == ARTICORE_MODE_MIT) {
    constexpr float kMitMaximumRadiansPerSecond = 3.4906585f;
    maximum = std::min(maximum, kMitMaximumRadiansPerSecond);
  }
  return maximum * speed_percent / 100.0f;
}

void SafetyRuntime::set_joint_mit_speed(
    const ArticoreJointMitTarget* targets, uint32_t count,
    float speed_percent) {
  set_joint_mit(targets, count,
                ordinary_velocity_from_percent(
                    ARTICORE_MODE_MIT, speed_percent));
}

void SafetyRuntime::set_joint_pv_speed(
    const ArticoreJointPvTarget* targets, uint32_t count,
    float speed_percent) {
  set_joint_pv(targets, count,
               ordinary_velocity_from_percent(
                   ARTICORE_MODE_PV, speed_percent));
}

void SafetyRuntime::update_joint_pv_motion_limits(
    float max_reference_velocity, float max_reference_acceleration,
    float pv_velocity_limit) {
  if (!finite(max_reference_velocity) || max_reference_velocity < 0.0f) {
    throw std::invalid_argument(
        "max_reference_velocity must be finite and non-negative");
  }
  if (!finite(pv_velocity_limit) || pv_velocity_limit < 0.0f) {
    throw std::invalid_argument(
        "pv_velocity_limit must be finite and non-negative");
  }
  if (!finite(max_reference_acceleration) ||
      max_reference_acceleration <= 0.0f) {
    throw std::invalid_argument(
        "max_reference_acceleration must be finite and positive");
  }

  std::lock_guard<std::mutex> command_lock(command_mutex_);
  if (!arm_mailbox_.valid || !arm_mailbox_.joint_position) return;
  if (arm_mailbox_.pv.empty()) {
    throw std::runtime_error(
        "PV motion-limit update requires an active ordinary PV command");
  }
  for (const auto& command : arm_mailbox_.pv) {
    const auto& limits = joint_config(command.motor);
    if (max_reference_velocity > limits.velocity_limit ||
        pv_velocity_limit > limits.velocity_limit) {
      throw std::invalid_argument(
          "ordinary PV velocity exceeds joint safety limit");
    }
  }

  arm_mailbox_.max_reference_velocity = max_reference_velocity;
  arm_mailbox_.max_reference_acceleration = max_reference_acceleration;
  arm_mailbox_.pv_velocity_limit = pv_velocity_limit;
  for (auto& command : arm_mailbox_.pv) {
    command.velocity_limit = std::max(
        config_.safe_pv_velocity_limit, pv_velocity_limit);
  }
  wakeup_.notify_all();
}

void SafetyRuntime::update_joint_pv_profile_limits(
    const std::vector<float>& maximum_velocities,
    const std::vector<float>& maximum_accelerations) {
  if (maximum_velocities.size() != maximum_accelerations.size() ||
      maximum_velocities.empty()) {
    throw std::invalid_argument(
        "ordinary PV per-joint limits must have the same non-zero size");
  }

  std::lock_guard<std::mutex> command_lock(command_mutex_);
  if (!arm_mailbox_.valid || !arm_mailbox_.user_command ||
      !arm_mailbox_.joint_position) {
    return;
  }
  // A complete Linear/Circular trajectory also owns a user-command mailbox,
  // but it has snapshotted motion limits and must not be retimed here. Keep the
  // new shared setting for later plans and update only active ordinary PV.
  if (arm_mailbox_.pv.empty() || !arm_mailbox_.pv_per_joint_profile) {
    return;
  }
  if (maximum_velocities.size() != arm_mailbox_.pv.size()) {
    throw std::runtime_error(
        "PV profile-limit update requires an active per-joint ordinary PV command");
  }

  for (std::size_t index = 0; index < arm_mailbox_.pv.size(); ++index) {
    const float velocity = maximum_velocities[index];
    const float acceleration = maximum_accelerations[index];
    const auto& limits = joint_config(arm_mailbox_.pv[index].motor);
    if (!finite(velocity) || velocity <= 0.0f ||
        velocity > limits.velocity_limit || !finite(acceleration) ||
        acceleration <= 0.0f) {
      throw std::invalid_argument(
          motor_roles_.at(arm_mailbox_.pv[index].motor) +
          ": invalid ordinary PV per-joint motion limit");
    }
  }

  arm_mailbox_.pv_reference_velocity_limits = maximum_velocities;
  arm_mailbox_.pv_reference_acceleration_limits = maximum_accelerations;
  arm_mailbox_.max_reference_velocity = *std::max_element(
      maximum_velocities.begin(), maximum_velocities.end());
  arm_mailbox_.max_reference_acceleration = *std::max_element(
      maximum_accelerations.begin(), maximum_accelerations.end());
  wakeup_.notify_all();
}

void SafetyRuntime::install_joint_position(
    ArticoreControlMode requested_mode,
    const std::vector<std::pair<void*, float>>& targets,
    float max_reference_velocity, float max_reference_acceleration,
    float pv_velocity_limit,
    CommandTransaction* transaction, uint64_t planning_token,
    const std::vector<float>* pv_reference_velocity_limits,
    const std::vector<float>* pv_reference_acceleration_limits,
    bool mit_direct) {
  const char* const label = mode_name(requested_mode);
  if (!finite(max_reference_velocity) || max_reference_velocity < 0.0f) {
    throw std::invalid_argument(
        "max_reference_velocity must be finite and non-negative");
  }
  if (requested_mode == ARTICORE_MODE_PV &&
      (!finite(pv_velocity_limit) || pv_velocity_limit < 0.0f)) {
    throw std::invalid_argument(
        "pv_velocity_limit must be finite and non-negative");
  }
  if (requested_mode != ARTICORE_MODE_MIT && mit_direct) {
    throw std::invalid_argument("direct MIT behavior requires MIT mode");
  }
  if (requested_mode == ARTICORE_MODE_PV &&
      (!finite(max_reference_acceleration) ||
       max_reference_acceleration <= 0.0f)) {
    throw std::invalid_argument(
        "max_reference_acceleration must be finite and positive");
  }
  const bool per_joint_profile =
      pv_reference_velocity_limits != nullptr ||
      pv_reference_acceleration_limits != nullptr;
  if (per_joint_profile &&
      (requested_mode != ARTICORE_MODE_PV ||
       !pv_reference_velocity_limits ||
       !pv_reference_acceleration_limits ||
       pv_reference_velocity_limits->size() != targets.size() ||
       pv_reference_acceleration_limits->size() != targets.size())) {
    throw std::invalid_argument(
        "ordinary PV per-joint limits must match the complete target layout");
  }

  const auto expected = static_cast<std::size_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      }));
  if (targets.size() != expected) {
    throw std::invalid_argument(
        std::string("ordinary ") + label +
        " position command must contain every active arm joint");
  }

  std::set<void*> unique;
  for (std::size_t target_index = 0;
       target_index < targets.size(); ++target_index) {
    const auto& [motor_handle, target_position] = targets[target_index];
    const auto motor = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
          return !record.descriptor.is_gripper &&
                 record.descriptor.motor == motor_handle;
        });
    if (motor == motors_.end()) {
      throw std::invalid_argument(
          std::string("ordinary ") + label +
          " position command contains an unexpected motor");
    }
    if (!unique.insert(motor_handle).second) {
      throw std::invalid_argument(
          std::string("ordinary ") + label +
          " position command contains duplicate motors");
    }

    const auto& limits = joint_config(motor_handle);
    if (max_reference_velocity > limits.velocity_limit) {
      throw std::invalid_argument(
          std::string(motor->descriptor.name) + ": shared " + label +
          " reference velocity exceeds joint safety limit");
    }
    if (requested_mode == ARTICORE_MODE_PV &&
        pv_velocity_limit > limits.velocity_limit) {
      throw std::invalid_argument(
          std::string(motor->descriptor.name) +
          ": PV velocity limit exceeds joint safety limit");
    }
    if (per_joint_profile) {
      const float joint_velocity =
          pv_reference_velocity_limits->at(target_index);
      const float joint_acceleration =
          pv_reference_acceleration_limits->at(target_index);
      if (!finite(joint_velocity) || joint_velocity <= 0.0f ||
          joint_velocity > limits.velocity_limit ||
          !finite(joint_acceleration) || joint_acceleration <= 0.0f) {
        throw std::invalid_argument(
            std::string(motor->descriptor.name) +
            ": invalid ordinary PV per-joint motion limit");
      }
    }
    validate_position_velocity_torque(
        motor_handle, target_position, 0.0f, 0.0f);
  }

  CommandTransaction owned_transaction;
  if (transaction) {
    if (!transaction->owns_lock() || transaction->mutex() != &command_mutex_) {
      throw std::logic_error(
          "planned joint position requires the Runtime command transaction");
    }
  } else {
    if (planning_token != 0) {
      throw std::logic_error(
          "planned joint position token requires a command transaction");
    }
    owned_transaction = begin_command_transaction();
  }

  const auto now = Clock::now();
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (planning_token != 0 &&
      active_command_planning_token_ != planning_token) {
    throw std::runtime_error(
        "Runtime state changed while planning the command");
  }
  require_state_for_command(false, false, planning_token);
  if (trajectory_control_.state == ARTICORE_MOTION_COMPLETED) {
    terminate_trajectory_locked(
        ARTICORE_MOTION_CANCELLED,
        std::string("completed trajectory replaced by ordinary ") + label +
            " position command");
  }
  if (mode_ != requested_mode) {
    throw std::runtime_error(
        std::string("ordinary ") + label +
        " position command requires matching runtime mode");
  }
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    if (!sides_[side].connected || !sides_[side].transport_healthy) {
      throw std::runtime_error(
          std::string(side == 0 ? "CH0" : "CH1") +
          " transport is not healthy for ordinary " + label +
          " position control");
    }
  }

  const bool continuing = arm_mailbox_.joint_position;
  const auto maximum_age_ns =
      static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  ArmMailbox next;
  next.valid = true;
  next.user_command = true;
  next.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
  next.generation = next_arm_generation();
  next.submitted_at = now;
  next.joint_position = true;
  next.mit_direct = requested_mode == ARTICORE_MODE_MIT && mit_direct;
  next.pv_per_joint_profile = per_joint_profile;
  next.max_reference_velocity = max_reference_velocity;
  next.max_reference_acceleration = max_reference_acceleration;
  next.pv_velocity_limit = pv_velocity_limit;
  next.final_positions.reserve(targets.size());
  if (requested_mode == ARTICORE_MODE_PV) {
    next.pv.reserve(targets.size());
    next.pv_reference_positions.reserve(targets.size());
    next.pv_reference_velocities.reserve(targets.size());
    next.pv_drive_velocity_commands.reserve(targets.size());
    next.pv_hold_confirmation_cycles.reserve(targets.size());
    next.pv_stationary_hold.reserve(targets.size());
    next.pv_hold_target_positions.reserve(targets.size());
    if (per_joint_profile) {
      next.pv_reference_velocity_limits =
          *pv_reference_velocity_limits;
      next.pv_reference_acceleration_limits =
          *pv_reference_acceleration_limits;
    }
  } else {
    next.mit.reserve(targets.size());
  }

  for (const auto& [motor_handle, final_position] : targets) {
    const auto motor = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
          return record.descriptor.motor == motor_handle;
        });
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (backend_->get_feedback_stats(motor_handle, &stats) != 0 ||
        !stats.has_feedback || stats.age_ns > maximum_age_ns ||
        backend_->get_state(motor_handle, &state) != 0 ||
        !state.has_value || !finite(state.pos) || state.status_code != 1) {
      throw std::runtime_error(
          "CH" + std::to_string(motor->descriptor.side) + "/" +
          motor->descriptor.name +
          ": complete fresh enabled feedback is required for ordinary " +
          label + " position control");
    }

    float current_position = state.pos;
    float current_command_position = state.pos;
    float current_reference_velocity = 0.0f;
    float current_drive_velocity = 0.0f;
    uint16_t current_hold_confirmations = 0;
    uint8_t current_stationary_hold = 0;
    float current_hold_target = final_position;
    if (continuing) {
      if (requested_mode == ARTICORE_MODE_PV) {
        const auto previous = std::find_if(
            arm_mailbox_.pv.begin(), arm_mailbox_.pv.end(),
            [&](const ArticorePosVelCommand& command) {
              return command.motor == motor_handle;
            });
        if (previous == arm_mailbox_.pv.end()) {
          throw std::runtime_error(
              "ordinary PV reference state does not match the active arm layout");
        }
        const auto previous_index = static_cast<std::size_t>(
            std::distance(arm_mailbox_.pv.begin(), previous));
        current_command_position = previous->target_position;
        if (arm_mailbox_.pv_reference_positions.size() !=
                arm_mailbox_.pv.size() ||
            arm_mailbox_.pv_reference_velocities.size() !=
            arm_mailbox_.pv.size()) {
          throw std::runtime_error(
              "ordinary PV reference velocity state is inconsistent");
        }
        current_position =
            arm_mailbox_.pv_reference_positions[previous_index];
        current_reference_velocity =
            arm_mailbox_.pv_reference_velocities[previous_index];
        if (arm_mailbox_.pv_drive_velocity_commands.size() ==
                arm_mailbox_.pv.size()) {
          current_drive_velocity =
              arm_mailbox_.pv_drive_velocity_commands[previous_index];
        }
        if (arm_mailbox_.pv_hold_confirmation_cycles.size() ==
                arm_mailbox_.pv.size() &&
            arm_mailbox_.pv_stationary_hold.size() ==
                arm_mailbox_.pv.size() &&
            arm_mailbox_.pv_hold_target_positions.size() ==
                arm_mailbox_.pv.size()) {
          const float previous_hold_target =
              arm_mailbox_.pv_hold_target_positions[previous_index];
          if (std::abs(final_position - previous_hold_target) <=
              kNativeOrdinaryPvHoldTargetTolerance) {
            current_hold_confirmations =
                arm_mailbox_.pv_hold_confirmation_cycles[previous_index];
            current_stationary_hold =
                arm_mailbox_.pv_stationary_hold[previous_index];
            current_hold_target = previous_hold_target;
          }
        }
      } else {
        const auto previous = std::find_if(
            arm_mailbox_.mit.begin(), arm_mailbox_.mit.end(),
            [&](const ArticoreMitCommand& command) {
              return command.motor == motor_handle;
            });
        if (previous == arm_mailbox_.mit.end()) {
          throw std::runtime_error(
              "ordinary MIT position state does not match the active arm layout");
        }
        current_position = previous->target_position;
      }
    }

    if (requested_mode == ARTICORE_MODE_PV) {
      next.pv.push_back(ArticorePosVelCommand{
          motor_handle,
          current_command_position,
          current_drive_velocity});
      next.pv_reference_positions.push_back(current_position);
      next.pv_reference_velocities.push_back(current_reference_velocity);
      next.pv_hold_confirmation_cycles.push_back(
          current_hold_confirmations);
      next.pv_stationary_hold.push_back(current_stationary_hold);
      next.pv_hold_target_positions.push_back(current_hold_target);
      next.pv_drive_velocity_commands.push_back(current_drive_velocity);
    } else {
      const auto& config = joint_config(motor_handle);
      next.mit.push_back(ArticoreMitCommand{
          motor_handle, mit_direct ? final_position : current_position, 0.0f,
          mit_direct ? config.mit_kp : config.mit_fast_follow_kp,
          mit_direct ? config.mit_kd : config.mit_fast_follow_kd, 0.0f});
    }
    next.final_positions.push_back(final_position);
  }

  clear_pending_arm_mailbox();
  arm_mailbox_ = std::move(next);
  first_command_accepted_ = true;
  if (planning_token != 0) active_command_planning_token_ = 0;
  wakeup_.notify_all();
}

}  // namespace articore
