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
    if (targets[i].struct_size < sizeof(Target) || !targets[i].motor ||
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

void SafetyRuntime::set_joint_pv(
    const ArticoreJointPvTarget* targets, uint32_t count,
    float max_reference_velocity) {
  install_joint_position(
      ARTICORE_MODE_PV, collect_targets(targets, count, "PV"),
      max_reference_velocity);
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

void SafetyRuntime::update_joint_position_velocity(
    float max_reference_velocity) {
  if (!finite(max_reference_velocity) || max_reference_velocity < 0.0f) {
    throw std::invalid_argument(
        "max_reference_velocity must be finite and non-negative");
  }

  std::lock_guard<std::mutex> command_lock(command_mutex_);
  if (!arm_mailbox_.valid || !arm_mailbox_.joint_position) return;

  for (const auto& command : arm_mailbox_.pv) {
    if (max_reference_velocity > joint_config(command.motor).velocity_limit) {
      throw std::invalid_argument(
          "ordinary reference velocity exceeds joint safety limit");
    }
  }
  for (const auto& command : arm_mailbox_.mit) {
    if (max_reference_velocity > joint_config(command.motor).velocity_limit) {
      throw std::invalid_argument(
          "ordinary reference velocity exceeds joint safety limit");
    }
  }

  arm_mailbox_.max_reference_velocity = max_reference_velocity;
  for (auto& command : arm_mailbox_.pv) {
    command.velocity_limit = std::max(
        config_.safe_pv_velocity_limit, max_reference_velocity);
  }
  wakeup_.notify_all();
}

void SafetyRuntime::install_joint_position(
    ArticoreControlMode requested_mode,
    const std::vector<std::pair<void*, float>>& targets,
    float max_reference_velocity) {
  const char* const label = mode_name(requested_mode);
  if (!finite(max_reference_velocity) || max_reference_velocity < 0.0f) {
    throw std::invalid_argument(
        "max_reference_velocity must be finite and non-negative");
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
  for (const auto& [motor_handle, target_position] : targets) {
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
    validate_position_velocity_torque(
        motor_handle, target_position, 0.0f, 0.0f);
  }

  const auto now = Clock::now();
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  require_state_for_command();
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
  next.max_reference_velocity = max_reference_velocity;
  next.final_positions.reserve(targets.size());
  if (requested_mode == ARTICORE_MODE_PV) {
    next.pv.reserve(targets.size());
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
    if (api_.motor_get_feedback_stats(motor_handle, &stats) != 0 ||
        !stats.has_feedback || stats.age_ns > maximum_age_ns ||
        api_.motor_get_state(motor_handle, &state) != 0 ||
        !state.has_value || !finite(state.pos) || state.status_code != 1) {
      throw std::runtime_error(
          "CH" + std::to_string(motor->descriptor.side) + "/" +
          motor->descriptor.name +
          ": complete fresh enabled feedback is required for ordinary " +
          label + " position control");
    }

    float current_position = state.pos;
    if (continuing) {
      if (requested_mode == ARTICORE_MODE_PV) {
        const auto previous = std::find_if(
            arm_mailbox_.pv.begin(), arm_mailbox_.pv.end(),
            [&](const ArticorePosVelCommand& command) {
              return command.motor == motor_handle;
            });
        if (previous == arm_mailbox_.pv.end()) {
          throw std::runtime_error(
              "ordinary PV position state does not match the active arm layout");
        }
        current_position = previous->target_position;
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
          motor_handle, current_position,
          std::max(config_.safe_pv_velocity_limit,
                   max_reference_velocity)});
    } else {
      const auto& config = joint_config(motor_handle);
      next.mit.push_back(ArticoreMitCommand{
          motor_handle, current_position, 0.0f,
          config.mit_kp, config.mit_kd, 0.0f});
    }
    next.final_positions.push_back(final_position);
  }

  clear_pending_arm_mailbox();
  arm_mailbox_ = std::move(next);
  wakeup_.notify_all();
}

}  // namespace articore
