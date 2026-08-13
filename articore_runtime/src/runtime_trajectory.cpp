#include "runtime.hpp"
#include "runtime_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace articore {

using detail::age_ns;
using detail::copy_text;

void SafetyRuntime::trim_trajectory_history_locked() {
  constexpr std::size_t kMaximumTrajectoryHistory = 64;
  while (trajectory_history_order_.size() > kMaximumTrajectoryHistory) {
    trajectory_history_.erase(trajectory_history_order_.front());
    trajectory_history_order_.pop_front();
  }
}

void SafetyRuntime::finish_trajectory_locked(
    uint64_t id, ArticoreTrajectoryStatus status, const std::string& error,
    Clock::time_point now) {
  if (!active_trajectory_ || active_trajectory_->id != id) return;
  active_trajectory_->status = status;
  active_trajectory_->error = error;
  active_trajectory_->finished_at = now;
  trajectory_history_[id] = *active_trajectory_;
  trajectory_history_order_.push_back(id);
  active_trajectory_.reset();
  trim_trajectory_history_locked();
  trajectory_cv_.notify_all();
}

void SafetyRuntime::cancel_active_trajectory_locked(
    ArticoreTrajectoryStatus status, const std::string& error) {
  if (!active_trajectory_) return;
  finish_trajectory_locked(active_trajectory_->id, status, error, Clock::now());
}

uint64_t SafetyRuntime::start_joint_trajectory(
    const ArticoreJointTrajectoryTarget* targets, uint32_t count,
    ArticoreTrajectoryProfile profile) {
  if (!targets || count == 0) {
    throw std::invalid_argument("joint trajectory is empty");
  }
  if (profile != ARTICORE_TRAJECTORY_MIN_JERK &&
      profile != ARTICORE_TRAJECTORY_LINEAR) {
    throw std::invalid_argument("unsupported trajectory profile");
  }
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      }));
  if (count != expected) {
    throw std::invalid_argument(
        "trajectory must contain the complete fixed arm layout");
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
  }
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  uint64_t trajectory_id = 0;
  ArticoreControlMode trajectory_mode = ARTICORE_MODE_PV;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (active_trajectory_) {
      throw std::runtime_error(
          "joint trajectory is already active; wait for it to complete");
    }
    trajectory_mode = mode_;
  }

  TrajectoryRecord trajectory;
  trajectory.profile = profile;
  trajectory.start_time = Clock::now();
  trajectory.joints.reserve(count);
  double duration_seconds = 0.0;
  std::set<void*> unique;
  for (uint32_t i = 0; i < count; ++i) {
    const auto& target = targets[i];
    if (!target.motor || !finite(target.target_position) ||
        !finite(target.velocity_limit) || target.velocity_limit <= 0.0f ||
        !unique.insert(target.motor).second) {
      throw std::invalid_argument("trajectory contains invalid targets");
    }
    const auto motor = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
          return !record.descriptor.is_gripper &&
                 record.descriptor.motor == target.motor;
        });
    if (motor == motors_.end()) {
      throw std::invalid_argument("trajectory contains an unexpected motor");
    }
    const auto& limits = joint_config(target.motor);
    if (target.velocity_limit > limits.velocity_limit) {
      throw std::invalid_argument("trajectory velocity exceeds joint limit");
    }
    validate_position_velocity_torque(
        target.motor, target.target_position, target.velocity_limit, 0.0f);

    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(target.motor, &stats) != 0 ||
        !stats.has_feedback ||
        stats.age_ns > static_cast<uint64_t>(config_.feedback_max_age_ms) *
                           1'000'000ULL ||
        api_.motor_get_state(target.motor, &state) != 0 || !state.has_value ||
        !finite(state.pos) || !finite(state.vel) || !finite(state.torq) ||
        state.status_code != 1) {
      throw std::runtime_error(
          std::string(motor->descriptor.name) +
          ": complete fresh enabled feedback is required for trajectory");
    }
    float start_position = state.pos;
    if (arm_mailbox_.valid && arm_mailbox_.trajectory_endpoint_hold) {
      if (trajectory_mode == ARTICORE_MODE_PV) {
        const auto commanded = std::find_if(
            arm_mailbox_.pv.begin(), arm_mailbox_.pv.end(),
            [&](const ArticorePosVelCommand& value) {
              return value.motor == target.motor;
            });
        if (commanded != arm_mailbox_.pv.end()) {
          start_position = commanded->target_position;
        }
      } else {
        const auto commanded = std::find_if(
            arm_mailbox_.mit.begin(), arm_mailbox_.mit.end(),
            [&](const ArticoreMitCommand& value) {
              return value.motor == target.motor;
            });
        if (commanded != arm_mailbox_.mit.end()) {
          start_position = commanded->target_position;
        }
      }
    }
    const auto distance = std::abs(
        static_cast<double>(target.target_position) - start_position);
    const auto factor = profile == ARTICORE_TRAJECTORY_MIN_JERK ? 1.875 : 1.0;
    duration_seconds = std::max(
        duration_seconds, factor * distance / target.velocity_limit);
    trajectory.joints.push_back(TrajectoryJoint{
        target.motor, start_position, target.target_position,
        target.velocity_limit});
  }

  const auto minimum_duration =
      std::chrono::duration<double>(1.0 / config_.control_hz);
  trajectory.duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::max(std::chrono::duration<double>(duration_seconds), minimum_duration));
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (active_trajectory_) {
      throw std::runtime_error(
          "joint trajectory is already active; wait for it to complete");
    }
    trajectory.id = next_trajectory_id_++;
    if (trajectory.id == 0) trajectory.id = next_trajectory_id_++;
    trajectory_id = trajectory.id;
    active_trajectory_ = std::move(trajectory);
  }
  wakeup_.notify_all();
  return trajectory_id;
}

ArticoreTrajectoryInfo SafetyRuntime::trajectory_info(
    uint64_t trajectory_id) const {
  if (trajectory_id == 0) {
    throw std::invalid_argument("trajectory id must be nonzero");
  }
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(state_mutex_);
  const TrajectoryRecord* record = nullptr;
  if (active_trajectory_ && active_trajectory_->id == trajectory_id) {
    record = &*active_trajectory_;
  } else {
    const auto found = trajectory_history_.find(trajectory_id);
    if (found != trajectory_history_.end()) record = &found->second;
  }
  if (!record) throw std::invalid_argument("unknown trajectory id");
  ArticoreTrajectoryInfo info{};
  info.struct_size = sizeof(info);
  info.trajectory_id = record->id;
  info.status = record->status;
  info.profile = record->profile;
  info.duration_ns = static_cast<uint64_t>(record->duration.count());
  const auto end = record->status == ARTICORE_TRAJECTORY_RUNNING
      ? now : record->finished_at;
  info.elapsed_ns = age_ns(record->start_time, true, end);
  if (info.duration_ns > 0) {
    info.elapsed_ns = std::min(info.elapsed_ns, info.duration_ns);
  }
  copy_text(info.error, record->error);
  return info;
}

ArticoreTrajectoryInfo SafetyRuntime::wait_trajectory(
    uint64_t trajectory_id, std::chrono::milliseconds timeout) {
  if (trajectory_id == 0) {
    throw std::invalid_argument("trajectory id must be nonzero");
  }
  std::unique_lock<std::mutex> lock(state_mutex_);
  const auto known = [&] {
    return (active_trajectory_ && active_trajectory_->id == trajectory_id) ||
           trajectory_history_.find(trajectory_id) != trajectory_history_.end();
  };
  if (!known()) throw std::invalid_argument("unknown trajectory id");
  const auto terminal = [&] {
    return trajectory_history_.find(trajectory_id) != trajectory_history_.end();
  };
  if (!terminal() && !trajectory_cv_.wait_for(lock, timeout, terminal)) {
    throw std::runtime_error("trajectory wait timed out");
  }
  lock.unlock();
  return trajectory_info(trajectory_id);
}

}  // namespace articore
