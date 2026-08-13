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

SafetyRuntime::TrajectorySample SafetyRuntime::sample_trajectory_joint(
    const TrajectoryRecord& trajectory, const TrajectoryJoint& joint,
    Clock::time_point now) const {
  const auto elapsed = std::max(
      std::chrono::nanoseconds::zero(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          now - trajectory.start_time));
  const bool complete = elapsed >= trajectory.duration;
  if (complete) return {joint.goal_position, 0.0f, 0.0f};
  const double seconds =
      std::chrono::duration<double>(trajectory.duration).count();
  const double u = std::clamp(
      static_cast<double>(elapsed.count()) / trajectory.duration.count(),
      0.0, 1.0);
  const double delta = static_cast<double>(joint.goal_position) -
                       joint.start_position;
  if (trajectory.profile == ARTICORE_TRAJECTORY_LINEAR) {
    return {
        static_cast<float>(joint.start_position + u * delta),
        static_cast<float>(delta / seconds),
        0.0f,
    };
  }

  // General quintic boundary polynomial. Zero start velocity/acceleration is
  // the ordinary minimum-jerk profile. Smooth replacement inherits the old
  // trajectory's q/v/a at one instant and ends at q_goal/0/0.
  const double c0 = joint.start_position;
  const double c1 = static_cast<double>(joint.start_velocity) * seconds;
  const double c2 = 0.5 * static_cast<double>(joint.start_acceleration) *
                    seconds * seconds;
  const double c3 = 10.0 * delta - 6.0 * c1 - 3.0 * c2;
  const double c4 = -15.0 * delta + 8.0 * c1 + 3.0 * c2;
  const double c5 = 6.0 * delta - 3.0 * c1 - c2;
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double position = c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 +
                          c5 * u5;
  const double velocity =
      (c1 + 2.0 * c2 * u + 3.0 * c3 * u2 + 4.0 * c4 * u3 +
       5.0 * c5 * u4) /
      seconds;
  const double acceleration =
      (2.0 * c2 + 6.0 * c3 * u + 12.0 * c4 * u2 + 20.0 * c5 * u3) /
      (seconds * seconds);
  return {
      static_cast<float>(position),
      static_cast<float>(velocity),
      static_cast<float>(acceleration),
  };
}

bool SafetyRuntime::trajectory_within_limits(
    const TrajectoryRecord& trajectory) const {
  constexpr uint32_t kSamples = 1024;
  constexpr double kVelocityTolerance = 1e-5;
  for (const auto& joint : trajectory.joints) {
    const auto& limits = joint_config(joint.motor);
    if (std::abs(joint.start_velocity) >
        joint.velocity_limit + kVelocityTolerance) {
      return false;
    }
    for (uint32_t index = 0; index <= kSamples; ++index) {
      const auto offset = std::chrono::duration_cast<Clock::duration>(
          trajectory.duration * index / kSamples);
      const auto sample = sample_trajectory_joint(
          trajectory, joint, trajectory.start_time + offset);
      if (!finite(sample.position) || !finite(sample.velocity) ||
          sample.position < limits.lower_position ||
          sample.position > limits.upper_position ||
          std::abs(sample.velocity) >
              joint.velocity_limit + kVelocityTolerance) {
        return false;
      }
    }
  }
  return true;
}

uint64_t SafetyRuntime::start_joint_trajectory(
    const ArticoreJointTrajectoryTarget* targets, uint32_t count,
    ArticoreTrajectoryProfile profile) {
  return start_joint_trajectory_ex(
      targets, count, profile, ARTICORE_TRAJECTORY_REJECT_IF_BUSY);
}

uint64_t SafetyRuntime::start_joint_trajectory_ex(
    const ArticoreJointTrajectoryTarget* targets, uint32_t count,
    ArticoreTrajectoryProfile profile,
    ArticoreTrajectoryReplacePolicy replace_policy) {
  if (!targets || count == 0) {
    throw std::invalid_argument("joint trajectory is empty");
  }
  if (profile != ARTICORE_TRAJECTORY_MIN_JERK &&
      profile != ARTICORE_TRAJECTORY_LINEAR) {
    throw std::invalid_argument("unsupported trajectory profile");
  }
  if (replace_policy != ARTICORE_TRAJECTORY_REJECT_IF_BUSY &&
      replace_policy != ARTICORE_TRAJECTORY_SMOOTH_REPLACE) {
    throw std::invalid_argument("unsupported trajectory replace policy");
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
  const auto start_time = Clock::now();
  std::optional<TrajectoryRecord> replaced;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (active_trajectory_ &&
        replace_policy == ARTICORE_TRAJECTORY_REJECT_IF_BUSY) {
      throw std::runtime_error(
          "joint trajectory is already active; wait for it to complete");
    }
    if (active_trajectory_) replaced = *active_trajectory_;
    trajectory_mode = mode_;
  }
  if (replaced && profile != ARTICORE_TRAJECTORY_MIN_JERK) {
    throw std::invalid_argument(
        "smooth trajectory replacement requires MIN_JERK profile");
  }

  TrajectoryRecord trajectory;
  trajectory.profile = profile;
  trajectory.start_time = start_time;
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
    float start_velocity = 0.0f;
    float start_acceleration = 0.0f;
    if (replaced) {
      const auto previous = std::find_if(
          replaced->joints.begin(), replaced->joints.end(),
          [&](const TrajectoryJoint& value) {
            return value.motor == target.motor;
          });
      if (previous == replaced->joints.end()) {
        throw std::runtime_error(
            "active trajectory does not match the fixed arm layout");
      }
      const auto sample = sample_trajectory_joint(
          *replaced, *previous, start_time);
      start_position = sample.position;
      start_velocity = sample.velocity;
      start_acceleration = sample.acceleration;
    } else if (arm_mailbox_.valid && arm_mailbox_.trajectory_endpoint_hold) {
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
        target.motor, start_position, start_velocity, start_acceleration,
        target.target_position,
        target.velocity_limit});
  }

  const auto minimum_duration =
      std::chrono::duration<double>(1.0 / config_.control_hz);
  trajectory.duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::max(std::chrono::duration<double>(duration_seconds), minimum_duration));
  if (replaced) {
    // Boundary velocity/acceleration can create an interior overshoot. Search
    // for a duration whose complete polynomial respects every new velocity
    // and position limit. This runs only at task submission, never in the
    // 500 Hz control hot path.
    const auto initial_duration = trajectory.duration;
    const auto maximum_duration = std::min(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::seconds(120)),
        initial_duration * 32);
    bool valid = trajectory_within_limits(trajectory);
    while (!valid && trajectory.duration < maximum_duration) {
      trajectory.duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
          trajectory.duration * 1.05);
      valid = trajectory_within_limits(trajectory);
    }
    if (!valid) {
      throw std::invalid_argument(
          "smooth replacement cannot satisfy position and velocity limits");
    }
  }
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (replace_policy == ARTICORE_TRAJECTORY_REJECT_IF_BUSY &&
        active_trajectory_) {
      throw std::runtime_error(
          "joint trajectory is already active; wait for it to complete");
    }
    if (replaced &&
        (!active_trajectory_ || active_trajectory_->id != replaced->id)) {
      throw std::runtime_error(
          "active trajectory changed while replacement was prepared");
    }
    trajectory.id = next_trajectory_id_++;
    if (trajectory.id == 0) trajectory.id = next_trajectory_id_++;
    trajectory_id = trajectory.id;
    if (replaced) {
      finish_trajectory_locked(
          replaced->id, ARTICORE_TRAJECTORY_PREEMPTED,
          "smoothly replaced by trajectory " + std::to_string(trajectory_id),
          start_time);
    }
    active_trajectory_ = std::move(trajectory);
  }
  wakeup_.notify_all();
  return trajectory_id;
}

void SafetyRuntime::install_cancellation_hold_locked(Clock::time_point now) {
  ArmMailbox hold;
  hold.valid = true;
  hold.user_command = false;
  hold.trajectory_endpoint_hold = true;
  hold.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
  hold.generation = arm_mailbox_.generation + 1;
  hold.submitted_at = now;
  const auto max_age =
      static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  for (const auto& motor : motors_) {
    if (motor.descriptor.is_gripper) continue;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool fresh =
        api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
        stats.has_feedback && stats.age_ns <= max_age &&
        api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
        state.has_value && state.status_code == 1 && finite(state.pos);
    float position = 0.0f;
    if (fresh) {
      position = state.pos;
    } else if (mode_ == ARTICORE_MODE_PV) {
      const auto previous = std::find_if(
          last_sent_pv_.begin(), last_sent_pv_.end(),
          [&](const ArticorePosVelCommand& value) {
            return value.motor == motor.descriptor.motor;
          });
      if (previous == last_sent_pv_.end()) {
        throw std::runtime_error(
            std::string(motor.descriptor.name) +
            ": no fresh feedback or sent target is available for cancel hold");
      }
      position = previous->target_position;
    } else {
      const auto previous = std::find_if(
          last_sent_mit_.begin(), last_sent_mit_.end(),
          [&](const ArticoreMitCommand& value) {
            return value.motor == motor.descriptor.motor;
          });
      if (previous == last_sent_mit_.end()) {
        throw std::runtime_error(
            std::string(motor.descriptor.name) +
            ": no fresh feedback or sent target is available for cancel hold");
      }
      position = previous->target_position;
    }
    if (mode_ == ARTICORE_MODE_PV) {
      hold.pv.push_back({motor.descriptor.motor, position,
                         config_.safe_pv_velocity_limit});
    } else {
      hold.mit.push_back({motor.descriptor.motor, position, 0.0f,
                          motor.descriptor.safe_kp,
                          motor.descriptor.safe_kd, 0.0f});
    }
  }
  arm_mailbox_ = std::move(hold);
}

void SafetyRuntime::cancel_trajectory(uint64_t trajectory_id) {
  if (trajectory_id == 0) {
    throw std::invalid_argument("trajectory id must be nonzero");
  }
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
  }
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (!active_trajectory_ || active_trajectory_->id != trajectory_id) {
      throw std::invalid_argument(
          "trajectory is not the currently active trajectory");
    }
  }
  install_cancellation_hold_locked(now);
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (!active_trajectory_ || active_trajectory_->id != trajectory_id) {
      throw std::runtime_error("active trajectory changed during cancellation");
    }
    finish_trajectory_locked(
        trajectory_id, ARTICORE_TRAJECTORY_CANCELED,
        "canceled by caller", now);
    next_control_tick_ = now;
  }
  wakeup_.notify_all();
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
