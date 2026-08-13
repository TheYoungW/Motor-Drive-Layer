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

float SafetyRuntime::allowed_outward_velocity(
    const JointControlConfig& limits, float position,
    bool toward_upper) const {
  const float distance = toward_upper
      ? limits.soft_upper_position - position
      : position - limits.soft_lower_position;
  if (distance <= 0.0f) return 0.0f;
  float allowed = limits.velocity_limit;
  if (limits.soft_limit_braking_zone > 0.0f &&
      distance < limits.soft_limit_braking_zone) {
    allowed = std::min(
        allowed,
        limits.velocity_limit *
            std::sqrt(distance / limits.soft_limit_braking_zone));
  }
  if (limits.braking_acceleration > 0.0f) {
    allowed = std::min(
        allowed, std::sqrt(2.0f * limits.braking_acceleration * distance));
  }
  return allowed;
}

bool SafetyRuntime::trajectory_within_limits(
    const TrajectoryRecord& trajectory, LimitViolation* violation) const {
  constexpr uint32_t kSamples = 1024;
  constexpr double kVelocityTolerance = 1e-5;
  const auto reject = [&](const TrajectoryJoint& joint,
                          const TrajectorySample& sample,
                          const char* reason) {
    if (violation) {
      violation->motor = joint.motor;
      violation->reason = reason;
      violation->position = sample.position;
      violation->velocity = sample.velocity;
      violation->acceleration = sample.acceleration;
    }
    return false;
  };
  for (const auto& joint : trajectory.joints) {
    const auto& limits = joint_config(joint.motor);
    if (std::abs(joint.start_velocity) >
        joint.velocity_limit + kVelocityTolerance) {
      return reject(joint,
                    {joint.start_position, joint.start_velocity,
                     joint.start_acceleration},
                    "initial velocity exceeds requested trajectory limit");
    }
    for (uint32_t index = 0; index <= kSamples; ++index) {
      const auto offset = std::chrono::duration_cast<Clock::duration>(
          trajectory.duration * index / kSamples);
      const auto sample = sample_trajectory_joint(
          trajectory, joint, trajectory.start_time + offset);
      if (!finite(sample.position) || !finite(sample.velocity) ||
          !finite(sample.acceleration)) {
        return reject(joint, sample, "trajectory sample is not finite");
      }
      if (sample.position < limits.hard_lower_position ||
          sample.position > limits.hard_upper_position) {
        return reject(joint, sample, "trajectory crosses a hard position limit");
      }
      if (std::abs(sample.velocity) >
          joint.velocity_limit + kVelocityTolerance) {
        return reject(joint, sample, "trajectory exceeds requested velocity limit");
      }
      if (sample.position > limits.soft_upper_position &&
          sample.velocity > kVelocityTolerance) {
        return reject(joint, sample,
                      "trajectory continues outward above the upper soft limit");
      }
      if (sample.position < limits.soft_lower_position &&
          sample.velocity < -kVelocityTolerance) {
        return reject(joint, sample,
                      "trajectory continues outward below the lower soft limit");
      }
      if (sample.position <= limits.soft_upper_position &&
          sample.velocity > kVelocityTolerance &&
          sample.velocity >
              allowed_outward_velocity(limits, sample.position, true) +
                  kVelocityTolerance) {
        return reject(joint, sample,
                      "trajectory exceeds upper soft-limit braking velocity");
      }
      if (sample.position >= limits.soft_lower_position &&
          sample.velocity < -kVelocityTolerance &&
          -sample.velocity >
              allowed_outward_velocity(limits, sample.position, false) +
                  kVelocityTolerance) {
        return reject(joint, sample,
                      "trajectory exceeds lower soft-limit braking velocity");
      }
    }
  }
  return true;
}

SafetyRuntime::TrajectoryProgress
SafetyRuntime::update_trajectory_progress_locked(
    TrajectoryRecord& trajectory, Clock::time_point now, std::string& error) {
  const auto profile_end = trajectory.start_time + trajectory.duration;
  if (!trajectory.settling && now >= profile_end) {
    trajectory.settling = true;
    trajectory.settling_started_at = profile_end;
    trajectory.settling_stable_started_at = {};
  }

  bool all_settled = trajectory.settling;
  float worst_position_error = -1.0f;
  float worst_velocity_error = -1.0f;
  const MotorRecord* worst_motor = nullptr;
  ArticoreMotorState worst_state{};
  for (auto& joint : trajectory.joints) {
    const auto motor = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& candidate) {
          return candidate.descriptor.motor == joint.motor;
        });
    if (motor == motors_.end()) {
      error = "trajectory references an unknown motor";
      return TrajectoryProgress::Failed;
    }
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool fresh =
        api_.motor_get_feedback_stats(joint.motor, &stats) == 0 &&
        stats.has_feedback &&
        stats.age_ns <=
            static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL &&
        api_.motor_get_state(joint.motor, &state) == 0 && state.has_value &&
        state.status_code == 1 && finite(state.pos) && finite(state.vel);
    if (!fresh) {
      joint.following_error_started_at = {};
      all_settled = false;
      continue;
    }

    const auto reference = sample_trajectory_joint(trajectory, joint, now);
    const float following_error = std::abs(reference.position - state.pos);
    if (following_error > trajectory_execution_.following_error_limit) {
      if (joint.following_error_started_at == Clock::time_point{}) {
        joint.following_error_started_at = now;
      } else if (now - joint.following_error_started_at >=
                 trajectory_execution_.following_error_timeout) {
        std::ostringstream message;
        message << "trajectory following error: channel=CH"
                << static_cast<unsigned>(motor->descriptor.side)
                << " motor=" << motor->descriptor.name
                << " can_id=" << static_cast<unsigned>(state.can_id)
                << " reference=" << reference.position
                << " actual=" << state.pos
                << " error=" << following_error
                << " limit=" << trajectory_execution_.following_error_limit;
        error = message.str();
        return TrajectoryProgress::Failed;
      }
    } else {
      joint.following_error_started_at = {};
    }

    if (!trajectory.settling) continue;
    const float position_error = std::abs(joint.goal_position - state.pos);
    const float velocity_error = std::abs(state.vel);
    if (position_error > trajectory_execution_.position_tolerance ||
        velocity_error > trajectory_execution_.velocity_tolerance) {
      all_settled = false;
    }
    if (position_error > worst_position_error ||
        (position_error == worst_position_error &&
         velocity_error > worst_velocity_error)) {
      worst_position_error = position_error;
      worst_velocity_error = velocity_error;
      worst_motor = &*motor;
      worst_state = state;
    }
  }

  if (!trajectory.settling) return TrajectoryProgress::Running;
  if (all_settled) {
    if (trajectory.settling_stable_started_at == Clock::time_point{}) {
      trajectory.settling_stable_started_at = now;
    }
    if (now - trajectory.settling_stable_started_at >=
        trajectory_execution_.settling_stable) {
      return TrajectoryProgress::Completed;
    }
  } else {
    trajectory.settling_stable_started_at = {};
  }

  if (now - trajectory.settling_started_at >=
      trajectory_execution_.settling_timeout) {
    std::ostringstream message;
    message << "trajectory settling timeout after "
            << trajectory_execution_.settling_timeout.count() << " ms";
    if (worst_motor) {
      message << ": channel=CH"
              << static_cast<unsigned>(worst_motor->descriptor.side)
              << " motor=" << worst_motor->descriptor.name
              << " can_id=" << static_cast<unsigned>(worst_state.can_id)
              << " position_error=" << worst_position_error
              << " position_tolerance="
              << trajectory_execution_.position_tolerance
              << " velocity_error=" << worst_velocity_error
              << " velocity_tolerance="
              << trajectory_execution_.velocity_tolerance;
    } else {
      message << ": no complete fresh enabled joint feedback";
    }
    error = message.str();
    return TrajectoryProgress::Failed;
  }
  return TrajectoryProgress::Running;
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
  if (replace_policy == ARTICORE_TRAJECTORY_SMOOTH_REPLACE_OR_HOLD) {
    throw std::invalid_argument(
        "SMOOTH_REPLACE_OR_HOLD requires the structured report API");
  }
  const auto report = start_joint_trajectory_report(
      targets, count, profile, replace_policy);
  if (report.outcome == ARTICORE_TRAJECTORY_START_STARTED ||
      report.outcome == ARTICORE_TRAJECTORY_START_REPLACED) {
    return report.new_trajectory_id;
  }
  const std::string reason = report.reason[0]
      ? std::string(report.reason) : std::string("trajectory start rejected");
  if (report.outcome == ARTICORE_TRAJECTORY_START_REJECTED) {
    if (reason.find("already active") != std::string::npos ||
        reason.find("runtime is not accepting") != std::string::npos ||
        reason.find("runtime state changed") != std::string::npos) {
      throw std::runtime_error(reason);
    }
    throw std::invalid_argument(reason);
  }
  throw std::runtime_error(reason);
}

ArticoreTrajectoryStartReport SafetyRuntime::start_joint_trajectory_report(
    const ArticoreJointTrajectoryTarget* targets, uint32_t count,
    ArticoreTrajectoryProfile profile,
    ArticoreTrajectoryReplacePolicy replace_policy) {
  ArticoreTrajectoryStartReport report{};
  report.struct_size = sizeof(report);
  report.outcome = ARTICORE_TRAJECTORY_START_REJECTED;
  const auto reject = [&](const std::string& reason) {
    copy_text(report.reason, reason);
    return report;
  };
  if (!targets || count == 0) return reject("joint trajectory is empty");
  if (profile != ARTICORE_TRAJECTORY_MIN_JERK &&
      profile != ARTICORE_TRAJECTORY_LINEAR) {
    return reject("unsupported trajectory profile");
  }
  if (replace_policy != ARTICORE_TRAJECTORY_REJECT_IF_BUSY &&
      replace_policy != ARTICORE_TRAJECTORY_SMOOTH_REPLACE &&
      replace_policy != ARTICORE_TRAJECTORY_SMOOTH_REPLACE_OR_HOLD) {
    return reject("unsupported trajectory replace policy");
  }
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      }));
  if (count != expected) {
    return reject("trajectory must contain the complete fixed arm layout");
  }

  std::lock_guard<std::mutex> command_lock(command_mutex_);
  const auto start_time = Clock::now();
  ArticoreControlMode trajectory_mode = ARTICORE_MODE_PV;
  std::optional<TrajectoryRecord> replaced;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (fault_latched_ || hardware_transition_ || enable_transaction_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING)) {
      report.outcome = fault_latched_ ? ARTICORE_TRAJECTORY_START_FAULTED
                                     : ARTICORE_TRAJECTORY_START_REJECTED;
      return reject("Articore runtime is not accepting motion commands");
    }
    if (active_trajectory_ &&
        replace_policy == ARTICORE_TRAJECTORY_REJECT_IF_BUSY) {
      report.old_trajectory_id = active_trajectory_->id;
      return reject("joint trajectory is already active; wait for it to complete");
    }
    if (active_trajectory_) {
      replaced = *active_trajectory_;
      report.old_trajectory_id = replaced->id;
    }
    trajectory_mode = mode_;
  }

  LimitViolation violation;
  bool planning_rejected = false;
  bool feedback_failure = false;
  void* feedback_failure_motor = nullptr;
  std::string failure_reason;
  if (replaced && profile != ARTICORE_TRAJECTORY_MIN_JERK) {
    planning_rejected = true;
    failure_reason = "smooth trajectory replacement requires MIN_JERK profile";
  }

  TrajectoryRecord trajectory;
  trajectory.profile = profile;
  trajectory.start_time = start_time;
  trajectory.joints.reserve(count);
  double duration_seconds = 0.0;
  std::set<void*> unique;
  for (uint32_t i = 0; i < count && !feedback_failure; ++i) {
    const auto& target = targets[i];
    if (!target.motor || !finite(target.target_position) ||
        !finite(target.velocity_limit) || target.velocity_limit <= 0.0f ||
        !unique.insert(target.motor).second) {
      return reject("trajectory contains invalid or duplicate targets");
    }
    const auto motor = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
          return !record.descriptor.is_gripper &&
                 record.descriptor.motor == target.motor;
        });
    if (motor == motors_.end()) {
      return reject("trajectory contains an unexpected motor");
    }
    const auto& limits = joint_config(target.motor);
    if (!planning_rejected && target.velocity_limit > limits.velocity_limit) {
      planning_rejected = true;
      violation = {target.motor, "trajectory velocity exceeds joint limit",
                   target.target_position, target.velocity_limit, 0.0f};
      failure_reason = violation.reason;
    }
    if (!planning_rejected &&
        (target.target_position < limits.hard_lower_position ||
         target.target_position > limits.hard_upper_position)) {
      planning_rejected = true;
      violation = {target.motor, "trajectory target exceeds a hard position limit",
                   target.target_position, 0.0f, 0.0f};
      failure_reason = violation.reason;
    }
    if (!planning_rejected &&
        (target.target_position < limits.soft_lower_position ||
         target.target_position > limits.soft_upper_position)) {
      planning_rejected = true;
      violation = {target.motor, "trajectory target exceeds a soft position limit",
                   target.target_position, 0.0f, 0.0f};
      failure_reason = violation.reason;
    }

    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(target.motor, &stats) != 0 ||
        !stats.has_feedback ||
        stats.age_ns > static_cast<uint64_t>(config_.feedback_max_age_ms) *
                           1'000'000ULL ||
        api_.motor_get_state(target.motor, &state) != 0 || !state.has_value ||
        !finite(state.pos) || !finite(state.vel) || !finite(state.torq) ||
        state.status_code != 1) {
      feedback_failure = true;
      feedback_failure_motor = target.motor;
      failure_reason = std::string(motor->descriptor.name) +
          ": complete fresh enabled feedback is required for trajectory";
      break;
    }
    if (state.pos < limits.hard_lower_position ||
        state.pos > limits.hard_upper_position) {
      feedback_failure = true;
      feedback_failure_motor = target.motor;
      failure_reason = std::string(motor->descriptor.name) +
          ": feedback position crossed a hard position limit";
      break;
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
        feedback_failure = true;
        feedback_failure_motor = target.motor;
        failure_reason = "active trajectory does not match the fixed arm layout";
        break;
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
        target.target_position, target.velocity_limit});
  }

  if (!feedback_failure && trajectory.joints.size() == count &&
      !planning_rejected) {
    const auto minimum_duration =
        std::chrono::duration<double>(1.0 / config_.control_hz);
    trajectory.duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::max(std::chrono::duration<double>(duration_seconds),
                 minimum_duration));
    const auto initial_duration = trajectory.duration;
    const auto maximum_duration = std::min(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::seconds(120)),
        std::max(initial_duration * 32,
                 std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::seconds(2))));
    bool valid = trajectory_within_limits(trajectory, &violation);
    while (!valid && trajectory.duration < maximum_duration) {
      trajectory.duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
          trajectory.duration * 1.05);
      valid = trajectory_within_limits(trajectory, &violation);
    }
    if (!valid) {
      planning_rejected = true;
      failure_reason = violation.reason;
    }
  }

  const bool replace_or_hold = replaced &&
      replace_policy == ARTICORE_TRAJECTORY_SMOOTH_REPLACE_OR_HOLD;
  if (feedback_failure || planning_rejected) {
    if (violation.motor) {
      populate_trajectory_start_report_limit(report, violation);
    } else if (feedback_failure_motor) {
      populate_trajectory_start_report_motor(
          report, feedback_failure_motor, false);
    }
    copy_text(report.reason, failure_reason);
    if (!replace_or_hold) return report;

    ArmMailbox hold;
    uint64_t maximum_age_ns = 0;
    std::string hold_error;
    void* limiting_motor = nullptr;
    const bool built = build_fresh_current_position_hold_locked(
        trajectory_mode, start_time, hold, maximum_age_ns,
        hold_error, limiting_motor);
    const bool sent = built &&
        transmit_hold_locked(hold, trajectory_mode, hold_error);
    if (!built || !sent) {
      report.outcome = ARTICORE_TRAJECTORY_START_FAULTED;
      report.hold_installed = 0;
      if (limiting_motor) {
        populate_trajectory_start_report_motor(report, limiting_motor, false);
      }
      const std::string fault = "replacement fallback failed: " + hold_error;
      copy_text(report.reason, fault);
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (active_trajectory_ && active_trajectory_->id == replaced->id) {
          finish_trajectory_locked(
              replaced->id, ARTICORE_TRAJECTORY_FAILED, fault, start_time);
        }
        if (built) {
          safe_pv_ = hold.pv;
          safe_mit_ = hold.mit;
        }
        arm_mailbox_ = ArmMailbox{};
        state_ = ARTICORE_FAULT;
        fault_latched_ = true;
        fault_reason_ = fault;
        fault_hold_active_ = built;
        next_safe_hold_ = start_time;
      }
      wakeup_.notify_all();
      return report;
    }
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (!active_trajectory_ || active_trajectory_->id != replaced->id ||
          mode_ != trajectory_mode || fault_latched_ || hardware_transition_) {
        report.outcome = ARTICORE_TRAJECTORY_START_FAULTED;
        copy_text(report.reason,
                  "runtime state changed during replacement hold transaction");
        return report;
      }
      finish_trajectory_locked(
          replaced->id, ARTICORE_TRAJECTORY_CANCELED,
          "replacement rejected; current-position hold installed: " +
              failure_reason,
          start_time);
      hold.sent_generation = hold.generation;
      arm_mailbox_ = std::move(hold);
      state_ = ARTICORE_RUNNING;
      fault_reason_.clear();
      last_fresh_feedback_ =
          start_time - std::chrono::nanoseconds(maximum_age_ns);
      next_control_tick_ = start_time;
      consecutive_send_failures_ = 0;
    }
    report.outcome =
        ARTICORE_TRAJECTORY_START_REPLACEMENT_REJECTED_HELD;
    report.hold_installed = 1;
    report.feedback_fresh = 1;
    copy_text(report.reason, failure_reason);
    wakeup_.notify_all();
    return report;
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (fault_latched_ || hardware_transition_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING)) {
      report.outcome = ARTICORE_TRAJECTORY_START_REJECTED;
      return reject("runtime state changed while trajectory was prepared");
    }
    if (replaced &&
        (!active_trajectory_ || active_trajectory_->id != replaced->id)) {
      return reject("active trajectory changed while replacement was prepared");
    }
    trajectory.id = next_trajectory_id_++;
    if (trajectory.id == 0) trajectory.id = next_trajectory_id_++;
    report.new_trajectory_id = trajectory.id;
    if (replaced) {
      finish_trajectory_locked(
          replaced->id, ARTICORE_TRAJECTORY_PREEMPTED,
          "smoothly replaced by trajectory " +
              std::to_string(trajectory.id),
          start_time);
    }
    active_trajectory_ = std::move(trajectory);
  }
  report.outcome = replaced ? ARTICORE_TRAJECTORY_START_REPLACED
                            : ARTICORE_TRAJECTORY_START_STARTED;
  report.feedback_fresh = 1;
  wakeup_.notify_all();
  return report;
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

bool SafetyRuntime::build_fresh_current_position_hold_locked(
    ArticoreControlMode mode, Clock::time_point now, ArmMailbox& hold,
    uint64_t& maximum_age_ns, std::string& error, void*& limiting_motor) {
  hold = ArmMailbox{};
  hold.valid = true;
  hold.user_command = false;
  hold.trajectory_endpoint_hold = true;
  hold.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
  hold.generation = arm_mailbox_.generation + 1;
  hold.submitted_at = now;
  maximum_age_ns = 0;
  limiting_motor = nullptr;
  const auto max_age =
      static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  for (const auto& motor : motors_) {
    if (motor.descriptor.is_gripper) continue;
    limiting_motor = motor.descriptor.motor;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
        !stats.has_feedback || stats.age_ns > max_age ||
        api_.motor_get_state(motor.descriptor.motor, &state) != 0 ||
        !state.has_value || state.status_code != 1 || !finite(state.pos) ||
        !finite(state.vel) || !finite(state.torq)) {
      error = std::string(motor.descriptor.name) +
          ": complete fresh enabled feedback is required for replacement hold";
      return false;
    }
    const auto& limits = joint_config(motor.descriptor.motor);
    if (state.pos < limits.hard_lower_position ||
        state.pos > limits.hard_upper_position) {
      std::ostringstream detail;
      detail << motor.descriptor.name
             << ": feedback position crossed a hard limit; position="
             << state.pos << " hard_lower=" << limits.hard_lower_position
             << " hard_upper=" << limits.hard_upper_position;
      error = detail.str();
      return false;
    }
    maximum_age_ns = std::max(maximum_age_ns, stats.age_ns);
    if (mode == ARTICORE_MODE_PV) {
      // DM POS_VEL exposes a positive velocity limit rather than a signed
      // target velocity. Holding therefore uses the dedicated low limit while
      // the reference position itself is stationary.
      hold.pv.push_back({motor.descriptor.motor, state.pos,
                         config_.safe_pv_velocity_limit});
    } else {
      hold.mit.push_back({motor.descriptor.motor, state.pos, 0.0f,
                          motor.descriptor.safe_kp,
                          motor.descriptor.safe_kd, 0.0f});
    }
  }
  limiting_motor = nullptr;
  if (hold.pv.empty() && hold.mit.empty()) {
    error = "replacement hold requires at least one active arm motor";
    return false;
  }
  return true;
}

bool SafetyRuntime::transmit_hold_locked(
    const ArmMailbox& hold, ArticoreControlMode mode, std::string& error) {
  const int32_t result = mode == ARTICORE_MODE_PV
      ? api_.group_send_pos_vel(
            controller_group_, hold.pv.data(),
            static_cast<uint32_t>(hold.pv.size()))
      : api_.group_send_mit(
            controller_group_, hold.mit.data(),
            static_cast<uint32_t>(hold.mit.size()));
  if (result != 0) {
    error = motor_error(mode == ARTICORE_MODE_PV
                            ? "replacement PV hold send failed"
                            : "replacement MIT hold send failed");
    return false;
  }
  if (mode == ARTICORE_MODE_PV) {
    last_sent_pv_ = hold.pv;
    last_sent_mit_.clear();
  } else {
    last_sent_mit_ = hold.mit;
    last_sent_pv_.clear();
  }
  return true;
}

void SafetyRuntime::populate_trajectory_start_report_motor(
    ArticoreTrajectoryStartReport& report, void* motor,
    bool feedback_fresh) const {
  if (!motor) return;
  const auto found = std::find_if(
      motors_.begin(), motors_.end(), [&](const MotorRecord& value) {
        return value.descriptor.motor == motor;
      });
  if (found == motors_.end()) return;
  report.limiting_channel = found->descriptor.side;
  copy_text(report.limiting_joint, std::string(found->descriptor.name));
  report.feedback_fresh = feedback_fresh ? 1 : 0;
  ArticoreMotorState state{};
  if (api_.motor_get_state(motor, &state) == 0 && state.has_value) {
    report.limiting_can_id = state.can_id;
    if (finite(state.pos)) report.position = state.pos;
    if (finite(state.vel)) report.velocity = state.vel;
  }
  const auto configured = joint_configs_.find(motor);
  if (configured != joint_configs_.end()) {
    report.soft_lower = configured->second.soft_lower_position;
    report.soft_upper = configured->second.soft_upper_position;
    report.hard_lower = configured->second.hard_lower_position;
    report.hard_upper = configured->second.hard_upper_position;
  }
}

void SafetyRuntime::populate_trajectory_start_report_limit(
    ArticoreTrajectoryStartReport& report,
    const LimitViolation& violation) const {
  populate_trajectory_start_report_motor(report, violation.motor, true);
  report.position = violation.position;
  report.velocity = violation.velocity;
  report.acceleration = violation.acceleration;
  copy_text(report.reason, violation.reason);
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
