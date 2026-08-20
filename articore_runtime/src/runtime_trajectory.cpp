#include "runtime.hpp"
#include "runtime_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace articore {
namespace {

constexpr double kPolynomialTolerance = 1e-10;
constexpr double kLimitTolerance = 1e-6;
constexpr float kStartPositionTolerance = 0.05f;
constexpr float kStartVelocityTolerance = 0.10f;
// Completion is based on physical feedback, not merely on the planned clock.
// Cartesian product motion is PV-only, while the native trajectory engine also
// supports explicit MIT trajectories, whose compliant control needs a wider
// position/velocity window.
constexpr float kPvArrivalPositionTolerance = 0.02f;
constexpr float kPvArrivalVelocityTolerance = 0.05f;
constexpr float kMitArrivalPositionTolerance = 0.05f;
constexpr float kMitArrivalVelocityTolerance = 0.10f;
constexpr uint32_t kArrivalStableFeedbackSamples = 5;
constexpr double kMinimumArrivalTimeoutSeconds = 2.0;
constexpr double kMaximumArrivalTimeoutSeconds = 10.0;

using Polynomial = std::vector<double>;

double evaluate(const Polynomial& coefficients, double x) {
  double result = 0.0;
  for (auto it = coefficients.rbegin(); it != coefficients.rend(); ++it) {
    result = result * x + *it;
  }
  return result;
}

Polynomial derivative(const Polynomial& coefficients) {
  if (coefficients.size() <= 1) return {0.0};
  Polynomial result(coefficients.size() - 1);
  for (std::size_t i = 1; i < coefficients.size(); ++i) {
    result[i - 1] = static_cast<double>(i) * coefficients[i];
  }
  return result;
}

void append_unique(std::vector<double>& roots, double value) {
  if (value < -kPolynomialTolerance || value > 1.0 + kPolynomialTolerance) {
    return;
  }
  value = std::clamp(value, 0.0, 1.0);
  if (std::none_of(roots.begin(), roots.end(), [&](double existing) {
        return std::abs(existing - value) <= 1e-8;
      })) {
    roots.push_back(value);
  }
}

std::vector<double> roots_on_unit_interval(Polynomial coefficients) {
  while (coefficients.size() > 1 &&
         std::abs(coefficients.back()) <= kPolynomialTolerance) {
    coefficients.pop_back();
  }
  const std::size_t degree = coefficients.size() - 1;
  if (degree == 0) return {};
  if (degree == 1) {
    std::vector<double> roots;
    append_unique(roots, -coefficients[0] / coefficients[1]);
    return roots;
  }

  auto critical = roots_on_unit_interval(derivative(coefficients));
  std::sort(critical.begin(), critical.end());
  std::vector<double> boundaries{0.0};
  boundaries.insert(boundaries.end(), critical.begin(), critical.end());
  boundaries.push_back(1.0);

  std::vector<double> roots;
  for (double point : boundaries) {
    if (std::abs(evaluate(coefficients, point)) <= 1e-9) {
      append_unique(roots, point);
    }
  }
  for (std::size_t i = 1; i < boundaries.size(); ++i) {
    double lower = boundaries[i - 1];
    double upper = boundaries[i];
    double lower_value = evaluate(coefficients, lower);
    double upper_value = evaluate(coefficients, upper);
    if (lower_value == 0.0 || upper_value == 0.0 ||
        std::signbit(lower_value) == std::signbit(upper_value)) {
      continue;
    }
    for (int iteration = 0; iteration < 80; ++iteration) {
      const double middle = (lower + upper) * 0.5;
      const double middle_value = evaluate(coefficients, middle);
      if (std::abs(middle_value) <= 1e-13) {
        lower = upper = middle;
        break;
      }
      if (std::signbit(lower_value) == std::signbit(middle_value)) {
        lower = middle;
        lower_value = middle_value;
      } else {
        upper = middle;
        upper_value = middle_value;
      }
    }
    append_unique(roots, (lower + upper) * 0.5);
  }
  std::sort(roots.begin(), roots.end());
  return roots;
}

std::array<double, 6> quintic_coefficients(
    double p0, double v0, double a0,
    double p1, double v1, double a1,
    double duration) {
  const double c0 = p0;
  const double c1 = v0 * duration;
  const double c2 = 0.5 * a0 * duration * duration;
  const double delta_position = p1 - (c0 + c1 + c2);
  const double delta_velocity = v1 * duration - (c1 + 2.0 * c2);
  const double delta_acceleration =
      a1 * duration * duration - 2.0 * c2;
  return {
      c0,
      c1,
      c2,
      10.0 * delta_position - 4.0 * delta_velocity +
          0.5 * delta_acceleration,
      -15.0 * delta_position + 7.0 * delta_velocity -
          delta_acceleration,
      6.0 * delta_position - 3.0 * delta_velocity +
          0.5 * delta_acceleration,
  };
}

Polynomial as_polynomial(const std::array<double, 6>& coefficients) {
  return Polynomial(coefficients.begin(), coefficients.end());
}

void validate_segment_extrema(
    double duration_s,
    const std::vector<std::array<double, 6>>& coefficients,
    const std::vector<NativeTrajectoryJoint>& joints,
    uint32_t segment_index) {
  for (std::size_t joint_index = 0; joint_index < joints.size(); ++joint_index) {
    const auto& joint = joints[joint_index];
    const auto position = as_polynomial(coefficients[joint_index]);
    const auto velocity_u = derivative(position);
    const auto acceleration_u = derivative(velocity_u);
    const auto jerk_u = derivative(acceleration_u);

    std::vector<double> position_candidates{0.0, 1.0};
    const auto position_roots = roots_on_unit_interval(velocity_u);
    position_candidates.insert(position_candidates.end(),
                               position_roots.begin(), position_roots.end());
    for (const double u : position_candidates) {
      const double value = evaluate(position, u);
      if (value < joint.lower_position - kLimitTolerance ||
          value > joint.upper_position + kLimitTolerance) {
        throw std::invalid_argument(
            "trajectory segment " + std::to_string(segment_index) +
            " joint " + std::to_string(joint_index) +
            " exceeds product position limits inside the segment");
      }
    }

    std::vector<double> velocity_candidates{0.0, 1.0};
    const auto velocity_roots = roots_on_unit_interval(acceleration_u);
    velocity_candidates.insert(velocity_candidates.end(),
                               velocity_roots.begin(), velocity_roots.end());
    for (const double u : velocity_candidates) {
      const double value =
          evaluate(velocity_u, u) / duration_s;
      if (std::abs(value) > joint.velocity_limit + kLimitTolerance) {
        throw std::invalid_argument(
            "trajectory segment " + std::to_string(segment_index) +
            " joint " + std::to_string(joint_index) +
            " exceeds product velocity limits inside the segment");
      }
      if (joint.pv_velocity_limit > 0.0f &&
          std::abs(value) > joint.pv_velocity_limit + kLimitTolerance) {
        throw std::invalid_argument(
            "trajectory segment " + std::to_string(segment_index) +
            " joint " + std::to_string(joint_index) +
            " exceeds its PV velocity limit inside the segment");
      }
    }

    std::vector<double> acceleration_candidates{0.0, 1.0};
    const auto acceleration_roots = roots_on_unit_interval(jerk_u);
    acceleration_candidates.insert(acceleration_candidates.end(),
                                   acceleration_roots.begin(),
                                   acceleration_roots.end());
    for (const double u : acceleration_candidates) {
      const double value = evaluate(acceleration_u, u) /
          (duration_s * duration_s);
      if (std::abs(value) > joint.acceleration_limit + kLimitTolerance) {
        throw std::invalid_argument(
            "trajectory segment " + std::to_string(segment_index) +
            " joint " + std::to_string(joint_index) +
            " exceeds product acceleration limits inside the segment");
      }
    }
  }
}

double sample_polynomial(const std::array<double, 6>& coefficients, double u) {
  double value = 0.0;
  for (auto it = coefficients.rbegin(); it != coefficients.rend(); ++it) {
    value = value * u + *it;
  }
  return value;
}

double sample_velocity(const std::array<double, 6>& coefficients,
                       double u, double duration) {
  double value = 0.0;
  for (int index = 5; index >= 1; --index) {
    value = value * u + static_cast<double>(index) * coefficients[index];
  }
  return value / duration;
}

double sample_acceleration(const std::array<double, 6>& coefficients,
                           double u, double duration) {
  double value = 0.0;
  for (int index = 5; index >= 2; --index) {
    value = value * u + static_cast<double>(index * (index - 1)) *
        coefficients[index];
  }
  return value / (duration * duration);
}

}  // namespace

uint64_t SafetyRuntime::start_trajectory(NativeTrajectoryRequest request,
                                         uint64_t replace_trajectory_id,
                                         CommandTransaction* transaction) {
  const bool planned_reference_transaction = transaction != nullptr;
  const std::size_t joint_count = request.joints.size();
  const std::size_t waypoint_count = request.waypoints.size();
  if (request.mode != ARTICORE_MODE_MIT && request.mode != ARTICORE_MODE_PV) {
    throw std::invalid_argument("trajectory control mode is invalid");
  }
  if (joint_count == 0 || joint_count > 32) {
    throw std::invalid_argument("trajectory joint count is invalid");
  }
  if (waypoint_count < 2 ||
      waypoint_count > ARTICORE_MAX_TRAJECTORY_WAYPOINTS) {
    throw std::invalid_argument("trajectory requires 2..10000 waypoints");
  }

  std::set<void*> unique_motors;
  const auto expected_arm_count = static_cast<std::size_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return !motor.descriptor.is_gripper;
      }));
  if (joint_count != expected_arm_count) {
    throw std::invalid_argument(
        "trajectory must cover the complete fixed arm layout");
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    const auto& joint = request.joints[index];
    const bool known_motor = std::any_of(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return !motor.descriptor.is_gripper &&
                 motor.descriptor.motor == joint.motor;
        });
    if (!joint.motor || !known_motor || !unique_motors.insert(joint.motor).second ||
        !finite(joint.direction) || std::abs(joint.direction) != 1.0f ||
        !finite(joint.velocity_command_scale) ||
        joint.velocity_command_scale <= 0.0f ||
        !finite(joint.velocity_feedback_scale) ||
        joint.velocity_feedback_scale <= 0.0f ||
        !finite(joint.torque_command_scale) ||
        joint.torque_command_scale <= 0.0f ||
        !finite(joint.lower_position) || !finite(joint.upper_position) ||
        joint.lower_position >= joint.upper_position ||
        !finite(joint.velocity_limit) || joint.velocity_limit <= 0.0f ||
        !finite(joint.acceleration_limit) ||
        joint.acceleration_limit <= 0.0f || !finite(joint.torque_limit) ||
        joint.torque_limit <= 0.0f) {
      throw std::invalid_argument(
          "trajectory contains invalid joint configuration at index " +
          std::to_string(index));
    }
    if (request.mode == ARTICORE_MODE_MIT) {
      if (!finite(joint.mit_kp) || joint.mit_kp < 0.0f ||
          joint.mit_kp > 500.0f || !finite(joint.mit_kd) ||
          joint.mit_kd < 0.0f || joint.mit_kd > 5.0f ||
          !finite(joint.mit_feedforward_torque) ||
          std::abs(joint.mit_feedforward_torque) >
              joint.torque_limit) {
        throw std::invalid_argument(
            "trajectory contains invalid MIT configuration at joint " +
            std::to_string(index));
      }
    } else if (!finite(joint.pv_velocity_limit) ||
               joint.pv_velocity_limit <= 0.0f ||
               joint.pv_velocity_limit > joint.velocity_limit) {
      throw std::invalid_argument(
          "trajectory contains invalid PV velocity limit at joint " +
          std::to_string(index));
    }
  }

  const uint32_t valid_joint_bits = joint_count == 32
      ? std::numeric_limits<uint32_t>::max()
      : (uint32_t{1} << joint_count) - 1U;
  const double time_origin = request.waypoints.front().time_s;
  if (!std::isfinite(time_origin) || time_origin < 0.0) {
    throw std::invalid_argument(
        "trajectory first timestamp must be finite and non-negative");
  }
  for (std::size_t waypoint_index = 0;
       waypoint_index < waypoint_count; ++waypoint_index) {
    auto& waypoint = request.waypoints[waypoint_index];
    if (!std::isfinite(waypoint.time_s) ||
        (waypoint_index > 0 &&
         waypoint.time_s <= request.waypoints[waypoint_index - 1].time_s)) {
      throw std::invalid_argument(
          "trajectory timestamps must be finite and strictly increasing");
    }
    if ((waypoint.velocity_valid_mask & ~valid_joint_bits) != 0 ||
        (waypoint.acceleration_valid_mask & ~valid_joint_bits) != 0) {
      throw std::invalid_argument(
          "trajectory derivative mask contains an unknown joint bit");
    }
    if (waypoint.positions.size() != joint_count ||
        waypoint.velocities.size() != joint_count ||
        waypoint.accelerations.size() != joint_count) {
      throw std::invalid_argument(
          "trajectory waypoint does not match the arm joint count");
    }
    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      const auto& joint = request.joints[joint_index];
      const float position = waypoint.positions[joint_index];
      if (!finite(position) || position < joint.lower_position ||
          position > joint.upper_position) {
        throw std::invalid_argument(
            "trajectory waypoint exceeds product position limits at joint " +
            std::to_string(joint_index));
      }
      const uint32_t bit = uint32_t{1} << joint_index;
      if ((waypoint.velocity_valid_mask & bit) != 0 &&
          (!finite(waypoint.velocities[joint_index]) ||
           std::abs(waypoint.velocities[joint_index]) > joint.velocity_limit)) {
        throw std::invalid_argument(
            "trajectory waypoint velocity is invalid at joint " +
            std::to_string(joint_index));
      }
      if ((waypoint.acceleration_valid_mask & bit) != 0 &&
          (!finite(waypoint.accelerations[joint_index]) ||
           std::abs(waypoint.accelerations[joint_index]) >
               joint.acceleration_limit)) {
        throw std::invalid_argument(
            "trajectory waypoint acceleration is invalid at joint " +
            std::to_string(joint_index));
      }
    }
  }
  for (auto& waypoint : request.waypoints) waypoint.time_s -= time_origin;
  const double duration_s = request.waypoints.back().time_s;
  if (!std::isfinite(duration_s) || duration_s <= 0.0 || duration_s > 3600.0) {
    throw std::invalid_argument(
        "trajectory duration must be positive and no greater than one hour");
  }

  for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
    const uint32_t bit = uint32_t{1} << joint_index;
    for (std::size_t waypoint_index = 0;
         waypoint_index < waypoint_count; ++waypoint_index) {
      auto& waypoint = request.waypoints[waypoint_index];
      if ((waypoint.velocity_valid_mask & bit) == 0) {
        if (waypoint_index == 0 || waypoint_index + 1 == waypoint_count) {
          waypoint.velocities[joint_index] = 0.0f;
        } else {
          const auto& previous = request.waypoints[waypoint_index - 1];
          const auto& next = request.waypoints[waypoint_index + 1];
          waypoint.velocities[joint_index] = static_cast<float>(
              (next.positions[joint_index] - previous.positions[joint_index]) /
              (next.time_s - previous.time_s));
        }
      }
      if ((waypoint.acceleration_valid_mask & bit) == 0) {
        if (waypoint_index == 0 || waypoint_index + 1 == waypoint_count) {
          waypoint.accelerations[joint_index] = 0.0f;
        } else {
          const auto& previous = request.waypoints[waypoint_index - 1];
          const auto& next = request.waypoints[waypoint_index + 1];
          const double previous_slope =
              (waypoint.positions[joint_index] -
               previous.positions[joint_index]) /
              (waypoint.time_s - previous.time_s);
          const double next_slope =
              (next.positions[joint_index] -
               waypoint.positions[joint_index]) /
              (next.time_s - waypoint.time_s);
          waypoint.accelerations[joint_index] = static_cast<float>(
              2.0 * (next_slope - previous_slope) /
              (next.time_s - previous.time_s));
        }
      }
    }
  }

  std::vector<TrajectorySegment> segments;
  segments.reserve(waypoint_count - 1);
  for (std::size_t segment_index = 0;
       segment_index + 1 < waypoint_count; ++segment_index) {
    const auto& start = request.waypoints[segment_index];
    const auto& end = request.waypoints[segment_index + 1];
    TrajectorySegment segment;
    segment.start_s = start.time_s;
    segment.duration_s = end.time_s - start.time_s;
    segment.coefficients.reserve(joint_count);
    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      segment.coefficients.push_back(quintic_coefficients(
          start.positions[joint_index], start.velocities[joint_index],
          start.accelerations[joint_index], end.positions[joint_index],
          end.velocities[joint_index], end.accelerations[joint_index],
          segment.duration_s));
    }
    validate_segment_extrema(segment.duration_s, segment.coefficients,
                             request.joints,
                             static_cast<uint32_t>(segment_index));
    segments.push_back(std::move(segment));
  }

  CommandTransaction owned_transaction;
  if (transaction) {
    if (!transaction->owns_lock() || transaction->mutex() != &command_mutex_) {
      throw std::logic_error("invalid Runtime command transaction");
    }
  } else {
    owned_transaction = begin_command_transaction();
  }
  std::set<void*> intentionally_disabled;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command(false, replace_trajectory_id != 0);
    if (state_ == ARTICORE_DEGRADED) {
      throw std::runtime_error(
          "cannot start a trajectory while Runtime is degraded");
    }
    if (mode_ != request.mode) {
      throw std::runtime_error(
          "trajectory mode does not match the active Runtime mode");
    }
    if (trajectory_control_.state == ARTICORE_TRAJECTORY_RUNNING &&
        (replace_trajectory_id == 0 ||
         trajectory_control_.id != replace_trajectory_id)) {
      throw std::runtime_error("a trajectory is already running");
    }
    if (replace_trajectory_id != 0 &&
        (trajectory_control_.state != ARTICORE_TRAJECTORY_RUNNING ||
         trajectory_control_.id != replace_trajectory_id)) {
      throw std::runtime_error(
          "trajectory changed while preparing its replacement");
    }
    intentionally_disabled = intentionally_disabled_motors_;
  }

  for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
    const auto& joint = request.joints[joint_index];
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(joint.motor, &stats) != 0 ||
        !stats.has_feedback || stats.age_ns > feedback_max_age_ns() ||
        api_.motor_get_state(joint.motor, &state) != 0 || !state.has_value ||
        !finite(state.pos) || !finite(state.vel) || state.status_code > 1) {
      throw std::runtime_error(
          "trajectory start requires fresh fault-free feedback at joint " +
          std::to_string(joint_index));
    }
    const bool disabled = intentionally_disabled.count(joint.motor) != 0;
    if ((!disabled && state.status_code != 1) ||
        (disabled && state.status_code != 0)) {
      throw std::runtime_error(
          "trajectory start power feedback disagrees with Runtime at joint " +
          std::to_string(joint_index));
    }
    if (replace_trajectory_id == 0 && !planned_reference_transaction) {
      const float requested_start =
          joint.direction * request.waypoints.front().positions[joint_index];
      if (std::abs(state.pos - requested_start) > kStartPositionTolerance) {
        throw std::invalid_argument(
            "trajectory first waypoint is not synchronized with current "
            "feedback at joint " + std::to_string(joint_index));
      }
      const float actual_velocity =
          joint.direction * state.vel * joint.velocity_feedback_scale;
      const float requested_velocity =
          request.waypoints.front().velocities[joint_index];
      if (std::abs(actual_velocity - requested_velocity) >
          kStartVelocityTolerance) {
        throw std::invalid_argument(
            "trajectory first waypoint velocity is not synchronized with "
            "current feedback at joint " + std::to_string(joint_index));
      }
    }
  }

  const auto now = Clock::now();
  uint64_t id = 0;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command(false, replace_trajectory_id != 0);
    if (mode_ != request.mode ||
        (trajectory_control_.state == ARTICORE_TRAJECTORY_RUNNING &&
         (replace_trajectory_id == 0 ||
          trajectory_control_.id != replace_trajectory_id)) ||
        (replace_trajectory_id != 0 &&
         (trajectory_control_.state != ARTICORE_TRAJECTORY_RUNNING ||
          trajectory_control_.id != replace_trajectory_id))) {
      throw std::runtime_error(
          "Runtime state changed while starting trajectory");
    }
    clear_pending_arm_mailbox();
    arm_mailbox_ = ArmMailbox{};
    trajectory_control_ = TrajectoryControl{};
    trajectory_control_.state = ARTICORE_TRAJECTORY_RUNNING;
    trajectory_control_.id = next_trajectory_id_++;
    trajectory_control_.waypoint_count =
        static_cast<uint32_t>(waypoint_count);
    trajectory_control_.duration_s = duration_s;
    trajectory_control_.started_at = now;
    trajectory_control_.operation = request.operation;
    trajectory_control_.joints = std::move(request.joints);
    trajectory_control_.segments = std::move(segments);
    id = trajectory_control_.id;
    next_control_tick_ = now;
  }
  wakeup_.notify_all();
  return id;
}

NativeTrajectorySample SafetyRuntime::trajectory_sample() const {
  const auto now = Clock::now();
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return trajectory_sample_locked(now);
}

SafetyRuntime::CommandTransaction SafetyRuntime::begin_command_transaction() {
  return CommandTransaction(command_mutex_);
}

NativeTrajectorySample SafetyRuntime::trajectory_sample_locked(
    Clock::time_point now) const {
  NativeTrajectorySample result;
  if (trajectory_control_.state != ARTICORE_TRAJECTORY_RUNNING ||
      trajectory_control_.segments.empty() ||
      trajectory_control_.joints.empty()) {
    return result;
  }

  double elapsed = std::chrono::duration<double>(
      now - trajectory_control_.started_at).count();
  elapsed = std::clamp(elapsed, 0.0, trajectory_control_.duration_s);
  std::size_t segment_index = trajectory_control_.active_segment;
  while (segment_index + 1 < trajectory_control_.segments.size() &&
         elapsed >= trajectory_control_.segments[segment_index].start_s +
                        trajectory_control_.segments[segment_index].duration_s) {
    ++segment_index;
  }
  const auto& segment = trajectory_control_.segments[segment_index];
  const double local = std::clamp(
      elapsed - segment.start_s, 0.0, segment.duration_s);
  const double u = local / segment.duration_s;

  result.active = true;
  result.trajectory_id = trajectory_control_.id;
  result.operation = trajectory_control_.operation;
  result.positions.reserve(segment.coefficients.size());
  result.velocities.reserve(segment.coefficients.size());
  result.accelerations.reserve(segment.coefficients.size());
  for (const auto& coefficients : segment.coefficients) {
    result.positions.push_back(static_cast<float>(
        sample_polynomial(coefficients, u)));
    result.velocities.push_back(static_cast<float>(
        sample_velocity(coefficients, u, segment.duration_s)));
    result.accelerations.push_back(static_cast<float>(
        sample_acceleration(coefficients, u, segment.duration_s)));
  }
  return result;
}

NativeTrajectorySample SafetyRuntime::planned_arm_sample(
    const std::vector<NativeTrajectoryJoint>& joints,
    const CommandTransaction& transaction) const {
  if (!transaction.owns_lock() || transaction.mutex() != &command_mutex_) {
    throw std::logic_error(
        "planned arm reference requires the Runtime command transaction");
  }
  if (joints.empty()) {
    throw std::invalid_argument("planned arm reference has no joints");
  }

  {
    const auto now = Clock::now();
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    auto active = trajectory_sample_locked(now);
    if (active.active) return active;
  }

  if (!arm_mailbox_.valid) {
    throw std::runtime_error(
        "current planned arm reference is unavailable");
  }

  NativeTrajectorySample result;
  result.positions.reserve(joints.size());
  result.velocities.reserve(joints.size());
  result.accelerations.assign(joints.size(), 0.0f);
  for (std::size_t index = 0; index < joints.size(); ++index) {
    const auto& joint = joints[index];
    if (!joint.motor || !finite(joint.direction) ||
        std::abs(joint.direction) != 1.0f) {
      throw std::invalid_argument(
          "planned arm reference contains an invalid joint at index " +
          std::to_string(index));
    }
    if (!arm_mailbox_.pv.empty()) {
      const auto found = std::find_if(
          arm_mailbox_.pv.begin(), arm_mailbox_.pv.end(),
          [&](const ArticorePosVelCommand& command) {
            return command.motor == joint.motor;
          });
      if (found == arm_mailbox_.pv.end()) {
        throw std::runtime_error(
            "current planned PV reference is incomplete at joint " +
            std::to_string(index));
      }
      result.positions.push_back(joint.direction * found->target_position);
      result.velocities.push_back(0.0f);
      continue;
    }
    const auto found = std::find_if(
        arm_mailbox_.mit.begin(), arm_mailbox_.mit.end(),
        [&](const ArticoreMitCommand& command) {
          return command.motor == joint.motor;
        });
    if (found == arm_mailbox_.mit.end() ||
        !finite(joint.velocity_command_scale) ||
        joint.velocity_command_scale <= 0.0f) {
      throw std::runtime_error(
          "current planned MIT reference is incomplete at joint " +
          std::to_string(index));
    }
    result.positions.push_back(joint.direction * found->target_position);
    result.velocities.push_back(
        joint.direction * found->target_velocity /
        joint.velocity_command_scale);
  }
  return result;
}

bool SafetyRuntime::prepare_trajectory_cycle(
    Clock::time_point now, bool& completing, std::string& error) {
  completing = false;
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (trajectory_control_.state != ARTICORE_TRAJECTORY_RUNNING) return false;
  if (trajectory_control_.segments.empty() ||
      trajectory_control_.joints.empty()) {
    error = "trajectory plan is internally empty";
    return false;
  }

  double elapsed = std::chrono::duration<double>(
      now - trajectory_control_.started_at).count();
  elapsed = std::clamp(elapsed, 0.0, trajectory_control_.duration_s);
  std::size_t segment_index = trajectory_control_.active_segment;
  while (segment_index + 1 < trajectory_control_.segments.size() &&
         elapsed >= trajectory_control_.segments[segment_index].start_s +
                        trajectory_control_.segments[segment_index].duration_s) {
    ++segment_index;
  }
  auto& segment = trajectory_control_.segments[segment_index];
  const double local = std::clamp(
      elapsed - segment.start_s, 0.0, segment.duration_s);
  const double u = local / segment.duration_s;

  arm_mailbox_.valid = true;
  arm_mailbox_.user_command = true;
  arm_mailbox_.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
  arm_mailbox_.generation = next_arm_generation();
  arm_mailbox_.submitted_at = now;
  arm_mailbox_.joint_position = false;
  arm_mailbox_.final_positions.clear();
  if (mode_ == ARTICORE_MODE_MIT) {
    arm_mailbox_.pv.clear();
    arm_mailbox_.mit.resize(trajectory_control_.joints.size());
  } else {
    arm_mailbox_.mit.clear();
    arm_mailbox_.pv.resize(trajectory_control_.joints.size());
  }
  for (std::size_t joint_index = 0;
       joint_index < trajectory_control_.joints.size(); ++joint_index) {
    const auto& joint = trajectory_control_.joints[joint_index];
    const auto& coefficients = segment.coefficients[joint_index];
    const float position = static_cast<float>(
        sample_polynomial(coefficients, u));
    const float velocity = static_cast<float>(
        sample_velocity(coefficients, u, segment.duration_s));
    if (!finite(position) || !finite(velocity)) {
      error = "trajectory produced a non-finite sample";
      return false;
    }
    if (mode_ == ARTICORE_MODE_MIT) {
      arm_mailbox_.mit[joint_index] = ArticoreMitCommand{
          joint.motor,
          joint.direction * position,
          joint.direction * velocity * joint.velocity_command_scale,
          joint.mit_kp,
          joint.mit_kd,
          joint.direction * joint.mit_feedforward_torque *
              joint.torque_command_scale};
    } else {
      arm_mailbox_.pv[joint_index] = ArticorePosVelCommand{
          joint.motor, joint.direction * position, joint.pv_velocity_limit};
    }
  }

  trajectory_control_.active_segment = static_cast<uint32_t>(segment_index);
  trajectory_control_.elapsed_s = elapsed;
  completing = elapsed >= trajectory_control_.duration_s;
  return true;
}

void SafetyRuntime::update_trajectory_completion(Clock::time_point now) {
  struct ArrivalJoint {
    NativeTrajectoryJoint joint;
    float target = 0.0f;
    bool intentionally_disabled = false;
    std::string role;
  };

  uint64_t trajectory_id = 0;
  ArticoreControlMode mode = ARTICORE_MODE_PV;
  Clock::time_point settling_started_at{};
  std::vector<uint64_t> previous_updates;
  bool feedback_initialized = false;
  std::vector<ArrivalJoint> arrivals;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (trajectory_control_.state != ARTICORE_TRAJECTORY_RUNNING ||
        trajectory_control_.segments.empty()) {
      return;
    }
    if (trajectory_control_.settling_started_at == Clock::time_point{}) {
      trajectory_control_.settling_started_at = now;
      trajectory_control_.settling_feedback_updates.assign(
          trajectory_control_.joints.size(), 0);
      trajectory_control_.settled_feedback_samples = 0;
      trajectory_control_.settling_feedback_initialized = false;
    }
    trajectory_id = trajectory_control_.id;
    mode = mode_;
    settling_started_at = trajectory_control_.settling_started_at;
    previous_updates = trajectory_control_.settling_feedback_updates;
    feedback_initialized =
        trajectory_control_.settling_feedback_initialized;
    const auto& final_coefficients =
        trajectory_control_.segments.back().coefficients;
    arrivals.reserve(trajectory_control_.joints.size());
    for (std::size_t index = 0;
         index < trajectory_control_.joints.size(); ++index) {
      const auto& joint = trajectory_control_.joints[index];
      ArrivalJoint arrival;
      arrival.joint = joint;
      arrival.target = static_cast<float>(
          sample_polynomial(final_coefficients[index], 1.0));
      arrival.intentionally_disabled =
          intentionally_disabled_motors_.count(joint.motor) != 0;
      const auto role = motor_roles_.find(joint.motor);
      arrival.role = role == motor_roles_.end()
          ? "joint " + std::to_string(index)
          : role->second;
      arrivals.push_back(std::move(arrival));
    }
  }

  const float position_tolerance = mode == ARTICORE_MODE_PV
      ? kPvArrivalPositionTolerance : kMitArrivalPositionTolerance;
  const float velocity_tolerance = mode == ARTICORE_MODE_PV
      ? kPvArrivalVelocityTolerance : kMitArrivalVelocityTolerance;
  bool all_arrived = true;
  bool all_feedback_new = true;
  bool has_active_joint = false;
  std::vector<uint64_t> current_updates(arrivals.size(), 0);
  float worst_position_error = 0.0f;
  float worst_velocity = 0.0f;
  std::string worst_role;
  float worst_target = 0.0f;
  float worst_actual = 0.0f;
  std::string unavailable_role;

  for (std::size_t index = 0; index < arrivals.size(); ++index) {
    const auto& arrival = arrivals[index];
    if (arrival.intentionally_disabled) continue;
    has_active_joint = true;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool available =
        api_.motor_get_feedback_stats(arrival.joint.motor, &stats) == 0 &&
        stats.has_feedback && stats.age_ns <= feedback_max_age_ns() &&
        api_.motor_get_state(arrival.joint.motor, &state) == 0 &&
        state.has_value && state.status_code == 1 && finite(state.pos) &&
        finite(state.vel);
    if (!available) {
      all_arrived = false;
      all_feedback_new = false;
      if (unavailable_role.empty()) unavailable_role = arrival.role;
      continue;
    }
    current_updates[index] = stats.update_count;
    if (feedback_initialized &&
        index < previous_updates.size() &&
        stats.update_count == previous_updates[index]) {
      all_feedback_new = false;
    }
    const float actual = arrival.joint.direction * state.pos;
    const float velocity = arrival.joint.direction * state.vel *
        arrival.joint.velocity_feedback_scale;
    const float position_error = std::abs(actual - arrival.target);
    const float speed = std::abs(velocity);
    if (position_error > position_tolerance ||
        speed > velocity_tolerance) {
      all_arrived = false;
    }
    if (position_error > worst_position_error ||
        (position_error == worst_position_error && speed > worst_velocity)) {
      worst_position_error = position_error;
      worst_velocity = speed;
      worst_role = arrival.role;
      worst_target = arrival.target;
      worst_actual = actual;
    }
  }

  bool completed = false;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (trajectory_control_.state != ARTICORE_TRAJECTORY_RUNNING ||
        trajectory_control_.id != trajectory_id) {
      return;
    }
    trajectory_control_.elapsed_s = trajectory_control_.duration_s;
    trajectory_control_.active_segment =
        static_cast<uint32_t>(trajectory_control_.segments.size() - 1);

    if (!has_active_joint) {
      completed = true;
    } else {
      if (!feedback_initialized || all_feedback_new) {
        trajectory_control_.settling_feedback_updates = current_updates;
        trajectory_control_.settling_feedback_initialized = true;
      }
      if (all_arrived && (!feedback_initialized || all_feedback_new)) {
        ++trajectory_control_.settled_feedback_samples;
      } else if (!all_arrived) {
        trajectory_control_.settled_feedback_samples = 0;
      }
      completed = trajectory_control_.settled_feedback_samples >=
          kArrivalStableFeedbackSamples;
    }

    if (completed) {
      trajectory_control_.state = ARTICORE_TRAJECTORY_COMPLETED;
      trajectory_control_.error.clear();
      return;
    }

    const double arrival_timeout_s = std::clamp(
        trajectory_control_.duration_s * 0.25,
        kMinimumArrivalTimeoutSeconds, kMaximumArrivalTimeoutSeconds);
    if (std::chrono::duration<double>(
            now - settling_started_at).count() >= arrival_timeout_s) {
      std::ostringstream stream;
      stream << "trajectory arrival timed out after " << arrival_timeout_s
             << " s";
      if (!unavailable_role.empty()) {
        stream << "; fresh enabled feedback unavailable for "
               << unavailable_role;
      } else if (!worst_role.empty()) {
        stream << "; " << worst_role
               << " target=" << worst_target
               << " actual=" << worst_actual
               << " position_error=" << worst_position_error
               << " velocity=" << worst_velocity
               << " tolerances=[position<=" << position_tolerance
               << ", velocity<=" << velocity_tolerance << "]";
      }
      const auto timeout_error = stream.str();
      terminate_trajectory_locked(
          ARTICORE_TRAJECTORY_FAULT, timeout_error);
      last_operation_ = trajectory_control_.operation;
      last_operation_code_ = ARTICORE_OPERATION_FEEDBACK;
      last_operation_error_ = timeout_error;
      operation_failed_motors_.clear();
      if (!unavailable_role.empty()) {
        operation_failed_motors_.push_back(unavailable_role);
      } else if (!worst_role.empty()) {
        operation_failed_motors_.push_back(worst_role);
      }
    }
  }
}

void SafetyRuntime::terminate_trajectory_locked(
    ArticoreTrajectoryState state, const std::string& error) {
  if (trajectory_control_.state != ARTICORE_TRAJECTORY_RUNNING) return;
  trajectory_control_.state = state;
  trajectory_control_.error = error;
  if (state == ARTICORE_TRAJECTORY_FAULT) {
    last_operation_ = trajectory_control_.operation;
    last_operation_code_ = ARTICORE_OPERATION_FEEDBACK;
    last_operation_error_ = error;
    operation_failed_motors_.clear();
  }
  // A cancelled trajectory becomes a stationary hold until the caller
  // replaces it or a lifecycle operation clears the mailbox. Never preserve
  // an in-flight MIT velocity/feedforward term as a persistent setpoint.
  if (arm_mailbox_.valid) {
    arm_mailbox_.user_command = true;
    arm_mailbox_.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
    for (auto& command : arm_mailbox_.mit) {
      command.target_velocity = 0.0f;
      command.feedforward_torque = 0.0f;
    }
  }
}

void SafetyRuntime::fault_trajectory(const std::string& error) {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  terminate_trajectory_locked(ARTICORE_TRAJECTORY_FAULT, error);
  last_operation_ = trajectory_control_.operation;
  last_operation_code_ = ARTICORE_OPERATION_MOTOR_COMMAND;
  last_operation_error_ = error;
  operation_failed_motors_.clear();
}

void SafetyRuntime::cancel_trajectory() {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  terminate_trajectory_locked(
      ARTICORE_TRAJECTORY_CANCELLED, "trajectory cancelled");
}

ArticoreTrajectoryStatus SafetyRuntime::trajectory_status() const {
  ArticoreTrajectoryStatus result{};
  result.struct_size = sizeof(result);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  result.state = trajectory_control_.state;
  result.trajectory_id = trajectory_control_.id;
  result.active_segment = trajectory_control_.active_segment;
  result.waypoint_count = trajectory_control_.waypoint_count;
  result.elapsed_s = trajectory_control_.elapsed_s;
  result.duration_s = trajectory_control_.duration_s;
  result.progress = trajectory_control_.duration_s > 0.0
      ? static_cast<float>(std::clamp(
            trajectory_control_.elapsed_s / trajectory_control_.duration_s,
            0.0, 1.0))
      : 0.0f;
  detail::copy_text(result.error, trajectory_control_.error);
  return result;
}

}  // namespace articore
