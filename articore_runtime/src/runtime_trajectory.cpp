#include "articore/detail/runtime.hpp"
#include "articore/detail/runtime_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
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
// Do not freeze the drive at zero speed merely because it entered the public
// 0.02 rad arrival window. First let the low-speed settling phase converge to
// a tighter target error, then install the stationary hold.
constexpr float kPvFinalPositionTolerance = 0.005f;
constexpr float kPvLoadedJointFinalPositionTolerance = 0.012f;
// Arrival accepts <=0.02 rad absolute error, but a completed PV hold must
// remain inside a tighter 0.012 rad peak-to-peak window.
// This rejects the reported ~0.0145 rad visible oscillation without treating
// one quantized feedback step as instability.
constexpr float kPvStablePositionRange = 0.012f;
constexpr float kMitArrivalPositionTolerance = 0.05f;
constexpr float kMitArrivalVelocityTolerance = 0.10f;
constexpr auto kArrivalStableDuration = std::chrono::milliseconds(200);
constexpr auto kCompletedHoldViolationDuration =
    std::chrono::milliseconds(50);
constexpr double kMinimumArrivalTimeoutSeconds = 4.0;
constexpr double kMaximumArrivalTimeoutSeconds = 10.0;

int32_t motion_type(ArticoreRuntimeOperation operation) {
  switch (operation) {
    case ARTICORE_OPERATION_START_TRAJECTORY:
      return ARTICORE_MOTION_JOINT_TRAJECTORY;
    case ARTICORE_OPERATION_MOVE_LINEAR:
      return ARTICORE_MOTION_CARTESIAN_LINEAR;
    case ARTICORE_OPERATION_MOVE_CIRCULAR:
      return ARTICORE_MOTION_CARTESIAN_CIRCULAR;
    default:
      return 0;
  }
}

bool is_loaded_joint4_role(const std::string& role) {
  constexpr std::string_view suffix = "joint4";
  return role.size() >= suffix.size() &&
      role.compare(role.size() - suffix.size(), suffix.size(), suffix) == 0;
}

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

std::array<double, 6> sampled_pv_coefficients(double p0, double p1) {
  return {p0, p1 - p0, 0.0, 0.0, 0.0, 0.0};
}

Polynomial as_polynomial(const std::array<double, 6>& coefficients) {
  return Polynomial(coefficients.begin(), coefficients.end());
}

std::string joint_role(const NativeTrajectoryJoint& joint) {
  return joint.role.empty() ? "unknown product joint" : joint.role;
}

std::string waypoint_context(std::size_t index, std::size_t count) {
  if (index == 0) return "trajectory start waypoint";
  if (index + 1 == count) return "trajectory target waypoint";
  return "trajectory waypoint " + std::to_string(index + 1);
}

std::string position_limit_error(
    const std::string& context, const NativeTrajectoryJoint& joint,
    double position) {
  std::ostringstream message;
  message << context << " exceeds product position limits: "
          << joint_role(joint) << " position=" << position
          << " rad, allowed=[" << joint.lower_position << ", "
          << joint.upper_position << "] rad";
  return message.str();
}

void validate_segment_extrema(
    double duration_s,
    const std::vector<std::array<double, 6>>& coefficients,
    const std::vector<NativeTrajectoryJoint>& joints,
    const std::vector<int8_t>& recovery_directions,
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
    const double segment_start = evaluate(position, 0.0);
    const bool recovering_below = recovery_directions[joint_index] > 0 &&
        segment_start < joint.lower_position;
    const bool recovering_above = recovery_directions[joint_index] < 0 &&
        segment_start > joint.upper_position;
    const double allowed_lower = recovering_below
        ? segment_start : joint.lower_position;
    const double allowed_upper = recovering_above
        ? segment_start : joint.upper_position;
    for (const double u : position_candidates) {
      const double value = evaluate(position, u);
      if (value < allowed_lower - kLimitTolerance ||
          value > allowed_upper + kLimitTolerance) {
        std::ostringstream message;
        message << "trajectory segment " << segment_index + 1;
        if (recovering_below || recovering_above) {
          message << " expands out-of-limit recovery: ";
        } else {
          message << " exceeds product position limits: ";
        }
        message << joint_role(joint) << " position=" << value
                << " rad, allowed=[" << allowed_lower << ", "
                << allowed_upper << "] rad inside the segment";
        throw std::invalid_argument(message.str());
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
        std::ostringstream message;
        message << "trajectory segment " << segment_index + 1 << ": "
                << joint_role(joint) << " velocity=" << value
                << " rad/s exceeds product limit=" << joint.velocity_limit
                << " rad/s inside the segment";
        throw std::invalid_argument(message.str());
      }
      if (joint.pv_velocity_limit > 0.0f &&
          std::abs(value) > joint.pv_velocity_limit + kLimitTolerance) {
        std::ostringstream message;
        message << "trajectory segment " << segment_index + 1 << ": "
                << joint_role(joint) << " velocity=" << value
                << " rad/s exceeds PV limit=" << joint.pv_velocity_limit
                << " rad/s inside the segment";
        throw std::invalid_argument(message.str());
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
        std::ostringstream message;
        message << "trajectory segment " << segment_index + 1 << ": "
                << joint_role(joint) << " acceleration=" << value
                << " rad/s^2 exceeds product limit="
                << joint.acceleration_limit
                << " rad/s^2 inside the segment";
        throw std::invalid_argument(message.str());
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
                                         uint64_t replace_motion_id,
                                         CommandTransaction* transaction,
                                         bool enqueue,
                                         uint64_t planning_token) {
  const bool planned_reference_transaction = transaction != nullptr;
  if (planning_token != 0 && transaction == nullptr) {
    throw std::logic_error(
        "planned trajectory token requires a command transaction");
  }
  if (enqueue && replace_motion_id != 0) {
    throw std::invalid_argument(
        "a queued trajectory cannot replace a running trajectory");
  }
  const std::size_t joint_count = request.joints.size();
  const std::size_t waypoint_count = request.waypoints.size();
  if (request.mode != ARTICORE_MODE_MIT && request.mode != ARTICORE_MODE_PV) {
    throw std::invalid_argument("trajectory control mode is invalid");
  }
  if (request.execution == NativeTrajectoryExecution::SampledPv &&
      request.mode != ARTICORE_MODE_PV) {
    throw std::invalid_argument(
        "PV trajectory execution requires PV control mode");
  }
  if (joint_count == 0 || joint_count > 32) {
    throw std::invalid_argument("trajectory joint count is invalid");
  }
  if (waypoint_count < 2 ||
      waypoint_count > ARTICORE_MAX_TRAJECTORY_WAYPOINTS) {
    throw std::invalid_argument("trajectory requires 2..10000 waypoints");
  }
  if (request.approach_segment_count >= waypoint_count) {
    throw std::invalid_argument(
        "trajectory approach segment count is invalid");
  }
  if (request.approach_segment_count != 0 &&
      request.mode != ARTICORE_MODE_PV) {
    throw std::invalid_argument(
        "trajectory approach requires PV control mode");
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
    auto& joint = request.joints[index];
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
          "trajectory contains invalid joint configuration for " +
          joint_role(joint));
    }
    if (request.mode == ARTICORE_MODE_MIT) {
      if (!finite(joint.mit_kp) || joint.mit_kp < 0.0f ||
          joint.mit_kp > 500.0f || !finite(joint.mit_kd) ||
          joint.mit_kd < 0.0f || joint.mit_kd > 5.0f ||
          !finite(joint.mit_feedforward_torque) ||
          std::abs(joint.mit_feedforward_torque) >
              joint.torque_limit) {
        throw std::invalid_argument(
            "trajectory contains invalid MIT configuration for " +
            joint_role(joint));
      }
    } else {
      if (!finite(joint.pv_velocity_limit) ||
          joint.pv_velocity_limit <= 0.0f ||
          joint.pv_velocity_limit > joint.velocity_limit) {
        throw std::invalid_argument(
            "trajectory contains invalid PV velocity limit for " +
            joint_role(joint));
      }
      if (!finite(joint.pv_hold_velocity_limit) ||
          joint.pv_hold_velocity_limit < 0.0f ||
          joint.pv_hold_velocity_limit > joint.pv_velocity_limit) {
        throw std::invalid_argument(
            "trajectory contains invalid PV final-hold velocity limit for " +
            joint_role(joint));
      }
    }
  }

  const uint32_t valid_joint_bits = joint_count == 32
      ? std::numeric_limits<uint32_t>::max()
      : (uint32_t{1} << joint_count) - 1U;
  std::vector<int8_t> recovery_directions(joint_count, 0);
  std::vector<bool> recovered(joint_count, false);
  std::vector<float> recovery_start_positions(
      joint_count, std::numeric_limits<float>::quiet_NaN());
  std::vector<float> previous_positions(
      joint_count, std::numeric_limits<float>::quiet_NaN());
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
      const auto context = waypoint_context(waypoint_index, waypoint_count);
      if (!finite(position)) {
        throw std::invalid_argument(
            context + " contains a non-finite position for " +
            joint_role(joint));
      }
      const bool below = position < joint.lower_position;
      const bool above = position > joint.upper_position;
      if (waypoint_index == 0 && (below || above)) {
        if (!request.allow_out_of_limit_start_recovery) {
          throw std::invalid_argument(
              position_limit_error(context, joint, position));
        }
        recovery_directions[joint_index] = below ? 1 : -1;
        recovery_start_positions[joint_index] = position;
      } else if (below || above) {
        const bool final_waypoint = waypoint_index + 1 == waypoint_count;
        const bool can_continue_recovery =
            request.allow_out_of_limit_start_recovery &&
            recovery_directions[joint_index] != 0 &&
            !recovered[joint_index] && !final_waypoint;
        const float previous = previous_positions[joint_index];
        const bool moves_inward = recovery_directions[joint_index] > 0
            ? position >= previous - static_cast<float>(kLimitTolerance)
            : position <= previous + static_cast<float>(kLimitTolerance);
        if (!can_continue_recovery || !moves_inward) {
          std::ostringstream message;
          message << context;
          if (recovery_directions[joint_index] != 0) {
            message << " cannot recover an out-of-limit start: ";
          } else {
            message << " exceeds product position limits: ";
          }
          message << joint_role(joint) << " position=" << position;
          if (recovery_directions[joint_index] != 0) {
            message << " rad, start_position="
                    << recovery_start_positions[joint_index]
                    << " rad, previous_position=" << previous;
          }
          message << " rad, allowed=[" << joint.lower_position << ", "
                  << joint.upper_position << "] rad";
          throw std::invalid_argument(message.str());
        }
      } else if (recovery_directions[joint_index] != 0 &&
                 waypoint_index > 0) {
        recovered[joint_index] = true;
      }
      previous_positions[joint_index] = position;
      const uint32_t bit = uint32_t{1} << joint_index;
      if ((waypoint.velocity_valid_mask & bit) != 0 &&
          (!finite(waypoint.velocities[joint_index]) ||
           std::abs(waypoint.velocities[joint_index]) > joint.velocity_limit)) {
        throw std::invalid_argument(
            context + " velocity is invalid for " + joint_role(joint));
      }
      if ((waypoint.acceleration_valid_mask & bit) != 0 &&
          (!finite(waypoint.accelerations[joint_index]) ||
           std::abs(waypoint.accelerations[joint_index]) >
               joint.acceleration_limit)) {
        throw std::invalid_argument(
            context + " acceleration is invalid for " + joint_role(joint));
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
      if (segment_index < request.approach_segment_count ||
          request.execution == NativeTrajectoryExecution::SampledPv) {
        segment.coefficients.push_back(sampled_pv_coefficients(
            start.positions[joint_index], end.positions[joint_index]));
      } else {
        segment.coefficients.push_back(quintic_coefficients(
            start.positions[joint_index], start.velocities[joint_index],
            start.accelerations[joint_index], end.positions[joint_index],
            end.velocities[joint_index], end.accelerations[joint_index],
            segment.duration_s));
      }
    }
    validate_segment_extrema(segment.duration_s, segment.coefficients,
                             request.joints, recovery_directions,
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
    if (planning_token != 0 &&
        active_command_planning_token_ != planning_token) {
      throw std::runtime_error(
          "Runtime state changed while planning the trajectory");
    }
    require_state_for_command(
        false, enqueue || replace_motion_id != 0, planning_token);
    if (bimanual_follow_.active) {
      throw std::runtime_error(
          "trajectory cannot replace active bimanual ordinary control");
    }
    if (state_ == ARTICORE_DEGRADED) {
      throw std::runtime_error(
          "cannot start a trajectory while Runtime is degraded");
    }
    if (mode_ != request.mode) {
      throw std::runtime_error(
          "trajectory mode does not match the active Runtime mode");
    }
    if (!enqueue &&
        trajectory_control_.state == ARTICORE_MOTION_RUNNING &&
        (replace_motion_id == 0 ||
         trajectory_control_.id != replace_motion_id)) {
      throw std::runtime_error("a trajectory is already running");
    }
    if (replace_motion_id != 0 &&
        (trajectory_control_.state != ARTICORE_MOTION_RUNNING ||
         trajectory_control_.id != replace_motion_id)) {
      throw std::runtime_error(
          "trajectory changed while preparing its replacement");
    }
    if (enqueue && trajectory_queue_.size() >= 64) {
      throw std::runtime_error("motion queue is full");
    }
    intentionally_disabled = intentionally_disabled_motors_;
  }

  for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
    const auto& joint = request.joints[joint_index];
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (backend_->get_feedback_stats(joint.motor, &stats) != 0 ||
        !stats.has_feedback || stats.age_ns > feedback_max_age_ns() ||
        backend_->get_state(joint.motor, &state) != 0 || !state.has_value ||
        !finite(state.pos) || !finite(state.vel) || state.status_code > 1) {
      throw std::runtime_error(
          "trajectory start requires fresh fault-free feedback at " +
          joint_role(joint));
    }
    const bool disabled = intentionally_disabled.count(joint.motor) != 0;
    if ((!disabled && state.status_code != 1) ||
        (disabled && state.status_code != 0)) {
      throw std::runtime_error(
          "trajectory start power feedback disagrees with Runtime at " +
          joint_role(joint));
    }
    if (replace_motion_id == 0 && !planned_reference_transaction) {
      const float requested_start =
          joint.direction * request.waypoints.front().positions[joint_index];
      if (std::abs(state.pos - requested_start) > kStartPositionTolerance) {
        throw std::invalid_argument(
            "trajectory first waypoint is not synchronized with current "
            "feedback at " + joint_role(joint));
      }
      const float actual_velocity =
          joint.direction * state.vel * joint.velocity_feedback_scale;
      const float requested_velocity =
          request.waypoints.front().velocities[joint_index];
      if (std::abs(actual_velocity - requested_velocity) >
          kStartVelocityTolerance) {
        throw std::invalid_argument(
            "trajectory first waypoint velocity is not synchronized with "
            "current feedback at " + joint_role(joint));
      }
    }
  }

  const auto now = Clock::now();
  uint64_t id = 0;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (planning_token != 0 &&
        active_command_planning_token_ != planning_token) {
      throw std::runtime_error(
          "Runtime state changed while planning the trajectory");
    }
    require_state_for_command(
        false, enqueue || replace_motion_id != 0, planning_token);
    if (bimanual_follow_.active) {
      throw std::runtime_error(
          "trajectory cannot replace active bimanual ordinary control");
    }
    if (mode_ != request.mode ||
        (!enqueue &&
         trajectory_control_.state == ARTICORE_MOTION_RUNNING &&
         (replace_motion_id == 0 ||
          trajectory_control_.id != replace_motion_id)) ||
        (replace_motion_id != 0 &&
         (trajectory_control_.state != ARTICORE_MOTION_RUNNING ||
          trajectory_control_.id != replace_motion_id))) {
      throw std::runtime_error(
          "Runtime state changed while starting trajectory");
    }
    if (enqueue && trajectory_queue_.size() >= 64) {
      throw std::runtime_error("motion queue is full");
    }

    TrajectoryControl prepared;
    prepared.state = ARTICORE_MOTION_QUEUED;
    prepared.id = next_motion_id_++;
    prepared.waypoint_count =
        static_cast<uint32_t>(waypoint_count);
    prepared.duration_s = duration_s;
    prepared.approach_segment_count = request.approach_segment_count;
    prepared.approach_duration_s = request.approach_segment_count == 0
        ? 0.0
        : segments[request.approach_segment_count - 1].start_s +
              segments[request.approach_segment_count - 1].duration_s;
    prepared.operation = request.operation;
    prepared.execution = request.execution;
    prepared.joints = std::move(request.joints);
    prepared.segments = std::move(segments);
    prepared.final_convergence_check =
        std::move(request.final_convergence_check);
    prepared.approach_convergence_check =
        std::move(request.approach_convergence_check);
    id = prepared.id;

    const bool active =
        trajectory_control_.state == ARTICORE_MOTION_RUNNING;
    if (enqueue && active) {
      trajectory_queue_.push_back(std::move(prepared));
    } else {
      if (trajectory_control_.id != 0 &&
          trajectory_control_.state != ARTICORE_MOTION_RUNNING) {
        archive_trajectory_locked(trajectory_control_);
      }
      clear_pending_arm_mailbox();
      arm_mailbox_ = ArmMailbox{};
      activate_trajectory_locked(std::move(prepared), now);
      next_control_tick_ = now;
    }
    if (planning_token != 0) active_command_planning_token_ = 0;
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

uint64_t SafetyRuntime::begin_command_planning(
    const CommandTransaction& transaction,
    bool allow_trajectory) {
  if (!transaction.owns_lock() || transaction.mutex() != &command_mutex_) {
    throw std::logic_error(
        "command planning requires the Runtime command transaction");
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  require_state_for_command(false, allow_trajectory);
  if (active_command_planning_token_ != 0) {
    throw std::runtime_error("another command is already being planned");
  }
  const uint64_t token = next_command_planning_token_++;
  active_command_planning_token_ = token == 0
      ? next_command_planning_token_++
      : token;
  return active_command_planning_token_;
}

void SafetyRuntime::cancel_command_planning(uint64_t token) noexcept {
  if (token == 0) return;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (active_command_planning_token_ != token) return;
    active_command_planning_token_ = 0;
  }
  wakeup_.notify_all();
}

NativeTrajectorySample SafetyRuntime::trajectory_final_sample_locked(
    const TrajectoryControl& trajectory) const {
  NativeTrajectorySample result;
  if (trajectory.segments.empty() || trajectory.joints.empty()) return result;
  const auto& coefficients = trajectory.segments.back().coefficients;
  if (coefficients.size() != trajectory.joints.size()) return result;
  result.active = true;
  result.motion_id = trajectory.id;
  result.operation = trajectory.operation;
  result.positions.reserve(coefficients.size());
  for (const auto& joint : coefficients) {
    result.positions.push_back(
        static_cast<float>(sample_polynomial(joint, 1.0)));
  }
  result.velocities.assign(coefficients.size(), 0.0f);
  result.accelerations.assign(coefficients.size(), 0.0f);
  return result;
}

NativeTrajectorySample SafetyRuntime::planned_trajectory_tail_sample(
    const std::vector<NativeTrajectoryJoint>& joints,
    const CommandTransaction& transaction) const {
  if (!transaction.owns_lock() || transaction.mutex() != &command_mutex_) {
    throw std::logic_error(
        "planned trajectory tail requires the Runtime command transaction");
  }
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (!trajectory_queue_.empty()) {
      auto result = trajectory_final_sample_locked(trajectory_queue_.back());
      if (result.active) return result;
    }
    if (trajectory_control_.state == ARTICORE_MOTION_RUNNING ||
        trajectory_control_.state == ARTICORE_MOTION_COMPLETED) {
      auto result = trajectory_final_sample_locked(trajectory_control_);
      if (result.active) return result;
    }
  }
  return planned_arm_sample(joints, transaction);
}

void SafetyRuntime::activate_trajectory_locked(
    TrajectoryControl trajectory, Clock::time_point now) {
  trajectory.state = ARTICORE_MOTION_RUNNING;
  trajectory.active_segment = 0;
  trajectory.elapsed_s = 0.0;
  trajectory.started_at = now;
  trajectory.tracking_updated_at = now;
  trajectory.tracking_pause_started_at = Clock::time_point{};
  trajectory.settling_started_at = Clock::time_point{};
  trajectory.settling_stable_started_at = Clock::time_point{};
  trajectory.hold_unstable_started_at = Clock::time_point{};
  trajectory.settled_feedback_samples = 0;
  trajectory.settling_feedback_initialized = false;
  trajectory.stationary_hold_active = false;
  trajectory.final_hold_limit_active = false;
  trajectory.final_hold_limit_mask = 0;
  trajectory.tracking_time_scale = 1.0f;
  trajectory.tracking_position_error = 0.0f;
  trajectory.tracking_feedback_valid = false;
  trajectory.tracking_worst_role.clear();
  trajectory.approach_complete = trajectory.approach_segment_count == 0;
  trajectory.error.clear();
  trajectory_control_ = std::move(trajectory);
}

ArticoreMotionStatus SafetyRuntime::motion_status_locked(
    const TrajectoryControl& trajectory) const {
  ArticoreMotionStatus result{};
  result.struct_size = sizeof(result);
  result.motion_id = trajectory.id;
  result.motion_type = motion_type(trajectory.operation);
  result.state = trajectory.state;
  result.active_segment = trajectory.active_segment;
  result.waypoint_count = trajectory.waypoint_count;
  result.elapsed_s = trajectory.elapsed_s;
  result.duration_s = trajectory.duration_s;
  result.progress = trajectory.duration_s > 0.0
      ? static_cast<float>(std::clamp(
            trajectory.elapsed_s / trajectory.duration_s, 0.0, 1.0))
      : 0.0f;
  detail::copy_text(result.error, trajectory.error);
  return result;
}

void SafetyRuntime::archive_trajectory_locked(
    const TrajectoryControl& trajectory) {
  if (trajectory.id == 0) return;
  trajectory_history_.push_back(motion_status_locked(trajectory));
  while (trajectory_history_.size() > 128) trajectory_history_.pop_front();
}

NativeTrajectorySample SafetyRuntime::trajectory_sample_locked(
    Clock::time_point now) const {
  NativeTrajectorySample result;
  if (trajectory_control_.state != ARTICORE_MOTION_RUNNING ||
      trajectory_control_.segments.empty() ||
      trajectory_control_.joints.empty()) {
    return result;
  }

  double elapsed = std::chrono::duration<double>(
      now - trajectory_control_.started_at).count();
  elapsed = std::clamp(elapsed, 0.0, trajectory_control_.duration_s);
  if (!trajectory_control_.approach_complete &&
      trajectory_control_.approach_segment_count != 0) {
    elapsed = std::min(elapsed, trajectory_control_.approach_duration_s);
  }
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
  const float time_scale =
      native_cartesian_operation(trajectory_control_.operation)
      ? trajectory_control_.tracking_time_scale
      : 1.0f;

  result.active = true;
  result.motion_id = trajectory_control_.id;
  result.operation = trajectory_control_.operation;
  result.positions.reserve(segment.coefficients.size());
  result.velocities.reserve(segment.coefficients.size());
  result.accelerations.reserve(segment.coefficients.size());
  for (const auto& coefficients : segment.coefficients) {
    result.positions.push_back(static_cast<float>(
        sample_polynomial(coefficients, u)));
    result.velocities.push_back(static_cast<float>(
        sample_velocity(coefficients, u, segment.duration_s) * time_scale));
    result.accelerations.push_back(static_cast<float>(
        sample_acceleration(coefficients, u, segment.duration_s) *
        time_scale * time_scale));
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
          "planned arm reference contains an invalid joint for " +
          joint_role(joint));
    }
    if (!arm_mailbox_.pv.empty()) {
      const auto found = std::find_if(
          arm_mailbox_.pv.begin(), arm_mailbox_.pv.end(),
          [&](const ArticorePosVelCommand& command) {
            return command.motor == joint.motor;
          });
      if (found == arm_mailbox_.pv.end()) {
        throw std::runtime_error(
            "current planned PV reference is incomplete at " +
            joint_role(joint));
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
          "current planned MIT reference is incomplete at " +
          joint_role(joint));
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
  if (trajectory_control_.state == ARTICORE_MOTION_COMPLETED) {
    // Keep checking physical feedback after reporting completion. The final
    // mailbox remains a constant low-speed PV hold; this flag asks the worker
    // to monitor that hold after the frame is sent.
    completing = true;
    return false;
  }
  if (trajectory_control_.state != ARTICORE_MOTION_RUNNING) return false;
  if (trajectory_control_.segments.empty() ||
      trajectory_control_.joints.empty()) {
    error = "trajectory plan is internally empty";
    return false;
  }

  const bool adaptive_cartesian_pv =
      mode_ == ARTICORE_MODE_PV &&
      native_cartesian_operation(trajectory_control_.operation) &&
      trajectory_control_.approach_complete;
  if (adaptive_cartesian_pv) {
    const double tracking_dt = std::max(
        0.0, std::chrono::duration<double>(
                 now - trajectory_control_.tracking_updated_at).count());
    const float target_scale = trajectory_control_.tracking_feedback_valid
        ? native_cartesian_tracking_scale(
              trajectory_control_.tracking_position_error)
        : 1.0f;
    if (target_scale <= 0.0f) {
      trajectory_control_.tracking_time_scale = 0.0f;
      if (trajectory_control_.tracking_pause_started_at ==
          Clock::time_point{}) {
        trajectory_control_.tracking_pause_started_at = now;
      } else if (now - trajectory_control_.tracking_pause_started_at >=
                 kNativeCartesianTrackingPauseTimeout) {
        error = "Cartesian tracking error did not recover";
        if (!trajectory_control_.tracking_worst_role.empty()) {
          error += " at " + trajectory_control_.tracking_worst_role;
        }
        error += ": position_error=" + std::to_string(
            trajectory_control_.tracking_position_error) +
            " rad, pause_threshold=" + std::to_string(
                kNativeCartesianTrackingPauseError) + " rad";
        return false;
      }
    } else {
      trajectory_control_.tracking_pause_started_at = Clock::time_point{};
      const float rate = target_scale < trajectory_control_.tracking_time_scale
          ? kNativeCartesianTrackingDecelerationPerSecond
          : kNativeCartesianTrackingAccelerationPerSecond;
      const float maximum_change = static_cast<float>(tracking_dt) * rate;
      trajectory_control_.tracking_time_scale += std::clamp(
          target_scale - trajectory_control_.tracking_time_scale,
          -maximum_change, maximum_change);
      trajectory_control_.tracking_time_scale = std::clamp(
          trajectory_control_.tracking_time_scale, 0.0f, 1.0f);
    }
    if (tracking_dt > 0.0 && trajectory_control_.tracking_time_scale < 1.0f) {
      trajectory_control_.started_at +=
          std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(
                  tracking_dt *
                  (1.0 - trajectory_control_.tracking_time_scale)));
    }
    trajectory_control_.tracking_updated_at = now;
  } else {
    trajectory_control_.tracking_time_scale = 1.0f;
    trajectory_control_.tracking_updated_at = now;
    trajectory_control_.tracking_pause_started_at = Clock::time_point{};
  }

  double elapsed = std::chrono::duration<double>(
      now - trajectory_control_.started_at).count();
  elapsed = std::clamp(elapsed, 0.0, trajectory_control_.duration_s);
  const bool at_approach_reference =
      !trajectory_control_.approach_complete &&
      trajectory_control_.approach_segment_count != 0 &&
      elapsed >= trajectory_control_.approach_duration_s;
  if (at_approach_reference) {
    elapsed = trajectory_control_.approach_duration_s;
  }
  const bool at_final_reference =
      elapsed >= trajectory_control_.duration_s;
  const bool at_settling_reference =
      at_approach_reference || at_final_reference;
  const bool use_stationary_hold =
      at_settling_reference && trajectory_control_.stationary_hold_active;
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
        sample_velocity(coefficients, u, segment.duration_s) *
        trajectory_control_.tracking_time_scale);
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
      const bool use_joint_final_hold_limit =
          at_settling_reference &&
          (trajectory_control_.final_hold_limit_mask &
           (uint32_t{1} << joint_index)) != 0;
      arm_mailbox_.pv[joint_index] = ArticorePosVelCommand{
          joint.motor, joint.direction * position,
          use_stationary_hold
              ? joint.pv_hold_velocity_limit
              : use_joint_final_hold_limit
              ? std::min(joint.pv_velocity_limit,
                         kNativePvSettlingVelocityLimit)
              : adaptive_cartesian_pv
              ? native_cartesian_pv_velocity_limit(
                    velocity, config_.safe_pv_velocity_limit,
                    joint.pv_velocity_limit)
              : joint.pv_velocity_limit};
    }
  }

  trajectory_control_.active_segment = static_cast<uint32_t>(segment_index);
  trajectory_control_.elapsed_s = elapsed;
  completing = at_settling_reference;
  return true;
}

void SafetyRuntime::update_trajectory_completion(Clock::time_point now) {
  struct ArrivalJoint {
    NativeTrajectoryJoint joint;
    float target = 0.0f;
    bool intentionally_disabled = false;
    std::string role;
  };

  uint64_t motion_id = 0;
  ArticoreControlMode mode = ARTICORE_MODE_PV;
  bool monitoring_completed_hold = false;
  bool waiting_at_approach = false;
  std::vector<uint64_t> previous_updates;
  bool feedback_initialized = false;
  std::function<bool(const std::vector<float>&, std::string&)>
      final_convergence_check;
  std::vector<ArrivalJoint> arrivals;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if ((trajectory_control_.state != ARTICORE_MOTION_RUNNING &&
         trajectory_control_.state != ARTICORE_MOTION_COMPLETED) ||
        trajectory_control_.segments.empty()) {
      return;
    }
    monitoring_completed_hold =
        trajectory_control_.state == ARTICORE_MOTION_COMPLETED;
    waiting_at_approach =
        !monitoring_completed_hold &&
        !trajectory_control_.approach_complete &&
        trajectory_control_.approach_segment_count != 0 &&
        trajectory_control_.elapsed_s >=
            trajectory_control_.approach_duration_s;
    if (!monitoring_completed_hold &&
        trajectory_control_.settling_started_at == Clock::time_point{}) {
      trajectory_control_.settling_started_at = now;
      trajectory_control_.settling_stable_started_at = Clock::time_point{};
      trajectory_control_.hold_unstable_started_at = Clock::time_point{};
      trajectory_control_.settling_feedback_updates.assign(
          trajectory_control_.joints.size(), 0);
      trajectory_control_.settling_position_min.assign(
          trajectory_control_.joints.size(),
          std::numeric_limits<float>::infinity());
      trajectory_control_.settling_position_max.assign(
          trajectory_control_.joints.size(),
          -std::numeric_limits<float>::infinity());
      trajectory_control_.settled_feedback_samples = 0;
      trajectory_control_.settling_feedback_initialized = false;
    }
    motion_id = trajectory_control_.id;
    mode = mode_;
    previous_updates = trajectory_control_.settling_feedback_updates;
    feedback_initialized =
        trajectory_control_.settling_feedback_initialized;
    final_convergence_check = waiting_at_approach
        ? trajectory_control_.approach_convergence_check
        : trajectory_control_.final_convergence_check;
    const auto& final_coefficients = waiting_at_approach
        ? trajectory_control_
              .segments[trajectory_control_.approach_segment_count - 1]
              .coefficients
        : trajectory_control_.segments.back().coefficients;
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
      arrival.role = !joint.role.empty()
          ? joint.role
          : role == motor_roles_.end() ? "unknown product joint"
                                       : role->second;
      arrivals.push_back(std::move(arrival));
    }
  }

  const float position_tolerance = mode == ARTICORE_MODE_PV
      ? kPvArrivalPositionTolerance : kMitArrivalPositionTolerance;
  const float velocity_tolerance = mode == ARTICORE_MODE_PV
      ? kPvArrivalVelocityTolerance : kMitArrivalVelocityTolerance;
  bool all_arrived = true;
  bool all_positions_arrived = true;
  bool final_position_ready = true;
  bool all_feedback_new = true;
  bool any_feedback_new = !feedback_initialized;
  bool has_active_joint = false;
  uint32_t active_joint_mask = 0;
  uint32_t available_joint_mask = 0;
  uint32_t arrived_joint_mask = 0;
  uint32_t position_arrived_mask = 0;
  std::vector<uint64_t> current_updates(arrivals.size(), 0);
  std::vector<float> actual_positions(
      arrivals.size(), std::numeric_limits<float>::quiet_NaN());
  float worst_position_error = 0.0f;
  float worst_velocity = 0.0f;
  std::string worst_role;
  float worst_target = 0.0f;
  float worst_actual = 0.0f;
  float worst_violation_score = -1.0f;
  float worst_position_range = 0.0f;
  std::string worst_range_role;
  float settling_worst_position_range = 0.0f;
  std::string settling_worst_range_role;
  std::string unavailable_role;
  std::string convergence_error;

  for (std::size_t index = 0; index < arrivals.size(); ++index) {
    const auto& arrival = arrivals[index];
    if (arrival.intentionally_disabled) continue;
    has_active_joint = true;
    active_joint_mask |= uint32_t{1} << index;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool available =
        backend_->get_feedback_stats(arrival.joint.motor, &stats) == 0 &&
        stats.has_feedback && stats.age_ns <= feedback_max_age_ns() &&
        backend_->get_state(arrival.joint.motor, &state) == 0 &&
        state.has_value && state.status_code == 1 && finite(state.pos) &&
        finite(state.vel);
    if (!available) {
      all_arrived = false;
      all_positions_arrived = false;
      all_feedback_new = false;
      if (unavailable_role.empty()) unavailable_role = arrival.role;
      continue;
    }
    available_joint_mask |= uint32_t{1} << index;
    current_updates[index] = stats.update_count;
    if (feedback_initialized &&
        index < previous_updates.size() &&
        stats.update_count == previous_updates[index]) {
      all_feedback_new = false;
    } else if (feedback_initialized && index < previous_updates.size()) {
      any_feedback_new = true;
    }
    const float actual = arrival.joint.direction * state.pos;
    const float velocity = arrival.joint.direction * state.vel *
        arrival.joint.velocity_feedback_scale;
    const float position_error = std::abs(actual - arrival.target);
    const float speed = std::abs(velocity);
    actual_positions[index] = actual;
    if (position_error > position_tolerance) {
      all_arrived = false;
      all_positions_arrived = false;
    } else {
      position_arrived_mask |= uint32_t{1} << index;
    }
    if (speed > velocity_tolerance) {
      all_arrived = false;
    } else if (position_error <= position_tolerance) {
      arrived_joint_mask |= uint32_t{1} << index;
    }
    const float final_position_tolerance = is_loaded_joint4_role(arrival.role)
        ? kPvLoadedJointFinalPositionTolerance
        : kPvFinalPositionTolerance;
    if (mode == ARTICORE_MODE_PV &&
        position_error > final_position_tolerance) {
      final_position_ready = false;
    }
    const float violation_score = std::max(
        position_error / position_tolerance, speed / velocity_tolerance);
    if (violation_score > worst_violation_score) {
      worst_violation_score = violation_score;
      worst_position_error = position_error;
      worst_velocity = speed;
      worst_role = arrival.role;
      worst_target = arrival.target;
      worst_actual = actual;
    }
  }

  if (mode == ARTICORE_MODE_PV && final_convergence_check &&
      unavailable_role.empty()) {
    try {
      final_position_ready =
          final_convergence_check(actual_positions, convergence_error);
    } catch (const std::exception& err) {
      final_position_ready = false;
      convergence_error = err.what();
    }
  }

  bool completed = false;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    const auto expected_state = monitoring_completed_hold
        ? ARTICORE_MOTION_COMPLETED
        : ARTICORE_MOTION_RUNNING;
    if (trajectory_control_.state != expected_state ||
        trajectory_control_.id != motion_id) {
      return;
    }
    if (waiting_at_approach) {
      trajectory_control_.elapsed_s =
          trajectory_control_.approach_duration_s;
      trajectory_control_.active_segment =
          trajectory_control_.approach_segment_count - 1;
    } else {
      trajectory_control_.elapsed_s = trajectory_control_.duration_s;
      trajectory_control_.active_segment =
          static_cast<uint32_t>(trajectory_control_.segments.size() - 1);
    }

    if (monitoring_completed_hold) {
      if (!has_active_joint) return;
      if (!any_feedback_new) return;
      trajectory_control_.settling_feedback_updates = current_updates;

      bool position_range_stable = true;
      for (std::size_t index = 0; index < arrivals.size(); ++index) {
        if (arrivals[index].intentionally_disabled ||
            !finite(actual_positions[index])) {
          continue;
        }
        trajectory_control_.settling_position_min[index] = std::min(
            trajectory_control_.settling_position_min[index],
            actual_positions[index]);
        trajectory_control_.settling_position_max[index] = std::max(
            trajectory_control_.settling_position_max[index],
            actual_positions[index]);
        const float position_range =
            trajectory_control_.settling_position_max[index] -
            trajectory_control_.settling_position_min[index];
        if (mode == ARTICORE_MODE_PV &&
            position_range > kPvStablePositionRange) {
          position_range_stable = false;
          if (position_range > worst_position_range) {
            worst_position_range = position_range;
            worst_range_role = arrivals[index].role;
          }
        }
      }
      const bool hold_stable = all_arrived && position_range_stable;
      if (hold_stable) {
        trajectory_control_.hold_unstable_started_at = Clock::time_point{};
        return;
      }
      if (trajectory_control_.hold_unstable_started_at == Clock::time_point{}) {
        trajectory_control_.hold_unstable_started_at = now;
        return;
      }
      if (now - trajectory_control_.hold_unstable_started_at <
          kCompletedHoldViolationDuration) {
        return;
      }

      std::ostringstream stream;
      stream << "completed trajectory hold became unstable; waiting for "
                "physical restabilization";
      if (!unavailable_role.empty()) {
        stream << "; fresh enabled feedback unavailable for "
               << unavailable_role;
      } else if (!position_range_stable && !worst_range_role.empty()) {
        stream << "; " << worst_range_role
               << " position_range=" << worst_position_range
               << " stable_range_limit=" << kPvStablePositionRange;
      } else if (!worst_role.empty()) {
        stream << "; " << worst_role
               << " target=" << worst_target
               << " actual=" << worst_actual
               << " position_error=" << worst_position_error
               << " velocity=" << worst_velocity;
      }
      const auto unstable_error = stream.str();
      trajectory_control_.state = ARTICORE_MOTION_RUNNING;
      trajectory_control_.settling_started_at = now;
      trajectory_control_.settling_stable_started_at = Clock::time_point{};
      trajectory_control_.hold_unstable_started_at = Clock::time_point{};
      trajectory_control_.settled_feedback_samples = 0;
      trajectory_control_.stationary_hold_active = all_positions_arrived;
      trajectory_control_.final_hold_limit_active = all_positions_arrived;
      trajectory_control_.settling_position_min.assign(
          arrivals.size(), std::numeric_limits<float>::infinity());
      trajectory_control_.settling_position_max.assign(
          arrivals.size(), -std::numeric_limits<float>::infinity());
      trajectory_control_.error = unstable_error;
      last_operation_ = trajectory_control_.operation;
      last_operation_code_ = ARTICORE_OPERATION_FEEDBACK;
      last_operation_error_ = unstable_error;
      operation_failed_motors_.clear();
      if (!unavailable_role.empty()) {
        operation_failed_motors_.push_back(unavailable_role);
      } else if (!worst_role.empty()) {
        operation_failed_motors_.push_back(worst_role);
      }
      return;
    }

    if (!has_active_joint) {
      completed = true;
    } else {
      if (!feedback_initialized || any_feedback_new) {
        trajectory_control_.settling_feedback_updates = current_updates;
        trajectory_control_.settling_feedback_initialized = true;
        // Once one PV joint has physically arrived, let it settle at the
        // low-speed limit independently while slower loaded joints continue
        // converging. Restore the normal limit only if that joint leaves the
        // position arrival window.
        trajectory_control_.final_hold_limit_mask &=
            position_arrived_mask | ~available_joint_mask;
        trajectory_control_.final_hold_limit_mask |= arrived_joint_mask;
        trajectory_control_.final_hold_limit_active =
            active_joint_mask != 0 &&
            (trajectory_control_.final_hold_limit_mask & active_joint_mask) ==
                active_joint_mask;
      }
      const bool fresh_arrival_feedback =
          all_arrived && (!feedback_initialized || any_feedback_new);
      const bool fresh_position_feedback =
          all_positions_arrived &&
          (!feedback_initialized || any_feedback_new);
      const bool fresh_final_feedback =
          fresh_position_feedback &&
          (final_position_ready || trajectory_control_.stationary_hold_active);
      if (fresh_arrival_feedback) {
        trajectory_control_.final_hold_limit_active = true;
      }
      if (fresh_final_feedback) {
        const bool entering_stationary_hold =
            !trajectory_control_.stationary_hold_active;
        trajectory_control_.stationary_hold_active = true;
        if (entering_stationary_hold) {
          // Low-speed convergence and zero-speed hold verification are two
          // distinct physical phases. Give the stationary phase its own
          // deadline/window instead of inheriting nearly expired settling
          // time and motion accumulated before the zero-speed command.
          trajectory_control_.settling_started_at = now;
          trajectory_control_.settling_stable_started_at = now;
          trajectory_control_.settled_feedback_samples = 0;
          trajectory_control_.settling_position_min.assign(
              arrivals.size(), std::numeric_limits<float>::infinity());
          trajectory_control_.settling_position_max.assign(
              arrivals.size(), -std::numeric_limits<float>::infinity());
        } else if (trajectory_control_.settling_stable_started_at ==
            Clock::time_point{}) {
          trajectory_control_.settling_stable_started_at = now;
        }
        for (std::size_t index = 0; index < arrivals.size(); ++index) {
          if (arrivals[index].intentionally_disabled ||
              !finite(actual_positions[index])) {
            continue;
          }
          trajectory_control_.settling_position_min[index] = std::min(
              trajectory_control_.settling_position_min[index],
              actual_positions[index]);
          trajectory_control_.settling_position_max[index] = std::max(
              trajectory_control_.settling_position_max[index],
              actual_positions[index]);
        }
        ++trajectory_control_.settled_feedback_samples;
      } else if (!feedback_initialized || any_feedback_new) {
        // A 500 Hz control cycle may legitimately observe the same coherent
        // feedback snapshot as the preceding cycle. That is not instability:
        // preserve the settling timer until a genuinely newer snapshot either
        // confirms or violates the window.
        const bool stationary_velocity_transient =
            trajectory_control_.stationary_hold_active &&
            all_positions_arrived;
        if (!stationary_velocity_transient) {
          trajectory_control_.settled_feedback_samples = 0;
          trajectory_control_.stationary_hold_active = false;
          if (!all_positions_arrived) {
            trajectory_control_.final_hold_limit_active = false;
          }
          trajectory_control_.settling_stable_started_at =
              Clock::time_point{};
          trajectory_control_.settling_position_min.assign(
              arrivals.size(), std::numeric_limits<float>::infinity());
          trajectory_control_.settling_position_max.assign(
              arrivals.size(), -std::numeric_limits<float>::infinity());
        }
      }
      bool position_range_stable = true;
      if (mode == ARTICORE_MODE_PV) {
        for (std::size_t index = 0; index < arrivals.size(); ++index) {
          if (arrivals[index].intentionally_disabled) continue;
          const float position_range =
              trajectory_control_.settling_position_max[index] -
              trajectory_control_.settling_position_min[index];
          if (position_range > settling_worst_position_range) {
            settling_worst_position_range = position_range;
            settling_worst_range_role = arrivals[index].role;
          }
          if (position_range > kPvStablePositionRange) {
            position_range_stable = false;
          }
        }
      }
      const bool stable_duration_met =
          trajectory_control_.settling_stable_started_at !=
              Clock::time_point{} &&
          now - trajectory_control_.settling_stable_started_at >=
              kArrivalStableDuration;
      completed = stable_duration_met && position_range_stable && all_arrived;
      if (stable_duration_met && !position_range_stable) {
        trajectory_control_.settling_stable_started_at = now;
        trajectory_control_.settled_feedback_samples = 1;
        trajectory_control_.settling_position_min = actual_positions;
        trajectory_control_.settling_position_max = actual_positions;
      }
    }

    if (completed) {
      if (waiting_at_approach) {
        trajectory_control_.approach_complete = true;
        trajectory_control_.started_at =
            now - std::chrono::duration_cast<Clock::duration>(
                      std::chrono::duration<double>(
                          trajectory_control_.approach_duration_s));
        trajectory_control_.tracking_updated_at = now;
        trajectory_control_.tracking_time_scale = 1.0f;
        trajectory_control_.tracking_position_error = 0.0f;
        trajectory_control_.tracking_feedback_valid = false;
        trajectory_control_.tracking_pause_started_at = Clock::time_point{};
        trajectory_control_.tracking_worst_role.clear();
        trajectory_control_.settling_started_at = Clock::time_point{};
        trajectory_control_.settling_stable_started_at = Clock::time_point{};
        trajectory_control_.hold_unstable_started_at = Clock::time_point{};
        trajectory_control_.settled_feedback_samples = 0;
        trajectory_control_.settling_feedback_initialized = false;
        trajectory_control_.stationary_hold_active = false;
        trajectory_control_.final_hold_limit_active = false;
        trajectory_control_.final_hold_limit_mask = 0;
        trajectory_control_.error.clear();
        next_control_tick_ = now;
        return;
      }
      trajectory_control_.state = ARTICORE_MOTION_COMPLETED;
      trajectory_control_.error.clear();
      trajectory_control_.final_hold_limit_active = true;
      trajectory_control_.final_hold_limit_mask =
          trajectory_control_.joints.size() >= 32
              ? std::numeric_limits<uint32_t>::max()
              : (uint32_t{1} << trajectory_control_.joints.size()) - 1U;
      trajectory_control_.stationary_hold_active = true;
      if (mode == ARTICORE_MODE_PV &&
          arm_mailbox_.pv.size() == trajectory_control_.joints.size()) {
        for (std::size_t index = 0; index < arm_mailbox_.pv.size(); ++index) {
          arm_mailbox_.pv[index].velocity_limit =
              trajectory_control_.joints[index].pv_hold_velocity_limit;
        }
      }
      trajectory_control_.hold_unstable_started_at = Clock::time_point{};
      trajectory_control_.settling_position_min = actual_positions;
      trajectory_control_.settling_position_max = actual_positions;
      if (!trajectory_queue_.empty()) {
        archive_trajectory_locked(trajectory_control_);
        auto next = std::move(trajectory_queue_.front());
        trajectory_queue_.pop_front();
        activate_trajectory_locked(std::move(next), now);
        next_control_tick_ = now;
      }
      return;
    }

    const double arrival_timeout_s = std::clamp(
        trajectory_control_.duration_s * 0.25,
        kMinimumArrivalTimeoutSeconds, kMaximumArrivalTimeoutSeconds);
    if (std::chrono::duration<double>(
            now - trajectory_control_.settling_started_at).count() >=
        arrival_timeout_s) {
      std::ostringstream stream;
      stream << (waiting_at_approach
                     ? "trajectory approach arrival timed out after "
                     : "trajectory arrival timed out after ")
             << arrival_timeout_s
             << " s"
             << "; stationary_hold="
             << (trajectory_control_.stationary_hold_active ? "true" : "false")
             << "; settling_hold="
             << (trajectory_control_.final_hold_limit_active ? "true" : "false")
             << "; stable_samples="
             << trajectory_control_.settled_feedback_samples
             << "; all_feedback_new="
             << (all_feedback_new ? "true" : "false")
             << "; any_feedback_new="
             << (any_feedback_new ? "true" : "false");
      if (!unavailable_role.empty()) {
        stream << "; fresh enabled feedback unavailable for "
               << unavailable_role;
      } else if (all_arrived &&
                 settling_worst_position_range > kPvStablePositionRange &&
                 !settling_worst_range_role.empty()) {
        stream << "; " << settling_worst_range_role
               << " position_range=" << settling_worst_position_range
               << " stable_range_limit=" << kPvStablePositionRange;
      } else if (!convergence_error.empty()) {
        stream << "; " << convergence_error;
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
          ARTICORE_MOTION_FAULT, timeout_error);
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
    ArticoreMotionState state, const std::string& error) {
  const bool active =
      trajectory_control_.state == ARTICORE_MOTION_RUNNING ||
      trajectory_control_.state == ARTICORE_MOTION_COMPLETED;
  if (active) {
    trajectory_control_.state = state;
    trajectory_control_.error = error;
  }
  for (auto& queued : trajectory_queue_) {
    queued.state = ARTICORE_MOTION_CANCELLED;
    queued.error = "queued motion cancelled: " + error;
    archive_trajectory_locked(queued);
  }
  trajectory_queue_.clear();
  if (!active) return;
  if (state == ARTICORE_MOTION_FAULT) {
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
  terminate_trajectory_locked(ARTICORE_MOTION_FAULT, error);
  last_operation_ = trajectory_control_.operation;
  last_operation_code_ = ARTICORE_OPERATION_MOTOR_COMMAND;
  last_operation_error_ = error;
  operation_failed_motors_.clear();
}

void SafetyRuntime::cancel_motion(uint64_t motion_id) {
  if (motion_id == 0) {
    throw std::invalid_argument("motion id must be non-zero");
  }
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (trajectory_control_.id == motion_id) {
    if (trajectory_control_.state == ARTICORE_MOTION_RUNNING) {
      // Queued tasks were planned from this task's final reference. They
      // cannot remain valid after an active cancellation, so cancel the
      // dependent tail and hold the last safely dispatched reference.
      terminate_trajectory_locked(
          ARTICORE_MOTION_CANCELLED, "motion cancelled");
    }
    return;
  }
  const auto queued = std::find_if(
      trajectory_queue_.begin(), trajectory_queue_.end(),
      [&](const TrajectoryControl& candidate) {
        return candidate.id == motion_id;
      });
  if (queued != trajectory_queue_.end()) {
    const auto successor = std::next(queued);
    if (successor != trajectory_queue_.end()) {
      const auto predecessor = queued == trajectory_queue_.begin()
          ? trajectory_final_sample_locked(trajectory_control_)
          : trajectory_final_sample_locked(*std::prev(queued));
      if (!predecessor.active ||
          predecessor.positions.size() != successor->joints.size() ||
          successor->segments.empty()) {
        throw std::runtime_error(
            "queued motion cannot be safely detached from its FIFO neighbors");
      }

      TrajectoryControl rebound = *successor;
      const auto& original_first = rebound.segments.front();
      double approach_duration = 0.10;
      for (std::size_t index = 0; index < rebound.joints.size(); ++index) {
        const auto& joint = rebound.joints[index];
        const double target = sample_polynomial(
            original_first.coefficients[index], 0.0);
        const double step = std::abs(
            target - static_cast<double>(predecessor.positions[index]));
        double velocity_limit = joint.velocity_limit;
        if (joint.pv_velocity_limit > 0.0f) {
          velocity_limit = std::min(
              velocity_limit, static_cast<double>(joint.pv_velocity_limit));
        }
        approach_duration = std::max(
            approach_duration, step / std::max(0.01, velocity_limit));
        approach_duration = std::max(
            approach_duration,
            std::sqrt(8.0 * step /
                      std::max(0.01, static_cast<double>(
                                         joint.acceleration_limit))));
      }

      TrajectorySegment approach;
      approach.start_s = 0.0;
      approach.duration_s = approach_duration;
      approach.coefficients.reserve(rebound.joints.size());
      for (std::size_t index = 0; index < rebound.joints.size(); ++index) {
        const auto& original = original_first.coefficients[index];
        const double target_position = sample_polynomial(original, 0.0);
        const double target_velocity = sample_velocity(
            original, 0.0, original_first.duration_s);
        const double target_acceleration = sample_acceleration(
            original, 0.0, original_first.duration_s);
        approach.coefficients.push_back(
            rebound.execution == NativeTrajectoryExecution::SampledPv
                ? sampled_pv_coefficients(
                      predecessor.positions[index], target_position)
                : quintic_coefficients(
                      predecessor.positions[index], 0.0, 0.0,
                      target_position, target_velocity,
                      target_acceleration, approach_duration));
      }
      validate_segment_extrema(
          approach.duration_s, approach.coefficients, rebound.joints,
          std::vector<int8_t>(rebound.joints.size(), 0), 0);
      for (auto& segment : rebound.segments) {
        segment.start_s += approach_duration;
      }
      rebound.segments.insert(
          rebound.segments.begin(), std::move(approach));
      rebound.duration_s += approach_duration;
      rebound.approach_duration_s += approach_duration;
      ++rebound.approach_segment_count;
      ++rebound.waypoint_count;
      *successor = std::move(rebound);
    }
    queued->state = ARTICORE_MOTION_CANCELLED;
    queued->error = "queued motion cancelled";
    archive_trajectory_locked(*queued);
    trajectory_queue_.erase(queued);
    return;
  }
  const bool terminal = std::any_of(
      trajectory_history_.begin(), trajectory_history_.end(),
      [&](const ArticoreMotionStatus& status) {
        return status.motion_id == motion_id;
      });
  if (!terminal) throw std::invalid_argument("motion id is not available");
}

void SafetyRuntime::cancel_all_motions() {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  terminate_trajectory_locked(
      ARTICORE_MOTION_CANCELLED, "all motions cancelled");
}

ArticoreMotionStatus SafetyRuntime::motion_status() const {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return motion_status_locked(trajectory_control_);
}

ArticoreMotionStatus SafetyRuntime::motion_status(
    uint64_t motion_id) const {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (trajectory_control_.id == motion_id) {
    return motion_status_locked(trajectory_control_);
  }
  for (const auto& queued : trajectory_queue_) {
    if (queued.id == motion_id) return motion_status_locked(queued);
  }
  for (auto it = trajectory_history_.rbegin();
       it != trajectory_history_.rend(); ++it) {
    if (it->motion_id == motion_id) return *it;
  }
  ArticoreMotionStatus result{};
  result.struct_size = sizeof(result);
  result.state = ARTICORE_MOTION_IDLE;
  result.motion_id = motion_id;
  detail::copy_text(result.error, "motion id is not available");
  return result;
}

}  // namespace articore
