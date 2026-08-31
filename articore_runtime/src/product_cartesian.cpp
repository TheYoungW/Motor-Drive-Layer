#include "articore/detail/product_cartesian.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "articore/detail/cartesian_math.hpp"
#include "articore/detail/yunyi_runtime.hpp"

namespace articore {
static_assert(
    kYunyiCartesianPvDriveVelocityLimit == kYunyiPvDriveVelocityLimit,
    "Cartesian trajectory PV must use the product drive velocity ceiling");

namespace {

// A sparse Cartesian IK path preserves geometry. It is then resampled through
// one global quintic time law into a finite 100 Hz real-time PV knot list. The
// Runtime executor linearly resamples adjacent knots on its 500 Hz clock.
constexpr double kCircularPvCommandPeriodSeconds =
    1.0 / static_cast<double>(kYunyiCircularReferenceHz);
constexpr double kLinearPvCommandPeriodSeconds =
    kNativeRealtimePvTrajectoryPeriodSeconds;
constexpr float kCartesianRealtimePvSpeedPercent = 50.0f;
constexpr uint32_t kCartesianMaximumExecutionSegments =
    kNativeMaximumTrajectoryWaypoints - 3U;
constexpr uint32_t kCircularMaximumGeometrySegments = 2048U;

uint32_t cartesian_execution_segment_count(
    double duration_s, uint32_t geometry_segments, double command_period_s) {
  const double requested = std::ceil(duration_s / command_period_s);
  if (!std::isfinite(requested) || requested < 1.0 ||
      requested > kCartesianMaximumExecutionSegments) {
    throw std::invalid_argument(
        "Cartesian duration_s requires more than the native point capacity");
  }
  return std::max<uint32_t>(
      geometry_segments, static_cast<uint32_t>(requested));
}

double quintic_progress(double amount) {
  amount = std::clamp(amount, 0.0, 1.0);
  const double amount2 = amount * amount;
  const double amount3 = amount2 * amount;
  return amount3 * (10.0 + amount * (-15.0 + 6.0 * amount));
}

std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>
resample_cartesian_path(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    double duration_s, double command_period_s) {
  if (path.size() < 2) {
    throw std::invalid_argument("Cartesian path requires at least two points");
  }
  const uint32_t segment_count = cartesian_execution_segment_count(
      duration_s, static_cast<uint32_t>(path.size() - 1U), command_period_s);
  std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>> result;
  result.reserve(segment_count + 1U);
  for (uint32_t sample = 0; sample <= segment_count; ++sample) {
    const double time_amount = static_cast<double>(sample) /
        static_cast<double>(segment_count);
    const double path_position = quintic_progress(time_amount) *
        static_cast<double>(path.size() - 1U);
    const auto lower = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(path_position)), path.size() - 2U);
    const auto upper = lower + 1U;
    const double local = sample == segment_count
        ? 1.0 : path_position - static_cast<double>(lower);
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> point{};
    for (uint32_t joint = 0; joint < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++joint) {
      point[joint] = static_cast<float>(
          static_cast<double>(path[lower][joint]) +
          local * static_cast<double>(path[upper][joint] - path[lower][joint]));
    }
    result.push_back(point);
  }
  result.front() = path.front();
  result.back() = path.back();
  return result;
}

double discrete_path_stretch_factor(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    double command_period_s, double maximum_velocity,
    double maximum_acceleration) {
  std::array<double, ARTICORE_PRODUCT_DUAL_ARM_DOF> previous_velocity{};
  double factor = 1.0;
  for (std::size_t sample = 1; sample < path.size(); ++sample) {
    for (uint32_t joint = 0; joint < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++joint) {
      const double velocity =
          static_cast<double>(path[sample][joint] - path[sample - 1U][joint]) /
          command_period_s;
      const double acceleration =
          (velocity - previous_velocity[joint]) / command_period_s;
      factor = std::max(factor, std::abs(velocity) / maximum_velocity);
      factor = std::max(
          factor, std::sqrt(std::abs(acceleration) / maximum_acceleration));
      previous_velocity[joint] = velocity;
    }
  }
  for (const double velocity : previous_velocity) {
    const double stopping_acceleration = velocity / command_period_s;
    factor = std::max(
        factor,
        std::sqrt(std::abs(stopping_acceleration) / maximum_acceleration));
  }
  return factor;
}

std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>
resample_realtime_pv_path_with_limits(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    double requested_duration_s, double maximum_velocity,
    double maximum_acceleration) {
  if (!std::isfinite(maximum_velocity) || maximum_velocity <= 0.0 ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration <= 0.0) {
    throw std::invalid_argument(
        "Cartesian PV velocity and acceleration must be finite and positive");
  }
  double candidate_duration_s = requested_duration_s;
  for (uint32_t attempt = 0; attempt < 12U; ++attempt) {
    auto result = resample_cartesian_path(
        path, candidate_duration_s, kLinearPvCommandPeriodSeconds);
    const double factor = discrete_path_stretch_factor(
        result, kLinearPvCommandPeriodSeconds, maximum_velocity,
        maximum_acceleration);
    if (factor <= 1.0001) return result;
    const double current_duration_s =
        static_cast<double>(result.size() - 1U) *
        kLinearPvCommandPeriodSeconds;
    candidate_duration_s = std::ceil(
        current_duration_s * factor * 1.01 /
        kLinearPvCommandPeriodSeconds) * kLinearPvCommandPeriodSeconds;
    if (!std::isfinite(candidate_duration_s) ||
        candidate_duration_s > 60.0) {
      throw std::invalid_argument(
          "Cartesian real-time PV path cannot satisfy velocity and "
          "acceleration limits within 60 seconds");
    }
  }
  throw std::invalid_argument(
      "Cartesian real-time PV path time parameterization did not converge");
}

std::string product_joint_role(uint32_t index) {
  return yunyi_joint_role(index);
}

ArticoreRobotPose robot_pose_from_rpy(const float* values) {
  if (!values) throw std::invalid_argument("target_pose is null");
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_POSE_DOF; ++index) {
    if (!std::isfinite(values[index])) {
      throw std::invalid_argument("target_pose contains NaN or Inf");
    }
  }
  const double roll = values[3];
  const double pitch = values[4];
  const double yaw = values[5];
  const double sr = std::sin(roll);
  const double cr = std::cos(roll);
  const double sp = std::sin(pitch);
  const double cp = std::cos(pitch);
  const double sy = std::sin(yaw);
  const double cy = std::cos(yaw);

  ArticoreRobotPose pose{};
  pose.struct_size = sizeof(pose);
  pose.position[0] = values[0];
  pose.position[1] = values[1];
  pose.position[2] = values[2];
  const double rotation[9] = {
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr};
  std::copy(std::begin(rotation), std::end(rotation), pose.rotation);
  return pose;
}

std::array<double, ARTICORE_PRODUCT_ARM_DOF> solve_path_ik(
    RobotModel& planning_model,
    YunyiRuntimeResources& product, uint32_t side,
    const ArticoreRobotPose& target,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& seed,
    const CartesianIkContinuityState* continuity,
    const char* context) {
  auto options = product_path_ik_options();
  auto preferred = predicted_cartesian_ik_posture(seed, continuity);
  const uint32_t offset = side * ARTICORE_PRODUCT_ARM_DOF;
  for (uint32_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
    preferred[joint] = std::clamp(
        preferred[joint],
        static_cast<double>(product.joints[offset + joint].lower),
        static_cast<double>(product.joints[offset + joint].upper));
  }
  ArticoreIkResult ik{};
  ik.struct_size = sizeof(ik);
  planning_model.ik_from_seed(
      &target, seed.data(), seed.size(), &options, &ik,
      preferred.data(), preferred.size());
  if (!ik.success || ik.dof != ARTICORE_PRODUCT_ARM_DOF) {
    throw std::invalid_argument(
        std::string(context) + " is unreachable; IK error_norm=" +
        std::to_string(ik.error_norm));
  }
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> result{};
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    const auto& joint = product.joints[offset + index];
    if (!std::isfinite(ik.q[index]) || ik.q[index] < joint.lower ||
        ik.q[index] > joint.upper) {
      std::ostringstream message;
      message << context << " IK result exceeds product position limits: "
              << product_joint_role(offset + index) << " position="
              << ik.q[index] << " rad, allowed=[" << joint.lower << ", "
              << joint.upper << "] rad";
      throw std::invalid_argument(message.str());
    }
    result[index] = ik.q[index];
  }
  return result;
}

std::array<double, ARTICORE_PRODUCT_ARM_DOF> solve_endpoint_ik(
    RobotModel& planning_model,
    YunyiRuntimeResources& product, uint32_t side,
    const ArticoreRobotPose& target,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& seed,
    const char* context,
    std::chrono::steady_clock::time_point deadline) {
  // set_pose has no sequential Cartesian samples to constrain the redundant arm.
  // Search the deterministic endpoint budget and select the valid joint
  // solution closest to the measured/planned seed.
  auto options = product_endpoint_ik_options();
  ArticoreIkResult ik{};
  ik.struct_size = sizeof(ik);
  planning_model.ik_nearest_until(
      &target, seed.data(), seed.size(), &options, deadline, &ik);
  if (!ik.success || ik.dof != ARTICORE_PRODUCT_ARM_DOF) {
    throw std::invalid_argument(
        std::string(context) + " is unreachable; IK error_norm=" +
        std::to_string(ik.error_norm));
  }
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> result{};
  const uint32_t offset = side * ARTICORE_PRODUCT_ARM_DOF;
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    const auto& joint = product.joints[offset + index];
    if (!std::isfinite(ik.q[index]) || ik.q[index] < joint.lower ||
        ik.q[index] > joint.upper) {
      std::ostringstream message;
      message << context << " IK result exceeds product position limits: "
              << product_joint_role(offset + index) << " position="
              << ik.q[index] << " rad, allowed=[" << joint.lower << ", "
              << joint.upper << "] rad";
      throw std::invalid_argument(message.str());
    }
    result[index] = ik.q[index];
  }
  return result;
}

NativeTrajectoryJoint trajectory_joint(
    const YunyiRuntimeResources::Joint& source,
    uint32_t product_index) {
  NativeTrajectoryJoint joint;
  joint.role = yunyi_joint_role(product_index);
  joint.motor = source.motor;
  joint.direction = source.direction;
  joint.velocity_command_scale = source.velocity_command_scale;
  joint.velocity_feedback_scale = source.velocity_feedback_scale;
  joint.torque_command_scale = source.torque_command_scale;
  joint.lower_position = source.lower;
  joint.upper_position = source.upper;
  joint.velocity_limit = source.velocity_limit;
  joint.acceleration_limit = source.acceleration_limit;
  joint.torque_limit = source.torque_limit;
  joint.mit_kp = source.kp;
  joint.mit_kd = source.kd;
  joint.mit_feedforward_torque = 0.0f;
  // Cartesian points execute through internal real-time PV with the speed-50
  // reference policy. Damiao's POS_VEL V field remains a separate 3 rad/s
  // trajectory catch-up ceiling.
  joint.pv_velocity_limit = product_pv_drive_velocity_limit(
      source.velocity_limit);
  joint.pv_hold_velocity_limit = std::min(
      joint.pv_velocity_limit, kNativePvFinalHoldVelocityLimit);
  return joint;
}

std::vector<double> plan_timestamps(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    double command_period_s) {
  std::vector<double> timestamps(path.size(), 0.0);
  for (std::size_t waypoint = 1; waypoint < path.size(); ++waypoint) {
    timestamps[waypoint] =
        timestamps[waypoint - 1] + command_period_s;
  }
  const double duration = timestamps.back();
  if (!std::isfinite(duration) || duration > 60.0) {
    throw std::invalid_argument("Cartesian motion requires an unsafe duration");
  }
  return timestamps;
}

void require_cartesian_duration(
    double duration_s, double command_period_s) {
  const double maximum_duration = std::min(
      60.0, kCartesianMaximumExecutionSegments * command_period_s);
  if (!std::isfinite(duration_s) || duration_s <= 0.0 ||
      duration_s > maximum_duration) {
    throw std::invalid_argument(
        "Cartesian duration_s must be finite and within (0," +
        std::to_string(maximum_duration) + "]");
  }
}

std::function<bool(const std::vector<float>&, std::string&)>
pose_convergence_check(
    YunyiRuntimeResources& product, uint32_t side,
    const ArticoreRobotPose& expected_pose, double position_tolerance,
    double orientation_tolerance, const char* context) {
  const uint32_t offset = side * ARTICORE_PRODUCT_ARM_DOF;
  auto* model = product.pose_models[side].get();
  auto* mutex = &product.pose_mutexes[side];
  const std::string label = context;
  return [model, mutex, offset, expected_pose, position_tolerance,
          orientation_tolerance, label](
             const std::vector<float>& actual_positions,
             std::string& error) {
    if (actual_positions.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
      error = label + " feedback has the wrong joint count";
      return false;
    }
    std::array<double, ARTICORE_PRODUCT_ARM_DOF> actual_q{};
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      actual_q[index] = actual_positions[offset + index];
    }
    ArticoreRobotPose actual_pose{};
    actual_pose.struct_size = sizeof(actual_pose);
    {
      std::lock_guard<std::mutex> lock(*mutex);
      model->fk(actual_q.data(), actual_q.size(), &actual_pose);
    }
    const double position_error = cartesian::norm(cartesian::subtract(
        actual_pose.position, expected_pose.position));
    const double orientation_error = cartesian::angular_distance(
        cartesian::quaternion_from_rotation(actual_pose.rotation),
        cartesian::quaternion_from_rotation(expected_pose.rotation));
    if (position_error <= position_tolerance &&
        orientation_error <= orientation_tolerance) {
      error.clear();
      return true;
    }
    error = label + " position_error=" + std::to_string(position_error) +
        " orientation_error=" + std::to_string(orientation_error) +
        " tolerances=[position<=" + std::to_string(position_tolerance) +
        ", orientation<=" + std::to_string(orientation_tolerance) + "]";
    return false;
  };
}

void prepend_cartesian_approach(
    NativeCartesianPlan& plan,
    YunyiRuntimeResources& product,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const ArticoreRobotPose& declared_start) {
  if (plan.trajectory.waypoints.empty() ||
      reference.positions.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
    throw std::runtime_error(
        "Cartesian approach requires a complete preplanned path");
  }
  const auto& approach_target = plan.trajectory.waypoints.front().positions;
  double minimum_approach_duration = 0.10;
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    const auto& joint = product.joints[index];
    const double velocity = std::max(
        0.01, static_cast<double>(std::min(
                  joint.velocity_limit,
                  yunyi_effective_pv_reference_velocity(
                      kCartesianRealtimePvSpeedPercent))));
    const double step = std::abs(
        static_cast<double>(approach_target[index]) -
        static_cast<double>(reference.positions[index]));
    minimum_approach_duration = std::max(
        minimum_approach_duration, step / velocity);
  }
  const double path_duration = plan.trajectory.waypoints.back().time_s;
  const double minimum_total_duration =
      minimum_approach_duration + path_duration;
  if (!std::isfinite(minimum_total_duration) ||
      minimum_total_duration > 60.0) {
    throw std::invalid_argument(
        "Cartesian approach requires an unsafe duration");
  }
  const double approach_duration = minimum_approach_duration;
  const double path_start_s = approach_duration;
  for (auto& waypoint : plan.trajectory.waypoints) {
    waypoint.time_s += path_start_s;
  }
  NativeTrajectoryWaypoint current;
  current.time_s = 0.0;
  current.positions = reference.positions;
  current.velocities.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  current.accelerations.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  const uint32_t all_joints =
      (uint32_t{1} << ARTICORE_PRODUCT_DUAL_ARM_DOF) - 1U;
  current.velocity_valid_mask = all_joints;
  current.acceleration_valid_mask = all_joints;
  // The shifted original first waypoint is already the approach endpoint and
  // the Cartesian path start. Reuse it so the boundary has one timestamp.
  auto& approach_end = plan.trajectory.waypoints.front();
  approach_end.velocities.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  approach_end.accelerations.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  approach_end.velocity_valid_mask = all_joints;
  approach_end.acceleration_valid_mask = all_joints;
  plan.trajectory.waypoints.insert(
      plan.trajectory.waypoints.begin(), std::move(current));
  plan.trajectory.approach_segment_count = 1;
  plan.trajectory.approach_deadline_s = path_start_s;
  plan.minimum_duration_s = minimum_total_duration;
  plan.trajectory.completion_deadline_s = minimum_total_duration;
  plan.trajectory.approach_convergence_check = pose_convergence_check(
      product, side, declared_start, 0.005, 0.035,
      "Cartesian approach start");
}

NativeCartesianPlan assemble_cartesian_plan(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    const std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& start_velocities,
    const std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& start_accelerations,
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    ArticoreRuntimeOperation operation,
    double duration_s,
    uint32_t completion_side,
    double command_period_s,
    float maximum_reference_acceleration =
        kYunyiTrajectoryPvAccelerationLimit) {
  require_cartesian_duration(duration_s, command_period_s);
  cartesian::require_pv_mode(mode);
  if (!std::isfinite(maximum_reference_acceleration) ||
      maximum_reference_acceleration <= 0.0f) {
    throw std::invalid_argument(
        "Cartesian PV acceleration must be finite and positive");
  }
  NativeCartesianPlan plan;
  auto timestamps = plan_timestamps(path, command_period_s);
  const double minimum_path_duration = timestamps.back();
  plan.minimum_duration_s = minimum_path_duration;
  auto& trajectory = plan.trajectory;
  trajectory.mode = mode;
  trajectory.operation = operation;
  trajectory.execution = NativeTrajectoryExecution::RealtimePv;
  trajectory.pv_reference_velocity =
      yunyi_effective_pv_reference_velocity(
          kCartesianRealtimePvSpeedPercent);
  trajectory.pv_reference_acceleration = maximum_reference_acceleration;
  trajectory.pv_drive_velocity_limit = kYunyiPvDriveVelocityLimit;
  trajectory.pv_reference_period_s = command_period_s;
  trajectory.allow_out_of_limit_start_recovery = true;
  trajectory.completion_deadline_s = minimum_path_duration;
  trajectory.cartesian_tracking_joint_mask =
      ((uint32_t{1} << ARTICORE_PRODUCT_ARM_DOF) - 1U) <<
      (completion_side * ARTICORE_PRODUCT_ARM_DOF);
  trajectory.joints.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    trajectory.joints.push_back(trajectory_joint(
        product.joints[index], index));
  }

  const uint32_t all_joints =
      (uint32_t{1} << ARTICORE_PRODUCT_DUAL_ARM_DOF) - 1U;
  trajectory.waypoints.reserve(path.size());
  for (std::size_t index = 0; index < path.size(); ++index) {
    NativeTrajectoryWaypoint waypoint;
    waypoint.time_s = timestamps[index];
    waypoint.positions.assign(path[index].begin(), path[index].end());
    waypoint.velocities.resize(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
    waypoint.accelerations.resize(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
    if (index == 0) {
      waypoint.velocities.assign(
          start_velocities.begin(), start_velocities.end());
      waypoint.accelerations.assign(
          start_accelerations.begin(), start_accelerations.end());
      waypoint.velocity_valid_mask = all_joints;
      waypoint.acceleration_valid_mask = all_joints;
    } else if (index + 1 == path.size()) {
      waypoint.velocity_valid_mask = all_joints;
      waypoint.acceleration_valid_mask = all_joints;
    }
    trajectory.waypoints.push_back(std::move(waypoint));
  }
  const uint32_t completion_offset =
      completion_side * ARTICORE_PRODUCT_ARM_DOF;
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> expected_q{};
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    expected_q[index] = path.back()[completion_offset + index];
  }
  ArticoreRobotPose expected_pose{};
  expected_pose.struct_size = sizeof(expected_pose);
  {
    std::lock_guard<std::mutex> lock(product.pose_mutexes[completion_side]);
    product.pose_models[completion_side]->fk(
        expected_q.data(), expected_q.size(), &expected_pose);
  }
  trajectory.final_convergence_check = pose_convergence_check(
      product, completion_side, expected_pose, 0.005, 0.035,
      "Cartesian endpoint");
  return plan;
}

void require_cartesian_reference(
    const NativeTrajectorySample& reference, const char* motion_name) {
  if (reference.active &&
      reference.operation != ARTICORE_OPERATION_SET_POSE &&
      reference.operation != ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY &&
      reference.operation != ARTICORE_OPERATION_MOVE_CIRCULAR_TRAJECTORY) {
    throw std::runtime_error(
        std::string(motion_name) +
        " cannot follow the current planned Runtime operation");
  }
  if (reference.positions.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF ||
      reference.velocities.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF ||
      reference.accelerations.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
    throw std::runtime_error(
        std::string(motion_name) +
        " requires a complete current planned reference");
  }
}

std::array<double, ARTICORE_PRODUCT_ARM_DOF> reference_q(
    const NativeTrajectorySample& reference, uint32_t side) {
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> q{};
  const uint32_t offset = side * ARTICORE_PRODUCT_ARM_DOF;
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    q[index] = reference.positions[offset + index];
  }
  return q;
}

struct CartesianCornerBlend {
  bool active = false;
  ArticoreRobotPose entry{};
  ArticoreRobotPose exit{};
  cartesian::Vector3 center{};
  cartesian::Vector3 radial_axis{};
  cartesian::Vector3 tangent_axis{};
  double radius = 0.0;
  double angle = 0.0;
  cartesian::Quaternion entry_orientation{};
  cartesian::Quaternion exit_orientation{};
};

ArticoreRobotPose cartesian_pose(
    const cartesian::Vector3& position,
    const cartesian::Quaternion& orientation) {
  ArticoreRobotPose result{};
  result.struct_size = sizeof(result);
  std::copy(position.begin(), position.end(), result.position);
  cartesian::rotation_from_quaternion(orientation, result.rotation);
  return result;
}

CartesianCornerBlend make_corner_blend(
    const ArticoreRobotPose& previous,
    const ArticoreRobotPose& corner,
    const ArticoreRobotPose& next) {
  constexpr double pi = 3.14159265358979323846;
  CartesianCornerBlend result;
  const auto incoming_delta = cartesian::subtract(
      corner.position, previous.position);
  const auto outgoing_delta = cartesian::subtract(
      next.position, corner.position);
  const double incoming_length = cartesian::norm(incoming_delta);
  const double outgoing_length = cartesian::norm(outgoing_delta);
  if (!std::isfinite(incoming_length) || !std::isfinite(outgoing_length) ||
      incoming_length <= 1e-6 || outgoing_length <= 1e-6) {
    return result;
  }
  const auto incoming = cartesian::scale(
      incoming_delta, 1.0 / incoming_length);
  const auto outgoing = cartesian::scale(
      outgoing_delta, 1.0 / outgoing_length);
  const double turn_angle = std::acos(std::clamp(
      cartesian::dot(incoming, outgoing), -1.0, 1.0));
  if (!std::isfinite(turn_angle) || turn_angle <= 1e-4 ||
      turn_angle >= pi - 1e-4) {
    return result;
  }
  const double tangent = std::tan(0.5 * turn_angle);
  if (!std::isfinite(tangent) || tangent <= 1e-6) return result;

  // Reserve at most one quarter of either adjacent segment for this corner.
  // Consequently two neighboring blends can never overlap on their shared
  // segment. Ten millimetres is the normal radius; short segments scale down.
  const double maximum_trim = 0.25 * std::min(
      incoming_length, outgoing_length);
  const double radius = std::min(
      kYunyiCartesianDefaultBlendRadius, maximum_trim / tangent);
  const double trim = radius * tangent;
  if (!std::isfinite(radius) || radius <= 1e-5 ||
      !std::isfinite(trim) || trim <= 1e-6) {
    return result;
  }

  const cartesian::Vector3 corner_position{
      corner.position[0], corner.position[1], corner.position[2]};
  const auto entry_position = cartesian::add(
      corner_position, cartesian::scale(incoming, -trim));
  const auto exit_position = cartesian::add(
      corner_position, cartesian::scale(outgoing, trim));
  auto normal = cartesian::cross(incoming, outgoing);
  const double normal_length = cartesian::norm(normal);
  if (!std::isfinite(normal_length) || normal_length <= 1e-8) return result;
  normal = cartesian::scale(normal, 1.0 / normal_length);
  const auto inward = cartesian::cross(normal, incoming);
  result.center = cartesian::add(
      entry_position, cartesian::scale(inward, radius));
  result.radial_axis = cartesian::scale(
      cartesian::subtract(entry_position.data(), result.center.data()),
      1.0 / radius);
  result.tangent_axis = cartesian::cross(normal, result.radial_axis);
  result.radius = radius;
  result.angle = turn_angle;

  const auto previous_orientation =
      cartesian::quaternion_from_rotation(previous.rotation);
  const auto corner_orientation =
      cartesian::quaternion_from_rotation(corner.rotation);
  const auto next_orientation =
      cartesian::quaternion_from_rotation(next.rotation);
  result.entry_orientation = cartesian::slerp(
      previous_orientation, corner_orientation,
      1.0 - trim / incoming_length);
  result.exit_orientation = cartesian::slerp(
      corner_orientation, next_orientation, trim / outgoing_length);
  result.entry = cartesian_pose(entry_position, result.entry_orientation);
  result.exit = cartesian_pose(exit_position, result.exit_orientation);
  result.active = true;
  return result;
}

void append_linear_pose_samples(
    std::vector<ArticoreRobotPose>& output,
    const ArticoreRobotPose& start,
    const ArticoreRobotPose& end) {
  const auto start_orientation =
      cartesian::quaternion_from_rotation(start.rotation);
  const auto end_orientation =
      cartesian::quaternion_from_rotation(end.rotation);
  const double translation = cartesian::norm(cartesian::subtract(
      end.position, start.position));
  const double orientation = cartesian::angular_distance(
      start_orientation, end_orientation);
  const uint32_t segments = std::max<uint32_t>(
      1U, static_cast<uint32_t>(std::ceil(std::max(
          translation / kYunyiLinearTranslationSampleDistance,
          orientation / kYunyiLinearOrientationSampleDistance))));
  for (uint32_t sample = 1; sample <= segments; ++sample) {
    const double amount = static_cast<double>(sample) /
        static_cast<double>(segments);
    const auto position = cartesian::lerp_position(
        start.position, end.position, amount);
    output.push_back(cartesian_pose(
        position, cartesian::slerp(
            start_orientation, end_orientation, amount)));
  }
}

void append_corner_pose_samples(
    std::vector<ArticoreRobotPose>& output,
    const CartesianCornerBlend& blend) {
  const double arc_length = blend.radius * blend.angle;
  const double orientation = cartesian::angular_distance(
      blend.entry_orientation, blend.exit_orientation);
  const uint32_t segments = std::max<uint32_t>(
      1U, static_cast<uint32_t>(std::ceil(std::max(
          arc_length / kYunyiLinearTranslationSampleDistance,
          orientation / kYunyiLinearOrientationSampleDistance))));
  for (uint32_t sample = 1; sample <= segments; ++sample) {
    const double amount = static_cast<double>(sample) /
        static_cast<double>(segments);
    const double angle = blend.angle * amount;
    const auto position = cartesian::add(
        blend.center,
        cartesian::scale(
            cartesian::add(
                cartesian::scale(blend.radial_axis, std::cos(angle)),
                cartesian::scale(blend.tangent_axis, std::sin(angle))),
            blend.radius));
    output.push_back(cartesian_pose(
        position, cartesian::slerp(
            blend.entry_orientation, blend.exit_orientation, amount)));
  }
  output.back() = blend.exit;
}

std::vector<ArticoreRobotPose> blended_linear_pose_samples(
    const std::vector<ArticoreRobotPose>& poses) {
  if (poses.size() < 2) {
    throw std::invalid_argument("linear path requires at least two poses");
  }
  std::vector<CartesianCornerBlend> blends(poses.size());
  for (std::size_t index = 1; index + 1 < poses.size(); ++index) {
    blends[index] = make_corner_blend(
        poses[index - 1], poses[index], poses[index + 1]);
  }
  std::vector<ArticoreRobotPose> result;
  result.push_back(poses.front());
  auto current = poses.front();
  for (std::size_t edge = 0; edge + 1 < poses.size(); ++edge) {
    const std::size_t corner = edge + 1;
    const auto& destination = blends[corner].active
        ? blends[corner].entry : poses[corner];
    append_linear_pose_samples(result, current, destination);
    if (blends[corner].active) {
      append_corner_pose_samples(result, blends[corner]);
      current = blends[corner].exit;
    } else {
      current = poses[corner];
    }
  }
  if (result.size() > 1025U) {
    throw std::invalid_argument(
        "blended linear path is too long for one native transaction");
  }
  result.front() = poses.front();
  result.back() = poses.back();
  return result;
}

NativeCartesianPlan build_linear_path_plan_common(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const std::vector<ArticoreRobotPose>& control_poses,
    double segment_duration_s,
    float pv_reference_acceleration) {
  cartesian::require_pv_mode(mode);
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  if (!std::isfinite(segment_duration_s) || segment_duration_s <= 0.0) {
    throw std::invalid_argument(
        "linear path segment_duration_s must be finite and positive");
  }
  const double total_duration_s = segment_duration_s *
      static_cast<double>(control_poses.size() - 1U);
  require_cartesian_duration(
      total_duration_s, kLinearPvCommandPeriodSeconds);
  require_cartesian_reference(reference, "linear path");

  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_positions{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_velocities{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_accelerations{};
  std::copy(reference.positions.begin(), reference.positions.end(),
            start_positions.begin());
  std::copy(reference.velocities.begin(), reference.velocities.end(),
            start_velocities.begin());
  std::copy(reference.accelerations.begin(), reference.accelerations.end(),
            start_accelerations.begin());

  RobotModel planning_model("yunyi_v1_0", side, product.tcp_offsets[side]);
  auto seed = reference_q(reference, side);
  const uint32_t side_offset = side * ARTICORE_PRODUCT_ARM_DOF;
  const auto pose_path = blended_linear_pose_samples(control_poses);
  std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>> joint_path;
  joint_path.reserve(pose_path.size());
  joint_path.push_back(start_positions);
  CartesianIkContinuityState ik_continuity;
  for (std::size_t sample = 1; sample < pose_path.size(); ++sample) {
    const char* const context = sample + 1U == pose_path.size()
        ? "Cartesian linear path target"
        : "Cartesian blended linear path sample";
    const auto solved = solve_path_ik(
        planning_model, product, side, pose_path[sample], seed,
        &ik_continuity, context);
    require_cartesian_ik_continuity(
        side, seed, solved, ik_continuity, context, sample);
    seed = solved;
    auto waypoint = start_positions;
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      waypoint[side_offset + index] = static_cast<float>(seed[index]);
    }
    joint_path.push_back(waypoint);
  }
  auto execution_path = resample_realtime_pv_path_with_limits(
      joint_path, total_duration_s,
      yunyi_effective_pv_reference_velocity(
          kCartesianRealtimePvSpeedPercent),
      pv_reference_acceleration);
  return assemble_cartesian_plan(
      execution_path, start_velocities, start_accelerations, product, mode,
      ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY, total_duration_s, side,
      kLinearPvCommandPeriodSeconds, pv_reference_acceleration);
}

NativeCartesianPlan build_linear_plan_common(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const ArticoreRobotPose* declared_start,
    const float* end_pose_values,
    double duration_s,
    float pv_reference_acceleration) {
  cartesian::require_pv_mode(mode);
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  require_cartesian_duration(duration_s, kLinearPvCommandPeriodSeconds);
  require_cartesian_reference(reference, "linear");
  if (declared_start) {
    validate_cartesian_start_pose(
        product.tcp_offsets[side], side, reference, *declared_start, "linear");
  }

  const auto end_pose = robot_pose_from_rpy(end_pose_values);
  RobotModel planning_model("yunyi_v1_0", side, product.tcp_offsets[side]);
  const auto start_q = reference_q(reference, side);
  ArticoreRobotPose actual_start{};
  actual_start.struct_size = sizeof(actual_start);
  planning_model.fk(start_q.data(), start_q.size(), &actual_start);
  const auto& geometric_start = declared_start ? *declared_start : actual_start;
  const auto start_orientation =
      cartesian::quaternion_from_rotation(geometric_start.rotation);
  const auto end_orientation =
      cartesian::quaternion_from_rotation(end_pose.rotation);
  const double orientation_distance = cartesian::angular_distance(
      start_orientation, end_orientation);
  const double translation_distance = cartesian::norm(cartesian::subtract(
      end_pose.position, geometric_start.position));
  const double required_segments = std::ceil(std::max(
      translation_distance / kYunyiLinearTranslationSampleDistance,
      orientation_distance / kYunyiLinearOrientationSampleDistance));
  if (!std::isfinite(required_segments) || required_segments > 255.0) {
    throw std::invalid_argument(
        "Cartesian linear path is too long for one native transaction");
  }
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_positions{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_velocities{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_accelerations{};
  std::copy(reference.positions.begin(), reference.positions.end(),
            start_positions.begin());
  std::copy(reference.velocities.begin(), reference.velocities.end(),
            start_velocities.begin());
  std::copy(reference.accelerations.begin(), reference.accelerations.end(),
            start_accelerations.begin());

  const uint32_t sample_count =
      static_cast<uint32_t>(required_segments) + 1U;
  const uint32_t side_offset = side * ARTICORE_PRODUCT_ARM_DOF;
  std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>> path;
  path.reserve(sample_count);
  path.push_back(start_positions);
  auto seed = start_q;
  CartesianIkContinuityState ik_continuity;
  for (uint32_t sample = 1; sample < sample_count; ++sample) {
    const double amount = static_cast<double>(sample) /
        static_cast<double>(sample_count - 1);
    ArticoreRobotPose sample_pose{};
    sample_pose.struct_size = sizeof(sample_pose);
    const auto position = cartesian::lerp_position(
        geometric_start.position, end_pose.position, amount);
    std::copy(position.begin(), position.end(), sample_pose.position);
    const auto orientation = cartesian::slerp(
        start_orientation, end_orientation, amount);
    cartesian::rotation_from_quaternion(orientation, sample_pose.rotation);
    const char* const context = sample + 1 == sample_count
        ? "Cartesian linear target"
        : "Cartesian linear path sample";
    const auto solved = solve_path_ik(
        planning_model, product, side, sample_pose, seed,
        &ik_continuity, context);
    require_cartesian_ik_continuity(
        side, seed, solved, ik_continuity, context, sample);
    seed = solved;
    auto waypoint = start_positions;
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      waypoint[side_offset + index] = static_cast<float>(seed[index]);
    }
    path.push_back(waypoint);
  }

  auto execution_path = resample_realtime_pv_path_with_limits(
      path, duration_s,
      yunyi_effective_pv_reference_velocity(
          kCartesianRealtimePvSpeedPercent),
      pv_reference_acceleration);
  return assemble_cartesian_plan(
      execution_path, start_velocities, start_accelerations, product, mode,
      ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY, duration_s, side,
      kLinearPvCommandPeriodSeconds, pv_reference_acceleration);
}

}  // namespace

void require_cartesian_ik_continuity(
    uint32_t side,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& previous,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& current,
    CartesianIkContinuityState& state,
    const char* context,
    std::size_t sample_index) {
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument("Cartesian IK continuity side is invalid");
  }
  const std::string label = context ? context : "Cartesian path";
  for (uint32_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
    const double step = current[joint] - previous[joint];
    if (!std::isfinite(step)) {
      throw std::invalid_argument(
          label + " contains a non-finite IK joint step");
    }
    if (std::abs(step) > kYunyiCartesianMaximumIkJointStep) {
      std::ostringstream message;
      message << label << " changes to a discontinuous IK branch at sample "
              << sample_index << ": "
              << product_joint_role(
                     side * ARTICORE_PRODUCT_ARM_DOF + joint)
              << " step=" << step << " rad exceeds "
              << kYunyiCartesianMaximumIkJointStep << " rad";
      throw std::invalid_argument(message.str());
    }

    state.previous_steps[joint] = step;
  }
  state.has_previous_step = true;
}

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* end_pose_values,
    double duration_s,
    float pv_reference_acceleration,
    uint32_t) {
  return build_linear_plan_common(
      product, mode, side, reference, nullptr, end_pose_values,
      duration_s, pv_reference_acceleration);
}

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose_values,
    const float* end_pose_values,
    double duration_s,
    float pv_reference_acceleration,
    uint32_t) {
  const auto declared_start = robot_pose_from_rpy(start_pose_values);
  try {
    validate_cartesian_start_pose(
        product.tcp_offsets[side], side, reference, declared_start, "linear");
    return build_linear_plan_common(
        product, mode, side, reference, &declared_start, end_pose_values,
        duration_s, pv_reference_acceleration);
  } catch (const std::invalid_argument& error) {
    if (std::string(error.what()).find(
            "start_pose does not match current planned pose") ==
        std::string::npos) {
      throw;
    }
  }

  const auto approach_target = solve_path_start_target_from_reference(
      product, mode, side, reference, start_pose_values);
  auto path_reference = reference;
  path_reference.positions.assign(
      approach_target.begin(), approach_target.end());
  path_reference.velocities.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  path_reference.accelerations.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  auto plan = build_linear_plan_common(
      product, mode, side, path_reference, &declared_start, end_pose_values,
      duration_s, pv_reference_acceleration);
  prepend_cartesian_approach(
      plan, product, side, reference, declared_start);
  return plan;
}

NativeCartesianPlan build_linear_path_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* pose_values,
    uint32_t pose_count,
    double segment_duration_s,
    float pv_reference_acceleration) {
  cartesian::require_pv_mode(mode);
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  if (!pose_values) {
    throw std::invalid_argument("linear path poses are null");
  }
  if (pose_count < 2U || pose_count > 64U) {
    throw std::invalid_argument(
        "linear path pose_count must be within [2,64]");
  }
  std::vector<ArticoreRobotPose> poses;
  poses.reserve(pose_count);
  for (uint32_t index = 0; index < pose_count; ++index) {
    poses.push_back(robot_pose_from_rpy(
        pose_values + index * ARTICORE_PRODUCT_POSE_DOF));
  }
  const auto& declared_start = poses.front();
  try {
    validate_cartesian_start_pose(
        product.tcp_offsets[side], side, reference, declared_start,
        "linear path");
    return build_linear_path_plan_common(
        product, mode, side, reference, poses, segment_duration_s,
        pv_reference_acceleration);
  } catch (const std::invalid_argument& error) {
    if (std::string(error.what()).find(
            "start_pose does not match current planned pose") ==
        std::string::npos) {
      throw;
    }
  }

  const auto approach_target = solve_path_start_target_from_reference(
      product, mode, side, reference, pose_values);
  auto path_reference = reference;
  path_reference.positions.assign(
      approach_target.begin(), approach_target.end());
  path_reference.velocities.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  path_reference.accelerations.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  auto plan = build_linear_path_plan_common(
      product, mode, side, path_reference, poses, segment_duration_s,
      pv_reference_acceleration);
  prepend_cartesian_approach(
      plan, product, side, reference, declared_start);
  return plan;
}

std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
solve_path_start_target_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* target_pose_values) {
  if (mode != ARTICORE_MODE_PV && mode != ARTICORE_MODE_MIT) {
    throw std::invalid_argument(
        "Cartesian point-to-point control mode is invalid");
  }
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  require_cartesian_reference(reference, "point-to-point");

  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_positions{};
  std::copy(reference.positions.begin(), reference.positions.end(),
            start_positions.begin());

  const uint32_t side_offset = side * ARTICORE_PRODUCT_ARM_DOF;
  const auto target = robot_pose_from_rpy(target_pose_values);
  const auto start_q = reference_q(reference, side);
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> target_q{};
  {
    std::lock_guard<std::mutex> lock(product.pose_mutexes[side]);
    if (!product.pose_models[side]) {
      throw std::runtime_error("Cartesian point-to-point model is unavailable");
    }
    target_q = solve_path_ik(
        *product.pose_models[side], product, side, target, start_q,
        nullptr, "Cartesian path start");
  }

  auto result = start_positions;
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    result[side_offset + index] = static_cast<float>(target_q[index]);
  }
  return result;
}

std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
solve_dual_point_to_point_targets_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    const NativeTrajectorySample& reference,
    const float* left_target_pose,
    const float* right_target_pose,
    std::chrono::steady_clock::time_point deadline) {
  if (mode != ARTICORE_MODE_PV && mode != ARTICORE_MODE_MIT) {
    throw std::invalid_argument("product inverse kinematics mode is invalid");
  }
  if (!left_target_pose || !right_target_pose) {
    throw std::invalid_argument(
        "dual point-to-point requires both left and right target poses");
  }
  require_cartesian_reference(reference, "dual point-to-point");

  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> result{};
  std::copy(reference.positions.begin(), reference.positions.end(),
            result.begin());
  auto solve_side = [&](uint32_t side) {
    const float* target_values = side == ARTICORE_ROBOT_LEFT
        ? left_target_pose : right_target_pose;
    const auto target = robot_pose_from_rpy(target_values);
    const auto start_q = reference_q(reference, side);
    std::lock_guard<std::mutex> lock(product.pose_mutexes[side]);
    if (!product.pose_models[side]) {
      throw std::runtime_error("dual point-to-point model is unavailable");
    }
    return solve_endpoint_ik(
        *product.pose_models[side], product, side, target, start_q,
        side == ARTICORE_ROBOT_LEFT
            ? "left Cartesian point-to-point target"
            : "right Cartesian point-to-point target",
        deadline);
  };
  auto left = std::async(std::launch::async, solve_side, ARTICORE_ROBOT_LEFT);
  auto right = std::async(std::launch::async, solve_side, ARTICORE_ROBOT_RIGHT);
  const std::array<std::array<double, ARTICORE_PRODUCT_ARM_DOF>, 2> targets{
      left.get(), right.get()};
  for (uint32_t side = ARTICORE_ROBOT_LEFT;
       side <= ARTICORE_ROBOT_RIGHT; ++side) {
    const uint32_t offset = side * ARTICORE_PRODUCT_ARM_DOF;
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      result[offset + index] = static_cast<float>(targets[side][index]);
    }
  }
  return result;
}

std::vector<NativeTrajectoryJoint> product_cartesian_joints(
    const YunyiRuntimeResources& product) {
  std::vector<NativeTrajectoryJoint> joints;
  joints.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    joints.push_back(trajectory_joint(product.joints[index], index));
  }
  return joints;
}

namespace {

NativeCartesianPlan build_circular_plan_common(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const ArticoreRobotPose* declared_start,
    const float* via_pose_values,
    const float* end_pose_values,
    double duration_s,
    float pv_reference_acceleration) {
  cartesian::require_pv_mode(mode);
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  require_cartesian_duration(duration_s, kCircularPvCommandPeriodSeconds);
  const auto via = robot_pose_from_rpy(via_pose_values);
  const auto end = robot_pose_from_rpy(end_pose_values);
  RobotModel planning_model("yunyi_v1_0", side, product.tcp_offsets[side]);
  require_cartesian_reference(reference, "circular");

  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_positions{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_velocities{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_accelerations{};
  std::copy(reference.positions.begin(), reference.positions.end(),
            start_positions.begin());
  std::copy(reference.velocities.begin(), reference.velocities.end(),
            start_velocities.begin());
  std::copy(reference.accelerations.begin(), reference.accelerations.end(),
            start_accelerations.begin());

  const uint32_t side_offset = side * ARTICORE_PRODUCT_ARM_DOF;
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> start_q{};
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    start_q[index] = start_positions[side_offset + index];
  }
  ArticoreRobotPose actual_start{};
  actual_start.struct_size = sizeof(actual_start);
  planning_model.fk(start_q.data(), start_q.size(), &actual_start);
  if (declared_start) {
    validate_cartesian_start_pose(
        product.tcp_offsets[side], side, reference, *declared_start, "circular");
  }
  const auto& geometric_start = declared_start ? *declared_start : actual_start;
  const auto geometric_start_orientation =
      cartesian::quaternion_from_rotation(geometric_start.rotation);
  const auto arc = cartesian::circular_arc_from_three_points(
      geometric_start.position, via.position, end.position);
  const auto via_orientation =
      cartesian::quaternion_from_rotation(via.rotation);
  const auto end_orientation =
      cartesian::quaternion_from_rotation(end.rotation);

  const double first_arc_length = arc.radius * arc.via_angle;
  const double second_arc_length =
      arc.radius * (arc.end_angle - arc.via_angle);
  const double first_orientation = cartesian::angular_distance(
      geometric_start_orientation, via_orientation);
  const double second_orientation = cartesian::angular_distance(
      via_orientation, end_orientation);
  const uint32_t geometry_first_segments = std::max<uint32_t>(
      1U, static_cast<uint32_t>(std::ceil(std::max(
          first_arc_length / kYunyiCircularTranslationSampleDistance,
          first_orientation / kYunyiCircularOrientationSampleDistance))));
  const uint32_t geometry_second_segments = std::max<uint32_t>(
      1U, static_cast<uint32_t>(std::ceil(std::max(
          second_arc_length / kYunyiCircularTranslationSampleDistance,
          second_orientation / kYunyiCircularOrientationSampleDistance))));
  const uint32_t geometry_segments =
      geometry_first_segments + geometry_second_segments;
  if (geometry_segments > kCircularMaximumGeometrySegments) {
    throw std::invalid_argument(
        "circular path is too long for one native transaction");
  }
  const uint32_t first_segments = geometry_first_segments;
  const uint32_t second_segments = geometry_second_segments;
  const double via_path_amount = arc.via_angle / arc.end_angle;

  std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>> path;
  path.reserve(first_segments + second_segments + 1U);
  path.push_back(start_positions);
  auto seed = start_q;
  CartesianIkContinuityState ik_continuity;
  const auto append_sample = [&](double angle,
                                 const cartesian::Quaternion& orientation,
                                 const char* context,
                                 uint32_t sample_index,
                                 auto& mutable_seed,
                                 auto& mutable_path) {
    ArticoreRobotPose sample_pose{};
    sample_pose.struct_size = sizeof(sample_pose);
    const auto position = arc.position(angle);
    std::copy(position.begin(), position.end(), sample_pose.position);
    cartesian::rotation_from_quaternion(
        orientation, sample_pose.rotation);
    const auto solved = solve_path_ik(
        planning_model, product, side, sample_pose, mutable_seed,
        &ik_continuity, context);
    require_cartesian_ik_continuity(
        side, mutable_seed, solved, ik_continuity, context, sample_index);
    mutable_seed = solved;
    auto waypoint = start_positions;
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      waypoint[side_offset + index] =
          static_cast<float>(mutable_seed[index]);
    }
    mutable_path.push_back(waypoint);
  };

  for (uint32_t sample = 1; sample <= first_segments; ++sample) {
    const double amount = static_cast<double>(sample) /
        static_cast<double>(first_segments);
    const double angle = amount * arc.via_angle;
    append_sample(
        angle,
        cartesian::circular_slerp_through_via(
            geometric_start_orientation, via_orientation, end_orientation,
            via_path_amount, angle / arc.end_angle),
        "Cartesian circular start-to-via path", sample, seed, path);
  }
  for (uint32_t sample = 1; sample <= second_segments; ++sample) {
    const double amount = static_cast<double>(sample) /
        static_cast<double>(second_segments);
    const double angle =
        arc.via_angle + amount * (arc.end_angle - arc.via_angle);
    append_sample(
        angle,
        cartesian::circular_slerp_through_via(
            geometric_start_orientation, via_orientation, end_orientation,
            via_path_amount, angle / arc.end_angle),
        "Cartesian circular via-to-end path",
        first_segments + sample, seed, path);
  }

  auto execution_path = resample_realtime_pv_path_with_limits(
      path, duration_s,
      yunyi_effective_pv_reference_velocity(
          kCartesianRealtimePvSpeedPercent),
      pv_reference_acceleration);
  return assemble_cartesian_plan(
      execution_path, start_velocities, start_accelerations, product, mode,
      ARTICORE_OPERATION_MOVE_CIRCULAR_TRAJECTORY, duration_s, side,
      kCircularPvCommandPeriodSeconds, pv_reference_acceleration);
}

}  // namespace

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* via_pose_values,
    const float* end_pose_values,
    double duration_s,
    float pv_reference_acceleration) {
  return build_circular_plan_common(
      product, mode, side, reference, nullptr, via_pose_values,
      end_pose_values, duration_s, pv_reference_acceleration);
}

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose_values,
    const float* via_pose_values,
    const float* end_pose_values,
    double duration_s,
    float pv_reference_acceleration) {
  const auto declared_start = robot_pose_from_rpy(start_pose_values);
  try {
    validate_cartesian_start_pose(
        product.tcp_offsets[side], side, reference, declared_start, "circular");
    return build_circular_plan_common(
        product, mode, side, reference, &declared_start, via_pose_values,
        end_pose_values, duration_s, pv_reference_acceleration);
  } catch (const std::invalid_argument& error) {
    if (std::string(error.what()).find(
            "start_pose does not match current planned pose") ==
        std::string::npos) {
      throw;
    }
  }

  const auto approach_target = solve_path_start_target_from_reference(
      product, mode, side, reference, start_pose_values);
  auto path_reference = reference;
  path_reference.positions.assign(
      approach_target.begin(), approach_target.end());
  path_reference.velocities.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  path_reference.accelerations.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  auto plan = build_circular_plan_common(
      product, mode, side, path_reference, &declared_start, via_pose_values,
      end_pose_values, duration_s, pv_reference_acceleration);
  prepend_cartesian_approach(
      plan, product, side, reference, declared_start);
  return plan;
}

}  // namespace articore
