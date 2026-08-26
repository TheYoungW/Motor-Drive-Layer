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
    "Cartesian and ordinary PV must share the same drive velocity ceiling");

namespace {

// IK samples describe Cartesian geometry; the 500 Hz worker still evaluates
// the resulting C2-continuous joint trajectory every control cycle.
constexpr double kCartesianTranslationSampleDistance = 0.005;
constexpr double kCartesianOrientationSampleDistance = 0.035;

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

std::array<double, ARTICORE_PRODUCT_ARM_DOF> solve_ik(
    RobotModel& planning_model,
    YunyiRuntimeResources& product, uint32_t side,
    const ArticoreRobotPose& target,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& seed,
    const char* context,
    CartesianIkSearch search = CartesianIkSearch::LocalPath) {
  auto options = product_cartesian_ik_options(search);
  ArticoreIkResult ik{};
  ik.struct_size = sizeof(ik);
  if (search == CartesianIkSearch::GlobalEndpoint) {
    planning_model.ik_nearest(
        &target, seed.data(), seed.size(), &options, &ik);
  } else {
    planning_model.ik(
        &target, seed.data(), seed.size(), &options, &ik);
    bool distant_from_seed = false;
    if (ik.success && ik.dof == ARTICORE_PRODUCT_ARM_DOF) {
      for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
        distant_from_seed = distant_from_seed ||
            std::abs(ik.q[index] - seed[index]) > 0.35;
      }
    }
    if (distant_from_seed) {
      ik = {};
      ik.struct_size = sizeof(ik);
      planning_model.ik_nearest(
          &target, seed.data(), seed.size(), &options, &ik);
    }
  }
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

std::array<double, ARTICORE_PRODUCT_ARM_DOF> solve_endpoint_ik(
    RobotModel& planning_model,
    YunyiRuntimeResources& product, uint32_t side,
    const ArticoreRobotPose& target,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& seed,
    const char* context,
    std::chrono::steady_clock::time_point deadline) {
  // PTP has no sequential Cartesian samples to constrain the redundant arm.
  // Search the deterministic endpoint budget and select the valid joint
  // solution closest to the measured/planned seed.
  auto options = product_cartesian_ik_options(
      CartesianIkSearch::GlobalEndpoint);
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
  // Cartesian duration controls native trajectory timing. Damiao's POS_VEL V
  // field is a separate product ceiling and remains 3 rad/s.
  joint.pv_velocity_limit = product_pv_drive_velocity_limit(
      source.velocity_limit);
  joint.pv_hold_velocity_limit = std::min(
      joint.pv_velocity_limit, kNativePvFinalHoldVelocityLimit);
  return joint;
}

std::vector<double> plan_timestamps(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    const YunyiRuntimeResources& product,
    float maximum_reference_velocity) {
  std::vector<double> timestamps(path.size(), 0.0);
  for (std::size_t waypoint = 1; waypoint < path.size(); ++waypoint) {
    double segment_duration = 0.002;
    for (uint32_t joint_index = 0;
         joint_index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++joint_index) {
      const auto& joint = product.joints[joint_index];
      const double velocity = std::max(
          0.01, static_cast<double>(
                    std::min(joint.velocity_limit,
                             maximum_reference_velocity)));
      const double step = std::abs(static_cast<double>(
          path[waypoint][joint_index] -
          path[waypoint - 1][joint_index]));
      segment_duration = std::max(segment_duration, step / velocity);
      // Cartesian samples are joined by quintics whose endpoint derivatives
      // are inferred from neighbouring samples. A velocity-only duration can
      // make the first/last derivative ramp arbitrarily sharp as the Cartesian
      // sampling interval gets smaller. Reserve enough time for that ramp as
      // well; the factor bounds the quintic extrema conservatively and gives
      // duration validation a conservative product minimum.
      const double acceleration = std::max(
          0.01, static_cast<double>(product_cartesian_acceleration_limit(
                    joint_index, joint.acceleration_limit)));
      segment_duration = std::max(
          segment_duration, std::sqrt(8.0 * step / acceleration));
    }
    timestamps[waypoint] = timestamps[waypoint - 1] + segment_duration;
  }
  const double duration = timestamps.back();
  if (!std::isfinite(duration) || duration > 60.0) {
    throw std::invalid_argument("Cartesian motion requires an unsafe duration");
  }
  if (duration < 0.10) {
    const double stretch = 0.10 / duration;
    for (std::size_t index = 1; index < timestamps.size(); ++index) {
      timestamps[index] *= stretch;
    }
  }
  return timestamps;
}

void require_cartesian_duration(double duration_s) {
  if (!std::isfinite(duration_s) || duration_s <= 0.0 ||
      duration_s > 3600.0) {
    throw std::invalid_argument(
        "Cartesian duration_s must be finite and within (0,3600]");
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
    const ArticoreRobotPose& declared_start,
    double total_duration_s) {
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
                  product.default_pv_reference_velocity)));
    const double step = std::abs(
        static_cast<double>(approach_target[index]) -
        static_cast<double>(reference.positions[index]));
    minimum_approach_duration = std::max(
        minimum_approach_duration, step / velocity);
    const double acceleration = std::max(
        0.01, static_cast<double>(product_cartesian_acceleration_limit(
                  index, joint.acceleration_limit)));
    minimum_approach_duration = std::max(
        minimum_approach_duration, std::sqrt(8.0 * step / acceleration));
  }
  const double minimum_total_duration =
      minimum_approach_duration + plan.minimum_duration_s;
  if (!std::isfinite(minimum_total_duration) ||
      minimum_total_duration > 60.0) {
    throw std::invalid_argument(
        "Cartesian approach requires an unsafe duration");
  }
  if (total_duration_s + 1e-9 < minimum_total_duration) {
    throw std::invalid_argument(
        "Cartesian duration_s is too short for the approach and path; "
        "minimum=" + std::to_string(minimum_total_duration) + " s");
  }
  const double approach_duration = total_duration_s *
      minimum_approach_duration / minimum_total_duration;
  const double path_duration = total_duration_s - approach_duration;
  const double original_path_duration =
      plan.trajectory.waypoints.back().time_s;
  for (auto& waypoint : plan.trajectory.waypoints) {
    waypoint.time_s = approach_duration +
        waypoint.time_s * path_duration / original_path_duration;
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
  plan.trajectory.waypoints.insert(
      plan.trajectory.waypoints.begin(), std::move(current));
  plan.trajectory.approach_segment_count = 1;
  plan.minimum_duration_s = minimum_total_duration;
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
    float maximum_reference_velocity = kYunyiCartesianMaximumVelocity) {
  require_cartesian_duration(duration_s);
  NativeCartesianPlan plan;
  auto timestamps = plan_timestamps(
      path, product, maximum_reference_velocity);
  plan.minimum_duration_s = timestamps.back();
  if (duration_s + 1e-9 < plan.minimum_duration_s) {
    throw std::invalid_argument(
        "Cartesian duration_s is too short for product limits; minimum=" +
        std::to_string(plan.minimum_duration_s) + " s");
  }
  const double stretch = duration_s / plan.minimum_duration_s;
  for (std::size_t index = 1; index < timestamps.size(); ++index) {
    timestamps[index] *= stretch;
  }
  auto& trajectory = plan.trajectory;
  trajectory.mode = mode;
  trajectory.operation = operation;
  trajectory.execution = NativeTrajectoryExecution::Quintic;
  trajectory.allow_out_of_limit_start_recovery = true;
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
      product, completion_side, expected_pose, 0.0025, 0.01,
      "Cartesian endpoint");
  return plan;
}

void require_cartesian_reference(
    const NativeTrajectorySample& reference, const char* motion_name) {
  if (reference.active &&
      reference.operation != ARTICORE_OPERATION_MOVE_POSE &&
      reference.operation != ARTICORE_OPERATION_START_TRAJECTORY &&
      reference.operation != ARTICORE_OPERATION_MOVE_LINEAR &&
      reference.operation != ARTICORE_OPERATION_MOVE_CIRCULAR) {
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

NativeCartesianPlan build_linear_plan_common(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const ArticoreRobotPose* declared_start,
    const float* end_pose_values,
    double duration_s) {
  cartesian::require_pv_mode(mode);
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  require_cartesian_duration(duration_s);
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
      translation_distance / kCartesianTranslationSampleDistance,
      orientation_distance / kCartesianOrientationSampleDistance));
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

  const uint32_t sample_count = std::max<uint32_t>(
      2U, static_cast<uint32_t>(required_segments) + 1U);
  const uint32_t side_offset = side * ARTICORE_PRODUCT_ARM_DOF;
  std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>> path;
  path.reserve(sample_count);
  path.push_back(start_positions);
  auto seed = start_q;
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
    const auto solved = solve_ik(
        planning_model, product, side, sample_pose, seed,
        sample + 1 == sample_count
            ? "Cartesian linear target"
            : "Cartesian linear path sample");
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      if (std::abs(solved[index] - seed[index]) > 0.35) {
        throw std::invalid_argument(
            "Cartesian linear path changes to a discontinuous IK branch at "
            "sample " + std::to_string(sample));
      }
    }
    seed = solved;
    auto waypoint = start_positions;
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      waypoint[side_offset + index] = static_cast<float>(seed[index]);
    }
    path.push_back(waypoint);
  }

  return assemble_cartesian_plan(
      path, start_velocities, start_accelerations, product, mode,
      ARTICORE_OPERATION_MOVE_LINEAR, duration_s, side);
}

}  // namespace

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* end_pose_values,
    double duration_s) {
  return build_linear_plan_common(
      product, mode, side, reference, nullptr, end_pose_values,
      duration_s);
}

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose_values,
    const float* end_pose_values,
    double duration_s) {
  const auto declared_start = robot_pose_from_rpy(start_pose_values);
  try {
    validate_cartesian_start_pose(
        product.tcp_offsets[side], side, reference, declared_start, "linear");
    return build_linear_plan_common(
        product, mode, side, reference, &declared_start, end_pose_values,
        duration_s);
  } catch (const std::invalid_argument& error) {
    if (std::string(error.what()).find(
            "start_pose does not match current planned pose") ==
        std::string::npos) {
      throw;
    }
  }

  const auto approach_target = solve_point_to_point_target_from_reference(
      product, mode, side, reference, start_pose_values);
  auto path_reference = reference;
  path_reference.positions.assign(
      approach_target.begin(), approach_target.end());
  path_reference.velocities.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  path_reference.accelerations.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  auto plan = build_linear_plan_common(
      product, mode, side, path_reference, &declared_start, end_pose_values,
      duration_s);
  prepend_cartesian_approach(
      plan, product, side, reference, declared_start, duration_s);
  return plan;
}

std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
solve_point_to_point_target_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* target_pose_values,
    std::chrono::steady_clock::time_point deadline) {
  cartesian::require_pv_mode(mode);
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
    target_q = solve_endpoint_ik(
        *product.pose_models[side], product, side, target, start_q,
        "Cartesian point-to-point target", deadline);
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
  cartesian::require_pv_mode(mode);
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
    double duration_s) {
  cartesian::require_pv_mode(mode);
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  require_cartesian_duration(duration_s);
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
  const uint32_t first_segments = std::max<uint32_t>(
      1U, static_cast<uint32_t>(std::ceil(std::max(
          first_arc_length / kCartesianTranslationSampleDistance,
          first_orientation / kCartesianOrientationSampleDistance))));
  const uint32_t second_segments = std::max<uint32_t>(
      1U, static_cast<uint32_t>(std::ceil(std::max(
          second_arc_length / kCartesianTranslationSampleDistance,
          second_orientation / kCartesianOrientationSampleDistance))));
  if (first_segments + second_segments > 255U) {
    throw std::invalid_argument(
        "circular path is too long for one native transaction");
  }

  std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>> path;
  path.reserve(first_segments + second_segments + 1U);
  path.push_back(start_positions);
  auto seed = start_q;
  const auto append_sample = [&](double angle,
                                 const cartesian::Quaternion& orientation,
                                 const char* context,
                                 uint32_t sample_index,
                                 CartesianIkSearch search,
                                 auto& mutable_seed,
                                 auto& mutable_path) {
    ArticoreRobotPose sample_pose{};
    sample_pose.struct_size = sizeof(sample_pose);
    const auto position = arc.position(angle);
    std::copy(position.begin(), position.end(), sample_pose.position);
    cartesian::rotation_from_quaternion(
        orientation, sample_pose.rotation);
    const auto solved = solve_ik(
        planning_model, product, side, sample_pose, mutable_seed, context,
        search);
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      if (std::abs(solved[index] - mutable_seed[index]) > 0.35) {
        throw std::invalid_argument(
            std::string(context) +
            " changes to a discontinuous IK branch at sample " +
            std::to_string(sample_index));
      }
    }
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
    append_sample(
        amount * arc.via_angle,
        cartesian::slerp(
            geometric_start_orientation, via_orientation, amount),
        "Cartesian circular start-to-via path", sample,
        CartesianIkSearch::LocalPath, seed, path);
  }
  for (uint32_t sample = 1; sample <= second_segments; ++sample) {
    const double amount = static_cast<double>(sample) /
        static_cast<double>(second_segments);
    append_sample(
        arc.via_angle + amount * (arc.end_angle - arc.via_angle),
        cartesian::slerp(via_orientation, end_orientation, amount),
        "Cartesian circular via-to-end path",
        first_segments + sample, CartesianIkSearch::LocalPath,
        seed, path);
  }

  return assemble_cartesian_plan(
      path, start_velocities, start_accelerations, product, mode,
      ARTICORE_OPERATION_MOVE_CIRCULAR, duration_s, side);
}

}  // namespace

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* via_pose_values,
    const float* end_pose_values,
    double duration_s) {
  return build_circular_plan_common(
      product, mode, side, reference, nullptr, via_pose_values,
      end_pose_values, duration_s);
}

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose_values,
    const float* via_pose_values,
    const float* end_pose_values,
    double duration_s) {
  const auto declared_start = robot_pose_from_rpy(start_pose_values);
  try {
    validate_cartesian_start_pose(
        product.tcp_offsets[side], side, reference, declared_start, "circular");
    return build_circular_plan_common(
        product, mode, side, reference, &declared_start, via_pose_values,
        end_pose_values, duration_s);
  } catch (const std::invalid_argument& error) {
    if (std::string(error.what()).find(
            "start_pose does not match current planned pose") ==
        std::string::npos) {
      throw;
    }
  }

  const auto approach_target = solve_point_to_point_target_from_reference(
      product, mode, side, reference, start_pose_values);
  auto path_reference = reference;
  path_reference.positions.assign(
      approach_target.begin(), approach_target.end());
  path_reference.velocities.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  path_reference.accelerations.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  auto plan = build_circular_plan_common(
      product, mode, side, path_reference, &declared_start, via_pose_values,
      end_pose_values, duration_s);
  prepend_cartesian_approach(
      plan, product, side, reference, declared_start, duration_s);
  return plan;
}

}  // namespace articore
