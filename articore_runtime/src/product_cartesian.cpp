#include "articore/detail/product_cartesian.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
    const char* context) {
  // PTP has no sequential Cartesian samples to constrain the redundant arm.
  // Search the deterministic endpoint budget and select the valid joint
  // solution closest to the measured/planned seed.
  return solve_ik(
      planning_model, product, side, target, seed, context,
      CartesianIkSearch::GlobalEndpoint);
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
  // The 0..100 value scales native trajectory timing. Damiao's POS_VEL V
  // field is a separate product ceiling and remains 3 rad/s.
  joint.pv_velocity_limit = product_pv_drive_velocity_limit(
      source.velocity_limit);
  joint.pv_hold_velocity_limit = std::min(
      joint.pv_velocity_limit, kNativePvFinalHoldVelocityLimit);
  return joint;
}

std::vector<double> plan_timestamps(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    const YunyiRuntimeResources& product, float scale,
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
                             maximum_reference_velocity) * scale));
      const double step = std::abs(static_cast<double>(
          path[waypoint][joint_index] -
          path[waypoint - 1][joint_index]));
      segment_duration = std::max(segment_duration, step / velocity);
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

NativeCartesianPlan assemble_cartesian_plan(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    const std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& start_velocities,
    const std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& start_accelerations,
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    ArticoreRuntimeOperation operation,
    float speed_percent,
    uint32_t completion_side,
    float maximum_reference_velocity = kYunyiCartesianMaximumVelocity) {
  NativeCartesianPlan plan;
  const float scale = speed_percent / 100.0f;
  const auto timestamps = plan_timestamps(
      path, product, scale, maximum_reference_velocity);
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
  auto* completion_model = product.pose_models[completion_side].get();
  auto* completion_mutex = &product.pose_mutexes[completion_side];
  trajectory.final_convergence_check =
      [completion_model, completion_mutex, completion_offset, expected_pose](
          const std::vector<float>& actual_positions,
          std::string& error) {
        if (actual_positions.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
          error = "Cartesian endpoint feedback has the wrong joint count";
          return false;
        }
        std::array<double, ARTICORE_PRODUCT_ARM_DOF> actual_q{};
        for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
          actual_q[index] = actual_positions[completion_offset + index];
        }
        ArticoreRobotPose actual_pose{};
        actual_pose.struct_size = sizeof(actual_pose);
        {
          std::lock_guard<std::mutex> lock(*completion_mutex);
          completion_model->fk(
              actual_q.data(), actual_q.size(), &actual_pose);
        }
        const double position_error = cartesian::norm(cartesian::subtract(
            actual_pose.position, expected_pose.position));
        const double orientation_error = cartesian::angular_distance(
            cartesian::quaternion_from_rotation(actual_pose.rotation),
            cartesian::quaternion_from_rotation(expected_pose.rotation));
        if (position_error <= 0.0025 && orientation_error <= 0.01) {
          error.clear();
          return true;
        }
        error = "Cartesian endpoint position_error=" +
            std::to_string(position_error) +
            " orientation_error=" + std::to_string(orientation_error) +
            " tolerances=[position<=0.0025, orientation<=0.01]";
        return false;
      };
  return plan;
}

void require_cartesian_reference(
    const NativeTrajectorySample& reference, const char* motion_name) {
  if (reference.active &&
      reference.operation != ARTICORE_OPERATION_MOVE_POSE &&
      reference.operation != ARTICORE_OPERATION_MOVE_LINEAR &&
      reference.operation != ARTICORE_OPERATION_MOVE_CIRCULAR) {
    throw std::runtime_error(
        std::string(motion_name) +
        " cannot replace an explicit multi-waypoint trajectory");
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
    float speed_percent) {
  cartesian::require_pv_mode(mode);
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  if (!std::isfinite(speed_percent) || speed_percent <= 0.0f ||
      speed_percent > 100.0f) {
    throw std::invalid_argument(
        "Cartesian motion speed_percent must be in (0,100]");
  }
  require_cartesian_reference(reference, "linear");
  if (declared_start) {
    validate_cartesian_start_pose(
        product.with_grippers, side, reference, *declared_start, "linear");
  }

  const auto end_pose = robot_pose_from_rpy(end_pose_values);
  RobotModel planning_model("yunyi_v1_0", side, product.with_grippers);
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
      translation_distance / 0.005, orientation_distance / 0.035));
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
      ARTICORE_OPERATION_MOVE_LINEAR, speed_percent, side);
}

}  // namespace

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* end_pose_values,
    float speed_percent) {
  return build_linear_plan_common(
      product, mode, side, reference, nullptr, end_pose_values,
      speed_percent);
}

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose_values,
    const float* end_pose_values,
    float speed_percent) {
  const auto declared_start = robot_pose_from_rpy(start_pose_values);
  return build_linear_plan_common(
      product, mode, side, reference, &declared_start, end_pose_values,
      speed_percent);
}

std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
solve_point_to_point_target_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* target_pose_values) {
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
  RobotModel planning_model("yunyi_v1_0", side, product.with_grippers);
  const auto target_q = solve_endpoint_ik(
      planning_model, product, side, target, start_q,
      "Cartesian point-to-point target");

  auto result = start_positions;
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    result[side_offset + index] = static_cast<float>(target_q[index]);
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
    float speed_percent) {
  cartesian::require_pv_mode(mode);
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  if (!std::isfinite(speed_percent) || speed_percent <= 0.0f ||
      speed_percent > 100.0f) {
    throw std::invalid_argument(
        "Cartesian motion speed_percent must be in (0,100]");
  }
  const auto via = robot_pose_from_rpy(via_pose_values);
  const auto end = robot_pose_from_rpy(end_pose_values);
  RobotModel planning_model("yunyi_v1_0", side, product.with_grippers);
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
        product.with_grippers, side, reference, *declared_start, "circular");
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
          first_arc_length / 0.005, first_orientation / 0.035))));
  const uint32_t second_segments = std::max<uint32_t>(
      1U, static_cast<uint32_t>(std::ceil(std::max(
          second_arc_length / 0.005, second_orientation / 0.035))));
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
      ARTICORE_OPERATION_MOVE_CIRCULAR, speed_percent, side);
}

}  // namespace

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* via_pose_values,
    const float* end_pose_values,
    float speed_percent) {
  return build_circular_plan_common(
      product, mode, side, reference, nullptr, via_pose_values,
      end_pose_values, speed_percent);
}

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose_values,
    const float* via_pose_values,
    const float* end_pose_values,
    float speed_percent) {
  const auto declared_start = robot_pose_from_rpy(start_pose_values);
  return build_circular_plan_common(
      product, mode, side, reference, &declared_start, via_pose_values,
      end_pose_values, speed_percent);
}

}  // namespace articore
