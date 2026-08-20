#include "product_cartesian.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "cartesian_math.hpp"
#include "motor_abi.h"

namespace articore {
namespace {

void read_product_arm_snapshot(
    SafetyRuntime& safety,
    YunyiRuntimeResources& product,
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& positions,
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& velocities) {
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    const auto& joint = product.joints[index];
    MotorState motor{};
    MotorFeedbackStats stats{};
    if (motor_handle_get_state(joint.motor, &motor) != 0 ||
        !motor.has_value ||
        motor_handle_get_feedback_stats(joint.motor, &stats) != 0 ||
        !stats.has_feedback || stats.age_ns > safety.feedback_max_age_ns()) {
      throw std::runtime_error(
          "Cartesian motion requires fresh feedback at joint " +
          std::to_string(index));
    }
    if (motor.status_code > 1 || !std::isfinite(motor.pos) ||
        !std::isfinite(motor.vel)) {
      throw std::runtime_error(
          "Cartesian motion requires fault-free finite feedback at joint " +
          std::to_string(index));
    }
    positions[index] = joint.direction * motor.pos;
    velocities[index] = joint.direction * motor.vel *
        joint.velocity_feedback_scale;
  }
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
    YunyiRuntimeResources& product, uint32_t side,
    const ArticoreRobotPose& target,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& seed,
    const char* context) {
  ArticoreIkOptions options{};
  options.struct_size = sizeof(options);
  ArticoreIkResult ik{};
  ik.struct_size = sizeof(ik);
  {
    std::lock_guard<std::mutex> lock(product.pose_mutexes[side]);
    product.pose_models[side]->ik(
        &target, seed.data(), seed.size(), &options, &ik);
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
      throw std::invalid_argument(
          std::string(context) + " IK result exceeds product limits at joint " +
          std::to_string(offset + index));
    }
    result[index] = ik.q[index];
  }
  return result;
}

NativeTrajectoryJoint trajectory_joint(
    const YunyiRuntimeResources::Joint& source,
    float scale, float start_velocity) {
  NativeTrajectoryJoint joint;
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
  joint.pv_velocity_limit = std::max(
      source.velocity_limit * scale,
      std::min(source.velocity_limit, std::abs(start_velocity) + 0.01f));
  return joint;
}

double plan_duration(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    const std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& start_velocities,
    const YunyiRuntimeResources& product, float scale) {
  double duration = 0.10;
  for (uint32_t joint_index = 0;
       joint_index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++joint_index) {
    double path_length = 0.0;
    for (std::size_t waypoint = 1; waypoint < path.size(); ++waypoint) {
      const double step = std::abs(
          static_cast<double>(path[waypoint][joint_index] -
                              path[waypoint - 1][joint_index]));
      path_length += step;
    }
    const auto& joint = product.joints[joint_index];
    const double velocity = std::max(
        0.01, static_cast<double>(joint.velocity_limit * scale));
    const double acceleration = std::max(
        0.01, static_cast<double>(joint.acceleration_limit * scale));
    duration = std::max(duration, 1.875 * path_length / velocity);
    duration = std::max(
        duration, std::sqrt(5.7736 * path_length / acceleration));
    duration = std::max(
        duration,
        2.0 * std::abs(static_cast<double>(start_velocities[joint_index])) /
            acceleration);
  }
  if (!std::isfinite(duration) || duration > 60.0) {
    throw std::invalid_argument("Cartesian motion requires an unsafe duration");
  }
  return duration;
}

NativeCartesianPlan assemble_cartesian_plan(
    const std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>& path,
    const std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& start_velocities,
    const std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>& start_accelerations,
    const YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    ArticoreRuntimeOperation operation,
    float speed_percent,
    uint64_t replace_trajectory_id) {
  NativeCartesianPlan plan;
  plan.replace_trajectory_id = replace_trajectory_id;
  const float scale = speed_percent / 100.0f;
  const double duration = plan_duration(
      path, start_velocities, product, scale);
  auto& trajectory = plan.trajectory;
  trajectory.mode = mode;
  trajectory.operation = operation;
  trajectory.joints.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    trajectory.joints.push_back(trajectory_joint(
        product.joints[index], scale, start_velocities[index]));
  }

  const uint32_t all_joints =
      (uint32_t{1} << ARTICORE_PRODUCT_DUAL_ARM_DOF) - 1U;
  trajectory.waypoints.reserve(path.size());
  for (std::size_t index = 0; index < path.size(); ++index) {
    NativeTrajectoryWaypoint waypoint;
    waypoint.time_s = duration * static_cast<double>(index) /
        static_cast<double>(path.size() - 1);
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
  return plan;
}

}  // namespace

NativeCartesianPlan build_cartesian_plan(
    SafetyRuntime& safety,
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const float* target_pose,
    float speed_percent,
    ArticoreCartesianInterpolation interpolation) {
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
  if (interpolation != ARTICORE_CARTESIAN_POINT_TO_POINT &&
      interpolation != ARTICORE_CARTESIAN_LINEAR) {
    throw std::invalid_argument("Cartesian interpolation is invalid");
  }
  const auto target = robot_pose_from_rpy(target_pose);
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> feedback_positions{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> feedback_velocities{};
  read_product_arm_snapshot(
      safety, product, feedback_positions, feedback_velocities);

  const uint32_t side_offset = side * ARTICORE_PRODUCT_ARM_DOF;
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_positions =
      feedback_positions;
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_velocities =
      feedback_velocities;
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_accelerations{};
  NativeCartesianPlan plan;
  const auto active = safety.trajectory_sample();
  if (active.active) {
    if (active.operation != ARTICORE_OPERATION_MOVE_POSE &&
        active.operation != ARTICORE_OPERATION_MOVE_LINEAR &&
        active.operation != ARTICORE_OPERATION_MOVE_CIRCULAR) {
      throw std::runtime_error(
          "Cartesian motion cannot replace an explicit multi-waypoint trajectory");
    }
    if (active.positions.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF ||
        active.velocities.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF ||
        active.accelerations.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
      throw std::runtime_error("active Cartesian plan is incomplete");
    }
    std::copy(active.positions.begin(), active.positions.end(),
              start_positions.begin());
    std::copy(active.velocities.begin(), active.velocities.end(),
              start_velocities.begin());
    std::copy(active.accelerations.begin(), active.accelerations.end(),
              start_accelerations.begin());
    plan.replace_trajectory_id = active.trajectory_id;
  }

  std::array<double, ARTICORE_PRODUCT_ARM_DOF> start_q{};
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    start_q[index] = start_positions[side_offset + index];
  }
  const auto target_q = solve_ik(
      product, side, target, start_q, "Cartesian target");

  std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>> path;
  if (interpolation == ARTICORE_CARTESIAN_POINT_TO_POINT) {
    path = {start_positions, start_positions};
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      path.back()[side_offset + index] = static_cast<float>(target_q[index]);
    }
  } else {
    ArticoreRobotPose start_pose{};
    start_pose.struct_size = sizeof(start_pose);
    {
      std::lock_guard<std::mutex> lock(product.pose_mutexes[side]);
      product.pose_models[side]->fk(
          start_q.data(), start_q.size(), &start_pose);
    }
    const auto start_orientation =
        cartesian::quaternion_from_rotation(start_pose.rotation);
    const auto target_orientation =
        cartesian::quaternion_from_rotation(target.rotation);
    const double orientation_distance = cartesian::angular_distance(
        start_orientation, target_orientation);
    const double dx = target.position[0] - start_pose.position[0];
    const double dy = target.position[1] - start_pose.position[1];
    const double dz = target.position[2] - start_pose.position[2];
    const double translation_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double required_segments = std::ceil(std::max(
        translation_distance / 0.005, orientation_distance / 0.035));
    if (!std::isfinite(required_segments) || required_segments > 255.0) {
      throw std::invalid_argument(
          "Cartesian linear path is too long for one native transaction");
    }
    const uint32_t sample_count = std::max<uint32_t>(
        2U, static_cast<uint32_t>(required_segments) + 1U);
    path.reserve(sample_count);
    path.push_back(start_positions);
    auto seed = start_q;
    for (uint32_t sample = 1; sample + 1 < sample_count; ++sample) {
      const double amount = static_cast<double>(sample) /
          static_cast<double>(sample_count - 1);
      ArticoreRobotPose sample_pose{};
      sample_pose.struct_size = sizeof(sample_pose);
      const auto position = cartesian::lerp_position(
          start_pose.position, target.position, amount);
      std::copy(position.begin(), position.end(), sample_pose.position);
      const auto orientation = cartesian::slerp(
          start_orientation, target_orientation, amount);
      cartesian::rotation_from_quaternion(
          orientation, sample_pose.rotation);
      const auto solved = solve_ik(
          product, side, sample_pose, seed, "Cartesian linear path sample");
      for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
        if (std::abs(solved[index] - seed[index]) > 0.35) {
          throw std::invalid_argument(
              "Cartesian linear path has a discontinuous IK branch at sample " +
              std::to_string(sample));
        }
      }
      seed = solved;
      auto waypoint = start_positions;
      for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
        waypoint[side_offset + index] = static_cast<float>(seed[index]);
      }
      path.push_back(waypoint);
    }
    const auto linear_target_q = solve_ik(
        product, side, target, seed, "Cartesian linear target");
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      if (std::abs(linear_target_q[index] - seed[index]) > 0.35) {
        throw std::invalid_argument(
            "Cartesian linear target changes to a discontinuous IK branch");
      }
    }
    auto end = start_positions;
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      end[side_offset + index] =
          static_cast<float>(linear_target_q[index]);
    }
    path.push_back(end);
  }

  return assemble_cartesian_plan(
      path, start_velocities, start_accelerations, product, mode,
      interpolation == ARTICORE_CARTESIAN_LINEAR
          ? ARTICORE_OPERATION_MOVE_LINEAR
          : ARTICORE_OPERATION_MOVE_POSE,
      speed_percent, plan.replace_trajectory_id);
}

NativeCartesianPlan build_circular_plan(
    SafetyRuntime& safety,
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const float* start_pose_values,
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
  const auto declared_start = robot_pose_from_rpy(start_pose_values);
  const auto via = robot_pose_from_rpy(via_pose_values);
  const auto end = robot_pose_from_rpy(end_pose_values);
  const auto arc = cartesian::circular_arc_from_three_points(
      declared_start.position, via.position, end.position);

  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_positions{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_velocities{};
  read_product_arm_snapshot(
      safety, product, start_positions, start_velocities);
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> start_accelerations{};
  uint64_t replace_trajectory_id = 0;
  const auto active = safety.trajectory_sample();
  if (active.active) {
    if (active.operation != ARTICORE_OPERATION_MOVE_POSE &&
        active.operation != ARTICORE_OPERATION_MOVE_LINEAR &&
        active.operation != ARTICORE_OPERATION_MOVE_CIRCULAR) {
      throw std::runtime_error(
          "Cartesian motion cannot replace an explicit multi-waypoint trajectory");
    }
    if (active.positions.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF ||
        active.velocities.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF ||
        active.accelerations.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
      throw std::runtime_error("active Cartesian plan is incomplete");
    }
    std::copy(active.positions.begin(), active.positions.end(),
              start_positions.begin());
    std::copy(active.velocities.begin(), active.velocities.end(),
              start_velocities.begin());
    std::copy(active.accelerations.begin(), active.accelerations.end(),
              start_accelerations.begin());
    replace_trajectory_id = active.trajectory_id;
  }

  const uint32_t side_offset = side * ARTICORE_PRODUCT_ARM_DOF;
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> start_q{};
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    start_q[index] = start_positions[side_offset + index];
  }
  ArticoreRobotPose actual_start{};
  actual_start.struct_size = sizeof(actual_start);
  {
    std::lock_guard<std::mutex> lock(product.pose_mutexes[side]);
    product.pose_models[side]->fk(
        start_q.data(), start_q.size(), &actual_start);
  }
  const auto position_error = cartesian::norm(cartesian::subtract(
      actual_start.position, declared_start.position));
  const auto actual_start_orientation =
      cartesian::quaternion_from_rotation(actual_start.rotation);
  const auto declared_start_orientation =
      cartesian::quaternion_from_rotation(declared_start.rotation);
  const auto via_orientation =
      cartesian::quaternion_from_rotation(via.rotation);
  const auto end_orientation =
      cartesian::quaternion_from_rotation(end.rotation);
  const double orientation_error = cartesian::angular_distance(
      actual_start_orientation, declared_start_orientation);
  if (position_error > 0.005 || orientation_error > 0.035) {
    throw std::invalid_argument(
        "circular start pose does not match the current planned flange pose");
  }

  const double first_arc_length = arc.radius * arc.via_angle;
  const double second_arc_length =
      arc.radius * (arc.end_angle - arc.via_angle);
  const double first_orientation = cartesian::angular_distance(
      actual_start_orientation, via_orientation);
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
                                 auto& mutable_seed,
                                 auto& mutable_path) {
    ArticoreRobotPose sample_pose{};
    sample_pose.struct_size = sizeof(sample_pose);
    const auto position = arc.position(angle);
    std::copy(position.begin(), position.end(), sample_pose.position);
    cartesian::rotation_from_quaternion(
        orientation, sample_pose.rotation);
    const auto solved = solve_ik(
        product, side, sample_pose, mutable_seed, context);
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
            actual_start_orientation, via_orientation, amount),
        "Cartesian circular start-to-via path", sample, seed, path);
  }
  for (uint32_t sample = 1; sample <= second_segments; ++sample) {
    const double amount = static_cast<double>(sample) /
        static_cast<double>(second_segments);
    append_sample(
        arc.via_angle + amount * (arc.end_angle - arc.via_angle),
        cartesian::slerp(via_orientation, end_orientation, amount),
        "Cartesian circular via-to-end path",
        first_segments + sample, seed, path);
  }

  return assemble_cartesian_plan(
      path, start_velocities, start_accelerations, product, mode,
      ARTICORE_OPERATION_MOVE_CIRCULAR, speed_percent,
      replace_trajectory_id);
}

}  // namespace articore
