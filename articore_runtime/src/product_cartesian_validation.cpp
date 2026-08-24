#include "articore/detail/product_cartesian.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include "articore/detail/cartesian_math.hpp"
#include "articore/detail/robot_model.hpp"

namespace articore {
namespace {

ArticoreRobotPose pose_from_rpy(const float* values) {
  if (!values) throw std::invalid_argument("start_pose is null");
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_POSE_DOF; ++index) {
    if (!std::isfinite(values[index])) {
      throw std::invalid_argument("start_pose contains NaN or Inf");
    }
  }
  const double sr = std::sin(values[3]);
  const double cr = std::cos(values[3]);
  const double sp = std::sin(values[4]);
  const double cp = std::cos(values[4]);
  const double sy = std::sin(values[5]);
  const double cy = std::cos(values[5]);
  ArticoreRobotPose pose{};
  pose.struct_size = sizeof(pose);
  pose.position[0] = values[0];
  pose.position[1] = values[1];
  pose.position[2] = values[2];
  const std::array<double, 9> rotation{
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr};
  std::copy(rotation.begin(), rotation.end(), pose.rotation);
  return pose;
}

}  // namespace

void validate_cartesian_start_pose(
    bool with_grippers,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const ArticoreRobotPose& declared,
    const char* motion_name) {
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    throw std::invalid_argument(
        "Cartesian motion side must be LEFT(0) or RIGHT(1)");
  }
  if (!motion_name || motion_name[0] == '\0') {
    throw std::invalid_argument("Cartesian motion name is empty");
  }
  if (reference.positions.size() != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
    throw std::runtime_error(
        std::string(motion_name) +
        " requires a complete current planned reference");
  }

  const uint32_t offset = side * ARTICORE_PRODUCT_ARM_DOF;
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> q{};
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
    q[index] = reference.positions[offset + index];
  }
  RobotModel model("yunyi_v1_0", side, with_grippers);
  ArticoreRobotPose current{};
  current.struct_size = sizeof(current);
  model.fk(q.data(), q.size(), &current);
  const double position_error = cartesian::norm(cartesian::subtract(
      current.position, declared.position));
  const double orientation_error = cartesian::angular_distance(
      cartesian::quaternion_from_rotation(current.rotation),
      cartesian::quaternion_from_rotation(declared.rotation));
  if (position_error <= 0.005 && orientation_error <= 0.035) return;

  std::ostringstream message;
  message << motion_name
          << " start_pose does not match current planned pose: "
          << "position_error=" << position_error << " m, "
          << "orientation_error=" << orientation_error << " rad, "
          << "tolerances=[position<=0.005, orientation<=0.035]";
  throw std::invalid_argument(message.str());
}

void validate_cartesian_start_pose(
    bool with_grippers,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose_values,
    const char* motion_name) {
  validate_cartesian_start_pose(
      with_grippers, side, reference, pose_from_rpy(start_pose_values),
      motion_name);
}

}  // namespace articore
