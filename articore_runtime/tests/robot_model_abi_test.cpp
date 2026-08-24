#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>

#include "articore/runtime_abi.h"

namespace {

bool near(double lhs, double rhs, double tolerance = 1e-12) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool test_side(uint32_t side, const char* prefix,
               const std::array<double, 3>& zero_position) {
  ArticoreRobotModel* model = articore_robot_model_create("yunyi_v1_0", side);
  if (!model) return false;

  ArticoreRobotModelInfo info{};
  info.struct_size = sizeof(info);
  if (articore_robot_model_get_info(model, &info) != 0 || info.dof != 7 ||
      info.side != side || std::strcmp(info.product_id, "yunyi_v1_0") != 0 ||
      std::strncmp(info.joint_names[0], prefix, 1) != 0) {
    articore_robot_model_free(model);
    return false;
  }

  std::array<double, 7> zero{};
  ArticoreRobotPose pose{};
  pose.struct_size = sizeof(pose);
  if (articore_robot_model_fk(model, zero.data(), zero.size(), &pose) != 0) {
    articore_robot_model_free(model);
    return false;
  }
  for (size_t i = 0; i < 3; ++i) {
    if (!near(pose.position[i], zero_position[i])) {
      articore_robot_model_free(model);
      return false;
    }
  }

  std::array<double, 7> gravity{};
  std::array<double, 7> rnea{};
  std::array<double, 7> acceleration{};
  if (articore_robot_model_gravity(model, zero.data(), 7, gravity.data(), 7) ||
      articore_robot_model_rnea(model, zero.data(), 7, zero.data(), 7,
                                zero.data(), 7, rnea.data(), 7) ||
      articore_robot_model_aba(model, zero.data(), 7, zero.data(), 7,
                               gravity.data(), 7, acceleration.data(), 7)) {
    articore_robot_model_free(model);
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    if (!near(gravity[i], rnea[i]) || !near(acceleration[i], 0.0, 1e-10)) {
      articore_robot_model_free(model);
      return false;
    }
  }

  std::array<double, 42> jacobian{};
  std::array<double, 49> mass{};
  const bool ok =
      articore_robot_model_jacobian(model, zero.data(), 7,
                                    ARTICORE_JACOBIAN_LOCAL,
                                    jacobian.data(), jacobian.size()) == 0 &&
      articore_robot_model_mass_matrix(model, zero.data(), 7, mass.data(),
                                       mass.size()) == 0 &&
      std::all_of(mass.begin(), mass.end(), [](double value) {
        return std::isfinite(value);
      });
  articore_robot_model_free(model);
  return ok;
}

bool test_first_joint_positive_direction_is_mirrored() {
  ArticoreRobotModel* left =
      articore_robot_model_create("yunyi_v1_0", ARTICORE_ROBOT_LEFT);
  ArticoreRobotModel* right =
      articore_robot_model_create("yunyi_v1_0", ARTICORE_ROBOT_RIGHT);
  if (!left || !right) {
    articore_robot_model_free(left);
    articore_robot_model_free(right);
    return false;
  }

  std::array<double, 7> zero{};
  auto positive_joint1 = zero;
  positive_joint1[0] = 0.1;
  ArticoreRobotPose left_zero{};
  ArticoreRobotPose left_positive{};
  ArticoreRobotPose right_zero{};
  ArticoreRobotPose right_positive{};
  for (auto* pose : {&left_zero, &left_positive, &right_zero,
                     &right_positive}) {
    pose->struct_size = sizeof(*pose);
  }
  const bool fk_ok =
      articore_robot_model_fk(left, zero.data(), zero.size(), &left_zero) == 0 &&
      articore_robot_model_fk(left, positive_joint1.data(),
                              positive_joint1.size(), &left_positive) == 0 &&
      articore_robot_model_fk(right, zero.data(), zero.size(), &right_zero) == 0 &&
      articore_robot_model_fk(right, positive_joint1.data(),
                              positive_joint1.size(), &right_positive) == 0;
  articore_robot_model_free(left);
  articore_robot_model_free(right);
  if (!fk_ok) return false;

  std::array<double, 3> left_delta{};
  std::array<double, 3> right_delta{};
  for (size_t axis = 0; axis < 3; ++axis) {
    left_delta[axis] = left_positive.position[axis] - left_zero.position[axis];
    right_delta[axis] =
        right_positive.position[axis] - right_zero.position[axis];
  }
  return left_delta[0] < 0.0 && right_delta[0] < 0.0 &&
      near(left_delta[0], right_delta[0], 1e-7) &&
      near(left_delta[1], -right_delta[1], 1e-7) &&
      near(left_delta[2], right_delta[2], 1e-7);
}

}  // namespace

int main() {
  if (!test_side(ARTICORE_ROBOT_LEFT, "l",
                 {0.001704439680308022, 0.2318916953069385,
                  0.18147106361002283}) ||
      !test_side(ARTICORE_ROBOT_RIGHT, "r",
                 {0.0017044146523629705, -0.23188869792476557,
                  0.18147150461594638}) ||
      !test_first_joint_positive_direction_is_mirrored()) {
    std::cerr << "native robot model ABI test failed: "
              << articore_runtime_last_error() << "\n";
    return 1;
  }
  std::cout << "native robot model ABI test passed\n";
  return 0;
}
