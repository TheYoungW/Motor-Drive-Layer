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

}  // namespace

int main() {
  if (!test_side(ARTICORE_ROBOT_LEFT, "l",
                 {0.001704439680308022, 0.2318916953069385,
                  0.18147106361002283}) ||
      !test_side(ARTICORE_ROBOT_RIGHT, "r",
                 {0.0017044146523629705, -0.23188869792476557,
                  0.18147150461594638})) {
    std::cerr << "native robot model ABI test failed: "
              << articore_runtime_last_error() << "\n";
    return 1;
  }
  std::cout << "native robot model ABI test passed\n";
  return 0;
}
