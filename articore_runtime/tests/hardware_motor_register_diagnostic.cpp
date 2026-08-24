#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "articore/detail/yunyi_runtime.hpp"

namespace {

struct FloatRegister {
  uint8_t rid;
  const char* name;
};

constexpr std::array<FloatRegister, 19> kFloatRegisters{{
    {4, "ACC"},       {5, "DEC"},       {6, "MAX_SPD"},
    {11, "Damp"},     {12, "Inertia"},  {17, "Rs"},
    {18, "Ls"},       {19, "Flux"},     {20, "Gr"},
    {21, "PMAX"},     {22, "VMAX"},     {23, "TMAX"},
    {24, "I_BW"},     {25, "KP_ASR"},   {26, "KI_ASR"},
    {27, "KP_APR"},   {28, "KI_APR"},   {31, "Deta"},
    {32, "V_BW"},
}};

constexpr std::array<FloatRegister, 2> kInternalGainRegisters{{
    {33, "IQ_c1"}, {34, "VL_c1"},
}};

struct UIntRegister {
  uint8_t rid;
  const char* name;
};

constexpr std::array<UIntRegister, 5> kUIntRegisters{{
    {10, "CTRL_MODE"}, {13, "hw_ver"}, {14, "sw_ver"},
    {16, "NPP"}, {36, "sub_ver"},
}};

void check_disabled(damiao::MotorHandle* motor, const char* role) {
  const auto state = motor->request_fresh_state(std::chrono::milliseconds(100));
  if (!state || state->status_code != 0) {
    throw std::runtime_error(std::string(role) +
                             " is not feedback-confirmed disabled");
  }
}

void print_motor(damiao::MotorHandle* motor, const char* role) {
  check_disabled(motor, role);
  std::cout << "motor=" << role << " model=" << motor->model()
            << " motor_id=" << motor->motor_id() << '\n';
  for (const auto& item : kUIntRegisters) {
    std::cout << "  rid=" << static_cast<unsigned>(item.rid)
              << " name=" << item.name << " value="
              << motor->get_register_u32(item.rid,
                                         std::chrono::milliseconds(100))
              << '\n';
  }
  std::cout << std::setprecision(9);
  for (const auto& item : kFloatRegisters) {
    std::cout << "  rid=" << static_cast<unsigned>(item.rid)
              << " name=" << item.name << " value="
              << motor->get_register_f32(item.rid,
                                         std::chrono::milliseconds(100))
              << '\n';
  }
  for (const auto& item : kInternalGainRegisters) {
    std::cout << "  rid=" << static_cast<unsigned>(item.rid)
              << " name=" << item.name << " value="
              << motor->get_register_f32(item.rid,
                                         std::chrono::milliseconds(100))
              << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  const bool read_only =
      argc == 2 && std::strcmp(argv[1], "--read-only-disabled") == 0;
  const bool read_gains =
      argc == 2 && std::strcmp(argv[1], "--read-gains-disabled") == 0;
  const bool set_left_j4_kp =
      argc == 5 &&
      std::strcmp(argv[1], "--set-left-j4-kp-apr") == 0 &&
      std::strcmp(argv[3], "--expect") == 0;
  const bool persist_joint4_kp =
      argc == 5 &&
      std::strcmp(argv[1], "--persist-joint4-kp-apr") == 0 &&
      std::strcmp(argv[3], "--expect") == 0;
  const bool set_left_j4_acc_dec =
      argc == 7 &&
      std::strcmp(argv[1], "--set-left-j4-acc-dec") == 0 &&
      std::strcmp(argv[4], "--expect") == 0;
  if (!read_only && !read_gains && !set_left_j4_kp && !persist_joint4_kp &&
      !set_left_j4_acc_dec) {
    std::cerr << "Refusing hardware access. Pass --read-only-disabled, "
                 "--read-gains-disabled, or "
                 "--set-left-j4-kp-apr VALUE --expect CURRENT_VALUE or "
                 "--persist-joint4-kp-apr VALUE --expect CURRENT_VALUE or "
                 "--set-left-j4-acc-dec ACC DEC --expect CURRENT_ACC "
                 "CURRENT_DEC\n";
    return 2;
  }
  try {
    auto product = articore::create_yunyi_runtime(ARTICORE_MODE_PV, true);
    product.runtime->connect();
    if (read_only) {
      print_motor(product.resources->arm_motors[2], "left/l-joint3");
      print_motor(product.resources->arm_motors[3], "left/l-joint4");
      print_motor(product.resources->arm_motors[9], "right/r-joint3");
      print_motor(product.resources->arm_motors[10], "right/r-joint4");
    } else if (read_gains) {
      for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF;
           ++index) {
        auto* motor = product.resources->arm_motors[index];
        const bool left = index < ARTICORE_PRODUCT_ARM_DOF;
        const uint32_t joint = index % ARTICORE_PRODUCT_ARM_DOF + 1;
        const std::string role = std::string(left ? "left/l-joint"
                                                   : "right/r-joint") +
            std::to_string(joint);
        check_disabled(motor, role.c_str());
        std::cout << "motor=" << role << " model=" << motor->model()
                  << " KP_ASR="
                  << motor->get_register_f32(25, std::chrono::milliseconds(100))
                  << " KI_ASR="
                  << motor->get_register_f32(26, std::chrono::milliseconds(100))
                  << " KP_APR="
                  << motor->get_register_f32(27, std::chrono::milliseconds(100))
                  << " KI_APR="
                  << motor->get_register_f32(28, std::chrono::milliseconds(100))
                  << '\n';
      }
    } else if (persist_joint4_kp) {
      const float requested = std::stof(argv[2]);
      const float expected = std::stof(argv[4]);
      if (!std::isfinite(requested) || requested <= 0.0f ||
          !std::isfinite(expected) || expected <= 0.0f) {
        throw std::invalid_argument("KP_APR values must be finite and positive");
      }
      for (uint32_t side = 0; side < 2; ++side) {
        auto* motor = product.resources->arm_motors[
            side * ARTICORE_PRODUCT_ARM_DOF + 3];
        const char* role = side == 0 ? "left/l-joint4" : "right/r-joint4";
        check_disabled(motor, role);
        constexpr uint8_t kKpAprRid = 27;
        const float before = motor->get_register_f32(
            kKpAprRid, std::chrono::milliseconds(100));
        if (std::abs(before - expected) > 1.0e-4f) {
          throw std::runtime_error(
              std::string(role) + " KP_APR precondition failed: expected " +
              std::to_string(expected) + ", actual " +
              std::to_string(before));
        }
        if (std::abs(before - requested) > 1.0e-4f) {
          motor->write_register_f32(kKpAprRid, requested);
        }
        motor->store_parameters();
        const float after = motor->get_register_f32(
            kKpAprRid, std::chrono::milliseconds(100));
        if (std::abs(after - requested) > 1.0e-4f) {
          throw std::runtime_error(
              std::string(role) + " KP_APR readback mismatch after store");
        }
        std::cout << "motor=" << role << " register=KP_APR value=" << after
                  << " persisted=true\n";
      }
    } else if (set_left_j4_kp) {
      const float requested = std::stof(argv[2]);
      const float expected = std::stof(argv[4]);
      if (!std::isfinite(requested) || requested <= 0.0f ||
          !std::isfinite(expected) || expected <= 0.0f) {
        throw std::invalid_argument("KP_APR values must be finite and positive");
      }
      auto* motor = product.resources->arm_motors[3];
      check_disabled(motor, "left/l-joint4");
      constexpr uint8_t kKpAprRid = 27;
      const float before = motor->get_register_f32(
          kKpAprRid, std::chrono::milliseconds(100));
      if (std::abs(before - expected) > 1.0e-4f) {
        throw std::runtime_error(
            "left/l-joint4 KP_APR precondition failed: expected " +
            std::to_string(expected) + ", actual " + std::to_string(before));
      }
      motor->write_register_f32(kKpAprRid, requested);
      const float after = motor->get_register_f32(
          kKpAprRid, std::chrono::milliseconds(100));
      if (std::abs(after - requested) > 1.0e-4f) {
        throw std::runtime_error(
            "left/l-joint4 KP_APR readback mismatch: requested " +
            std::to_string(requested) + ", actual " + std::to_string(after));
      }
      std::cout << "motor=left/l-joint4 register=KP_APR before=" << before
                << " after=" << after << " persisted=false\n";
    } else {
      const float requested_acc = std::stof(argv[2]);
      const float requested_dec = std::stof(argv[3]);
      const float expected_acc = std::stof(argv[5]);
      const float expected_dec = std::stof(argv[6]);
      if (!std::isfinite(requested_acc) || requested_acc <= 0.0f ||
          !std::isfinite(requested_dec) || requested_dec >= 0.0f ||
          !std::isfinite(expected_acc) || expected_acc <= 0.0f ||
          !std::isfinite(expected_dec) || expected_dec >= 0.0f) {
        throw std::invalid_argument(
            "ACC must be positive and DEC must be negative");
      }
      auto* motor = product.resources->arm_motors[3];
      check_disabled(motor, "left/l-joint4");
      constexpr uint8_t kAccRid = 4;
      constexpr uint8_t kDecRid = 5;
      const float before_acc = motor->get_register_f32(
          kAccRid, std::chrono::milliseconds(100));
      const float before_dec = motor->get_register_f32(
          kDecRid, std::chrono::milliseconds(100));
      if (std::abs(before_acc - expected_acc) > 1.0e-4f ||
          std::abs(before_dec - expected_dec) > 1.0e-4f) {
        throw std::runtime_error(
            "left/l-joint4 ACC/DEC precondition failed: expected [" +
            std::to_string(expected_acc) + ", " +
            std::to_string(expected_dec) + "], actual [" +
            std::to_string(before_acc) + ", " +
            std::to_string(before_dec) + "]");
      }
      motor->write_register_f32(kAccRid, requested_acc);
      try {
        motor->write_register_f32(kDecRid, requested_dec);
      } catch (...) {
        motor->write_register_f32(kAccRid, before_acc);
        throw;
      }
      const float after_acc = motor->get_register_f32(
          kAccRid, std::chrono::milliseconds(100));
      const float after_dec = motor->get_register_f32(
          kDecRid, std::chrono::milliseconds(100));
      if (std::abs(after_acc - requested_acc) > 1.0e-4f ||
          std::abs(after_dec - requested_dec) > 1.0e-4f) {
        motor->write_register_f32(kAccRid, before_acc);
        motor->write_register_f32(kDecRid, before_dec);
        throw std::runtime_error("left/l-joint4 ACC/DEC readback mismatch");
      }
      std::cout << "motor=left/l-joint4 registers=ACC,DEC before=["
                << before_acc << ',' << before_dec << "] after=["
                << after_acc << ',' << after_dec
                << "] persisted=false\n";
    }
    product.runtime->disconnect();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
