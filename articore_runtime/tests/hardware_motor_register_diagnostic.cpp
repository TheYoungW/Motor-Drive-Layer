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

struct PvGains {
  float kp_asr;
  float ki_asr;
  float kp_apr;
  float ki_apr;
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

damiao::MotorHandle* find_arm_motor(
    articore::YunyiRuntimeResources& resources, const std::string& requested) {
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    const bool left = index < ARTICORE_PRODUCT_ARM_DOF;
    const uint32_t joint = index % ARTICORE_PRODUCT_ARM_DOF + 1;
    const std::string role = std::string(left ? "left/l-joint"
                                               : "right/r-joint") +
        std::to_string(joint);
    if (role == requested) return resources.arm_motors[index];
  }
  throw std::invalid_argument("unknown arm motor role: " + requested);
}

void check_all_arm_motors_disabled(articore::YunyiRuntimeResources& resources) {
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    const bool left = index < ARTICORE_PRODUCT_ARM_DOF;
    const uint32_t joint = index % ARTICORE_PRODUCT_ARM_DOF + 1;
    const std::string role = std::string(left ? "left/l-joint"
                                               : "right/r-joint") +
        std::to_string(joint);
    check_disabled(resources.arm_motors[index], role.c_str());
  }
}

PvGains read_pv_gains(damiao::MotorHandle* motor) {
  const auto timeout = std::chrono::milliseconds(100);
  return {
      motor->get_register_f32(25, timeout),
      motor->get_register_f32(26, timeout),
      motor->get_register_f32(27, timeout),
      motor->get_register_f32(28, timeout),
  };
}

bool gain_matches(float actual, float expected) {
  return std::abs(actual - expected) <= 1.0e-6f *
      std::max(1.0f, std::max(std::abs(actual), std::abs(expected)));
}

bool gains_match(const PvGains& actual, const PvGains& expected) {
  return gain_matches(actual.kp_asr, expected.kp_asr) &&
      gain_matches(actual.ki_asr, expected.ki_asr) &&
      gain_matches(actual.kp_apr, expected.kp_apr) &&
      gain_matches(actual.ki_apr, expected.ki_apr);
}

void validate_pv_gains(const PvGains& value) {
  if (!std::isfinite(value.kp_asr) || value.kp_asr <= 0.0f ||
      value.kp_asr > 0.05f ||
      !std::isfinite(value.ki_asr) || value.ki_asr < 0.0f ||
      value.ki_asr > 0.05f ||
      !std::isfinite(value.kp_apr) || value.kp_apr <= 0.0f ||
      value.kp_apr > 200.0f ||
      !std::isfinite(value.ki_apr) || value.ki_apr < 0.0f ||
      value.ki_apr > 10.0f) {
    throw std::invalid_argument("PV gains are outside diagnostic safety bounds");
  }
}

void write_pv_gains(damiao::MotorHandle* motor, const PvGains& value) {
  motor->write_register_f32(25, value.kp_asr);
  motor->write_register_f32(26, value.ki_asr);
  motor->write_register_f32(27, value.kp_apr);
  motor->write_register_f32(28, value.ki_apr);
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
  const bool read_acc_dec =
      argc == 2 && std::strcmp(argv[1], "--read-acc-dec-disabled") == 0;
  const bool set_pv_gains =
      argc == 12 &&
      std::strcmp(argv[1], "--set-pv-gains-disabled") == 0 &&
      std::strcmp(argv[7], "--expect") == 0;
  const bool persist_pv_gains =
      argc == 12 &&
      std::strcmp(argv[1], "--persist-pv-gains-disabled") == 0 &&
      std::strcmp(argv[7], "--expect") == 0;
  const bool set_deta =
      argc == 6 && std::strcmp(argv[1], "--set-deta-disabled") == 0 &&
      std::strcmp(argv[4], "--expect") == 0;
  const bool set_acc_dec =
      argc == 8 && std::strcmp(argv[1], "--set-acc-dec-disabled") == 0 &&
      std::strcmp(argv[5], "--expect") == 0;
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
  if (!read_only && !read_gains && !read_acc_dec && !set_pv_gains &&
      !persist_pv_gains && !set_deta && !set_acc_dec && !set_left_j4_kp &&
      !persist_joint4_kp && !set_left_j4_acc_dec) {
    std::cerr << "Refusing hardware access. Pass --read-only-disabled, "
                 "--read-gains-disabled, --read-acc-dec-disabled, or "
                 "--set-pv-gains-disabled ROLE KP_ASR KI_ASR KP_APR KI_APR "
                 "--expect KP_ASR KI_ASR KP_APR KI_APR, or "
                 "--persist-pv-gains-disabled ROLE KP_ASR KI_ASR KP_APR "
                 "KI_APR --expect KP_ASR KI_ASR KP_APR KI_APR, or "
                 "--set-deta-disabled ROLE VALUE --expect CURRENT, or "
                 "--set-acc-dec-disabled ROLE ACC DEC --expect CURRENT_ACC "
                 "CURRENT_DEC, or "
                 "--set-left-j4-kp-apr VALUE --expect CURRENT_VALUE or "
                 "--persist-joint4-kp-apr VALUE --expect CURRENT_VALUE or "
                 "--set-left-j4-acc-dec ACC DEC --expect CURRENT_ACC "
                 "CURRENT_DEC\n";
    return 2;
  }
  try {
    auto product = articore::create_yunyi_runtime(ARTICORE_MODE_PV, true);
    const bool strict_read_only = read_only || read_gains || read_acc_dec;
    if (!strict_read_only) product.runtime->connect();
    if (read_only) {
      for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF;
           ++index) {
        const bool left = index < ARTICORE_PRODUCT_ARM_DOF;
        const uint32_t joint = index % ARTICORE_PRODUCT_ARM_DOF + 1;
        const std::string role = std::string(left ? "left/l-joint"
                                                   : "right/r-joint") +
            std::to_string(joint);
        print_motor(product.resources->arm_motors[index], role.c_str());
      }
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
    } else if (read_acc_dec) {
      for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF;
           ++index) {
        auto* motor = product.resources->arm_motors[index];
        const bool left = index < ARTICORE_PRODUCT_ARM_DOF;
        const uint32_t joint = index % ARTICORE_PRODUCT_ARM_DOF + 1;
        const std::string role = std::string(left ? "left/l-joint"
                                                   : "right/r-joint") +
            std::to_string(joint);
        check_disabled(motor, role.c_str());
        std::cout << std::setprecision(9)
                  << "motor=" << role << " model=" << motor->model()
                  << " ACC="
                  << motor->get_register_f32(4, std::chrono::milliseconds(100))
                  << " DEC="
                  << motor->get_register_f32(5, std::chrono::milliseconds(100))
                  << '\n';
      }
    } else if (set_pv_gains || persist_pv_gains) {
      const std::string role = argv[2];
      const PvGains requested{
          std::stof(argv[3]), std::stof(argv[4]),
          std::stof(argv[5]), std::stof(argv[6]),
      };
      const PvGains expected{
          std::stof(argv[8]), std::stof(argv[9]),
          std::stof(argv[10]), std::stof(argv[11]),
      };
      validate_pv_gains(requested);
      validate_pv_gains(expected);
      check_all_arm_motors_disabled(*product.resources);
      auto* motor = find_arm_motor(*product.resources, role);
      const PvGains before = read_pv_gains(motor);
      if (!gains_match(before, expected)) {
        throw std::runtime_error(role + " PV gain precondition failed");
      }
      try {
        write_pv_gains(motor, requested);
        const PvGains after = read_pv_gains(motor);
        if (!gains_match(after, requested)) {
          throw std::runtime_error(role + " PV gain readback mismatch");
        }
        if (persist_pv_gains) {
          motor->store_parameters();
          const PvGains stored = read_pv_gains(motor);
          if (!gains_match(stored, requested)) {
            throw std::runtime_error(
                role + " PV gain readback mismatch after store");
          }
        }
      } catch (...) {
        write_pv_gains(motor, before);
        if (persist_pv_gains) motor->store_parameters();
        throw;
      }
      std::cout << std::setprecision(9)
                << "motor=" << role
                << " gains_before=[" << before.kp_asr << ','
                << before.ki_asr << ',' << before.kp_apr << ','
                << before.ki_apr << "] gains_after=[" << requested.kp_asr
                << ',' << requested.ki_asr << ',' << requested.kp_apr << ','
                << requested.ki_apr << "] persisted="
                << (persist_pv_gains ? "true" : "false") << '\n';
    } else if (set_deta) {
      const std::string role = argv[2];
      const float requested = std::stof(argv[3]);
      const float expected = std::stof(argv[5]);
      if (!std::isfinite(requested) || requested < 1.0f || requested > 30.0f ||
          !std::isfinite(expected) || expected < 1.0f || expected > 30.0f) {
        throw std::invalid_argument("Deta must be finite and within 1..30");
      }
      check_all_arm_motors_disabled(*product.resources);
      auto* motor = find_arm_motor(*product.resources, role);
      constexpr uint8_t kDetaRid = 31;
      const auto timeout = std::chrono::milliseconds(100);
      const float before = motor->get_register_f32(kDetaRid, timeout);
      if (!gain_matches(before, expected)) {
        throw std::runtime_error(role + " Deta precondition failed");
      }
      motor->write_register_f32(kDetaRid, requested);
      const float after = motor->get_register_f32(kDetaRid, timeout);
      if (!gain_matches(after, requested)) {
        motor->write_register_f32(kDetaRid, before);
        throw std::runtime_error(role + " Deta readback mismatch");
      }
      std::cout << std::setprecision(9) << "motor=" << role
                << " register=Deta before=" << before << " after=" << after
                << " persisted=false\n";
    } else if (set_acc_dec) {
      const std::string role = argv[2];
      const float requested_acc = std::stof(argv[3]);
      const float requested_dec = std::stof(argv[4]);
      const float expected_acc = std::stof(argv[6]);
      const float expected_dec = std::stof(argv[7]);
      if (!std::isfinite(requested_acc) || requested_acc <= 0.0f ||
          requested_acc > 100.0f || !std::isfinite(requested_dec) ||
          requested_dec >= 0.0f || requested_dec < -100.0f ||
          !std::isfinite(expected_acc) || expected_acc <= 0.0f ||
          !std::isfinite(expected_dec) || expected_dec >= 0.0f) {
        throw std::invalid_argument("ACC/DEC values are outside safety bounds");
      }
      check_all_arm_motors_disabled(*product.resources);
      auto* motor = find_arm_motor(*product.resources, role);
      constexpr uint8_t kAccRid = 4;
      constexpr uint8_t kDecRid = 5;
      const auto timeout = std::chrono::milliseconds(100);
      const float before_acc = motor->get_register_f32(kAccRid, timeout);
      const float before_dec = motor->get_register_f32(kDecRid, timeout);
      if (!gain_matches(before_acc, expected_acc) ||
          !gain_matches(before_dec, expected_dec)) {
        throw std::runtime_error(role + " ACC/DEC precondition failed");
      }
      try {
        motor->write_register_f32(kAccRid, requested_acc);
        motor->write_register_f32(kDecRid, requested_dec);
        const float after_acc = motor->get_register_f32(kAccRid, timeout);
        const float after_dec = motor->get_register_f32(kDecRid, timeout);
        if (!gain_matches(after_acc, requested_acc) ||
            !gain_matches(after_dec, requested_dec)) {
          throw std::runtime_error(role + " ACC/DEC readback mismatch");
        }
      } catch (...) {
        motor->write_register_f32(kAccRid, before_acc);
        motor->write_register_f32(kDecRid, before_dec);
        throw;
      }
      std::cout << std::setprecision(9) << "motor=" << role
                << " registers=ACC,DEC before=[" << before_acc << ','
                << before_dec << "] after=[" << requested_acc << ','
                << requested_dec << "] persisted=false\n";
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
    if (!strict_read_only) product.runtime->disconnect();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
