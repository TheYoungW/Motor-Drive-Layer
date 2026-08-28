#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "articore/runtime_abi.h"

namespace {

using Clock = std::chrono::steady_clock;
using Product = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr std::size_t kRightJ1 = ARTICORE_PRODUCT_ARM_DOF;
constexpr std::size_t kRightJ2 = ARTICORE_PRODUCT_ARM_DOF + 1;
constexpr float kJ1TravelRad = 0.12f;
constexpr float kJ2TravelRad = 0.06f;
constexpr float kSpeedPercent = 10.0f;
constexpr float kAccelerationRadS2 = 2.0f;
constexpr float kAbortVelocityRadS = 1.0f;
constexpr float kPositionToleranceRad = 0.01f;
constexpr float kVelocityToleranceRadS = 0.05f;
constexpr auto kStableWindow = std::chrono::milliseconds(250);
constexpr auto kLegTimeout = std::chrono::seconds(8);
constexpr const char* kTracePath =
    "/tmp/articore_joint_ptp_septic_pilot.csv";

void check(int32_t result, const char* operation) {
  if (result == ARTICORE_OPERATION_OK) return;
  throw std::runtime_error(
      std::string(operation) + " failed: " + articore_runtime_last_error());
}

ArticoreProductState read_state(ArticoreRuntime* runtime) {
  ArticoreProductState state{};
  state.struct_size = sizeof(state);
  check(articore_runtime_get_state(runtime, &state), "get_state");
  return state;
}

Product positions(const ArticoreProductState& state) {
  Product result{};
  std::copy(std::begin(state.left.positions), std::end(state.left.positions),
            result.begin());
  std::copy(std::begin(state.right.positions), std::end(state.right.positions),
            result.begin() + ARTICORE_PRODUCT_ARM_DOF);
  return result;
}

Product velocities(const ArticoreProductState& state) {
  Product result{};
  std::copy(std::begin(state.left.velocities), std::end(state.left.velocities),
            result.begin());
  std::copy(std::begin(state.right.velocities), std::end(state.right.velocities),
            result.begin() + ARTICORE_PRODUCT_ARM_DOF);
  return result;
}

void require_healthy(ArticoreRuntime* runtime) {
  ArticoreSafetyHealth health{};
  health.struct_size = sizeof(health);
  check(articore_runtime_get_health(runtime, &health), "get_health");
  if (health.state == ARTICORE_FAULT || health.state == ARTICORE_SAFE_STOP ||
      health.degraded || !health.left_transport.healthy ||
      !health.right_transport.healthy) {
    const char* reason = health.fault_reason[0] != '\0'
        ? health.fault_reason : health.safety_reason;
    throw std::runtime_error(std::string("unsafe Runtime state: ") + reason);
  }
}

void wait_enabled_and_stationary(ArticoreRuntime* runtime) {
  const auto deadline = Clock::now() + std::chrono::seconds(4);
  Clock::time_point stable_since{};
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_state(runtime);
    float maximum_velocity = 0.0f;
    for (float value : state.left.velocities) {
      maximum_velocity = std::max(maximum_velocity, std::abs(value));
    }
    for (float value : state.right.velocities) {
      maximum_velocity = std::max(maximum_velocity, std::abs(value));
    }
    const bool ready = state.left.enabled_valid_mask == 0x7fU &&
                       state.right.enabled_valid_mask == 0x7fU &&
                       state.left.enabled_mask == 0x7fU &&
                       state.right.enabled_mask == 0x7fU &&
                       maximum_velocity <= kVelocityToleranceRadS;
    if (ready) {
      if (stable_since == Clock::time_point{}) stable_since = Clock::now();
      if (Clock::now() - stable_since >= std::chrono::milliseconds(150)) {
        return;
      }
    } else {
      stable_since = Clock::time_point{};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("all joints did not become enabled and stationary");
}

float choose_delta(float position, float lower, float upper, float magnitude) {
  constexpr float margin = 0.10f;
  if (upper - position >= magnitude + margin) return magnitude;
  if (position - lower >= magnitude + margin) return -magnitude;
  throw std::runtime_error("selected joint lacks safe pilot travel margin");
}

struct LegResult {
  double settled_s = 0.0;
  double arrival_difference_s = 0.0;
  float peak_velocity_rad_s = 0.0f;
  float maximum_phase_error = 0.0f;
  float rms_phase_error = 0.0f;
  float endpoint_error_rad = 0.0f;
  uint64_t phase_samples = 0;
  uint64_t feedback_samples = 0;
};

LegResult move_and_measure(ArticoreRuntime* runtime, const Product& start,
                           const Product& target, const char* leg,
                           std::ofstream& trace) {
  const float delta_j1 = target[kRightJ1] - start[kRightJ1];
  const float delta_j2 = target[kRightJ2] - start[kRightJ2];
  if (std::abs(delta_j1) < 0.01f || std::abs(delta_j2) < 0.01f) {
    throw std::runtime_error("pilot leg requires two non-zero joint moves");
  }

  const auto started = Clock::now();
  check(articore_runtime_set_joint_pv(
            runtime, target.data(), static_cast<uint32_t>(target.size()),
            kSpeedPercent),
        "set_joint_pv");

  LegResult result;
  Clock::time_point stable_since{};
  std::array<double, 2> arrival_s{-1.0, -1.0};
  double squared_phase_error_sum = 0.0;
  uint64_t last_sequence = 0;
  while (Clock::now() - started < kLegTimeout) {
    require_healthy(runtime);
    const auto state = read_state(runtime);
    if (state.sequence == last_sequence) {
      std::this_thread::sleep_for(std::chrono::microseconds(250));
      continue;
    }
    last_sequence = state.sequence;
    ++result.feedback_samples;
    const auto current = positions(state);
    const auto velocity = velocities(state);
    const double elapsed_s =
        std::chrono::duration<double>(Clock::now() - started).count();
    const float phase_j1 =
        (current[kRightJ1] - start[kRightJ1]) / delta_j1;
    const float phase_j2 =
        (current[kRightJ2] - start[kRightJ2]) / delta_j2;
    const float phase_error = std::abs(phase_j1 - phase_j2);

    float maximum_error = 0.0f;
    float maximum_velocity = 0.0f;
    for (std::size_t index = 0; index < current.size(); ++index) {
      maximum_error = std::max(
          maximum_error, std::abs(current[index] - target[index]));
      maximum_velocity = std::max(maximum_velocity, std::abs(velocity[index]));
    }
    result.peak_velocity_rad_s =
        std::max(result.peak_velocity_rad_s, maximum_velocity);
    result.endpoint_error_rad = maximum_error;

    if (phase_j1 >= 0.10f && phase_j1 <= 0.90f &&
        phase_j2 >= 0.10f && phase_j2 <= 0.90f) {
      result.maximum_phase_error =
          std::max(result.maximum_phase_error, phase_error);
      squared_phase_error_sum +=
          static_cast<double>(phase_error) * phase_error;
      ++result.phase_samples;
    }
    // Five percent corresponds to 0.006/0.003 rad for this guarded pilot.
    // A 98% threshold falls below the encoder/steady-state resolution of the
    // shorter J2 leg and can reject a physically settled endpoint.
    if (arrival_s[0] < 0.0 && phase_j1 >= 0.95f) arrival_s[0] = elapsed_s;
    if (arrival_s[1] < 0.0 && phase_j2 >= 0.95f) arrival_s[1] = elapsed_s;

    trace << leg << ',' << std::setprecision(10) << elapsed_s << ','
          << state.sequence << ',' << current[kRightJ1] << ','
          << current[kRightJ2] << ',' << velocity[kRightJ1] << ','
          << velocity[kRightJ2] << ',' << phase_j1 << ',' << phase_j2 << ','
          << phase_error << ',' << maximum_error << ',' << maximum_velocity
          << '\n';

    if (maximum_velocity > kAbortVelocityRadS) {
      throw std::runtime_error(
          "measured velocity exceeded 1 rad/s pilot abort threshold");
    }
    if (maximum_error <= kPositionToleranceRad &&
        maximum_velocity <= kVelocityToleranceRadS) {
      if (stable_since == Clock::time_point{}) stable_since = Clock::now();
      if (Clock::now() - stable_since >= kStableWindow) {
        result.settled_s = elapsed_s;
        if (result.phase_samples > 0) {
          result.rms_phase_error = static_cast<float>(std::sqrt(
              squared_phase_error_sum /
              static_cast<double>(result.phase_samples)));
        }
        if (arrival_s[0] < 0.0 || arrival_s[1] < 0.0) {
          throw std::runtime_error("both pilot joints did not reach 95% phase");
        }
        result.arrival_difference_s = std::abs(arrival_s[0] - arrival_s[1]);
        trace.flush();
        return result;
      }
    } else {
      stable_since = Clock::time_point{};
    }
  }
  throw std::runtime_error(std::string(leg) + " did not settle within 8 s");
}

void print_result(const char* leg, const LegResult& result) {
  std::cout << "LEG " << leg
            << " settled_s=" << result.settled_s
            << " peak_velocity_rad_s=" << result.peak_velocity_rad_s
            << " arrival_difference_s=" << result.arrival_difference_s
            << " maximum_phase_error=" << result.maximum_phase_error
            << " rms_phase_error=" << result.rms_phase_error
            << " endpoint_error_rad=" << result.endpoint_error_rad
            << " feedback_samples=" << result.feedback_samples << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || std::strcmp(
          argv[1],
          "--i-understand-right-arm-j1-j2-will-move") != 0) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-right-arm-j1-j2-will-move\n";
    return 2;
  }

  ArticoreRuntime* runtime = nullptr;
  bool connected = false;
  bool enabled = false;
  Product initial{};
  std::ofstream trace(kTracePath);
  trace << "leg,elapsed_s,sequence,right_j1_position_rad,"
           "right_j2_position_rad,right_j1_velocity_rad_s,"
           "right_j2_velocity_rad_s,right_j1_phase,right_j2_phase,"
           "phase_error,maximum_product_error_rad,"
           "maximum_product_velocity_rad_s\n";
  try {
    check(articore_runtime_create_yunyi(ARTICORE_MODE_PV, 0, &runtime),
          "create_yunyi");
    check(articore_runtime_connect(runtime), "connect");
    connected = true;
    require_healthy(runtime);

    ArticoreProductJointAngleVelLimits limits{};
    limits.struct_size = sizeof(limits);
    check(articore_runtime_get_joint_angle_vel_limits(runtime, &limits),
          "get_joint_angle_vel_limits");
    initial = positions(read_state(runtime));
    for (std::size_t index = 0; index < initial.size(); ++index) {
      if (!std::isfinite(initial[index]) ||
          initial[index] < limits.lower_angles[index] ||
          initial[index] > limits.upper_angles[index]) {
        throw std::runtime_error(
            "initial feedback is outside product limits at joint " +
            std::to_string(index));
      }
    }

    Product target = initial;
    target[kRightJ1] += choose_delta(
        initial[kRightJ1], limits.lower_angles[kRightJ1],
        limits.upper_angles[kRightJ1], kJ1TravelRad);
    target[kRightJ2] += choose_delta(
        initial[kRightJ2], limits.lower_angles[kRightJ2],
        limits.upper_angles[kRightJ2], kJ2TravelRad);
    std::cout << "START right_j1=" << initial[kRightJ1]
              << " target_j1=" << target[kRightJ1]
              << " right_j2=" << initial[kRightJ2]
              << " target_j2=" << target[kRightJ2]
              << " speed_percent=" << kSpeedPercent
              << " acceleration_rad_s2=" << kAccelerationRadS2 << '\n';

    check(articore_runtime_enable(runtime), "enable");
    enabled = true;
    wait_enabled_and_stationary(runtime);
    check(articore_runtime_set_max_acceleration(runtime, kAccelerationRadS2),
          "set_max_acceleration");

    const auto outward = move_and_measure(
        runtime, positions(read_state(runtime)), target, "outward", trace);
    print_result("outward", outward);
    const auto returned = move_and_measure(
        runtime, positions(read_state(runtime)), initial, "return", trace);
    print_result("return", returned);

    if (outward.maximum_phase_error > 0.20f ||
        returned.maximum_phase_error > 0.20f ||
        outward.arrival_difference_s > 0.20 ||
        returned.arrival_difference_s > 0.20) {
      throw std::runtime_error(
          "measured joint synchronization exceeded pilot acceptance limits");
    }

    check(articore_runtime_disable(runtime), "disable");
    enabled = false;
    check(articore_runtime_disconnect(runtime), "disconnect");
    connected = false;
    articore_runtime_free(runtime);
    runtime = nullptr;
    std::cout << "RESULT pass=true trace=" << kTracePath << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (runtime && enabled) {
      try {
        articore_runtime_set_joint_pv(
            runtime, initial.data(), static_cast<uint32_t>(initial.size()),
            kSpeedPercent);
        std::this_thread::sleep_for(std::chrono::seconds(3));
      } catch (...) {
      }
      articore_runtime_disable(runtime);
    }
    if (runtime && connected) articore_runtime_disconnect(runtime);
    if (runtime) articore_runtime_free(runtime);
    return 1;
  }
}
