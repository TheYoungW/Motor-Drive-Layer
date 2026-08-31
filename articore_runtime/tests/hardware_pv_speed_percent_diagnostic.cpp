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

constexpr std::size_t kRightJoint1 = ARTICORE_PRODUCT_ARM_DOF;
constexpr float kTravelRad = 0.30f;
constexpr float kConfiguredMaximumSpeedRadS = 0.60f;
constexpr float kConfiguredMaximumAccelerationRadS2 = 4.00f;
constexpr float kPositionToleranceRad = 0.02f;
constexpr float kVelocityToleranceRadS = 0.05f;
constexpr float kAbortVelocityRadS = 3.0f;
constexpr auto kStableWindow = std::chrono::milliseconds(250);
constexpr auto kLegTimeout = std::chrono::seconds(8);
constexpr const char* kTracePath =
    "/tmp/articore_pv_user_limits_trace.csv";

void check(int32_t result, const char* operation) {
  if (result != ARTICORE_OPERATION_OK) {
    throw std::runtime_error(
        std::string(operation) + " failed: " + articore_runtime_last_error());
  }
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

void require_healthy(ArticoreRuntime* runtime) {
  ArticoreSafetyHealth health{};
  health.struct_size = sizeof(health);
  check(articore_runtime_get_health(runtime, &health), "get_health");
  if (health.state == ARTICORE_FAULT || health.state == ARTICORE_SAFE_STOP ||
      health.degraded || !health.left_transport.healthy ||
      !health.right_transport.healthy) {
    const char* reason = health.fault_reason[0] != '\0'
        ? health.fault_reason : health.safety_reason;
    throw std::runtime_error(
        std::string("unsafe Runtime state: ") + reason);
  }
}

struct LegResult {
  double duration_s = 0.0;
  float peak_right_j1_velocity = 0.0f;
  float endpoint_error = 0.0f;
  uint64_t samples = 0;
};

LegResult move_and_measure(
    ArticoreRuntime* runtime, const Product& target, float speed_percent,
    const char* leg, std::ofstream& trace) {
  const auto started = Clock::now();
  check(articore_runtime_set_joint_pv(
            runtime, target.data(), static_cast<uint32_t>(target.size()),
            speed_percent),
        "set_joint_pv");

  LegResult result;
  Clock::time_point stable_since{};
  uint64_t last_sequence = 0;
  while (Clock::now() - started < kLegTimeout) {
    require_healthy(runtime);
    const auto state = read_state(runtime);
    if (state.sequence == last_sequence) {
      std::this_thread::sleep_for(std::chrono::microseconds(250));
      continue;
    }
    last_sequence = state.sequence;
    ++result.samples;

    const double elapsed_s =
        std::chrono::duration<double>(Clock::now() - started).count();
    float maximum_error = 0.0f;
    float maximum_velocity = 0.0f;
    for (std::size_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      maximum_error = std::max(
          maximum_error,
          std::abs(state.left.positions[index] - target[index]));
      maximum_error = std::max(
          maximum_error,
          std::abs(state.right.positions[index] -
                   target[index + ARTICORE_PRODUCT_ARM_DOF]));
      maximum_velocity = std::max(
          maximum_velocity, std::abs(state.left.velocities[index]));
      maximum_velocity = std::max(
          maximum_velocity, std::abs(state.right.velocities[index]));
    }
    const float right_j1_position = state.right.positions[0];
    const float right_j1_velocity = state.right.velocities[0];
    result.peak_right_j1_velocity = std::max(
        result.peak_right_j1_velocity, std::abs(right_j1_velocity));
    result.endpoint_error = std::abs(right_j1_position - target[kRightJoint1]);
    trace << leg << ',' << speed_percent << ',' << std::setprecision(10)
          << elapsed_s << ',' << state.sequence << ',' << right_j1_position
          << ',' << right_j1_velocity << ',' << target[kRightJoint1] << ','
          << result.endpoint_error << ',' << maximum_error << ','
          << maximum_velocity << '\n';

    if (maximum_velocity > kAbortVelocityRadS) {
      throw std::runtime_error(
          "measured joint velocity exceeded 3 rad/s abort threshold");
    }
    if (maximum_error <= kPositionToleranceRad &&
        maximum_velocity <= kVelocityToleranceRadS) {
      if (stable_since == Clock::time_point{}) stable_since = Clock::now();
      if (Clock::now() - stable_since >= kStableWindow) {
        result.duration_s = elapsed_s;
        trace.flush();
        return result;
      }
    } else {
      stable_since = Clock::time_point{};
    }
  }
  throw std::runtime_error(std::string(leg) + " did not settle within 8 s");
}

void print_result(const char* leg, float speed, const LegResult& result) {
  std::cout << "LEG " << leg << " speed_percent=" << speed
            << " duration_s=" << result.duration_s
            << " peak_right_j1_velocity_rad_s="
            << result.peak_right_j1_velocity
            << " endpoint_error_rad=" << result.endpoint_error
            << " samples=" << result.samples << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || std::strcmp(
          argv[1], "--i-understand-right-arm-j1-will-move") != 0) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-right-arm-j1-will-move\n";
    return 2;
  }

  ArticoreRuntime* runtime = nullptr;
  bool connected = false;
  bool enabled = false;
  Product initial{};
  std::ofstream trace(kTracePath);
  trace << "leg,speed_percent,elapsed_s,sequence,right_j1_position_rad,"
           "right_j1_velocity_rad_s,target_rad,right_j1_error_rad,"
           "maximum_product_error_rad,maximum_product_velocity_rad_s\n";
  try {
    check(articore_runtime_create_yunyi(
              ARTICORE_MODE_PV, 0, &runtime),
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
    const float positive_room =
        limits.upper_angles[kRightJoint1] - initial[kRightJoint1];
    const float negative_room =
        initial[kRightJoint1] - limits.lower_angles[kRightJoint1];
    const float direction = positive_room >= kTravelRad + 0.10f
        ? 1.0f : (negative_room >= kTravelRad + 0.10f ? -1.0f : 0.0f);
    if (direction == 0.0f) {
      throw std::runtime_error("right J1 lacks 0.3 rad of safe travel margin");
    }
    target[kRightJoint1] += direction * kTravelRad;
    std::cout << "START right_j1=" << initial[kRightJoint1]
              << " target=" << target[kRightJoint1]
              << " travel_rad=" << kTravelRad << '\n';

    check(articore_runtime_enable(runtime), "enable");
    enabled = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    require_healthy(runtime);

    check(articore_runtime_set_max_speed(
              runtime, kConfiguredMaximumSpeedRadS),
          "set_max_speed");
    check(articore_runtime_set_max_acceleration(
              runtime, kConfiguredMaximumAccelerationRadS2),
          "set_max_acceleration");
    float configured_speed = 0.0f;
    float configured_acceleration = 0.0f;
    check(articore_runtime_get_max_speed(runtime, &configured_speed),
          "get_max_speed");
    check(articore_runtime_get_max_acceleration(
              runtime, &configured_acceleration),
          "get_max_acceleration");
    if (std::abs(configured_speed - kConfiguredMaximumSpeedRadS) > 1e-6f ||
        std::abs(configured_acceleration -
                 kConfiguredMaximumAccelerationRadS2) > 1e-6f) {
      throw std::runtime_error("ordinary PV user limits did not round-trip");
    }
    std::cout << "LIMITS max_speed_rad_s=" << configured_speed
              << " max_acceleration_rad_s2=" << configured_acceleration
              << '\n';

    const auto fifty_out =
        move_and_measure(runtime, target, 50.0f, "50_out", trace);
    print_result("50_out", 50.0f, fifty_out);
    const auto fifty_back =
        move_and_measure(runtime, initial, 50.0f, "50_back", trace);
    print_result("50_back", 50.0f, fifty_back);
    const auto hundred_out =
        move_and_measure(runtime, target, 100.0f, "100_out", trace);
    print_result("100_out", 100.0f, hundred_out);
    const auto hundred_back =
        move_and_measure(runtime, initial, 100.0f, "100_back", trace);
    print_result("100_back", 100.0f, hundred_back);

    const float peak_50 = std::max(
        fifty_out.peak_right_j1_velocity,
        fifty_back.peak_right_j1_velocity);
    const float peak_100 = std::max(
        hundred_out.peak_right_j1_velocity,
        hundred_back.peak_right_j1_velocity);
    const double duration_50 = 0.5 *
        (fifty_out.duration_s + fifty_back.duration_s);
    const double duration_100 = 0.5 *
        (hundred_out.duration_s + hundred_back.duration_s);
    const float peak_ratio = peak_100 / peak_50;
    const double duration_ratio = duration_50 / duration_100;
    if (peak_50 < 0.10f ||
        peak_50 > kConfiguredMaximumSpeedRadS * 0.85f ||
        peak_100 > kConfiguredMaximumSpeedRadS * 1.75f ||
        peak_ratio < 1.60f || peak_ratio > 2.60f ||
        duration_ratio < 1.25) {
      throw std::runtime_error(
          "measured motion does not reflect configured 50/100 percent limits");
    }
    std::cout << "VERIFY peak_ratio=" << peak_ratio
              << " duration_ratio=" << duration_ratio << '\n';

    check(articore_runtime_set_max_speed(runtime, 0.0f),
          "clear_max_speed");
    check(articore_runtime_set_max_acceleration(runtime, 0.0f),
          "clear_max_acceleration");
    check(articore_runtime_get_max_speed(runtime, &configured_speed),
          "get_cleared_max_speed");
    check(articore_runtime_get_max_acceleration(
              runtime, &configured_acceleration),
          "get_cleared_max_acceleration");
    if (configured_speed != 0.0f || configured_acceleration != 0.0f) {
      throw std::runtime_error("ordinary PV user limits did not clear");
    }

    check(articore_runtime_disable(runtime), "disable");
    enabled = false;
    check(articore_runtime_disconnect(runtime), "disconnect");
    connected = false;
    articore_runtime_free(runtime);
    runtime = nullptr;
    std::cout << "RESULT trace=" << kTracePath << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (runtime && enabled) {
      try {
        articore_runtime_set_joint_pv(
            runtime, initial.data(), static_cast<uint32_t>(initial.size()),
            20.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));
      } catch (...) {
      }
      articore_runtime_disable(runtime);
    }
    if (runtime && connected) articore_runtime_disconnect(runtime);
    if (runtime) articore_runtime_free(runtime);
    return 1;
  }
}
