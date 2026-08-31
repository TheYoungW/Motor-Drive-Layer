#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include "articore/runtime_abi.h"

namespace {

using Clock = std::chrono::steady_clock;
using Product = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kSpeedPercent = 50.0f;
constexpr float kPositionToleranceRad = 0.02f;
constexpr float kVelocityToleranceRadS = 0.05f;
constexpr float kAbortVelocityRadS = 3.0f;
constexpr auto kStableWindow = std::chrono::milliseconds(250);
constexpr auto kTimeout = std::chrono::seconds(12);
constexpr const char* kTracePath =
    "/tmp/articore_joint_ptp_j4_90_trace.csv";

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
    const auto velocity = velocities(state);
    float maximum_velocity = 0.0f;
    for (float value : velocity) {
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

void print_product(const char* label, const Product& values) {
  std::cout << label << '=';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index) std::cout << ',';
    std::cout << values[index];
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || std::strcmp(
          argv[1], "--i-understand-both-arms-will-move") != 0) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-both-arms-will-move\n";
    return 2;
  }

  Product target{};
  target[3] = kPi / 2.0f;
  target[ARTICORE_PRODUCT_ARM_DOF + 3] = kPi / 2.0f;

  ArticoreRuntime* runtime = nullptr;
  bool connected = false;
  bool enabled = false;
  std::ofstream trace(kTracePath);
  trace << "elapsed_s,sequence";
  for (std::size_t index = 0; index < target.size(); ++index) {
    trace << ",q" << index + 1;
  }
  for (std::size_t index = 0; index < target.size(); ++index) {
    trace << ",dq" << index + 1;
  }
  trace << ",maximum_error_rad,maximum_velocity_rad_s\n";

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
    for (std::size_t index = 0; index < target.size(); ++index) {
      if (target[index] < limits.lower_angles[index] ||
          target[index] > limits.upper_angles[index]) {
        throw std::runtime_error(
            "requested J4=90/others=0 target exceeds limits at joint " +
            std::to_string(index));
      }
    }

    const Product connected_position = positions(read_state(runtime));
    print_product("connected_q", connected_position);
    print_product("target_q", target);
    std::cout << "speed_percent=" << kSpeedPercent << '\n';

    check(articore_runtime_enable(runtime), "enable");
    enabled = true;
    wait_enabled_and_stationary(runtime);
    const Product start = positions(read_state(runtime));
    const auto started = Clock::now();
    check(articore_runtime_set_joint_pv(
              runtime, target.data(), static_cast<uint32_t>(target.size()),
              kSpeedPercent),
          "set_joint_pv");

    std::array<double, ARTICORE_PRODUCT_DUAL_ARM_DOF> arrival_s{};
    arrival_s.fill(-1.0);
    Clock::time_point stable_since{};
    uint64_t last_sequence = 0;
    uint64_t feedback_samples = 0;
    float peak_velocity = 0.0f;
    float final_error = std::numeric_limits<float>::infinity();
    bool settled = false;
    while (Clock::now() - started < kTimeout) {
      require_healthy(runtime);
      const auto state = read_state(runtime);
      if (state.sequence == last_sequence) {
        std::this_thread::sleep_for(std::chrono::microseconds(250));
        continue;
      }
      last_sequence = state.sequence;
      ++feedback_samples;
      const auto current = positions(state);
      const auto velocity = velocities(state);
      const double elapsed_s =
          std::chrono::duration<double>(Clock::now() - started).count();
      float maximum_error = 0.0f;
      float maximum_velocity = 0.0f;
      for (std::size_t index = 0; index < target.size(); ++index) {
        maximum_error = std::max(
            maximum_error, std::abs(current[index] - target[index]));
        maximum_velocity = std::max(
            maximum_velocity, std::abs(velocity[index]));
        const float displacement = target[index] - start[index];
        if (arrival_s[index] < 0.0 && std::abs(displacement) >= 0.05f) {
          const float phase = (current[index] - start[index]) / displacement;
          if (phase >= 0.95f) arrival_s[index] = elapsed_s;
        }
      }
      peak_velocity = std::max(peak_velocity, maximum_velocity);
      final_error = maximum_error;
      trace << std::setprecision(10) << elapsed_s << ',' << state.sequence;
      for (float value : current) trace << ',' << value;
      for (float value : velocity) trace << ',' << value;
      trace << ',' << maximum_error << ',' << maximum_velocity << '\n';

      if (maximum_velocity > kAbortVelocityRadS) {
        throw std::runtime_error(
            "measured velocity exceeded 3 rad/s abort threshold");
      }
      if (maximum_error <= kPositionToleranceRad &&
          maximum_velocity <= kVelocityToleranceRadS) {
        if (stable_since == Clock::time_point{}) stable_since = Clock::now();
        if (Clock::now() - stable_since >= kStableWindow) {
          settled = true;
          break;
        }
      } else {
        stable_since = Clock::time_point{};
      }
    }
    if (!settled) {
      throw std::runtime_error("J4=90/others=0 target did not settle in 12 s");
    }
    trace.flush();

    double earliest_arrival = std::numeric_limits<double>::infinity();
    double latest_arrival = 0.0;
    uint32_t moving_joints = 0;
    for (std::size_t index = 0; index < target.size(); ++index) {
      if (std::abs(target[index] - start[index]) < 0.05f) continue;
      if (arrival_s[index] < 0.0) {
        throw std::runtime_error(
            "moving joint did not cross 95% phase at index " +
            std::to_string(index));
      }
      earliest_arrival = std::min(earliest_arrival, arrival_s[index]);
      latest_arrival = std::max(latest_arrival, arrival_s[index]);
      ++moving_joints;
      std::cout << "arrival joint=" << index
                << " elapsed_s=" << arrival_s[index] << '\n';
    }
    const double settled_s =
        std::chrono::duration<double>(Clock::now() - started).count();
    const Product final_position = positions(read_state(runtime));
    print_product("final_q", final_position);
    std::cout << "RESULT pass=true settled_s=" << settled_s
              << " feedback_hz=" << feedback_samples / settled_s
              << " moving_joints=" << moving_joints
              << " arrival_spread_s=" << latest_arrival - earliest_arrival
              << " peak_velocity_rad_s=" << peak_velocity
              << " final_error_rad=" << final_error
              << " trace=" << kTracePath << '\n';

    check(articore_runtime_disable(runtime), "disable");
    enabled = false;
    check(articore_runtime_disconnect(runtime), "disconnect");
    connected = false;
    articore_runtime_free(runtime);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (runtime && enabled) articore_runtime_disable(runtime);
    if (runtime && connected) articore_runtime_disconnect(runtime);
    if (runtime) articore_runtime_free(runtime);
    return 1;
  }
}
