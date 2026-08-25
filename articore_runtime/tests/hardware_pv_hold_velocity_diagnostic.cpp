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
#include <vector>

#include "articore/detail/yunyi_runtime.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using Arm = std::array<float, ARTICORE_PRODUCT_ARM_DOF>;
using Product = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr Arm kPoint17{
    -0.308804512f, -0.254253387f, 0.548752785f, 1.296826363f,
    -0.279048920f, -0.192454338f, 0.111581802f};
constexpr Arm kPoint19{
    -0.369077682f, 0.123407364f, 0.153924942f, 1.584839821f,
    0.040246010f, -0.516708374f, 0.071526527f};
constexpr std::array<float, 4> kHoldVelocityLimits{3.0f, 1.0f, 0.5f, 0.2f};
constexpr float kMoveReferenceVelocity = 0.5f;
constexpr float kMoveVelocityLimit = 3.0f;
constexpr float kAbortVelocity = 3.5f;
constexpr auto kHoldDuration = std::chrono::seconds(8);

struct Snapshot {
  uint64_t update_count = 0;
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> q{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> dq{};
};

struct HoldSample {
  uint64_t timestamp_ns = 0;
  Snapshot state;
};

uint64_t now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch()).count());
}

void require_healthy(const articore::SafetyRuntime& runtime) {
  const auto health = runtime.health_v2();
  if (health.health.state == ARTICORE_FAULT ||
      health.health.state == ARTICORE_SAFE_STOP ||
      health.health.state == ARTICORE_DEGRADED ||
      !health.health.left_transport.healthy ||
      !health.health.right_transport.healthy) {
    throw std::runtime_error(
        std::string("unsafe Runtime state: ") +
        (health.health.fault_reason[0] ? health.health.fault_reason
                                      : health.safety_reason));
  }
}

Snapshot read_snapshot(const articore::YunyiRuntimeResources& resources,
                       bool require_enabled = true) {
  Snapshot result;
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    ArticoreMotorState state{};
    ArticoreFeedbackStats stats{};
    if (!articore::read_yunyi_motor_state(
            resources.joints[index].motor, state, stats) ||
        !state.has_value || !stats.has_feedback || stats.age_ns > 20'000'000ULL ||
        !std::isfinite(state.pos) || !std::isfinite(state.vel)) {
      throw std::runtime_error("joint feedback is incomplete or stale at index " +
                               std::to_string(index));
    }
    if (require_enabled && state.status_code != 1) {
      throw std::runtime_error("joint is not enabled at index " +
                               std::to_string(index));
    }
    const auto& joint = resources.joints[index];
    result.q[index] = joint.direction * state.pos;
    result.dq[index] =
        joint.direction * state.vel * joint.velocity_feedback_scale;
    if (index == 0) result.update_count = stats.update_count;
  }
  return result;
}

std::vector<ArticoreJointPvTarget> make_targets(
    const articore::YunyiRuntimeResources& resources,
    const Product& logical_positions) {
  std::vector<ArticoreJointPvTarget> targets;
  targets.reserve(resources.joints.size());
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    targets.push_back(ArticoreJointPvTarget{
        sizeof(ArticoreJointPvTarget), resources.joints[index].motor,
        resources.joints[index].direction * logical_positions[index]});
  }
  return targets;
}

void wait_stable(articore::SafetyRuntime& runtime,
                 const articore::YunyiRuntimeResources& resources,
                 const Product& target,
                 std::chrono::seconds timeout = std::chrono::seconds(20)) {
  const auto deadline = Clock::now() + timeout;
  Clock::time_point stable_since{};
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_snapshot(resources);
    float max_error = 0.0f;
    float max_velocity = 0.0f;
    for (std::size_t index = 0; index < state.q.size(); ++index) {
      max_error = std::max(max_error, std::abs(state.q[index] - target[index]));
      max_velocity = std::max(max_velocity, std::abs(state.dq[index]));
    }
    if (max_velocity > kAbortVelocity) {
      throw std::runtime_error("measured velocity exceeded abort limit");
    }
    if (max_error <= 0.01f && max_velocity <= 0.05f) {
      if (stable_since == Clock::time_point{}) stable_since = Clock::now();
      if (Clock::now() - stable_since >= std::chrono::milliseconds(400)) return;
    } else {
      stable_since = Clock::time_point{};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("ordinary PV target did not settle within 20 s");
}

void move(articore::SafetyRuntime& runtime,
          const articore::YunyiRuntimeResources& resources,
          const Product& target) {
  const auto commands = make_targets(resources, target);
  runtime.set_joint_pv(
      commands.data(), static_cast<uint32_t>(commands.size()),
      kMoveReferenceVelocity, kMoveVelocityLimit);
  wait_stable(runtime, resources, target);
}

std::vector<HoldSample> collect_hold(
    articore::SafetyRuntime& runtime,
    const articore::YunyiRuntimeResources& resources) {
  std::vector<HoldSample> samples;
  samples.reserve(4200);
  const auto deadline = Clock::now() + kHoldDuration;
  uint64_t last_update_count = 0;
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_snapshot(resources);
    if (state.update_count != last_update_count) {
      last_update_count = state.update_count;
      const float max_velocity = *std::max_element(
          state.dq.begin(), state.dq.end(),
          [](float lhs, float rhs) {
            return std::abs(lhs) < std::abs(rhs);
          });
      if (std::abs(max_velocity) > kAbortVelocity) {
        throw std::runtime_error("abort velocity exceeded during hold");
      }
      samples.push_back(HoldSample{now_ns(), state});
    }
    std::this_thread::sleep_for(std::chrono::microseconds(250));
  }
  return samples;
}

void write_samples(std::ofstream& output, int point_index,
                   float hold_velocity_limit,
                   const std::vector<HoldSample>& samples) {
  for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
    const auto& sample = samples[sample_index];
    output << point_index << ',' << hold_velocity_limit << ',' << sample_index
           << ',' << sample.timestamp_ns;
    for (float value : sample.state.q) output << ',' << value;
    for (float value : sample.state.dq) output << ',' << value;
    output << '\n';
  }
  output.flush();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 ||
      std::strcmp(argv[1], "--i-understand-left-arm-will-move") != 0) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-left-arm-will-move\n";
    return 2;
  }

  std::ofstream output("/tmp/articore_pv_hold_velocity_ab_trace.csv");
  output << "point_index,hold_velocity_limit_rad_s,sample_index,timestamp_ns";
  for (const char* side : {"left", "right"}) {
    for (int joint = 1; joint <= 7; ++joint) {
      output << ",q_" << side << "_j" << joint;
    }
  }
  for (const char* side : {"left", "right"}) {
    for (int joint = 1; joint <= 7; ++joint) {
      output << ",dq_" << side << "_j" << joint;
    }
  }
  output << '\n' << std::setprecision(9);

  auto bundle = articore::create_yunyi_runtime(ARTICORE_MODE_PV, false);
  auto& runtime = *bundle.runtime;
  auto& resources = *bundle.resources;
  bool connected = false;
  bool enabled = false;
  Product initial{};
  try {
    runtime.connect();
    connected = true;
    const auto initial_state = read_snapshot(resources, false);
    initial = initial_state.q;
    for (const auto& joint : resources.joints) {
      ArticoreMotorState state{};
      ArticoreFeedbackStats stats{};
      if (!articore::read_yunyi_motor_state(joint.motor, state, stats) ||
          state.status_code != 0) {
        throw std::runtime_error("all arm joints must start disabled");
      }
    }

    runtime.enable(ARTICORE_MODE_PV);
    enabled = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    require_healthy(runtime);
    read_snapshot(resources);

    for (const auto& point : std::array<std::pair<int, Arm>, 2>{
             std::pair<int, Arm>{17, kPoint17},
             std::pair<int, Arm>{19, kPoint19}}) {
      Product target = initial;
      std::copy(point.second.begin(), point.second.end(), target.begin());
      std::cout << "MOVE point=" << point.first << std::endl;
      move(runtime, resources, target);
      const auto commands = make_targets(resources, target);
      for (float hold_limit : kHoldVelocityLimits) {
        runtime.set_joint_pv(
            commands.data(), static_cast<uint32_t>(commands.size()),
            kMoveReferenceVelocity, hold_limit);
        std::cout << "HOLD point=" << point.first
                  << " v_des=" << hold_limit << " rad/s" << std::endl;
        const auto samples = collect_hold(runtime, resources);
        write_samples(output, point.first, hold_limit, samples);
        std::cout << "DONE point=" << point.first
                  << " v_des=" << hold_limit
                  << " samples=" << samples.size() << std::endl;
      }
    }

    std::cout << "RESTORE initial" << std::endl;
    move(runtime, resources, initial);
    runtime.disable();
    enabled = false;
    runtime.disconnect();
    connected = false;
    std::cout << "RESULT trace=/tmp/articore_pv_hold_velocity_ab_trace.csv"
              << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (enabled) {
      try {
        move(runtime, resources, initial);
      } catch (const std::exception& restore_error) {
        std::cerr << "RESTORE_ERROR " << restore_error.what() << '\n';
      }
      try {
        runtime.disable();
      } catch (...) {
      }
    }
    if (connected) {
      try {
        runtime.disconnect();
      } catch (...) {
      }
    }
    return 1;
  }
  return 0;
}
