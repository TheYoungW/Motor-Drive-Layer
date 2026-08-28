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
#include <utility>
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
constexpr Arm kPoint86{
    -0.861181259f, -0.277523041f, -1.302929878f, -0.1744f,
    -2.0933f, -0.047112465f, 0.325589180f};
constexpr Arm kPoint118{
    -0.000190735f, -0.000190735f, -0.000190735f, 0.000190735f,
    -0.018882751f, -0.000572205f, -0.000572205f};
constexpr Arm kPoint25{
    0.237468719f, 0.423247337f, -0.399595261f, 1.027885437f,
    0.298504829f, -0.124933243f, 0.056267738f};
constexpr Arm kPoint84{
    -0.627717972f, -0.347715378f, -1.415083885f, -0.174400000f,
    -2.093300000f, 0.029563904f, 0.318722725f};
constexpr Arm kPoint58{
    -0.946249962f, 0.043679237f, -0.554474831f, -0.174400000f,
    -0.438124657f, 0.596437454f, -0.151254654f};
constexpr Arm kPoint131{
    0.019264221f, 0.014305115f, -0.000190735f, 0.022697449f,
    -0.026130676f, 0.000190735f, -0.030326843f};
constexpr Arm kPoint154{
    0.248149872f, -0.020790100f, 0.625429153f, 1.662280083f,
    -0.198176384f, 0.025367737f, -0.032616615f};
constexpr Arm kPoint27{
    0.378232956f, 0.448042870f, -0.159266472f, 0.532349586f,
    0.510223389f, 0.322155952f, 0.016975403f};
constexpr Arm kPoint2{
    0.257305145f, -0.189402580f, -0.406844139f, 0.304226875f,
    -0.461012840f, -0.785000000f, -0.250057220f};
constexpr Arm kPoint61{
    -0.790608406f, -0.174525261f, -0.458724022f, -0.174400000f,
    -0.514038086f, 0.497253418f, -0.127985001f};
constexpr Arm kPoint87{
    -0.887121201f, -0.201609612f, -1.280040741f, -0.174400000f,
    -2.093300000f, -0.063134193f, 0.325970650f};
constexpr Arm kPoint98{
    0.135614395f, 0.530822754f, -1.274700165f, -0.174400000f,
    -0.888647079f, 0.785000000f, -0.394636154f};

// Internal trajectory speed 50 maps to a 1 rad/s reference. Keep the
// independent Damiao POS_VEL catch-up ceiling at 3 rad/s.
constexpr float kReferenceVelocity = 1.0f;
constexpr float kVelocityLimit = 3.0f;
constexpr float kAbortMoveVelocity = 3.5f;
constexpr float kAbortHoldVelocity = 0.30f;
constexpr float kAbortHoldError = 0.030f;
constexpr auto kHoldDuration = std::chrono::seconds(1);

struct PvGains {
  float kp_asr;
  float ki_asr;
  float kp_apr;
  float ki_apr;
};

struct Profile {
  const char* family;
  const char* label;
  PvGains gains;
};

constexpr PvGains kExpectedBaseline{0.0092f, 0.002f, 54.0f, 0.0f};

constexpr std::array<Profile, 19> kProfiles{{
    {"kp_apr", "kp_apr_36", {0.0092f, 0.002f, 36.0f, 0.0f}},
    {"kp_apr", "kp_apr_45", {0.0092f, 0.002f, 45.0f, 0.0f}},
    {"kp_apr", "kp_apr_54", kExpectedBaseline},
    {"kp_apr", "kp_apr_63", {0.0092f, 0.002f, 63.0f, 0.0f}},
    {"kp_apr", "kp_apr_72", {0.0092f, 0.002f, 72.0f, 0.0f}},
    {"kp_asr", "kp_asr_0.0048", {0.0048f, 0.002f, 54.0f, 0.0f}},
    {"kp_asr", "kp_asr_0.0058", {0.0058f, 0.002f, 54.0f, 0.0f}},
    {"kp_asr", "kp_asr_0.0068", kExpectedBaseline},
    {"kp_asr", "kp_asr_0.0080", {0.0080f, 0.002f, 54.0f, 0.0f}},
    {"kp_asr", "kp_asr_0.0092", {0.0092f, 0.002f, 54.0f, 0.0f}},
    {"ki_asr", "ki_asr_0.0010", {0.0092f, 0.0010f, 54.0f, 0.0f}},
    {"ki_asr", "ki_asr_0.0015", {0.0092f, 0.0015f, 54.0f, 0.0f}},
    {"ki_asr", "ki_asr_0.0020", kExpectedBaseline},
    {"ki_asr", "ki_asr_0.0025", {0.0092f, 0.0025f, 54.0f, 0.0f}},
    {"ki_asr", "ki_asr_0.0030", {0.0092f, 0.0030f, 54.0f, 0.0f}},
    {"ki_apr", "ki_apr_0.000", kExpectedBaseline},
    {"ki_apr", "ki_apr_0.020", {0.0092f, 0.002f, 54.0f, 0.020f}},
    {"ki_apr", "ki_apr_0.050", {0.0092f, 0.002f, 54.0f, 0.050f}},
    {"ki_apr", "ki_apr_0.100", {0.0092f, 0.002f, 54.0f, 0.100f}},
}};

constexpr std::array<Profile, 5> kValidationProfiles{{
    {"kp_apr", "baseline_54_a1", kExpectedBaseline},
    {"mixed_j2", "j1base_j2p58_vp0p0120_vi0p0015_b1",
     {0.0120f, 0.0015f, 58.0f, 0.0f}},
    {"kp_apr", "baseline_54_a2", kExpectedBaseline},
    {"mixed_j2", "j1base_j2p58_vp0p0120_vi0p0015_b2",
     {0.0120f, 0.0015f, 58.0f, 0.0f}},
    {"kp_apr", "baseline_54_a3", kExpectedBaseline},
}};

PvGains validation_j1_gains(const Profile& profile) {
  return std::strcmp(profile.family, "mixed_j2") == 0
      ? kExpectedBaseline : profile.gains;
}

constexpr std::array<std::pair<int, Arm>, 3> kValidationPoints{{
    {25, kPoint25}, {17, kPoint17}, {19, kPoint19},
}};

struct Snapshot {
  uint64_t update_count = 0;
  Product q{};
  Product dq{};
};

uint64_t now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch()).count());
}

bool close(float lhs, float rhs) {
  return std::abs(lhs - rhs) <=
      1.0e-6f * std::max(1.0f, std::max(std::abs(lhs), std::abs(rhs)));
}

PvGains read_gains(damiao::MotorHandle* motor) {
  const auto timeout = std::chrono::milliseconds(100);
  return {motor->get_register_f32(25, timeout),
          motor->get_register_f32(26, timeout),
          motor->get_register_f32(27, timeout),
          motor->get_register_f32(28, timeout)};
}

uint32_t read_control_mode(damiao::MotorHandle* motor) {
  return motor->get_register_u32(10, std::chrono::milliseconds(100));
}

void print_validation_modes(const char* checkpoint,
                            damiao::MotorHandle* motor_j1,
                            damiao::MotorHandle* motor_j2) {
  std::cout << "MODES checkpoint=" << checkpoint
            << " J1=" << read_control_mode(motor_j1)
            << " J2=" << read_control_mode(motor_j2) << std::endl;
}

void require_validation_pv_mode(const char* checkpoint,
                                damiao::MotorHandle* motor_j1,
                                damiao::MotorHandle* motor_j2) {
  const auto j1_mode = read_control_mode(motor_j1);
  const auto j2_mode = read_control_mode(motor_j2);
  std::cout << "MODES checkpoint=" << checkpoint
            << " J1=" << j1_mode << " J2=" << j2_mode << std::endl;
  if (j1_mode != 2 || j2_mode != 2) {
    throw std::runtime_error(
        std::string("PV control-mode precondition failed at ") + checkpoint);
  }
}

bool gains_match(const PvGains& lhs, const PvGains& rhs) {
  return close(lhs.kp_asr, rhs.kp_asr) && close(lhs.ki_asr, rhs.ki_asr) &&
      close(lhs.kp_apr, rhs.kp_apr) && close(lhs.ki_apr, rhs.ki_apr);
}

void write_gains(damiao::MotorHandle* motor, const PvGains& gains) {
  motor->write_register_f32(25, gains.kp_asr);
  motor->write_register_f32(26, gains.ki_asr);
  motor->write_register_f32(27, gains.kp_apr);
  motor->write_register_f32(28, gains.ki_apr);
  if (!gains_match(read_gains(motor), gains)) {
    throw std::runtime_error("PV gain readback mismatch");
  }
}

void require_all_disabled(articore::YunyiRuntimeResources& resources) {
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    const auto state = resources.arm_motors[index]->request_fresh_state(
        std::chrono::milliseconds(100));
    if (!state || state->status_code != 0) {
      throw std::runtime_error("joint is not feedback-confirmed disabled at index " +
                               std::to_string(index));
    }
  }
}

void require_healthy(const articore::SafetyRuntime& runtime) {
  const auto health = runtime.health();
  if (health.state == ARTICORE_FAULT ||
      health.state == ARTICORE_SAFE_STOP ||
      health.state == ARTICORE_DEGRADED ||
      !health.left_transport.healthy ||
      !health.right_transport.healthy) {
    throw std::runtime_error(
        std::string("unsafe Runtime state: ") +
        (health.fault_reason[0] ? health.fault_reason
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
      throw std::runtime_error("joint feedback incomplete/stale at index " +
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
    const Product& positions) {
  std::vector<ArticoreJointPvTarget> targets;
  targets.reserve(resources.joints.size());
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    targets.push_back(ArticoreJointPvTarget{
        sizeof(ArticoreJointPvTarget), resources.joints[index].motor,
        resources.joints[index].direction * positions[index]});
  }
  return targets;
}

void wait_stable(articore::SafetyRuntime& runtime,
                 const articore::YunyiRuntimeResources& resources,
                 const Product& target) {
  const auto deadline = Clock::now() + std::chrono::seconds(20);
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
    if (max_velocity > kAbortMoveVelocity) {
      throw std::runtime_error("move velocity exceeded safety threshold");
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
  runtime.set_joint_pv(commands.data(), static_cast<uint32_t>(commands.size()),
                       kReferenceVelocity, kVelocityLimit);
  wait_stable(runtime, resources, target);
}

void seed_trace_joint_layout(
    articore::SafetyRuntime& runtime,
    const articore::YunyiRuntimeResources& resources,
    const Product& current) {
  articore::NativeTrajectoryRequest request;
  request.mode = ARTICORE_MODE_PV;
  request.joints.reserve(resources.joints.size());
  for (const auto& source : resources.joints) {
    articore::NativeTrajectoryJoint joint;
    joint.role = articore::yunyi_joint_role(
        static_cast<uint32_t>(request.joints.size()));
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
    joint.pv_velocity_limit = kVelocityLimit;
    joint.pv_hold_velocity_limit = 0.0f;
    request.joints.push_back(joint);
  }
  for (double time_s : {0.0, 0.1}) {
    articore::NativeTrajectoryWaypoint waypoint;
    waypoint.time_s = time_s;
    waypoint.positions.assign(current.begin(), current.end());
    waypoint.velocities.assign(current.size(), 0.0f);
    waypoint.accelerations.assign(current.size(), 0.0f);
    request.waypoints.push_back(std::move(waypoint));
  }
  runtime.start_trajectory(std::move(request));
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  while (Clock::now() < deadline) {
    const auto status = runtime.motion_status();
    if (status.state == ARTICORE_MOTION_COMPLETED) return;
    if (status.state == ARTICORE_MOTION_FAULT ||
        status.state == ARTICORE_MOTION_CANCELLED) {
      throw std::runtime_error(std::string("trace seed trajectory failed: ") +
                               status.error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("trace seed trajectory timed out");
}

void collect_hold(std::ofstream& output,
                  articore::SafetyRuntime& runtime,
                  const articore::YunyiRuntimeResources& resources,
                  const Profile& profile, int tested_joint, int point_index,
                  const Product& target) {
  const auto deadline = Clock::now() + kHoldDuration;
  uint64_t last_update_count = 0;
  std::size_t sample_index = 0;
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_snapshot(resources);
    if (state.update_count == last_update_count) {
      std::this_thread::sleep_for(std::chrono::microseconds(250));
      continue;
    }
    last_update_count = state.update_count;
    float max_velocity = 0.0f;
    float max_error = 0.0f;
    for (std::size_t index = 0; index < state.q.size(); ++index) {
      max_velocity = std::max(max_velocity, std::abs(state.dq[index]));
      max_error = std::max(max_error, std::abs(state.q[index] - target[index]));
    }
    if (max_velocity > kAbortHoldVelocity || max_error > kAbortHoldError) {
      throw std::runtime_error("hold safety threshold exceeded");
    }
    output << tested_joint << ',' << profile.family << ',' << profile.label
           << ',' << profile.gains.kp_asr << ',' << profile.gains.ki_asr
           << ',' << profile.gains.kp_apr << ',' << profile.gains.ki_apr
           << ',' << point_index << ',' << sample_index++ << ',' << now_ns();
    for (float value : state.q) output << ',' << value;
    for (float value : state.dq) output << ',' << value;
    output << '\n';
  }
  output.flush();
}

}  // namespace

int main(int argc, char** argv) {
  const bool j1 = argc == 2 &&
      std::strcmp(argv[1], "--i-understand-test-left-j1-gains") == 0;
  const bool j2 = argc == 2 &&
      std::strcmp(argv[1], "--i-understand-test-left-j2-gains") == 0;
  const bool validate = argc == 2 &&
      std::strcmp(argv[1], "--i-understand-validate-left-j12-gains") == 0;
  if (!j1 && !j2 && !validate) {
    std::cerr << "Refusing hardware movement/register writes. Pass "
                 "--i-understand-test-left-j1-gains or "
                 "--i-understand-test-left-j2-gains or "
                 "--i-understand-validate-left-j12-gains\n";
    return 2;
  }

  const int tested_joint = j1 ? 1 : (j2 ? 2 : 12);
  const std::string path = validate
      ? "/tmp/articore_left_j12_pv_gain_validation.csv"
      : std::string("/tmp/articore_left_j") +
            std::to_string(tested_joint) + "_pv_gain_sweep.csv";
  std::ofstream output(path);
  output << "tested_joint,family,profile,kp_asr,ki_asr,kp_apr,ki_apr,"
            "point_index,sample_index,timestamp_ns";
  for (const char* side : {"left", "right"}) {
    for (int joint = 1; joint <= 7; ++joint) output << ",q_" << side << "_j" << joint;
  }
  for (const char* side : {"left", "right"}) {
    for (int joint = 1; joint <= 7; ++joint) output << ",dq_" << side << "_j" << joint;
  }
  output << '\n' << std::setprecision(9);

  auto bundle = articore::create_yunyi_runtime(ARTICORE_MODE_PV, false);
  auto& runtime = *bundle.runtime;
  auto& resources = *bundle.resources;
  auto* tested_motor = validate ? nullptr : resources.arm_motors[tested_joint - 1];
  auto* motor_j1 = resources.arm_motors[0];
  auto* motor_j2 = resources.arm_motors[1];
  bool connected = false;
  bool enabled = false;
  bool have_initial = false;
  Product initial{};
  PvGains original{};
  bool have_original = false;
  PvGains original_j1{};
  PvGains original_j2{};
  bool have_validation_originals = false;
  try {
    runtime.connect();
    connected = true;
    const auto configure_result =
        runtime.configure_mode_for_connect(ARTICORE_MODE_PV);
    if (configure_result != ARTICORE_OPERATION_OK) {
      throw std::runtime_error(
          "failed to configure native POS_VEL mode after connect");
    }
    require_all_disabled(resources);
    initial = read_snapshot(resources, false).q;
    have_initial = true;
    if (validate) {
      original_j1 = read_gains(motor_j1);
      original_j2 = read_gains(motor_j2);
      have_validation_originals = true;
      print_validation_modes("after_connect", motor_j1, motor_j2);
      if (!gains_match(original_j1, kExpectedBaseline) ||
          !gains_match(original_j2, kExpectedBaseline)) {
        throw std::runtime_error(
            "left J1/J2 gains do not match pre-registered baseline");
      }
      for (const auto& profile : kValidationProfiles) {
        require_all_disabled(resources);
        const auto j1_gains = validation_j1_gains(profile);
        write_gains(motor_j1, j1_gains);
        try {
          write_gains(motor_j2, profile.gains);
        } catch (...) {
          write_gains(motor_j1, original_j1);
          throw;
        }
        print_validation_modes("after_gain_write", motor_j1, motor_j2);
        std::cout << "PROFILE joints=J1/J2 label=" << profile.label
                  << " J1_KP_ASR=" << j1_gains.kp_asr
                  << " J1_KI_ASR=" << j1_gains.ki_asr
                  << " J1_KP_APR=" << j1_gains.kp_apr
                  << " J2_KP_ASR=" << profile.gains.kp_asr
                  << " J2_KI_ASR=" << profile.gains.ki_asr
                  << " J2_KP_APR=" << profile.gains.kp_apr
                  << std::endl;
        runtime.enable(ARTICORE_MODE_PV);
        enabled = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        require_healthy(runtime);
        require_validation_pv_mode("after_enable", motor_j1, motor_j2);
        seed_trace_joint_layout(runtime, resources, read_snapshot(resources).q);
        require_validation_pv_mode("after_trace_seed", motor_j1, motor_j2);
        for (const auto& point : kValidationPoints) {
          Product target = initial;
          std::copy(point.second.begin(), point.second.end(), target.begin());
          move(runtime, resources, target);
          std::cout << "HOLD joints=J1/J2 profile=" << profile.label
                    << " point=" << point.first << std::endl;
          collect_hold(output, runtime, resources, profile, tested_joint,
                       point.first, target);
          move(runtime, resources, initial);
        }
        runtime.disable();
        enabled = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        require_all_disabled(resources);
      }
      write_gains(motor_j1, original_j1);
      write_gains(motor_j2, original_j2);
      std::cout << "RESTORED joints=J1/J2 gains=baseline" << std::endl;
    } else {
      original = read_gains(tested_motor);
      have_original = true;
      if (!gains_match(original, kExpectedBaseline)) {
        throw std::runtime_error("left J" + std::to_string(tested_joint) +
                                 " gains do not match pre-registered baseline");
      }

      const std::array<std::pair<int, Arm>, 2> points = j1
          ? std::array<std::pair<int, Arm>, 2>{
                std::pair<int, Arm>{17, kPoint17}, {86, kPoint86}}
          : std::array<std::pair<int, Arm>, 2>{
                std::pair<int, Arm>{19, kPoint19}, {118, kPoint118}};

      for (const auto& profile : kProfiles) {
        require_all_disabled(resources);
        write_gains(tested_motor, profile.gains);
        std::cout << "PROFILE joint=J" << tested_joint
                  << " family=" << profile.family
                  << " label=" << profile.label << std::endl;

        runtime.enable(ARTICORE_MODE_PV);
        enabled = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        require_healthy(runtime);
        read_snapshot(resources);

        for (const auto& point : points) {
          Product target = initial;
          std::copy(point.second.begin(), point.second.end(), target.begin());
          move(runtime, resources, target);
          std::cout << "HOLD joint=J" << tested_joint
                    << " profile=" << profile.label
                    << " point=" << point.first << std::endl;
          collect_hold(output, runtime, resources, profile, tested_joint,
                       point.first, target);
          move(runtime, resources, initial);
        }

        runtime.disable();
        enabled = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        require_all_disabled(resources);
      }

      write_gains(tested_motor, original);
      std::cout << "RESTORED joint=J" << tested_joint << " gains=baseline"
                << std::endl;
    }
    runtime.disconnect();
    connected = false;
    std::cout << "RESULT trace=" << path << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (enabled) {
      if (have_initial) {
        try {
          move(runtime, resources, initial);
        } catch (const std::exception& restore_error) {
          std::cerr << "POSE_RESTORE_ERROR " << restore_error.what() << '\n';
        }
      }
      try {
        runtime.disable();
        enabled = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      } catch (...) {
      }
    }
    if (have_validation_originals) {
      try {
        require_all_disabled(resources);
        write_gains(motor_j1, original_j1);
        write_gains(motor_j2, original_j2);
        std::cerr << "GAINS_RESTORED joints=J1/J2\n";
      } catch (const std::exception& restore_error) {
        std::cerr << "GAIN_RESTORE_ERROR " << restore_error.what() << '\n';
      }
    } else if (have_original) {
      try {
        require_all_disabled(resources);
        write_gains(tested_motor, original);
        std::cerr << "GAINS_RESTORED joint=J" << tested_joint << '\n';
      } catch (const std::exception& restore_error) {
        std::cerr << "GAIN_RESTORE_ERROR " << restore_error.what() << '\n';
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
