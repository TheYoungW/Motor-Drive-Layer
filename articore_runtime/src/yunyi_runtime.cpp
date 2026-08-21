#include "articore/detail/yunyi_runtime.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "damiao/socketcan_fd_bus.hpp"

namespace articore {
namespace {

constexpr const char* kProductId = "yunyi_v1_0";
constexpr const char* kGripperProfile = "yunyi_gripper_v1";
constexpr const char* kChannels[2] = {"can-left", "can-right"};
constexpr const char* kModels[8] = {
    "8009", "8009", "4340P", "4340P", "4310", "4310", "4310", "4310"};
constexpr float kKp[7] = {190, 190, 70, 125, 10, 22, 28};
constexpr float kKd[7] = {4.55f, 4.5f, 2, 2.9f, .7f, .89f, .84f};
constexpr float kDirection[2][7] = {
    {1, 1, 1, -1, -1, 1, 1},
    {1, 1, 1, 1, -1, -1, 1},
};
constexpr float kLower[2][7] = {
    {-2.745f, -.3489f, -2.5294f, -.1744f, -2.0933f, -.785f, -1.3956f},
    {-2.745f, -2.2678f, -2.5294f, -.1744f, -2.0933f, -.785f, -1.3956f},
};
constexpr float kUpper[2][7] = {
    {2.745f, 2.2678f, 2.5294f, 2.2678f, 2.0933f, .785f, 1.3956f},
    {2.745f, .3489f, 2.5294f, 2.2678f, 2.0933f, .785f, 1.3956f},
};
constexpr float kVelocityLimit[7] = {16, 16, 5, 5, 20, 20, 20};
// Conservative logical-coordinate trajectory limits. These are native product
// policy, not user-tunable scheduler parameters.
constexpr float kAccelerationLimit[7] = {10, 10, 8, 8, 20, 20, 20};
constexpr float kLogicalVelocityRange[2][7] = {
    {20, 20, 10, 10, 10, 10, 10},
    {10, 10, 10, 10, 10, 10, 10},
};
constexpr float kNativeVelocityRange[7] = {45, 45, 10, 10, 30, 30, 30};
constexpr float kLogicalTorqueRange[7] = {40, 40, 27, 27, 7, 7, 7};
constexpr float kNativeTorqueRange[7] = {54, 54, 28, 28, 10, 10, 10};

thread_local std::string g_motor_error = "ok";

template <typename Fn>
int32_t native_call(Fn&& fn) noexcept {
  try {
    std::forward<Fn>(fn)();
    g_motor_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    g_motor_error = error.what();
    return -1;
  } catch (...) {
    g_motor_error = "unknown native Motor exception";
    return -1;
  }
}

const char* last_motor_error() { return g_motor_error.c_str(); }

std::shared_ptr<damiao::MotorHandle> owned_motor(
    YunyiRuntimeResources& resources, void* handle) {
  const auto iterator = resources.motor_owners.find(
      static_cast<damiao::MotorHandle*>(handle));
  if (iterator == resources.motor_owners.end()) {
    throw std::invalid_argument("command contains a foreign Motor handle");
  }
  return iterator->second;
}

int32_t group_send_pv(void* group, const ArticorePosVelCommand* commands,
                      uint32_t count) {
  if (!group || (!commands && count > 0)) return -1;
  return native_call([&] {
    auto& resources = *static_cast<YunyiRuntimeResources*>(group);
    std::vector<damiao::PosVelBatchCommand> native;
    native.reserve(count);
    for (uint32_t index = 0; index < count; ++index) {
      native.push_back({owned_motor(resources, commands[index].motor),
                        commands[index].target_position,
                        commands[index].velocity_limit});
    }
    resources.group->send_pos_vel(native);
  });
}

int32_t group_send_mit(void* group, const ArticoreMitCommand* commands,
                       uint32_t count) {
  if (!group || (!commands && count > 0)) return -1;
  return native_call([&] {
    auto& resources = *static_cast<YunyiRuntimeResources*>(group);
    std::vector<damiao::MitBatchCommand> native;
    native.reserve(count);
    for (uint32_t index = 0; index < count; ++index) {
      native.push_back({owned_motor(resources, commands[index].motor),
                        commands[index].target_position,
                        commands[index].target_velocity,
                        commands[index].stiffness,
                        commands[index].damping,
                        commands[index].feedforward_torque});
    }
    resources.group->send_mit(native);
  });
}

int32_t request_feedback(void* controller, uint32_t timeout_ms,
                         ArticoreFeedbackReport* report, uint32_t* missing,
                         uint32_t capacity) {
  if (!controller || !report || (!missing && capacity > 0) || timeout_ms == 0) {
    g_motor_error = "invalid native feedback request";
    return 1;
  }
  try {
    const auto native = static_cast<damiao::Controller*>(controller)
        ->request_feedback_all_report(std::chrono::milliseconds(timeout_ms));
    std::memset(report, 0, sizeof(*report));
    report->struct_size = sizeof(*report);
    report->timeout_ms = timeout_ms;
    report->expected_count = native.expected_count;
    report->received_count = native.received_count;
    report->missing_count = static_cast<uint32_t>(native.missing_motor_ids.size());
    const auto copy_count = std::min<std::size_t>(capacity,
                                                  native.missing_motor_ids.size());
    for (std::size_t index = 0; index < copy_count; ++index) {
      missing[index] = native.missing_motor_ids[index];
    }
    if (native.status == damiao::FeedbackBatchStatus::Ok) {
      g_motor_error = "ok";
      return 0;
    }
    g_motor_error = native.error.empty() ? "fresh feedback is incomplete"
                                         : native.error;
    if (native.status == damiao::FeedbackBatchStatus::TransportError) return 2;
    if (native.status == damiao::FeedbackBatchStatus::Timeout) return 3;
    return 4;
  } catch (const std::exception& error) {
    g_motor_error = error.what();
    return 2;
  }
}

int32_t get_state(void* motor, ArticoreMotorState* state) {
  if (!motor || !state) return -1;
  return native_call([&] {
    std::memset(state, 0, sizeof(*state));
    const auto value = static_cast<damiao::MotorHandle*>(motor)->latest_state();
    if (!value) return;
    state->has_value = 1;
    state->can_id = value->can_id;
    state->arbitration_id = value->arbitration_id;
    state->status_code = value->status_code;
    state->pos = value->pos;
    state->vel = value->vel;
    state->torq = value->torq;
    state->t_mos = value->t_mos;
    state->t_rotor = value->t_rotor;
  });
}

int32_t get_feedback_stats(void* motor, ArticoreFeedbackStats* stats) {
  if (!motor || !stats) return -1;
  return native_call([&] {
    std::memset(stats, 0, sizeof(*stats));
    const auto value = static_cast<damiao::MotorHandle*>(motor)->feedback_stats();
    stats->has_feedback = value.has_feedback ? 1 : 0;
    stats->update_count = value.update_count;
    stats->age_ns = value.age.count() < 0
        ? 0 : static_cast<uint64_t>(value.age.count());
  });
}

int32_t get_transport_health(void* controller,
                             ArticoreDriverTransportHealth* health) {
  if (!controller || !health) return -1;
  return native_call([&] {
    const auto value = static_cast<damiao::Controller*>(controller)
        ->transport_health();
    std::memset(health, 0, sizeof(*health));
    health->connected = value.connected ? 1 : 0;
    health->healthy = value.healthy ? 1 : 0;
    health->tx_frames = value.tx_frames;
    health->rx_frames = value.rx_frames;
    health->send_errors = value.send_errors;
    health->receive_errors = value.receive_errors;
    health->last_tx_age_ns = value.last_tx_age
        ? static_cast<uint64_t>(std::max<int64_t>(0, value.last_tx_age->count()))
        : std::numeric_limits<uint64_t>::max();
    health->last_rx_age_ns = value.last_rx_age
        ? static_cast<uint64_t>(std::max<int64_t>(0, value.last_rx_age->count()))
        : std::numeric_limits<uint64_t>::max();
    std::strncpy(health->last_error, value.last_error.c_str(),
                 sizeof(health->last_error) - 1);
  });
}

int32_t controller_enable_all(void* controller) {
  if (!controller) return -1;
  return native_call([&] {
    static_cast<damiao::Controller*>(controller)->enable_all();
  });
}

int32_t controller_disable_all(void* controller) {
  if (!controller) return -1;
  return native_call([&] {
    static_cast<damiao::Controller*>(controller)->disable_all();
  });
}

int32_t motor_enable(void* motor) {
  if (!motor) return -1;
  return native_call([&] { static_cast<damiao::MotorHandle*>(motor)->enable(); });
}

int32_t motor_disable(void* motor) {
  if (!motor) return -1;
  return native_call([&] { static_cast<damiao::MotorHandle*>(motor)->disable(); });
}

int32_t motor_clear_error(void* motor) {
  if (!motor) return -1;
  return native_call([&] {
    static_cast<damiao::MotorHandle*>(motor)->clear_error();
  });
}

int32_t motor_set_zero(void* motor) {
  if (!motor) return -1;
  return native_call([&] {
    static_cast<damiao::MotorHandle*>(motor)->set_zero_position();
  });
}

int32_t motor_ensure_mode(void* motor, uint32_t mode, uint32_t timeout_ms) {
  if (!motor) return -1;
  return native_call([&] {
    static_cast<damiao::MotorHandle*>(motor)->ensure_mode(
        mode, std::chrono::milliseconds(timeout_ms));
  });
}

int32_t motor_set_timeout(void* motor, uint32_t timeout_ms) {
  if (!motor) return -1;
  return native_call([&] {
    static_cast<damiao::MotorHandle*>(motor)->set_can_timeout_ms(timeout_ms);
  });
}

class YunyiMotorBackend final : public MotorBackend {
 public:
  int32_t send_pos_vel(void* group, const ArticorePosVelCommand* commands,
                       uint32_t count) override {
    return group_send_pv(group, commands, count);
  }
  int32_t send_mit(void* group, const ArticoreMitCommand* commands,
                   uint32_t count) override {
    return group_send_mit(group, commands, count);
  }
  int32_t disable_all(void* controller) override {
    return controller_disable_all(controller);
  }
  int32_t request_feedback(
      void* controller, uint32_t timeout_ms, ArticoreFeedbackReport* report,
      uint32_t* missing, uint32_t capacity) override {
    return ::articore::request_feedback(
        controller, timeout_ms, report, missing, capacity);
  }
  int32_t get_state(void* motor, ArticoreMotorState* state) override {
    return ::articore::get_state(motor, state);
  }
  int32_t get_feedback_stats(
      void* motor, ArticoreFeedbackStats* stats) override {
    return ::articore::get_feedback_stats(motor, stats);
  }
  int32_t get_transport_health(
      void* controller, ArticoreDriverTransportHealth* health) override {
    return ::articore::get_transport_health(controller, health);
  }
  int32_t disable_motor(void* motor) override {
    return ::articore::motor_disable(motor);
  }
  const char* last_error_message() const override { return last_motor_error(); }
  bool can_enable_all() const override { return true; }
  int32_t enable_all(void* controller) override {
    return controller_enable_all(controller);
  }
  bool can_enable_motor() const override { return true; }
  int32_t enable_motor(void* motor) override {
    return ::articore::motor_enable(motor);
  }
  bool can_clear_error() const override { return true; }
  int32_t clear_error(void* motor) override {
    return motor_clear_error(motor);
  }
  bool can_set_zero() const override { return true; }
  int32_t set_zero(void* motor) override { return motor_set_zero(motor); }
  bool can_ensure_mode() const override { return true; }
  int32_t ensure_mode(void* motor, uint32_t mode,
                      uint32_t timeout_ms) override {
    return motor_ensure_mode(motor, mode, timeout_ms);
  }
  bool can_set_timeout() const override { return true; }
  int32_t set_timeout_ms(void* motor, uint32_t timeout_ms) override {
    return motor_set_timeout(motor, timeout_ms);
  }
  uint32_t communication_timeout_ms() const override { return 500; }
};

void add_motors(YunyiRuntimeResources& resources,
                std::vector<ArticoreMotorDescriptor>& descriptors,
                std::vector<ArticoreMotorIdentity>& identities) {
  const uint16_t motors_per_side = resources.with_grippers ? 8 : 7;
  for (uint8_t side = 0; side < 2; ++side) {
    for (uint16_t index = 0; index < motors_per_side; ++index) {
      const uint16_t id = index + 1;
      auto motor = resources.controllers[side]->add_damiao_motor(
          id, 0x10 + id, kModels[index]);
      auto* motor_handle = motor.get();
      resources.motor_owners.emplace(motor_handle, motor);
      if (index < ARTICORE_PRODUCT_ARM_DOF) {
        resources.arm_motors[side * ARTICORE_PRODUCT_ARM_DOF + index] =
            motor_handle;
      } else {
        resources.grippers[side] = motor_handle;
      }

      ArticoreMotorDescriptor descriptor{};
      descriptor.motor = motor_handle;
      descriptor.side = side;
      descriptor.is_gripper = index == ARTICORE_PRODUCT_ARM_DOF;
      const std::string name = std::string(side == 0 ? "left/l-" : "right/r-") +
          (descriptor.is_gripper ? "gripper"
                                 : "joint" + std::to_string(index + 1));
      std::strncpy(descriptor.name, name.c_str(), sizeof(descriptor.name) - 1);
      if (!descriptor.is_gripper) {
        descriptor.safe_kp = 5.0f;
        descriptor.safe_kd = 1.0f;
      }
      descriptors.push_back(descriptor);

      ArticoreMotorIdentity identity{};
      identity.struct_size = sizeof(identity);
      identity.motor = motor_handle;
      identity.can_id = id;
      identities.push_back(identity);
    }
  }
}

std::vector<ArticoreJointControlConfig> configure_joint_table(
    YunyiRuntimeResources& resources) {
  std::vector<ArticoreJointControlConfig> joints;
  joints.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
  for (uint32_t side = 0; side < 2; ++side) {
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      const auto product_index = side * ARTICORE_PRODUCT_ARM_DOF + index;
      auto& product_joint = resources.joints[product_index];
      product_joint.motor = resources.arm_motors[product_index];
      product_joint.direction = kDirection[side][index];
      product_joint.lower = kLower[side][index];
      product_joint.upper = kUpper[side][index];
      product_joint.velocity_limit = kVelocityLimit[index];
      product_joint.acceleration_limit = kAccelerationLimit[index];
      product_joint.torque_limit = kLogicalTorqueRange[index];
      product_joint.velocity_command_scale =
          kNativeVelocityRange[index] / kLogicalVelocityRange[side][index];
      product_joint.velocity_feedback_scale =
          kLogicalVelocityRange[side][index] / kNativeVelocityRange[index];
      product_joint.torque_command_scale =
          kNativeTorqueRange[index] / kLogicalTorqueRange[index];
      product_joint.torque_feedback_scale =
          kLogicalTorqueRange[index] / kNativeTorqueRange[index];
      product_joint.kp = kKp[index];
      product_joint.kd = kKd[index];

      ArticoreJointControlConfig joint{};
      joint.motor = product_joint.motor;
      joint.lower_position = product_joint.direction > 0
          ? product_joint.lower : -product_joint.upper;
      joint.upper_position = product_joint.direction > 0
          ? product_joint.upper : -product_joint.lower;
      joint.velocity_limit = kNativeVelocityRange[index];
      joint.torque_limit = kNativeTorqueRange[index];
      joint.mit_kp = kKp[index];
      joint.mit_kd = kKd[index];
      joints.push_back(joint);
    }
  }
  return joints;
}

}  // namespace

bool read_yunyi_motor_state(
    damiao::MotorHandle* motor, ArticoreMotorState& state,
    ArticoreFeedbackStats& stats) noexcept {
  if (!motor) return false;
  try {
    std::memset(&state, 0, sizeof(state));
    std::memset(&stats, 0, sizeof(stats));
    const auto snapshot = motor->state_snapshot();
    if (snapshot.state) {
      const auto& value = *snapshot.state;
      state.has_value = 1;
      state.can_id = value.can_id;
      state.arbitration_id = value.arbitration_id;
      state.status_code = value.status_code;
      state.pos = value.pos;
      state.vel = value.vel;
      state.torq = value.torq;
      state.t_mos = value.t_mos;
      state.t_rotor = value.t_rotor;
    }
    stats.has_feedback = snapshot.feedback.has_feedback ? 1 : 0;
    stats.update_count = snapshot.feedback.update_count;
    stats.age_ns = snapshot.feedback.age.count() < 0
        ? 0 : static_cast<uint64_t>(snapshot.feedback.age.count());
    return true;
  } catch (...) {
    return false;
  }
}

YunyiRuntimeBundle create_yunyi_runtime(
    ArticoreControlMode mode, bool with_grippers) {
  if (mode != ARTICORE_MODE_PV && mode != ARTICORE_MODE_MIT) {
    throw std::invalid_argument("unsupported Yunyi control mode");
  }

  auto resources = std::make_unique<YunyiRuntimeResources>();
  resources->with_grippers = with_grippers;
  for (uint32_t side = 0; side < 2; ++side) {
    resources->pose_models[side] =
        std::make_unique<RobotModel>(kProductId, side, with_grippers);
    auto bus = damiao::SocketCanFdBus::open(kChannels[side], true);
    resources->controllers[side] = std::make_unique<damiao::Controller>(
        std::move(bus), std::string("socketcanfd ") + kChannels[side]);
  }

  const uint32_t motor_count = with_grippers ? 16 : 14;
  std::vector<ArticoreMotorDescriptor> descriptors;
  std::vector<ArticoreMotorIdentity> identities;
  descriptors.reserve(motor_count);
  identities.reserve(motor_count);
  add_motors(*resources, descriptors, identities);

  resources->group = std::make_unique<damiao::ControllerGroup>(
      std::vector<damiao::Controller*>{resources->controllers[0].get(),
                                       resources->controllers[1].get()});

  ArticoreRuntimeTransportCapabilities transports[2]{};
  for (uint32_t side = 0; side < 2; ++side) {
    transports[side].struct_size = sizeof(transports[side]);
    transports[side].side = side;
    transports[side].can_fd = 1;
    transports[side].can_fd_brs = 1;
    std::strncpy(transports[side].transport, "socketcanfd",
                 sizeof(transports[side].transport) - 1);
  }

  const ArticoreRuntimeConfig config{
      0, 250, 2000, 100, 100, 3, 300, 1, 50, 0.2f, 0,
      ARTICORE_GRIPPER_FAULT_HOLD};
  auto runtime = std::make_unique<SafetyRuntime>(
      config, std::make_shared<YunyiMotorBackend>(), resources.get(),
      resources->controllers[0].get(), resources->controllers[1].get(),
      descriptors,
      with_grippers,
      std::vector<ArticoreRuntimeTransportCapabilities>(
          std::begin(transports), std::end(transports)));
  runtime->configure_motor_identities(identities.data(), identities.size());

  const auto joints = configure_joint_table(*resources);
  runtime->configure_joints(joints.data(), joints.size());

  if (with_grippers) {
    ArticoreGripperProductBinding grippers[2]{};
    for (uint32_t side = 0; side < 2; ++side) {
      grippers[side].struct_size = sizeof(grippers[side]);
      grippers[side].motor = resources->grippers[side];
      std::strncpy(grippers[side].profile_id, kGripperProfile,
                   sizeof(grippers[side].profile_id) - 1);
    }
    runtime->configure_gripper_products(grippers, 2);
  } else {
    runtime->configure_gripper_products(nullptr, 0);
  }

  ArticoreGravityProductBinding gravity[2]{};
  for (uint32_t side = 0; side < 2; ++side) {
    gravity[side].struct_size = sizeof(gravity[side]);
    gravity[side].runtime_side = side;
    gravity[side].robot_side = side;
    std::strncpy(gravity[side].product_id, kProductId,
                 sizeof(gravity[side].product_id) - 1);
  }
  runtime->configure_gravity_products(gravity, 2);

  return YunyiRuntimeBundle{
      std::move(runtime), std::move(resources), mode};
}

}  // namespace articore
