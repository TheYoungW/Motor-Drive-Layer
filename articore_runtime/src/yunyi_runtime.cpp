#include "yunyi_runtime.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

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

int32_t group_send_pv(void* group, const ArticorePosVelCommand* commands,
                      uint32_t count) {
  return motor_controller_group_send_pos_vel(
      static_cast<MotorControllerGroup*>(group),
      reinterpret_cast<const MotorPosVelBatchCommand*>(commands), count);
}

int32_t group_send_mit(void* group, const ArticoreMitCommand* commands,
                       uint32_t count) {
  return motor_controller_group_send_mit(
      static_cast<MotorControllerGroup*>(group),
      reinterpret_cast<const MotorMitBatchCommand*>(commands), count);
}

int32_t request_feedback(void* controller, uint32_t timeout_ms,
                         ArticoreFeedbackReport* report, uint32_t* missing,
                         uint32_t capacity) {
  return motor_controller_request_feedback_all_ex(
      static_cast<MotorController*>(controller), timeout_ms,
      reinterpret_cast<MotorFeedbackReport*>(report), missing, capacity);
}

int32_t get_state(void* motor, ArticoreMotorState* state) {
  return motor_handle_get_state(static_cast<MotorHandle*>(motor),
                                reinterpret_cast<MotorState*>(state));
}

int32_t get_feedback_stats(void* motor, ArticoreFeedbackStats* stats) {
  return motor_handle_get_feedback_stats(
      static_cast<MotorHandle*>(motor),
      reinterpret_cast<MotorFeedbackStats*>(stats));
}

int32_t get_transport_health(void* controller,
                             ArticoreDriverTransportHealth* health) {
  return motor_controller_get_transport_health(
      static_cast<MotorController*>(controller),
      reinterpret_cast<MotorTransportHealth*>(health));
}

ArticoreMotorApi motor_api() {
  return ArticoreMotorApi{
      group_send_pv,
      group_send_mit,
      reinterpret_cast<ArticoreControllerCallFn>(motor_controller_disable_all),
      request_feedback,
      get_state,
      get_feedback_stats,
      motor_last_error_message,
      get_transport_health,
      reinterpret_cast<ArticoreControllerCallFn>(motor_handle_disable),
  };
}

ArticoreMotorMaintenanceApi maintenance_api() {
  ArticoreMotorMaintenanceApi api{};
  api.struct_size = sizeof(api);
  api.motor_clear_error =
      reinterpret_cast<ArticoreControllerCallFn>(motor_handle_clear_error);
  api.motor_set_zero_position =
      reinterpret_cast<ArticoreControllerCallFn>(motor_handle_set_zero_position);
  api.motor_ensure_mode =
      reinterpret_cast<ArticoreMotorEnsureModeFn>(motor_handle_ensure_mode);
  api.motor_set_can_timeout_ms =
      reinterpret_cast<ArticoreMotorSetCanTimeoutFn>(
          motor_handle_set_can_timeout_ms);
  api.communication_timeout_ms = 500;
  return api;
}

void add_motors(YunyiRuntimeResources& resources,
                std::vector<ArticoreMotorDescriptor>& descriptors,
                std::vector<ArticoreMotorIdentity>& identities) {
  const uint16_t motors_per_side = resources.with_grippers ? 8 : 7;
  for (uint8_t side = 0; side < 2; ++side) {
    for (uint16_t index = 0; index < motors_per_side; ++index) {
      const uint16_t id = index + 1;
      auto* motor = motor_controller_add_damiao_motor(
          resources.controllers[side], id, 0x10 + id, kModels[index]);
      if (!motor) throw std::runtime_error(motor_last_error_message());
      resources.motors.push_back(motor);
      if (index < ARTICORE_PRODUCT_ARM_DOF) {
        resources.arm_motors[side * ARTICORE_PRODUCT_ARM_DOF + index] = motor;
      } else {
        resources.grippers[side] = motor;
      }

      ArticoreMotorDescriptor descriptor{};
      descriptor.motor = motor;
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
      identity.motor = motor;
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

YunyiRuntimeResources::~YunyiRuntimeResources() {
  if (group) motor_controller_group_free(group);
  for (auto* motor : motors) motor_handle_free(motor);
  for (auto* controller : controllers) {
    if (controller) motor_controller_free(controller);
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
    resources->controllers[side] =
        motor_controller_new_socketcanfd_ex(kChannels[side], 1);
    if (!resources->controllers[side]) {
      throw std::runtime_error(motor_last_error_message());
    }
  }

  const uint32_t motor_count = with_grippers ? 16 : 14;
  std::vector<ArticoreMotorDescriptor> descriptors;
  std::vector<ArticoreMotorIdentity> identities;
  descriptors.reserve(motor_count);
  identities.reserve(motor_count);
  add_motors(*resources, descriptors, identities);

  MotorController* controllers[2] = {
      resources->controllers[0], resources->controllers[1]};
  resources->group = motor_controller_group_new(controllers, 2);
  if (!resources->group) throw std::runtime_error(motor_last_error_message());

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
      config, motor_api(), resources->group, resources->controllers[0],
      resources->controllers[1], descriptors,
      reinterpret_cast<ArticoreControllerCallFn>(motor_controller_enable_all),
      reinterpret_cast<ArticoreControllerCallFn>(motor_handle_enable),
      with_grippers,
      std::vector<ArticoreRuntimeTransportCapabilities>(
          std::begin(transports), std::end(transports)),
      maintenance_api());
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
