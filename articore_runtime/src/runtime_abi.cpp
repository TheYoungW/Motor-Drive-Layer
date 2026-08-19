#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "articore/runtime_abi.h"
#include "robot_model.hpp"
#include "runtime.hpp"
#include "motor_abi.h"

struct ProductRuntimeResources {
  struct Joint {
    MotorHandle* motor = nullptr;
    float direction = 1.0f;
    float lower = 0.0f;
    float upper = 0.0f;
    float velocity_limit = 0.0f;
    float torque_limit = 0.0f;
    float velocity_command_scale = 1.0f;
    float velocity_feedback_scale = 1.0f;
    float torque_command_scale = 1.0f;
    float torque_feedback_scale = 1.0f;
    float kp = 0.0f;
    float kd = 0.0f;
  };

  MotorController* controllers[2]{};
  MotorControllerGroup* group = nullptr;
  std::vector<MotorHandle*> motors;
  std::array<MotorHandle*, ARTICORE_PRODUCT_DUAL_ARM_DOF> arm_motors{};
  std::array<Joint, ARTICORE_PRODUCT_DUAL_ARM_DOF> joints{};
  std::array<MotorHandle*, 2> grippers{};
  std::array<std::unique_ptr<articore::RobotModel>, 2> pose_models;
  std::array<std::mutex, 2> pose_mutexes;
  std::array<std::atomic<int32_t>, 2> gripper_levels{};
  bool with_grippers = true;
  float default_mit_reference_velocity = 3.4906585f;
  float default_pv_reference_velocity = 5.0f;

  ProductRuntimeResources() {
    for (auto& level : gripper_levels) {
      level.store(3);
    }
  }

  ~ProductRuntimeResources() {
    if (group) motor_controller_group_free(group);
    for (auto* motor : motors) motor_handle_free(motor);
    for (auto* controller : controllers) {
      if (controller) motor_controller_free(controller);
    }
  }
};

struct ArticoreRuntime {
  explicit ArticoreRuntime(
      std::unique_ptr<articore::SafetyRuntime> value,
      std::unique_ptr<ProductRuntimeResources> owned = {},
      ArticoreControlMode product_mode = ARTICORE_MODE_PV)
      : resources(std::move(owned)), runtime(std::move(value)),
        product_mode(product_mode) {}
  std::unique_ptr<ProductRuntimeResources> resources;
  std::unique_ptr<articore::SafetyRuntime> runtime;
  ArticoreControlMode product_mode = ARTICORE_MODE_PV;
};

struct ArticoreRobotModel {
  explicit ArticoreRobotModel(std::unique_ptr<articore::RobotModel> value)
      : model(std::move(value)) {}
  std::unique_ptr<articore::RobotModel> model;
};

namespace {

thread_local std::string g_last_error = "ok";

template <typename Function>
int32_t call(Function&& function) {
  try {
    function();
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return -1;
  } catch (...) {
    g_last_error = "unknown Articore runtime exception";
    return -1;
  }
}

articore::SafetyRuntime& checked(ArticoreRuntime* runtime) {
  if (!runtime || !runtime->runtime) throw std::invalid_argument("runtime is null");
  return *runtime->runtime;
}

ProductRuntimeResources& checked_product(ArticoreRuntime* runtime) {
  checked(runtime);
  if (!runtime->resources) {
    throw std::runtime_error("operation requires a product-owned Runtime");
  }
  return *runtime->resources;
}

void require_product_count(uint32_t count) {
  if (count != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
    throw std::invalid_argument(
        "yunyi_v1_0 command must contain exactly 14 joints");
  }
}

void require_finite(const float* values, uint32_t count, const char* name) {
  if (!values) throw std::invalid_argument(std::string(name) + " is null");
  for (uint32_t i = 0; i < count; ++i) {
    if (!std::isfinite(values[i])) {
      throw std::invalid_argument(std::string(name) + " contains NaN or Inf");
    }
  }
}

void validate_product_position(
    const ProductRuntimeResources::Joint& joint, float position,
    uint32_t index) {
  if (position < joint.lower || position > joint.upper) {
    throw std::invalid_argument(
        "joint " + std::to_string(index) + " exceeds product position limits");
  }
}

int32_t record_product_command_error(
    ArticoreRuntime* runtime, int32_t code, const std::string& error) {
  if (runtime && runtime->runtime) {
    runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_COMMAND, code, error);
  }
  g_last_error = error;
  return code;
}

articore::RobotModel& checked(ArticoreRobotModel* model) {
  if (!model || !model->model) throw std::invalid_argument("robot model is null");
  return *model->model;
}

int32_t native_group_send_pv(void* group,
                             const ArticorePosVelCommand* commands,
                             uint32_t count) {
  return motor_controller_group_send_pos_vel(
      static_cast<MotorControllerGroup*>(group),
      reinterpret_cast<const MotorPosVelBatchCommand*>(commands), count);
}

int32_t native_group_send_mit(void* group,
                              const ArticoreMitCommand* commands,
                              uint32_t count) {
  return motor_controller_group_send_mit(
      static_cast<MotorControllerGroup*>(group),
      reinterpret_cast<const MotorMitBatchCommand*>(commands), count);
}

int32_t native_feedback(void* controller, uint32_t timeout_ms,
                        ArticoreFeedbackReport* report, uint32_t* missing,
                        uint32_t capacity) {
  return motor_controller_request_feedback_all_ex(
      static_cast<MotorController*>(controller), timeout_ms,
      reinterpret_cast<MotorFeedbackReport*>(report), missing, capacity);
}

int32_t native_state(void* motor, ArticoreMotorState* state) {
  return motor_handle_get_state(static_cast<MotorHandle*>(motor),
                                reinterpret_cast<MotorState*>(state));
}

int32_t native_feedback_stats(void* motor, ArticoreFeedbackStats* stats) {
  return motor_handle_get_feedback_stats(
      static_cast<MotorHandle*>(motor),
      reinterpret_cast<MotorFeedbackStats*>(stats));
}

int32_t native_transport_health(void* controller,
                                ArticoreDriverTransportHealth* health) {
  return motor_controller_get_transport_health(
      static_cast<MotorController*>(controller),
      reinterpret_cast<MotorTransportHealth*>(health));
}

ArticoreMotorApi native_motor_api() {
  return ArticoreMotorApi{
      native_group_send_pv,
      native_group_send_mit,
      reinterpret_cast<ArticoreControllerCallFn>(motor_controller_disable_all),
      native_feedback,
      native_state,
      native_feedback_stats,
      motor_last_error_message,
      native_transport_health,
      reinterpret_cast<ArticoreControllerCallFn>(motor_handle_disable),
  };
}

ArticoreMotorMaintenanceApi native_maintenance_api() {
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

}  // namespace

extern "C" {

ARTICORE_RUNTIME_API uint32_t articore_runtime_abi_version(void) {
  return (2U << 16) | 15U;
}

ARTICORE_RUNTIME_API uint64_t articore_runtime_capabilities(void) {
  return ARTICORE_CAP_COMMAND_WATCHDOG |
         ARTICORE_CAP_SAFE_HOLD |
         ARTICORE_CAP_GRIPPER_PROTECTION |
         ARTICORE_CAP_SINGLE_CHANNEL |
         ARTICORE_CAP_DUAL_CHANNEL |
         ARTICORE_CAP_TRANSPORT_HEALTH |
         ARTICORE_CAP_CURRENT_POSITION_HOLD |
         ARTICORE_CAP_MOTOR_PRESENCE |
         ARTICORE_CAP_REALTIME_JOINT_MAILBOX |
         ARTICORE_CAP_ATOMIC_ENABLE |
         ARTICORE_CAP_COMMAND_LIFETIME |
         ARTICORE_CAP_PROTECTIVE_FAULT_HOLD |
         ARTICORE_CAP_DETERMINISTIC_DISABLE |
         ARTICORE_CAP_LAYERED_JOINT_LIMITS |
         ARTICORE_CAP_GRIPPER_COMMAND_PROFILES |
         ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS |
         ARTICORE_CAP_JOINT_MIT_POSITION |
         ARTICORE_CAP_JOINT_PV_POSITION |
         ARTICORE_CAP_EFFECTIVE_CONTROL_RATE |
         ARTICORE_CAP_BUILTIN_GRIPPER_PRODUCT_PROFILES |
         ARTICORE_CAP_CONNECT_FEEDBACK_BARRIER |
         ARTICORE_CAP_STRUCTURED_CONNECT_REPORT |
         ARTICORE_CAP_TRANSPORT_AWARE_CONTROL_RATE |
         ARTICORE_CAP_PER_CYCLE_MIT_TORQUE_LIMIT |
         ARTICORE_CAP_NATIVE_ROBOT_MODEL |
         ARTICORE_CAP_NATIVE_GRAVITY_COMPENSATION |
         ARTICORE_CAP_RUNTIME_MAINTENANCE |
         ARTICORE_CAP_PRODUCT_RUNTIME_FACTORY |
         ARTICORE_CAP_UNIFIED_OPERATION_HEALTH |
         ARTICORE_CAP_PRODUCT_COMMAND_FRAMES |
         ARTICORE_CAP_PRODUCT_STATE |
         ARTICORE_CAP_OPTIONAL_PRODUCT_GRIPPERS |
         ARTICORE_CAP_GRADED_FEEDBACK_SAFETY |
         ARTICORE_CAP_NORMALIZED_ORDINARY_SPEED |
         ARTICORE_CAP_RUNTIME_MOTOR_POWER |
         ARTICORE_CAP_PRODUCT_POSE;
}

ARTICORE_RUNTIME_API ArticoreRobotModel* articore_robot_model_create(
    const char* product_id, uint32_t side) {
  if (!product_id) {
    g_last_error = "robot product_id is null";
    return nullptr;
  }
  try {
    auto value = std::make_unique<articore::RobotModel>(product_id, side);
    g_last_error = "ok";
    return new ArticoreRobotModel(std::move(value));
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return nullptr;
  } catch (...) {
    g_last_error = "unknown robot model creation error";
    return nullptr;
  }
}

ARTICORE_RUNTIME_API void articore_robot_model_free(ArticoreRobotModel* model) {
  delete model;
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_get_info(
    ArticoreRobotModel* model, ArticoreRobotModelInfo* info) {
  return call([&] { checked(model).get_info(info); });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_fk(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    ArticoreRobotPose* pose) {
  return call([&] { checked(model).fk(q, q_count, pose); });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_jacobian(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    uint32_t reference, double* output, uint32_t output_count) {
  return call([&] {
    checked(model).jacobian(q, q_count, reference, output, output_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_gravity(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    double* output, uint32_t output_count) {
  return call([&] { checked(model).gravity(q, q_count, output, output_count); });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_mass_matrix(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    double* output, uint32_t output_count) {
  return call([&] { checked(model).mass_matrix(q, q_count, output, output_count); });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_coriolis_matrix(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, double* output,
    uint32_t output_count) {
  return call([&] {
    checked(model).coriolis_matrix(q, q_count, dq, dq_count, output, output_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_nonlinear_effects(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, double* output,
    uint32_t output_count) {
  return call([&] {
    checked(model).nonlinear_effects(q, q_count, dq, dq_count, output, output_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_rnea(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, const double* ddq,
    uint32_t ddq_count, double* output, uint32_t output_count) {
  return call([&] {
    checked(model).rnea(q, q_count, dq, dq_count, ddq, ddq_count, output,
                        output_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_aba(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, const double* torque,
    uint32_t torque_count, double* output, uint32_t output_count) {
  return call([&] {
    checked(model).aba(q, q_count, dq, dq_count, torque, torque_count, output,
                       output_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_robot_model_ik(
    ArticoreRobotModel* model, const ArticoreRobotPose* target,
    const double* initial_q, uint32_t initial_q_count,
    const ArticoreIkOptions* options, ArticoreIkResult* result) {
  return call([&] {
    checked(model).ik(target, initial_q, initial_q_count, options, result);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_control_hz(
    ArticoreRuntime* runtime, uint32_t* control_hz) {
  return call([&] {
    if (!control_hz) throw std::invalid_argument("control_hz output is null");
    *control_hz = checked(runtime).control_hz();
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_control_mode(
    ArticoreRuntime* runtime, int32_t* mode) {
  return call([&] {
    if (!mode) throw std::invalid_argument("control mode output is null");
    checked(runtime);
    *mode = runtime->resources
        ? static_cast<int32_t>(runtime->product_mode)
        : static_cast<int32_t>(runtime->runtime->control_mode());
  });
}

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create(
    const ArticoreRuntimeConfig* config,
    const ArticoreMotorApi* motor_api,
    void* controller_group,
    void* left_controller,
    void* right_controller,
    const ArticoreMotorDescriptor* motors,
    uint32_t motor_count) {
  return articore_runtime_create_ex(
      config, motor_api, controller_group, left_controller, right_controller,
      motors, motor_count, nullptr, nullptr);
}

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create_ex(
    const ArticoreRuntimeConfig* config,
    const ArticoreMotorApi* motor_api,
    void* controller_group,
    void* left_controller,
    void* right_controller,
    const ArticoreMotorDescriptor* motors,
    uint32_t motor_count,
    ArticoreControllerCallFn controller_enable_all,
    ArticoreControllerCallFn motor_enable) {
  return articore_runtime_create_ex2(
      config, motor_api, controller_group, left_controller, right_controller,
      motors, motor_count, controller_enable_all, motor_enable, nullptr, 0);
}

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create_ex2(
    const ArticoreRuntimeConfig* config,
    const ArticoreMotorApi* motor_api,
    void* controller_group,
    void* left_controller,
    void* right_controller,
    const ArticoreMotorDescriptor* motors,
    uint32_t motor_count,
    ArticoreControllerCallFn controller_enable_all,
    ArticoreControllerCallFn motor_enable,
    const ArticoreRuntimeTransportCapabilities* transport_capabilities,
    uint32_t transport_capability_count) {
  return articore_runtime_create_ex3(
      config, motor_api, nullptr, controller_group, left_controller,
      right_controller, motors, motor_count, controller_enable_all,
      motor_enable, transport_capabilities, transport_capability_count);
}

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create_ex3(
    const ArticoreRuntimeConfig* config,
    const ArticoreMotorApi* motor_api,
    const ArticoreMotorMaintenanceApi* maintenance_api,
    void* controller_group,
    void* left_controller,
    void* right_controller,
    const ArticoreMotorDescriptor* motors,
    uint32_t motor_count,
    ArticoreControllerCallFn controller_enable_all,
    ArticoreControllerCallFn motor_enable,
    const ArticoreRuntimeTransportCapabilities* transport_capabilities,
    uint32_t transport_capability_count) {
  if (!config || !motor_api || (!motors && motor_count > 0)) {
    g_last_error = "invalid Articore runtime creation arguments";
    return nullptr;
  }
  if (!transport_capabilities && transport_capability_count > 0) {
    g_last_error = "transport capabilities pointer is null";
    return nullptr;
  }
  try {
    ArticoreMotorMaintenanceApi maintenance{};
    if (maintenance_api) {
      constexpr std::size_t kMaintenanceV29Size =
          offsetof(ArticoreMotorMaintenanceApi, motor_set_can_timeout_ms);
      if (maintenance_api->struct_size < kMaintenanceV29Size) {
        throw std::invalid_argument("maintenance API struct_size is too small");
      }
      std::memcpy(
          &maintenance, maintenance_api,
          std::min<std::size_t>(maintenance_api->struct_size,
                                sizeof(maintenance)));
      maintenance.struct_size = sizeof(maintenance);
    }
    std::vector<ArticoreMotorDescriptor> descriptors(motors, motors + motor_count);
    std::vector<ArticoreRuntimeTransportCapabilities> capabilities;
    if (transport_capability_count > 0) {
      capabilities.assign(
          transport_capabilities,
          transport_capabilities + transport_capability_count);
    }
    auto value = std::make_unique<articore::SafetyRuntime>(
        *config, *motor_api, controller_group, left_controller, right_controller,
        std::move(descriptors), controller_enable_all, motor_enable, true,
        std::move(capabilities), maintenance);
    g_last_error = "ok";
    return new ArticoreRuntime(std::move(value));
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return nullptr;
  } catch (...) {
    g_last_error = "unknown Articore runtime creation error";
    return nullptr;
  }
}

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create_product(
    const char* product_id, int32_t requested_mode, int32_t with_grippers) {
  if (!product_id || std::strcmp(product_id, "yunyi_v1_0") != 0) {
    g_last_error = "unsupported Articore product profile";
    return nullptr;
  }
  if (requested_mode != ARTICORE_MODE_PV &&
      requested_mode != ARTICORE_MODE_MIT) {
    g_last_error = "unsupported Articore product control mode";
    return nullptr;
  }
  if (with_grippers != 0 && with_grippers != 1) {
    g_last_error = "with_grippers must be 0 or 1";
    return nullptr;
  }
  try {
    const ArticoreRuntimeConfig config{
        500, 250, 2000, 100, 100, 3, 300, 1, 50, 0.2f, 500,
        ARTICORE_GRIPPER_FAULT_HOLD};
    auto resources = std::make_unique<ProductRuntimeResources>();
    resources->with_grippers = with_grippers != 0;
    resources->pose_models[ARTICORE_ROBOT_LEFT] =
        std::make_unique<articore::RobotModel>(
            "yunyi_v1_0", ARTICORE_ROBOT_LEFT);
    resources->pose_models[ARTICORE_ROBOT_RIGHT] =
        std::make_unique<articore::RobotModel>(
            "yunyi_v1_0", ARTICORE_ROBOT_RIGHT);
    resources->controllers[0] = motor_controller_new_socketcanfd_ex("can-left", 1);
    if (!resources->controllers[0]) {
      throw std::runtime_error(motor_last_error_message());
    }
    resources->controllers[1] = motor_controller_new_socketcanfd_ex("can-right", 1);
    if (!resources->controllers[1]) {
      throw std::runtime_error(motor_last_error_message());
    }

    static constexpr const char* kModels[8] = {
        "8009", "8009", "4340P", "4340P",
        "4310", "4310", "4310", "4310"};
    std::vector<ArticoreMotorDescriptor> descriptors;
    std::vector<ArticoreMotorIdentity> identities;
    const uint32_t motor_count = resources->with_grippers ? 16 : 14;
    descriptors.reserve(motor_count);
    identities.reserve(motor_count);
    for (uint8_t side = 0; side < 2; ++side) {
      const uint16_t side_motor_count = resources->with_grippers ? 8 : 7;
      for (uint16_t index = 0; index < side_motor_count; ++index) {
        const uint16_t id = index + 1;
        auto* motor = motor_controller_add_damiao_motor(
            resources->controllers[side], id, 0x10 + id, kModels[index]);
        if (!motor) throw std::runtime_error(motor_last_error_message());
        resources->motors.push_back(motor);
        if (index < ARTICORE_PRODUCT_ARM_DOF) {
          resources->arm_motors[side * ARTICORE_PRODUCT_ARM_DOF + index] = motor;
        } else {
          resources->grippers[side] = motor;
        }
        ArticoreMotorDescriptor descriptor{};
        descriptor.motor = motor;
        descriptor.side = side;
        descriptor.is_gripper = index == 7;
        const std::string name = std::string(side == 0 ? "left/l-" : "right/r-") +
            (index == 7 ? "gripper" : "joint" + std::to_string(index + 1));
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
    auto api = native_motor_api();
    auto maintenance = native_maintenance_api();
    auto value = std::make_unique<articore::SafetyRuntime>(
        config, api, resources->group, resources->controllers[0],
        resources->controllers[1], descriptors,
        reinterpret_cast<ArticoreControllerCallFn>(motor_controller_enable_all),
        reinterpret_cast<ArticoreControllerCallFn>(motor_handle_enable),
        resources->with_grippers,
        std::vector<ArticoreRuntimeTransportCapabilities>(
            std::begin(transports), std::end(transports)), maintenance);
    value->configure_motor_identities(identities.data(), identities.size());

    static constexpr float kKp[7] = {190, 190, 70, 125, 10, 22, 28};
    static constexpr float kKd[7] = {4.55f, 4.5f, 2, 2.9f, .7f, .89f, .84f};
    static constexpr float kDirection[2][7] = {
        {1, 1, 1, -1, -1, 1, 1},
        {1, 1, 1, 1, -1, -1, 1},
    };
    static constexpr float kLower[2][7] = {
        {-2.745f, -.3489f, -2.5294f, -.1744f, -2.0933f, -.785f, -1.3956f},
        {-2.745f, -2.2678f, -2.5294f, -.1744f, -2.0933f, -.785f, -1.3956f},
    };
    static constexpr float kUpper[2][7] = {
        {2.745f, 2.2678f, 2.5294f, 2.2678f, 2.0933f, .785f, 1.3956f},
        {2.745f, .3489f, 2.5294f, 2.2678f, 2.0933f, .785f, 1.3956f},
    };
    static constexpr float kVelocityLimit[7] = {16, 16, 5, 5, 20, 20, 20};
    static constexpr float kLogicalVelocityRange[2][7] = {
        {20, 20, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10},
    };
    static constexpr float kNativeVelocityRange[7] = {45, 45, 10, 10, 30, 30, 30};
    static constexpr float kLogicalTorqueRange[7] = {40, 40, 27, 27, 7, 7, 7};
    static constexpr float kNativeTorqueRange[7] = {54, 54, 28, 28, 10, 10, 10};
    std::vector<ArticoreJointControlConfig> joints;
    for (uint32_t side = 0; side < 2; ++side) {
      for (uint32_t index = 0; index < 7; ++index) {
        const auto product_index = side * 7 + index;
        auto& product_joint = resources->joints[product_index];
        product_joint.motor = resources->arm_motors[product_index];
        product_joint.direction = kDirection[side][index];
        product_joint.lower = kLower[side][index];
        product_joint.upper = kUpper[side][index];
        product_joint.velocity_limit = kVelocityLimit[index];
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
    value->configure_joints(joints.data(), joints.size());
    if (resources->with_grippers) {
      ArticoreGripperProductBinding grippers[2]{};
      for (uint32_t side = 0; side < 2; ++side) {
        grippers[side].struct_size = sizeof(grippers[side]);
        grippers[side].motor = resources->grippers[side];
        std::strncpy(grippers[side].profile_id, "yunyi_gripper_v1",
                     sizeof(grippers[side].profile_id) - 1);
      }
      value->configure_gripper_products(grippers, 2);
    } else {
      value->configure_gripper_products(nullptr, 0);
    }
    ArticoreGravityProductBinding gravity[2]{};
    for (uint32_t side = 0; side < 2; ++side) {
      gravity[side].struct_size = sizeof(gravity[side]);
      gravity[side].runtime_side = side;
      gravity[side].robot_side = side;
      std::strncpy(gravity[side].product_id, "yunyi_v1_0",
                   sizeof(gravity[side].product_id) - 1);
    }
    value->configure_gravity_products(gravity, 2);
    g_last_error = "ok";
    return new ArticoreRuntime(
        std::move(value), std::move(resources),
        static_cast<ArticoreControlMode>(requested_mode));
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return nullptr;
  }
}

ARTICORE_RUNTIME_API void articore_runtime_free(ArticoreRuntime* runtime) {
  delete runtime;
}

ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime) {
  try {
    checked(runtime).connect();
    if (runtime->resources) {
      const auto configured = checked(runtime).configure_mode(
          runtime->product_mode);
      if (configured != ARTICORE_OPERATION_OK) {
        g_last_error = checked(runtime).health_v2().last_operation_error;
        return configured;
      }
    }
    checked(runtime).record_operation_result(ARTICORE_OPERATION_CONNECT,
                                             ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_CONNECT, ARTICORE_OPERATION_FEEDBACK,
          error.what());
    }
    g_last_error = error.what();
    return -1;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_disconnect(
    ArticoreRuntime* runtime) {
  try {
    checked(runtime).disconnect();
    checked(runtime).record_operation_result(ARTICORE_OPERATION_DISCONNECT,
                                             ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_DISCONNECT, ARTICORE_OPERATION_VERIFICATION,
        error.what());
    g_last_error = error.what();
    return -1;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_configure_mode(
    ArticoreRuntime* runtime, int32_t mode) {
  try {
    const auto result = checked(runtime).configure_mode(
        static_cast<ArticoreControlMode>(mode));
    if (result == ARTICORE_OPERATION_OK && runtime->resources) {
      runtime->product_mode = static_cast<ArticoreControlMode>(mode);
    }
    g_last_error = result == ARTICORE_OPERATION_OK
        ? "ok" : checked(runtime).health_v2().last_operation_error;
    return result;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_clear_faults(
    ArticoreRuntime* runtime) {
  try {
    const auto result = checked(runtime).clear_faults();
    g_last_error = result == ARTICORE_OPERATION_OK
        ? "ok" : checked(runtime).health_v2().last_operation_error;
    return result;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_zero(
    ArticoreRuntime* runtime) {
  try {
    const auto result = checked(runtime).set_zero();
    g_last_error = result == ARTICORE_OPERATION_OK
        ? "ok" : checked(runtime).health_v2().last_operation_error;
    return result;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_positions(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
    float speed_percent) {
  try {
    auto& product = checked_product(runtime);
    require_product_count(count);
    require_finite(positions, count, "positions");
    if (!std::isfinite(speed_percent) || speed_percent < 0.0f ||
        speed_percent > 100.0f) {
      throw std::invalid_argument(
          "ordinary speed must be finite and within 0..100");
    }
    const float maximum_velocity = runtime->product_mode == ARTICORE_MODE_MIT
        ? product.default_mit_reference_velocity
        : product.default_pv_reference_velocity;
    const float selected_velocity =
        maximum_velocity * speed_percent / 100.0f;
    std::array<ArticoreJointMitTarget, ARTICORE_PRODUCT_DUAL_ARM_DOF> mit{};
    std::array<ArticoreJointPvTarget, ARTICORE_PRODUCT_DUAL_ARM_DOF> pv{};
    for (uint32_t i = 0; i < count; ++i) {
      const auto& joint = product.joints[i];
      validate_product_position(joint, positions[i], i);
      if (selected_velocity > joint.velocity_limit) {
        throw std::invalid_argument(
            "reference velocity exceeds product joint limit");
      }
      const float motor_position = joint.direction * positions[i];
      mit[i] = {sizeof(ArticoreJointMitTarget), joint.motor, motor_position};
      pv[i] = {sizeof(ArticoreJointPvTarget), joint.motor, motor_position};
    }
    if (runtime->product_mode == ARTICORE_MODE_MIT) {
      checked(runtime).set_joint_mit(mit.data(), count,
                                     selected_velocity);
    } else {
      checked(runtime).set_joint_pv(pv.data(), count,
                                    selected_velocity);
    }
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE, error.what());
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocities, const float* feedforward_torques,
    const float* kp, const float* kd, uint32_t count) {
  try {
    auto& product = checked_product(runtime);
    require_product_count(count);
    require_finite(positions, count, "positions");
    if (velocities) require_finite(velocities, count, "velocities");
    if (feedforward_torques) {
      require_finite(feedforward_torques, count, "feedforward_torques");
    }
    if (kp) require_finite(kp, count, "kp");
    if (kd) require_finite(kd, count, "kd");
    if (runtime->product_mode != ARTICORE_MODE_MIT) {
      throw std::runtime_error("raw MIT frame requires product MIT mode");
    }
    std::array<ArticoreMitCommand, ARTICORE_PRODUCT_DUAL_ARM_DOF> commands{};
    for (uint32_t i = 0; i < count; ++i) {
      const auto& joint = product.joints[i];
      validate_product_position(joint, positions[i], i);
      const float velocity = velocities ? velocities[i] : 0.0f;
      const float torque = feedforward_torques
          ? feedforward_torques[i] : 0.0f;
      const float stiffness = kp ? kp[i] : joint.kp;
      const float damping = kd ? kd[i] : joint.kd;
      if (std::fabs(velocity) > joint.velocity_limit) {
        throw std::invalid_argument("velocity exceeds product joint limit");
      }
      if (std::fabs(torque) > joint.torque_limit) {
        throw std::invalid_argument("torque exceeds product joint limit");
      }
      if (stiffness < 0.0f || stiffness > 500.0f ||
          damping < 0.0f || damping > 5.0f) {
        throw std::invalid_argument("MIT gain exceeds protocol limits");
      }
      commands[i] = ArticoreMitCommand{
          joint.motor,
          joint.direction * positions[i],
          joint.direction * velocity * joint.velocity_command_scale,
          stiffness, damping,
          joint.direction * torque *
              joint.torque_command_scale};
    }
    checked(runtime).submit_mit(commands.data(), count);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE, error.what());
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pv_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocity_limits, uint32_t count) {
  try {
    auto& product = checked_product(runtime);
    require_product_count(count);
    require_finite(positions, count, "positions");
    require_finite(velocity_limits, count, "velocity_limits");
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error("raw PV frame requires product PV mode");
    }
    std::array<ArticorePosVelCommand, ARTICORE_PRODUCT_DUAL_ARM_DOF> commands{};
    for (uint32_t i = 0; i < count; ++i) {
      const auto& joint = product.joints[i];
      validate_product_position(joint, positions[i], i);
      if (velocity_limits[i] <= 0.0f ||
          velocity_limits[i] > joint.velocity_limit) {
        throw std::invalid_argument(
            "PV velocity limit exceeds product joint limit");
      }
      commands[i] = ArticorePosVelCommand{
          joint.motor, joint.direction * positions[i], velocity_limits[i]};
    }
    checked(runtime).submit_pos_vel(commands.data(), count);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE, error.what());
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_state(
    ArticoreRuntime* runtime, ArticoreProductState* state) {
  if (!state || state->struct_size < sizeof(*state)) {
    g_last_error = "product state output is null or too small";
    return -1;
  }
  try {
    auto& product = checked_product(runtime);
    const uint32_t caller_size = state->struct_size;
    ArticoreProductState output{};
    output.struct_size = caller_size;
    output.has_grippers = product.with_grippers ? 1 : 0;
    uint64_t maximum_age = 0;
    uint64_t sequence = std::numeric_limits<uint64_t>::max();
    for (uint32_t i = 0; i < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++i) {
      const auto& joint = product.joints[i];
      MotorState motor{};
      MotorFeedbackStats stats{};
      if (motor_handle_get_state(joint.motor, &motor) != 0 ||
          !motor.has_value ||
          motor_handle_get_feedback_stats(joint.motor, &stats) != 0 ||
          !stats.has_feedback) {
        throw std::runtime_error(
            "complete product feedback is unavailable for joint " +
            std::to_string(i));
      }
      auto& arm = i < ARTICORE_PRODUCT_ARM_DOF ? output.left : output.right;
      const uint32_t index = i % ARTICORE_PRODUCT_ARM_DOF;
      arm.positions[index] = joint.direction * motor.pos;
      arm.velocities[index] = joint.direction * motor.vel *
                              joint.velocity_feedback_scale;
      arm.torques[index] = joint.direction * motor.torq *
                           joint.torque_feedback_scale;
      maximum_age = std::max(maximum_age, stats.age_ns);
      sequence = std::min(sequence, stats.update_count);
    }
    if (product.with_grippers) {
      const auto health = checked(runtime).health();
      for (uint32_t side = 0; side < 2; ++side) {
        MotorState motor{};
        MotorFeedbackStats stats{};
        if (motor_handle_get_state(product.grippers[side], &motor) != 0 ||
            !motor.has_value ||
            motor_handle_get_feedback_stats(product.grippers[side], &stats) != 0 ||
            !stats.has_feedback) {
          throw std::runtime_error(
              "complete product gripper feedback is unavailable");
        }
        if (side == 0) {
          output.left_gripper_available = 1;
          output.left_gripper_level = product.gripper_levels[side].load();
        } else {
          output.right_gripper_available = 1;
          output.right_gripper_level = product.gripper_levels[side].load();
        }
        for (uint32_t i = 0; i < health.gripper_count && i < 2; ++i) {
          if (health.grippers[i].side == side) {
            if (side == 0) {
              output.left_gripper_opening = health.grippers[i].opening;
            } else {
              output.right_gripper_opening = health.grippers[i].opening;
            }
          }
        }
        maximum_age = std::max(maximum_age, stats.age_ns);
        sequence = std::min(sequence, stats.update_count);
      }
    }
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    output.timestamp_ns = static_cast<uint64_t>(now) > maximum_age
        ? static_cast<uint64_t>(now) - maximum_age : 0;
    output.sequence = sequence == std::numeric_limits<uint64_t>::max()
        ? 0 : sequence;
    *state = output;
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return -1;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_pose(
    ArticoreRuntime* runtime, uint32_t side, ArticoreProductPose* pose) {
  if (!pose || pose->struct_size < sizeof(*pose)) {
    g_last_error = "product pose output is null or too small";
    return -1;
  }
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    g_last_error = "product pose side must be LEFT(0) or RIGHT(1)";
    return -1;
  }
  try {
    auto& product = checked_product(runtime);
    std::array<double, ARTICORE_PRODUCT_ARM_DOF> q{};
    uint64_t maximum_age = 0;
    uint64_t sequence = std::numeric_limits<uint64_t>::max();
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      const auto& joint = product.joints[
          side * ARTICORE_PRODUCT_ARM_DOF + index];
      MotorState motor{};
      MotorFeedbackStats stats{};
      if (motor_handle_get_state(joint.motor, &motor) != 0 ||
          !motor.has_value ||
          motor_handle_get_feedback_stats(joint.motor, &stats) != 0 ||
          !stats.has_feedback) {
        throw std::runtime_error(
            "complete pose feedback is unavailable for " +
            std::string(side == ARTICORE_ROBOT_LEFT ? "left/joint" :
                                                        "right/joint") +
            std::to_string(index + 1));
      }
      q[index] = static_cast<double>(joint.direction * motor.pos);
      if (!std::isfinite(q[index])) {
        throw std::runtime_error("pose feedback contains NaN or Inf");
      }
      maximum_age = std::max(maximum_age, stats.age_ns);
      sequence = std::min(sequence, stats.update_count);
    }

    ArticoreRobotPose native_pose{};
    native_pose.struct_size = sizeof(native_pose);
    {
      std::lock_guard<std::mutex> lock(product.pose_mutexes[side]);
      product.pose_models[side]->fk(q.data(), q.size(), &native_pose);
    }
    const double* rotation = native_pose.rotation;
    const double pitch = std::asin(std::clamp(-rotation[6], -1.0, 1.0));
    const double cos_pitch = std::cos(pitch);
    double roll = 0.0;
    double yaw = 0.0;
    if (std::abs(cos_pitch) > 1e-9) {
      roll = std::atan2(rotation[7], rotation[8]);
      yaw = std::atan2(rotation[3], rotation[0]);
    } else {
      // At gimbal lock yaw is not unique. Keep yaw at zero and preserve the
      // equivalent orientation in roll for a deterministic output.
      roll = std::atan2(-rotation[5], rotation[4]);
    }

    const uint32_t caller_size = pose->struct_size;
    ArticoreProductPose output{};
    output.struct_size = caller_size;
    output.side = side;
    output.values[0] = static_cast<float>(native_pose.position[0]);
    output.values[1] = static_cast<float>(native_pose.position[1]);
    output.values[2] = static_cast<float>(native_pose.position[2]);
    output.values[3] = static_cast<float>(roll);
    output.values[4] = static_cast<float>(pitch);
    output.values[5] = static_cast<float>(yaw);
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    output.timestamp_ns = static_cast<uint64_t>(now) > maximum_age
        ? static_cast<uint64_t>(now) - maximum_age : 0;
    output.sequence = sequence == std::numeric_limits<uint64_t>::max()
        ? 0 : sequence;
    *pose = output;
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return -1;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_grippers(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
    int32_t gripper_level) {
  try {
    auto& product = checked_product(runtime);
    if (!std::isfinite(left_opening) || !std::isfinite(right_opening)) {
      throw std::invalid_argument("gripper opening contains NaN or Inf");
    }
    if (gripper_level < 1 || gripper_level > 5) {
      throw std::invalid_argument("gripper_level must be in the range 1..5");
    }
    if (!product.with_grippers) {
      checked(runtime).record_operation_result(
          ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
      g_last_error = "ok";
      return 0;
    }
    const float openings[2] = {
        std::clamp(left_opening, 0.0f, 1000.0f),
        std::clamp(right_opening, 0.0f, 1000.0f)};
    static constexpr int32_t kNativeForceLevels[5] = {1, 3, 5, 8, 10};
    const int32_t native_force_level = kNativeForceLevels[gripper_level - 1];
    ArticoreGripperCommand commands[2]{};
    for (uint32_t side = 0; side < 2; ++side) {
      commands[side].struct_size = sizeof(ArticoreGripperCommand);
      commands[side].motor = product.grippers[side];
      commands[side].opening = openings[side];
      commands[side].speed = 1000.0f;
      commands[side].force_level = native_force_level;
    }
    checked(runtime).set_gripper_commands(commands, 2);
    for (auto& level : product.gripper_levels) level.store(gripper_level);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE, error.what());
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_has_grippers(
    ArticoreRuntime* runtime, int32_t* has_grippers) {
  return call([&] {
    if (!has_grippers) throw std::invalid_argument("has_grippers is null");
    *has_grippers = checked_product(runtime).with_grippers ? 1 : 0;
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_configure_motor_identities(
    ArticoreRuntime* runtime, const ArticoreMotorIdentity* identities,
    uint32_t identity_count) {
  return call([&] {
    checked(runtime).configure_motor_identities(identities, identity_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_connect_report(
    ArticoreRuntime* runtime, ArticoreConnectReport* report) {
  return call([&] {
    if (!report) throw std::invalid_argument("connect report is null");
    if (report->struct_size < sizeof(ArticoreConnectReport)) {
      throw std::invalid_argument("connect report struct_size is too small");
    }
    *report = checked(runtime).last_connect_report();
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_enable(ArticoreRuntime* runtime,
                                                     int32_t mode) {
  try {
    checked(runtime).enable(static_cast<ArticoreControlMode>(mode));
    checked(runtime).record_operation_result(ARTICORE_OPERATION_ENABLE,
                                             ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_ENABLE, ARTICORE_OPERATION_MOTOR_COMMAND,
        error.what());
    g_last_error = error.what();
    return -1;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_motor_power(
    ArticoreRuntime* runtime, const char* motor_name, int32_t enabled,
    int32_t* confirmed_state) {
  if (enabled != 0 && enabled != 1) {
    g_last_error = "enabled must be 0 or 1";
    return -1;
  }
  try {
    auto& value = checked(runtime);
    const std::string role = motor_name ? motor_name : "";
    ArticoreMotorPowerState result = ARTICORE_MOTOR_POWER_UNKNOWN;
    if (role.empty()) {
      if (enabled) {
        value.enable(runtime->resources ? runtime->product_mode
                                        : value.control_mode());
        result = ARTICORE_MOTOR_POWER_ENABLED;
      } else {
        value.disable();
        result = ARTICORE_MOTOR_POWER_DISABLED;
      }
    } else {
      result = value.set_motor_power(role, enabled != 0);
    }
    if (confirmed_state) *confirmed_state = static_cast<int32_t>(result);
    value.record_operation_result(
        enabled ? ARTICORE_OPERATION_ENABLE : ARTICORE_OPERATION_DISABLE,
        ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) runtime->runtime->record_operation_result(
        enabled ? ARTICORE_OPERATION_ENABLE : ARTICORE_OPERATION_DISABLE,
        ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    g_last_error = error.what();
    return -1;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) runtime->runtime->record_operation_result(
        enabled ? ARTICORE_OPERATION_ENABLE : ARTICORE_OPERATION_DISABLE,
        ARTICORE_OPERATION_VERIFICATION, error.what());
    g_last_error = error.what();
    return -1;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_motor_power(
    ArticoreRuntime* runtime, const char* motor_name, int32_t* state) {
  return call([&] {
    if (!state) throw std::invalid_argument("motor power state output is null");
    const std::string role = motor_name ? motor_name : "";
    *state = static_cast<int32_t>(
        checked(runtime).motor_power_state(role));
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_enable_report(
    ArticoreRuntime* runtime, ArticoreEnableReport* report) {
  return call([&] {
    if (!report) throw std::invalid_argument("enable report is null");
    if (report->struct_size < sizeof(ArticoreEnableReport)) {
      throw std::invalid_argument("enable report struct_size is too small");
    }
    *report = checked(runtime).last_enable_report();
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_configure_joints(
    ArticoreRuntime* runtime,
    const ArticoreJointControlConfig* configs,
    uint32_t config_count) {
  return call([&] { checked(runtime).configure_joints(configs, config_count); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_configure_joint_safety_limits(
    ArticoreRuntime* runtime,
    const ArticoreJointSafetyLimits* limits,
    uint32_t limit_count) {
  return call([&] {
    checked(runtime).configure_joint_safety_limits(limits, limit_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pos_vel(
    ArticoreRuntime* runtime,
    const ArticorePosVelCommand* commands,
    uint32_t command_count) {
  return call([&] { checked(runtime).submit_pos_vel(commands, command_count); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count) {
  return call([&] { checked(runtime).submit_mit(commands, command_count); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pos_vel_ex(
    ArticoreRuntime* runtime,
    const ArticorePosVelCommand* commands,
    uint32_t command_count,
    int32_t lifetime) {
  return call([&] {
    checked(runtime).submit_pos_vel_ex(
        commands, command_count, static_cast<ArticoreCommandLifetime>(lifetime));
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit_ex(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count,
    int32_t lifetime) {
  return call([&] {
    checked(runtime).submit_mit_ex(
        commands, command_count, static_cast<ArticoreCommandLifetime>(lifetime));
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit(
    ArticoreRuntime* runtime,
    const ArticoreJointMitTarget* targets,
    uint32_t target_count,
    float speed_percent) {
  return call([&] {
    checked(runtime).set_joint_mit_speed(
        targets, target_count, speed_percent);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_pv(
    ArticoreRuntime* runtime,
    const ArticoreJointPvTarget* targets,
    uint32_t target_count,
    float speed_percent) {
  return call([&] {
    checked(runtime).set_joint_pv_speed(
        targets, target_count, speed_percent);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_gripper_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count) {
  return call([&] {
    checked(runtime).submit_gripper_mit(commands, command_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_gripper_openings(
    ArticoreRuntime* runtime,
    const ArticoreGripperTarget* targets,
    uint32_t target_count) {
  return call([&] {
    checked(runtime).set_gripper_openings(targets, target_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gripper_products(
    ArticoreRuntime* runtime,
    const ArticoreGripperProductBinding* bindings,
    uint32_t binding_count) {
  return call([&] {
    checked(runtime).configure_gripper_products(bindings, binding_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gravity_products(
    ArticoreRuntime* runtime,
    const ArticoreGravityProductBinding* bindings,
    uint32_t binding_count) {
  return call([&] {
    checked(runtime).configure_gravity_products(bindings, binding_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_start_gravity_compensation(
    ArticoreRuntime* runtime,
    const ArticoreGravityCompensationConfig* config) {
  return call([&] { checked(runtime).start_gravity_compensation(config); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_stop_gravity_compensation(
    ArticoreRuntime* runtime) {
  return call([&] { checked(runtime).stop_gravity_compensation(); });
}

ARTICORE_RUNTIME_API int32_t
articore_runtime_get_gravity_compensation_status(
    ArticoreRuntime* runtime,
    ArticoreGravityCompensationStatus* status) {
  return call([&] {
    if (!status || status->struct_size < sizeof(*status)) {
      throw std::invalid_argument(
          "gravity compensation status is null or too small");
    }
    const auto size = status->struct_size;
    *status = checked(runtime).gravity_compensation_status();
    status->struct_size = size;
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gripper_force_profiles(
    ArticoreRuntime* runtime,
    const ArticoreGripperForceProfile* profiles,
    uint32_t profile_count) {
  return call([&] {
    checked(runtime).configure_gripper_force_profiles(profiles, profile_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_gripper_commands(
    ArticoreRuntime* runtime,
    const ArticoreGripperCommand* commands,
    uint32_t command_count) {
  return call([&] {
    checked(runtime).set_gripper_commands(commands, command_count);
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_report_feedback_failure(
    ArticoreRuntime* runtime, uint8_t side, const char* reason) {
  return call([&] {
    checked(runtime).report_feedback_failure(
        side, reason ? std::string(reason) : std::string("feedback failure"));
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_disable(ArticoreRuntime* runtime) {
  try {
    checked(runtime).disable();
    checked(runtime).record_operation_result(ARTICORE_OPERATION_DISABLE,
                                             ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_DISABLE, ARTICORE_OPERATION_VERIFICATION,
        error.what());
    g_last_error = error.what();
    return -1;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_estop(ArticoreRuntime* runtime,
                                                    const char* reason) {
  return call([&] {
    checked(runtime).estop(reason ? std::string(reason) : std::string());
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_recover(ArticoreRuntime* runtime) {
  return call([&] { checked(runtime).recover(); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_disable_report(
    ArticoreRuntime* runtime, ArticoreDisableReport* report) {
  return call([&] {
    if (!report) throw std::invalid_argument("disable report is null");
    if (report->struct_size < sizeof(ArticoreDisableReport)) {
      throw std::invalid_argument("disable report struct_size is too small");
    }
    *report = checked(runtime).last_disable_report();
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_health(
    ArticoreRuntime* runtime, ArticoreSafetyHealth* health) {
  if (!health) {
    g_last_error = "health output is null";
    return -1;
  }
  return call([&] { *health = checked(runtime).health(); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_health_v2(
    ArticoreRuntime* runtime, ArticoreSafetyHealthV2* health) {
  if (!health || health->struct_size < sizeof(*health)) {
    g_last_error = "health V2 output is null or too small";
    return -1;
  }
  return call([&] {
    const auto caller_size = health->struct_size;
    *health = checked(runtime).health_v2();
    health->struct_size = caller_size;
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_mit_torque_limit_stats(
    ArticoreRuntime* runtime, ArticoreMitTorqueLimitStats* stats) {
  return call([&] {
    if (!stats) throw std::invalid_argument("MIT torque limit stats are null");
    if (stats->struct_size < sizeof(ArticoreMitTorqueLimitStats)) {
      throw std::invalid_argument(
          "MIT torque limit stats struct_size is too small");
    }
    *stats = checked(runtime).mit_torque_limit_stats();
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_declare_motor_presence(
    ArticoreRuntime* runtime, const char* motor_role, int32_t state) {
  if (!motor_role) {
    g_last_error = "motor_role is null";
    return -1;
  }
  return call([&] {
    checked(runtime).declare_motor_presence(
        motor_role, static_cast<ArticorePresenceState>(state));
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_motor_presence(
    ArticoreRuntime* runtime, const char* motor_role, int32_t* state) {
  if (!motor_role || !state) {
    g_last_error = "motor_role or state output is null";
    return -1;
  }
  return call([&] { *state = checked(runtime).motor_presence(motor_role); });
}

ARTICORE_RUNTIME_API uint64_t articore_runtime_active_capabilities(
    ArticoreRuntime* runtime) {
  try {
    const auto value = checked(runtime).active_capabilities();
    g_last_error = "ok";
    return value;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return 0;
  } catch (...) {
    g_last_error = "unknown Articore runtime exception";
    return 0;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_close(ArticoreRuntime* runtime) {
  try {
    checked(runtime).close();
    checked(runtime).record_operation_result(ARTICORE_OPERATION_CLOSE,
                                             ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_CLOSE, ARTICORE_OPERATION_VERIFICATION,
        error.what());
    g_last_error = error.what();
    return -1;
  }
}

ARTICORE_RUNTIME_API const char* articore_runtime_last_error(void) {
  return g_last_error.c_str();
}

}  // extern "C"
