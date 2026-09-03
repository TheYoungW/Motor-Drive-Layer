#include <algorithm>
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
#include <utility>
#include <vector>

#include "articore/detail/native_types.hpp"
#include "articore/detail/yunyi_runtime_core.hpp"
#include "articore/detail/product_cartesian.hpp"
#include "articore/detail/robot_model.hpp"
#include "articore/detail/runtime.hpp"
#include "articore/detail/yunyi_runtime.hpp"

struct YunyiRuntimeCoreState {
  explicit YunyiRuntimeCoreState(
      std::unique_ptr<articore::SafetyRuntime> value,
      std::unique_ptr<articore::YunyiRuntimeResources> owned = {},
      ArticoreControlMode product_mode = ARTICORE_MODE_PV)
      : yunyi_owned(owned != nullptr), yunyi(std::move(owned)),
        runtime(std::move(value)), product_mode(product_mode) {}
  std::mutex terminal_mutex;
  bool yunyi_owned = false;
  bool terminally_disconnected = false;
  std::unique_ptr<articore::YunyiRuntimeResources> yunyi;
  std::unique_ptr<articore::SafetyRuntime> runtime;
  ArticoreControlMode product_mode = ARTICORE_MODE_PV;
  std::mutex product_pv_limits_mutex;
  float product_pv_max_velocity =
      articore::kYunyiOrdinaryPvDefaultLimitSelection;
  float product_pv_max_acceleration =
      articore::kYunyiOrdinaryPvDefaultLimitSelection;
  float product_speed_percent = 100.0f;
  std::mutex motion_mutex;
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

YunyiRuntimeCoreState* create_yunyi_runtime_checked(
    int32_t requested_mode,
    const articore::YunyiNativeConfig& native_config = {}) {
  if (requested_mode != ARTICORE_MODE_PV &&
      requested_mode != ARTICORE_MODE_MIT) {
    throw std::invalid_argument("unsupported Yunyi control mode");
  }
  auto bundle = articore::create_yunyi_runtime(
      static_cast<ArticoreControlMode>(requested_mode), native_config);
  return new YunyiRuntimeCoreState(
      std::move(bundle.runtime), std::move(bundle.resources), bundle.mode);
}

articore::SafetyRuntime& checked(YunyiRuntimeCoreState* runtime) {
  if (!runtime || !runtime->runtime) throw std::invalid_argument("runtime is null");
  return *runtime->runtime;
}

articore::YunyiRuntimeResources& checked_yunyi(YunyiRuntimeCoreState* runtime) {
  checked(runtime);
  if (!runtime->yunyi) {
    throw std::runtime_error("operation requires the Yunyi dual-arm Runtime");
  }
  return *runtime->yunyi;
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

float require_optional_pv_limit(
    float value, float maximum, const char* name) {
  if (!std::isfinite(value) || value < 0.0f || value > maximum) {
    throw std::invalid_argument(
        std::string(name) + " must be 0 to use product defaults or within " +
        std::to_string(articore::kYunyiPvMotionLimitResolution) + ".." +
        std::to_string(maximum));
  }
  if (value == 0.0f) return value;
  const float quantized = std::round(
      value / articore::kYunyiPvMotionLimitResolution) *
      articore::kYunyiPvMotionLimitResolution;
  if (std::abs(value - quantized) > 1.0e-5f) {
    throw std::invalid_argument(
        std::string(name) + " must use 0.01 physical-unit resolution");
  }
  return quantized;
}

std::pair<std::vector<float>, std::vector<float>>
effective_product_pv_limits(
    float speed_percent, float configured_maximum_velocity,
    float configured_maximum_acceleration) {
  std::vector<float> velocities;
  std::vector<float> accelerations;
  velocities.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
  accelerations.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
  for (uint32_t index = 0;
       index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    velocities.push_back(articore::yunyi_ordinary_pv_velocity_limit(
        index, speed_percent, configured_maximum_velocity));
    accelerations.push_back(articore::yunyi_ordinary_pv_acceleration_limit(
        index, speed_percent, configured_maximum_acceleration));
  }
  return {std::move(velocities), std::move(accelerations)};
}

struct CartesianSpeedScale {
  float reference_velocity = articore::kYunyiCartesianMaximumVelocity;
  float reference_acceleration =
      articore::kYunyiTrajectoryPvAccelerationLimit;
};

CartesianSpeedScale cartesian_speed_scale(YunyiRuntimeCoreState* runtime) {
  std::lock_guard<std::mutex> lock(runtime->product_pv_limits_mutex);
  const float scale = runtime->product_speed_percent / 100.0f;
  return {
      articore::kYunyiCartesianMaximumVelocity * scale,
      articore::kYunyiTrajectoryPvAccelerationLimit * scale * scale};
}

void validate_product_position(
    const articore::YunyiRuntimeResources::Joint& joint, float position,
    uint32_t index) {
  if (position < joint.lower || position > joint.upper) {
    throw std::invalid_argument(
        articore::yunyi_joint_role(index) + " position=" +
        std::to_string(position) + " rad exceeds product limits, allowed=[" +
        std::to_string(joint.lower) + ", " +
        std::to_string(joint.upper) + "] rad");
  }
}

class CommandPlanningScope {
 public:
  explicit CommandPlanningScope(articore::SafetyRuntime& runtime)
      : runtime_(runtime) {}
  CommandPlanningScope(const CommandPlanningScope&) = delete;
  CommandPlanningScope& operator=(const CommandPlanningScope&) = delete;
  ~CommandPlanningScope() { runtime_.cancel_command_planning(token_); }

  void begin(const articore::SafetyRuntime::CommandTransaction& transaction,
             bool allow_trajectory = false) {
    token_ = runtime_.begin_command_planning(transaction, allow_trajectory);
  }

  uint64_t token() const { return token_; }

 private:
  articore::SafetyRuntime& runtime_;
  uint64_t token_ = 0;
};

void install_product_joint_positions(
    YunyiRuntimeCoreState* runtime, const float* positions, uint32_t count,
    float speed_percent,
    articore::SafetyRuntime::CommandTransaction* transaction = nullptr,
    uint64_t planning_token = 0,
    bool direct_mit = false,
    std::unique_lock<std::mutex>* product_limits_transaction = nullptr) {
  auto& product = checked_yunyi(runtime);
  require_product_count(count);
  require_finite(positions, count, "positions");
  const float minimum_speed =
      runtime->product_mode == ARTICORE_MODE_PV ? 1.0f : 0.0f;
  if (!std::isfinite(speed_percent) || speed_percent < minimum_speed ||
      speed_percent > 100.0f) {
    throw std::invalid_argument(
        runtime->product_mode == ARTICORE_MODE_PV
            ? "ordinary PV speed must be finite and within 1..100"
            : "fast MIT speed must be finite and within 0..100");
  }
  float selected_reference_velocity = 0.0f;
  std::unique_lock<std::mutex> pv_limits_lock;
  if (runtime->product_mode == ARTICORE_MODE_MIT && !direct_mit) {
    selected_reference_velocity =
        articore::yunyi_mit_fast_follow_reference_velocity(speed_percent);
  }
  std::array<ArticoreJointMitTarget, ARTICORE_PRODUCT_DUAL_ARM_DOF> mit{};
  std::array<ArticoreJointPvTarget, ARTICORE_PRODUCT_DUAL_ARM_DOF> pv{};
  std::vector<float> pv_velocity_limits;
  std::vector<float> pv_acceleration_limits;
  if (runtime->product_mode == ARTICORE_MODE_PV) {
    if (product_limits_transaction) {
      if (!product_limits_transaction->owns_lock() ||
          product_limits_transaction->mutex() !=
              &runtime->product_pv_limits_mutex) {
        throw std::logic_error(
            "PV command requires the product motion-limit transaction");
      }
    } else {
      pv_limits_lock =
          std::unique_lock<std::mutex>(runtime->product_pv_limits_mutex);
    }
    auto limits = effective_product_pv_limits(
        speed_percent, runtime->product_pv_max_velocity,
        runtime->product_pv_max_acceleration);
    pv_velocity_limits = std::move(limits.first);
    pv_acceleration_limits = std::move(limits.second);
  }
  for (uint32_t i = 0; i < count; ++i) {
    const auto& joint = product.joints[i];
    validate_product_position(joint, positions[i], i);
    if (selected_reference_velocity > joint.velocity_limit) {
      throw std::invalid_argument(
          "reference velocity exceeds product joint limit");
    }
    const float motor_position = joint.direction * positions[i];
    mit[i] = {sizeof(ArticoreJointMitTarget), joint.motor, motor_position};
    pv[i] = {sizeof(ArticoreJointPvTarget), joint.motor, motor_position};
  }
  auto& safety = checked(runtime);
  if (runtime->product_mode == ARTICORE_MODE_MIT) {
    if (direct_mit && transaction) {
      safety.set_joint_mit_direct_planned(
          mit.data(), count, *transaction, planning_token);
    } else if (direct_mit) {
      safety.set_joint_mit_direct(mit.data(), count);
    } else if (transaction) {
      safety.set_joint_mit_planned(
          mit.data(), count, selected_reference_velocity,
          *transaction, planning_token);
    } else {
      safety.set_joint_mit(mit.data(), count, selected_reference_velocity);
    }
  } else if (transaction) {
    safety.set_joint_pv_profile_planned(
        pv.data(), count, pv_velocity_limits, pv_acceleration_limits,
        *transaction, planning_token);
  } else {
    safety.set_joint_pv_profile(
        pv.data(), count, pv_velocity_limits, pv_acceleration_limits);
  }
  if (runtime->product_mode == ARTICORE_MODE_PV) {
    runtime->product_speed_percent = speed_percent;
  }
}

int32_t record_product_command_error(
    YunyiRuntimeCoreState* runtime, int32_t code, const std::string& error) {
  if (runtime && runtime->runtime) {
    runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_COMMAND, code, error);
  }
  g_last_error = error;
  return code;
}

articore::NativeTrajectorySample product_ik_reference(
    articore::SafetyRuntime& safety,
    const articore::YunyiRuntimeResources& product,
    const std::vector<articore::NativeTrajectoryJoint>& joints) {
  try {
    auto transaction = safety.begin_command_transaction();
    return safety.planned_arm_sample(joints, transaction);
  } catch (const std::runtime_error& error) {
    if (std::string(error.what()) !=
        "current planned arm reference is unavailable") {
      throw;
    }
  }

  articore::NativeTrajectorySample reference;
  reference.positions.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
  reference.velocities.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  reference.accelerations.assign(ARTICORE_PRODUCT_DUAL_ARM_DOF, 0.0f);
  const uint64_t fresh_limit = safety.feedback_max_age_ns();
  for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
    const auto& joint = product.joints[index];
    ArticoreMotorState motor{};
    ArticoreFeedbackStats stats{};
    if (!articore::read_yunyi_motor_state(joint.motor, motor, stats) ||
        !motor.has_value || !stats.has_feedback ||
        stats.age_ns > fresh_limit || !std::isfinite(motor.pos)) {
      throw std::runtime_error(
          "solve_ik requires fresh complete joint feedback at " +
          articore::yunyi_joint_role(index));
    }
    reference.positions.push_back(joint.direction * motor.pos);
  }
  return reference;
}

int32_t solve_product_ik_impl(
    YunyiRuntimeCoreState* runtime, const float* left_target_pose,
    const float* right_target_pose, float* positions, uint32_t count) {
  try {
    if (!left_target_pose || !right_target_pose) {
      throw std::invalid_argument(
          "solve_ik requires both left and right target poses");
    }
    if (!positions) {
      throw std::invalid_argument("solve_ik joint output is null");
    }
    if (count != ARTICORE_PRODUCT_DUAL_ARM_DOF) {
      throw std::invalid_argument("solve_ik joint output count must be 14");
    }
    require_finite(
        left_target_pose, ARTICORE_PRODUCT_POSE_DOF, "left target pose");
    require_finite(
        right_target_pose, ARTICORE_PRODUCT_POSE_DOF, "right target pose");
    if (!runtime) throw std::invalid_argument("runtime is null");

    std::lock_guard<std::mutex> motion_lock(runtime->motion_mutex);
    auto& safety = checked(runtime);
    auto& product = checked_yunyi(runtime);
    const auto joints = articore::product_cartesian_joints(product);
    const auto reference = product_ik_reference(safety, product, joints);
    const auto ik_deadline = std::chrono::steady_clock::now() +
        articore::kYunyiProductIkBudget;
    const auto target =
        articore::solve_dual_point_to_point_targets_from_reference(
            product, runtime->product_mode, reference,
            left_target_pose, right_target_pose, ik_deadline);
    std::copy(target.begin(), target.end(), positions);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t move_linear_trajectory_impl(
    YunyiRuntimeCoreState* runtime, uint32_t side, const float* start_pose,
    const float* end_pose, uint64_t* motion_id, bool enqueue = true,
    ArticoreRuntimeOperation operation = ARTICORE_OPERATION_MOVE_LINEAR) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    if (!motion_id) throw std::invalid_argument("motion_id output is null");
    std::lock_guard<std::mutex> motion_lock(runtime->motion_mutex);
    auto& safety = checked(runtime);
    auto& product = checked_yunyi(runtime);
    const auto joints = articore::product_cartesian_joints(product);

    articore::NativeTrajectorySample reference;
    CommandPlanningScope planning(safety);
    {
      auto snapshot = safety.begin_command_transaction();
      planning.begin(snapshot, enqueue);
      reference = enqueue
          ? safety.planned_trajectory_tail_sample(joints, snapshot)
          : safety.planned_arm_sample(joints, snapshot);
    }
    const auto speed = cartesian_speed_scale(runtime);
    auto plan = start_pose
        ? articore::build_linear_trajectory_plan_from_reference(
              product, runtime->product_mode, side, reference,
              start_pose, end_pose, speed.reference_acceleration,
              speed.reference_velocity)
        : articore::build_linear_trajectory_plan_from_reference(
              product, runtime->product_mode, side, reference,
              end_pose, speed.reference_acceleration,
              speed.reference_velocity);
    plan.trajectory.operation = operation;

    auto transaction = safety.begin_command_transaction();
    const auto current = enqueue
        ? safety.planned_trajectory_tail_sample(joints, transaction)
        : safety.planned_arm_sample(joints, transaction);
    articore::require_unchanged_planned_reference(
        reference, current, "linear");

    const uint64_t new_id = safety.start_trajectory(
        plan.trajectory, 0, &transaction, enqueue, planning.token());
    *motion_id = new_id;
    safety.record_operation_result(
        operation, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          operation,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          operation,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t move_circular_trajectory_impl(
    YunyiRuntimeCoreState* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, uint64_t* motion_id,
    bool enqueue = true) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    if (!motion_id) throw std::invalid_argument("motion_id output is null");
    std::lock_guard<std::mutex> motion_lock(runtime->motion_mutex);
    auto& safety = checked(runtime);
    auto& product = checked_yunyi(runtime);
    const auto joints = articore::product_cartesian_joints(product);

    articore::NativeTrajectorySample reference;
    CommandPlanningScope planning(safety);
    {
      auto snapshot = safety.begin_command_transaction();
      planning.begin(snapshot, enqueue);
      reference = enqueue
          ? safety.planned_trajectory_tail_sample(joints, snapshot)
          : safety.planned_arm_sample(joints, snapshot);
    }
    const auto speed = cartesian_speed_scale(runtime);
    auto plan = articore::build_circular_trajectory_plan_from_reference(
        product, runtime->product_mode, side, reference, start_pose,
        via_pose, end_pose,
        speed.reference_acceleration, speed.reference_velocity);

    auto transaction = safety.begin_command_transaction();
    const auto current = enqueue
        ? safety.planned_trajectory_tail_sample(joints, transaction)
        : safety.planned_arm_sample(joints, transaction);
    articore::require_unchanged_planned_reference(
        reference, current, "circular");
    const uint64_t new_id = safety.start_trajectory(
        plan.trajectory, 0, &transaction, enqueue, planning.token());

    *motion_id = new_id;
    safety.record_operation_result(
        ARTICORE_OPERATION_MOVE_CIRCULAR_TRAJECTORY, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_CIRCULAR_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_CIRCULAR_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t set_product_grippers_impl(
    YunyiRuntimeCoreState* runtime, float left_opening, float right_opening,
    int32_t strength, int32_t minimum_strength, int32_t mode,
    const char* strength_error) {
  try {
    if (!std::isfinite(left_opening) || !std::isfinite(right_opening)) {
      throw std::invalid_argument("gripper opening contains NaN or Inf");
    }
    if (strength < minimum_strength ||
        strength > ARTICORE_GRIPPER_STRENGTH_MAX) {
      throw std::invalid_argument(strength_error);
    }
    if (mode != ARTICORE_GRIPPER_MODE_PROTECTED &&
        mode != ARTICORE_GRIPPER_MODE_DIRECT) {
      throw std::invalid_argument("gripper mode must be PROTECTED or DIRECT");
    }
    auto& product = checked_yunyi(runtime);
    if (!product.gripper_present[0] && !product.gripper_present[1]) {
      checked(runtime).record_operation_result(
          ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
      g_last_error = "ok";
      return 0;
    }
    const float openings[2] = {
        std::clamp(left_opening, 0.0f, 1000.0f),
        std::clamp(right_opening, 0.0f, 1000.0f)};
    ArticoreGripperCommand commands[2]{};
    uint32_t command_count = 0;
    for (uint32_t side = 0; side < 2; ++side) {
      if (!product.gripper_present[side]) continue;
      auto& command = commands[command_count++];
      command.struct_size = sizeof(ArticoreGripperCommand);
      command.motor = product.grippers[side];
      command.opening = openings[side];
      command.speed = 1000.0f;
      command.force_level = strength;
    }
    checked(runtime).set_gripper_commands(commands, command_count, mode);
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

}  // namespace

namespace {

int32_t native_runtime_get_control_mode(
    YunyiRuntimeCoreState* runtime, int32_t* mode) {
  return call([&] {
    if (!mode) throw std::invalid_argument("control mode output is null");
    checked(runtime);
    *mode = runtime->yunyi
        ? static_cast<int32_t>(runtime->product_mode)
        : static_cast<int32_t>(runtime->runtime->control_mode());
  });
}

int32_t native_runtime_create_yunyi_configured(
    int32_t requested_mode,
    const char* left_can_interface, const char* right_can_interface,
    int32_t realtime, int32_t lock_memory, int32_t control_cpu,
    int32_t can_tx_cpu, int32_t can_rx_cpu, int32_t control_priority,
    int32_t can_tx_priority, int32_t can_rx_priority,
    uint32_t feedback_max_age_ms, uint32_t motor_watchdog_ms,
    uint32_t motor_discovery_timeout_ms,
    uint32_t motor_discovery_retries,
    YunyiRuntimeCoreState** runtime) {
  if (!runtime || !left_can_interface || !right_can_interface) {
    g_last_error = "configured Runtime arguments are null";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  *runtime = nullptr;
  try {
    articore::YunyiNativeConfig native_config;
    native_config.can_interfaces = {left_can_interface, right_can_interface};
    native_config.realtime = realtime != 0;
    native_config.lock_memory = lock_memory != 0;
    native_config.control_cpu = control_cpu;
    native_config.can_tx_cpu = can_tx_cpu;
    native_config.can_rx_cpu = can_rx_cpu;
    native_config.control_priority = control_priority;
    native_config.can_tx_priority = can_tx_priority;
    native_config.can_rx_priority = can_rx_priority;
    native_config.feedback_max_age_ms = feedback_max_age_ms;
    native_config.motor_watchdog_ms = motor_watchdog_ms;
    native_config.motor_discovery_timeout_ms = motor_discovery_timeout_ms;
    native_config.motor_discovery_retries = motor_discovery_retries;
    *runtime = create_yunyi_runtime_checked(requested_mode, native_config);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  } catch (...) {
    g_last_error = "unknown configured Yunyi Runtime creation error";
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_connect(YunyiRuntimeCoreState* runtime) {
  try {
    checked(runtime).connect();
    if (runtime->yunyi) {
      const auto connected_health = checked(runtime).health();
      if (connected_health.state == ARTICORE_FAULT &&
          connected_health.motor_fault_count > 0) {
        checked(runtime).record_operation_result(ARTICORE_OPERATION_CONNECT,
                                                 ARTICORE_OPERATION_OK);
        g_last_error = "ok";
        return ARTICORE_OPERATION_OK;
      }
      const auto configured = checked(runtime).configure_mode_for_connect(
          runtime->product_mode);
      if (configured != ARTICORE_OPERATION_OK) {
        g_last_error = checked(runtime).health().last_operation_error;
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

int32_t native_runtime_disconnect(
    YunyiRuntimeCoreState* runtime) {
  if (!runtime) {
    g_last_error = "runtime is null";
    return -1;
  }
  std::lock_guard<std::mutex> terminal_lock(runtime->terminal_mutex);
  if (runtime->yunyi_owned && runtime->terminally_disconnected) {
    g_last_error = "ok";
    return 0;
  }
  if (runtime->yunyi_owned) {
    std::string failure;
    try {
      checked(runtime).disconnect();
      checked(runtime).record_operation_result(ARTICORE_OPERATION_DISCONNECT,
                                               ARTICORE_OPERATION_OK);
    } catch (const std::exception& error) {
      failure = error.what();
      if (runtime->runtime) {
        runtime->runtime->record_operation_result(
            ARTICORE_OPERATION_DISCONNECT, ARTICORE_OPERATION_VERIFICATION,
            failure);
      }
    }
    // The SafetyRuntime worker is terminal at this point, including the
    // disable-confirmation failure path. Destroy it before releasing Motors,
    // ControllerGroup, Controllers, and their two CAN transports.
    runtime->runtime.reset();
    runtime->yunyi.reset();
    runtime->terminally_disconnected = true;
    if (!failure.empty()) {
      g_last_error = failure;
      return -1;
    }
    g_last_error = "ok";
    return 0;
  }
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

int32_t native_runtime_configure_mode(
    YunyiRuntimeCoreState* runtime, int32_t mode) {
  try {
    const auto result = checked(runtime).configure_mode(
        static_cast<ArticoreControlMode>(mode));
    if (result == ARTICORE_OPERATION_OK && runtime->yunyi) {
      runtime->product_mode = static_cast<ArticoreControlMode>(mode);
    }
    g_last_error = result == ARTICORE_OPERATION_OK
        ? "ok" : checked(runtime).health().last_operation_error;
    return result;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
}

int32_t native_runtime_clear_faults(
    YunyiRuntimeCoreState* runtime) {
  try {
    const auto result = checked(runtime).clear_faults();
    g_last_error = result == ARTICORE_OPERATION_OK
        ? "ok" : checked(runtime).health().last_operation_error;
    return result;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
}

int32_t native_runtime_set_zero(
    YunyiRuntimeCoreState* runtime) {
  try {
    const auto result = checked(runtime).set_zero();
    g_last_error = result == ARTICORE_OPERATION_OK
        ? "ok" : checked(runtime).health().last_operation_error;
    return result;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
}

int32_t native_runtime_set_joint_pv(
    YunyiRuntimeCoreState* runtime, const float* positions, uint32_t count,
    float speed_percent) {
  try {
    if (!std::isfinite(speed_percent) || speed_percent < 1.0f ||
        speed_percent > 100.0f) {
      throw std::invalid_argument(
          "PV command speed must be finite and within 1..100");
    }
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "PV joint command requires product PV mode");
    }
    install_product_joint_positions(
        runtime, positions, count, speed_percent);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT,
        std::string("PV joint command: ") + error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE,
        std::string("PV joint command: ") + error.what());
  }
}

int32_t native_runtime_set_joint_mit(
    YunyiRuntimeCoreState* runtime, const float* positions,
    const float* velocities, const float* kp, const float* kd,
    const float* feedforward_torques, uint32_t count) {
  try {
    auto& product = checked_yunyi(runtime);
    require_product_count(count);
    require_finite(positions, count, "positions");
    require_finite(velocities, count, "velocities");
    require_finite(kp, count, "kp");
    require_finite(kd, count, "kd");
    require_finite(
        feedforward_torques, count, "feedforward_torques");
    if (runtime->product_mode != ARTICORE_MODE_MIT) {
      throw std::runtime_error(
          "standard MIT joint command requires product MIT mode");
    }
    std::array<ArticoreMitCommand, ARTICORE_PRODUCT_DUAL_ARM_DOF> commands{};
    for (uint32_t i = 0; i < count; ++i) {
      const auto& joint = product.joints[i];
      validate_product_position(joint, positions[i], i);
      if (std::fabs(velocities[i]) > joint.velocity_limit) {
        throw std::invalid_argument("velocity exceeds product joint limit");
      }
      if (std::fabs(feedforward_torques[i]) > joint.torque_limit) {
        throw std::invalid_argument("torque exceeds product joint limit");
      }
      if (kp[i] < 0.0f || kp[i] > 500.0f ||
          kd[i] < 0.0f || kd[i] > 5.0f) {
        throw std::invalid_argument("MIT gain exceeds protocol limits");
      }
      commands[i] = ArticoreMitCommand{
          joint.motor,
          joint.direction * positions[i],
          joint.direction * velocities[i] * joint.velocity_command_scale,
          kp[i], kd[i],
          joint.direction * feedforward_torques[i] *
              joint.torque_command_scale};
    }
    checked(runtime).submit_mit(commands.data(), count);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT,
        std::string("standard MIT joint command: ") + error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE,
        std::string("standard MIT joint command: ") + error.what());
  }
}

int32_t native_runtime_set_joint_mit_fast(
    YunyiRuntimeCoreState* runtime, const float* positions, uint32_t count,
    float speed_percent) {
  try {
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_MIT) {
      throw std::runtime_error(
          "fast MIT joint command requires product MIT mode");
    }
    install_product_joint_positions(
        runtime, positions, count, speed_percent);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT,
        std::string("fast MIT joint command: ") + error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE,
        std::string("fast MIT joint command: ") + error.what());
  }
}

int32_t native_runtime_set_speed_percent(
    YunyiRuntimeCoreState* runtime, float speed_percent) {
  try {
    if (!std::isfinite(speed_percent) || speed_percent < 1.0f ||
        speed_percent > 100.0f) {
      throw std::invalid_argument(
          "speed_percent must be finite and within 1..100");
    }
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "speed percentage setting is available only in product PV mode");
    }
    std::lock_guard<std::mutex> lock(runtime->product_pv_limits_mutex);
    auto limits = effective_product_pv_limits(
        speed_percent, runtime->product_pv_max_velocity,
        runtime->product_pv_max_acceleration);
    checked(runtime).update_joint_pv_profile_limits(
        limits.first, limits.second);
    runtime->product_speed_percent = speed_percent;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE, error.what());
  }
}

int32_t native_runtime_get_speed_percent(
    YunyiRuntimeCoreState* runtime, float* speed_percent) {
  if (!speed_percent) {
    g_last_error = "speed_percent output is null";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "speed percentage setting is available only in product PV mode");
    }
    std::lock_guard<std::mutex> lock(runtime->product_pv_limits_mutex);
    *speed_percent = runtime->product_speed_percent;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_set_max_speed(
    YunyiRuntimeCoreState* runtime, float max_speed_rad_s) {
  try {
    const float validated = require_optional_pv_limit(
        max_speed_rad_s,
        articore::kYunyiOrdinaryPvMaximumConfigurableVelocity,
        "ordinary PV maximum speed in rad/s");
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "maximum speed setting is available only in product PV mode");
    }
    std::lock_guard<std::mutex> lock(runtime->product_pv_limits_mutex);
    auto limits = effective_product_pv_limits(
        runtime->product_speed_percent, validated,
        runtime->product_pv_max_acceleration);
    checked(runtime).update_joint_pv_profile_limits(
        limits.first, limits.second);
    runtime->product_pv_max_velocity = validated;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE, error.what());
  }
}

int32_t native_runtime_get_max_speed(
    YunyiRuntimeCoreState* runtime, float* max_speed_rad_s) {
  if (!max_speed_rad_s) {
    g_last_error = "max_speed_rad_s output is null";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "maximum speed setting is available only in product PV mode");
    }
    std::lock_guard<std::mutex> lock(runtime->product_pv_limits_mutex);
    *max_speed_rad_s = runtime->product_pv_max_velocity;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_set_max_acceleration(
    YunyiRuntimeCoreState* runtime, float max_acceleration_rad_s2) {
  try {
    const float validated = require_optional_pv_limit(
        max_acceleration_rad_s2,
        articore::kYunyiOrdinaryPvMaximumConfigurableAcceleration,
        "ordinary PV maximum acceleration in rad/s^2");
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "maximum acceleration setting is available only in product PV mode");
    }
    std::lock_guard<std::mutex> lock(runtime->product_pv_limits_mutex);
    auto limits = effective_product_pv_limits(
        runtime->product_speed_percent,
        runtime->product_pv_max_velocity, validated);
    checked(runtime).update_joint_pv_profile_limits(
        limits.first, limits.second);
    runtime->product_pv_max_acceleration = validated;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE, error.what());
  }
}

int32_t native_runtime_get_max_acceleration(
    YunyiRuntimeCoreState* runtime, float* max_acceleration_rad_s2) {
  if (!max_acceleration_rad_s2) {
    g_last_error = "max_acceleration_rad_s2 output is null";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "maximum acceleration setting is available only in product PV mode");
    }
    std::lock_guard<std::mutex> lock(runtime->product_pv_limits_mutex);
    *max_acceleration_rad_s2 = runtime->product_pv_max_acceleration;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_move_pose(
    YunyiRuntimeCoreState* runtime, uint32_t side, const float* target_pose) {
  uint64_t ignored_motion_id = 0;
  return move_linear_trajectory_impl(
      runtime, side, nullptr, target_pose, &ignored_motion_id, false,
      ARTICORE_OPERATION_MOVE_POSE);
}

int32_t native_runtime_move_linear(
    YunyiRuntimeCoreState* runtime, uint32_t side, const float* start_pose,
    const float* end_pose) {
  uint64_t ignored_motion_id = 0;
  return move_linear_trajectory_impl(
      runtime, side, start_pose, end_pose, &ignored_motion_id, false);
}

int32_t native_runtime_move_circular(
    YunyiRuntimeCoreState* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose) {
  uint64_t ignored_motion_id = 0;
  return move_circular_trajectory_impl(
      runtime, side, start_pose, via_pose, end_pose, &ignored_motion_id,
      false);
}

int32_t native_runtime_stop_motion(
    YunyiRuntimeCoreState* runtime) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    std::lock_guard<std::mutex> motion_lock(runtime->motion_mutex);
    checked(runtime).cancel_all_motions();
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_STOP_MOTION, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_STOP_MOTION,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_solve_ik(
    YunyiRuntimeCoreState* runtime, const float* left_target_pose,
    const float* right_target_pose, float* positions, uint32_t count) {
  return solve_product_ik_impl(
      runtime, left_target_pose, right_target_pose, positions, count);
}

 int32_t native_runtime_get_state(
    YunyiRuntimeCoreState* runtime, ArticoreProductState* state) {
  if (!state || state->struct_size != sizeof(*state)) {
    g_last_error = "product state output is null or too small";
    return -1;
  }
  try {
    auto& product = checked_yunyi(runtime);
    auto& safety = checked(runtime);
    const uint32_t caller_size = state->struct_size;
    ArticoreProductState output{};
    output.struct_size = caller_size;
    output.has_grippers =
        product.gripper_present[0] || product.gripper_present[1] ? 1 : 0;
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    output.left_gripper_mos_temperature = unavailable;
    output.left_gripper_rotor_temperature = unavailable;
    output.right_gripper_mos_temperature = unavailable;
    output.right_gripper_rotor_temperature = unavailable;
    output.left_gripper_opening = unavailable;
    output.right_gripper_opening = unavailable;
    output.motion_arrived = safety.motion_arrived() ? 1 : 0;
    uint64_t maximum_age = 0;
    uint64_t sequence = std::numeric_limits<uint64_t>::max();
    bool complete_timing = true;
    const uint64_t fresh_limit = safety.feedback_max_age_ns();

    for (uint32_t i = 0; i < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++i) {
      const auto& joint = product.joints[i];
      ArticoreMotorState motor{};
      ArticoreFeedbackStats stats{};
      const bool cached = articore::read_yunyi_motor_state(
          joint.motor, motor, stats);
      auto& arm = i < ARTICORE_PRODUCT_ARM_DOF ? output.left : output.right;
      const uint32_t index = i % ARTICORE_PRODUCT_ARM_DOF;
      const uint32_t bit = 1U << index;
      if (cached && motor.has_value) {
        arm.positions[index] = joint.direction * motor.pos;
        arm.velocities[index] = joint.direction * motor.vel *
                                joint.velocity_feedback_scale;
        arm.torques[index] = joint.direction * motor.torq *
                             joint.torque_feedback_scale;
      } else {
        arm.positions[index] = unavailable;
        arm.velocities[index] = unavailable;
        arm.torques[index] = unavailable;
      }
      const bool feedback_present =
          cached && motor.has_value && stats.has_feedback;
      const bool fresh = feedback_present && stats.age_ns <= fresh_limit;
      const bool power_valid = fresh && motor.status_code <= 1;
      if (power_valid) {
        arm.enabled_valid_mask |= bit;
        if (motor.status_code == 1) arm.enabled_mask |= bit;
      }
      const bool temperature_valid =
          fresh && std::isfinite(motor.t_mos) && std::isfinite(motor.t_rotor);
      if (temperature_valid) {
        arm.mos_temperatures[index] = motor.t_mos;
        arm.rotor_temperatures[index] = motor.t_rotor;
        arm.temperature_valid_mask |= bit;
      } else {
        arm.mos_temperatures[index] = unavailable;
        arm.rotor_temperatures[index] = unavailable;
      }
      if (feedback_present) {
        maximum_age = std::max(maximum_age, stats.age_ns);
        sequence = std::min(sequence, stats.update_count);
      } else {
        complete_timing = false;
      }
    }

    if (product.gripper_present[0] || product.gripper_present[1]) {
      const auto health = safety.health();
      for (uint32_t side = 0; side < 2; ++side) {
        if (!product.gripper_present[side]) continue;
        ArticoreMotorState motor{};
        ArticoreFeedbackStats stats{};
        const bool cached = articore::read_yunyi_motor_state(
            product.grippers[side], motor, stats);
        const bool feedback_present =
            cached && motor.has_value && stats.has_feedback;
        const bool fresh = feedback_present && stats.age_ns <= fresh_limit;
        const bool power_valid = fresh && motor.status_code <= 1;
        const bool temperature_valid =
            fresh && std::isfinite(motor.t_mos) && std::isfinite(motor.t_rotor);
        if (side == 0) {
          output.left_gripper_available = 1;
          output.left_gripper_feedback_valid = fresh ? 1 : 0;
          output.left_gripper_level = safety.gripper_force_level(side);
          output.left_gripper_enabled_valid = power_valid ? 1 : 0;
          output.left_gripper_enabled =
              power_valid && motor.status_code == 1 ? 1 : 0;
          output.left_gripper_temperature_valid = temperature_valid ? 1 : 0;
          if (temperature_valid) {
            output.left_gripper_mos_temperature = motor.t_mos;
            output.left_gripper_rotor_temperature = motor.t_rotor;
          }
        } else {
          output.right_gripper_available = 1;
          output.right_gripper_feedback_valid = fresh ? 1 : 0;
          output.right_gripper_level = safety.gripper_force_level(side);
          output.right_gripper_enabled_valid = power_valid ? 1 : 0;
          output.right_gripper_enabled =
              power_valid && motor.status_code == 1 ? 1 : 0;
          output.right_gripper_temperature_valid = temperature_valid ? 1 : 0;
          if (temperature_valid) {
            output.right_gripper_mos_temperature = motor.t_mos;
            output.right_gripper_rotor_temperature = motor.t_rotor;
          }
        }
        for (uint32_t i = 0; i < health.gripper_count && i < 2; ++i) {
          if (health.grippers[i].side != side) continue;
          if (side == 0) {
            output.left_gripper_opening = health.grippers[i].opening;
          } else {
            output.right_gripper_opening = health.grippers[i].opening;
          }
        }
        if (feedback_present) {
          maximum_age = std::max(maximum_age, stats.age_ns);
          sequence = std::min(sequence, stats.update_count);
        } else {
          complete_timing = false;
        }
      }
    }

    if (complete_timing) {
      const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      output.timestamp_ns = static_cast<uint64_t>(now) > maximum_age
          ? static_cast<uint64_t>(now) - maximum_age : 0;
      output.sequence = sequence == std::numeric_limits<uint64_t>::max()
          ? 0 : sequence;
    }
    *state = output;
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return -1;
  }
}

int32_t native_runtime_get_joint_angle_vel_limits(
    YunyiRuntimeCoreState* runtime, ArticoreProductJointAngleVelLimits* limits) {
  if (!limits || limits->struct_size != sizeof(*limits)) {
    g_last_error = "joint angle/velocity limits output is null or too small";
    return -1;
  }
  try {
    const auto& product = checked_yunyi(runtime);
    const uint32_t caller_size = limits->struct_size;
    ArticoreProductJointAngleVelLimits output{};
    output.struct_size = caller_size;
    output.joint_count = ARTICORE_PRODUCT_DUAL_ARM_DOF;
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
      output.lower_angles[index] = product.joints[index].lower;
      output.upper_angles[index] = product.joints[index].upper;
      output.velocity_limits[index] = product.joints[index].velocity_limit;
    }
    *limits = output;
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return -1;
  }
}

int32_t native_runtime_get_pose(
    YunyiRuntimeCoreState* runtime, uint32_t side, ArticoreProductPose* pose) {
  if (!pose || pose->struct_size != sizeof(*pose)) {
    g_last_error = "product pose output is null or too small";
    return -1;
  }
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    g_last_error = "product pose side must be LEFT(0) or RIGHT(1)";
    return -1;
  }
  try {
    auto& product = checked_yunyi(runtime);
    std::array<double, ARTICORE_PRODUCT_ARM_DOF> q{};
    uint64_t maximum_age = 0;
    uint64_t sequence = std::numeric_limits<uint64_t>::max();
    for (uint32_t index = 0; index < ARTICORE_PRODUCT_ARM_DOF; ++index) {
      const auto& joint = product.joints[
          side * ARTICORE_PRODUCT_ARM_DOF + index];
      ArticoreMotorState motor{};
      ArticoreFeedbackStats stats{};
      if (!articore::read_yunyi_motor_state(joint.motor, motor, stats) ||
          !motor.has_value ||
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

int32_t native_runtime_set_tcp_offset(
    YunyiRuntimeCoreState* runtime, const ArticoreTcpOffset* offset) {
  try {
    if (!offset || offset->struct_size != sizeof(*offset)) {
      throw std::invalid_argument("TCP offset input is null or too small");
    }
    if (offset->side != ARTICORE_ROBOT_LEFT &&
        offset->side != ARTICORE_ROBOT_RIGHT) {
      throw std::invalid_argument("TCP offset side must be LEFT(0) or RIGHT(1)");
    }
    std::array<float, ARTICORE_PRODUCT_POSE_DOF> values{};
    for (uint32_t index = 0; index < values.size(); ++index) {
      if (!std::isfinite(offset->values[index])) {
        throw std::invalid_argument("TCP offset contains NaN or Inf");
      }
      values[index] = offset->values[index];
    }

    auto& safety = checked(runtime);
    auto& product = checked_yunyi(runtime);
    const auto health = safety.health();
    if (health.state != ARTICORE_DISCONNECTED &&
        health.state != ARTICORE_READY) {
      throw std::runtime_error(
          "TCP offset can only be changed while disconnected or READY");
    }
    if (health.state == ARTICORE_READY &&
        !health.disable_confirmed) {
      throw std::runtime_error(
          "TCP offset requires confirmed physical disable in READY");
    }

    auto model = std::make_unique<articore::RobotModel>(
        "yunyi_v1_0", offset->side, values);
    {
      std::lock_guard<std::mutex> lock(product.pose_mutexes[offset->side]);
      product.tcp_offsets[offset->side] = values;
      product.pose_models[offset->side] = std::move(model);
    }
    safety.record_operation_result(
        ARTICORE_OPERATION_SET_TCP_OFFSET, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_SET_TCP_OFFSET,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_SET_TCP_OFFSET,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_get_tcp_offset(
    YunyiRuntimeCoreState* runtime, uint32_t side, ArticoreTcpOffset* offset) {
  if (!offset || offset->struct_size != sizeof(*offset)) {
    g_last_error = "TCP offset output is null or too small";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
    g_last_error = "TCP offset side must be LEFT(0) or RIGHT(1)";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    auto& product = checked_yunyi(runtime);
    const uint32_t caller_size = offset->struct_size;
    ArticoreTcpOffset output{};
    output.struct_size = caller_size;
    output.side = side;
    {
      std::lock_guard<std::mutex> lock(product.pose_mutexes[side]);
      std::copy(product.tcp_offsets[side].begin(),
                product.tcp_offsets[side].end(), output.values);
    }
    *offset = output;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_reset_tcp_offset(
    YunyiRuntimeCoreState* runtime, uint32_t side) {
  try {
    auto& product = checked_yunyi(runtime);
    ArticoreTcpOffset offset{};
    offset.struct_size = sizeof(offset);
    offset.side = side;
    const auto values = articore::default_yunyi_tcp_offset(
        product.gripper_present[side]);
    std::copy(values.begin(), values.end(), offset.values);
    return native_runtime_set_tcp_offset(runtime, &offset);
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_set_grippers(
    YunyiRuntimeCoreState* runtime, float left_opening, float right_opening,
    int32_t strength, int32_t mode) {
  return set_product_grippers_impl(
      runtime, left_opening, right_opening, strength,
      ARTICORE_GRIPPER_STRENGTH_MIN, mode,
      "gripper strength must be in the range 0..10");
}

int32_t native_runtime_has_grippers(
    YunyiRuntimeCoreState* runtime, int32_t* has_grippers) {
  return call([&] {
    if (!has_grippers) throw std::invalid_argument("has_grippers is null");
    const auto& product = checked_yunyi(runtime);
    *has_grippers =
        product.gripper_present[0] || product.gripper_present[1] ? 1 : 0;
  });
}

int32_t native_runtime_get_gripper_presence(
    YunyiRuntimeCoreState* runtime, int32_t* left, int32_t* right) {
  return call([&] {
    if (!left || !right) {
      throw std::invalid_argument("gripper presence output is null");
    }
    const auto& product = checked_yunyi(runtime);
    *left = product.gripper_present[0] ? 1 : 0;
    *right = product.gripper_present[1] ? 1 : 0;
  });
}

int32_t native_runtime_enable(YunyiRuntimeCoreState* runtime) {
  try {
    checked_yunyi(runtime);
    auto& safety = checked(runtime);
    safety.enable(runtime->product_mode);
    safety.record_operation_result(ARTICORE_OPERATION_ENABLE,
                                   ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const articore::InvalidRuntimeState& error) {
    if (runtime && runtime->runtime) runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_ENABLE, ARTICORE_OPERATION_INVALID_STATE,
        error.what());
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_ENABLE, ARTICORE_OPERATION_MOTOR_COMMAND,
        error.what());
    g_last_error = error.what();
    return -1;
  }
}

int32_t native_runtime_start_gravity_compensation(
    YunyiRuntimeCoreState* runtime,
    const ArticoreGravityCompensationConfig* config) {
  return call([&] { checked(runtime).start_gravity_compensation(config); });
}

int32_t native_runtime_stop_gravity_compensation(
    YunyiRuntimeCoreState* runtime) {
  return call([&] { checked(runtime).stop_gravity_compensation(); });
}

int32_t
native_runtime_get_gravity_compensation_status(
    YunyiRuntimeCoreState* runtime,
    ArticoreGravityCompensationStatus* status) {
  return call([&] {
    if (!status || status->struct_size != sizeof(*status)) {
      throw std::invalid_argument(
          "gravity compensation status is null or too small");
    }
    const auto size = status->struct_size;
    *status = checked(runtime).gravity_compensation_status();
    status->struct_size = size;
  });
}

int32_t native_runtime_start_bimanual_follow(
    YunyiRuntimeCoreState* runtime, uint32_t leader_side) {
  try {
    checked(runtime).start_bimanual_follow(leader_side);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_START_BIMANUAL_FOLLOW, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_START_BIMANUAL_FOLLOW,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_START_BIMANUAL_FOLLOW,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_stop_bimanual_follow(
    YunyiRuntimeCoreState* runtime) {
  try {
    checked(runtime).stop_bimanual_follow();
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_STOP_BIMANUAL_FOLLOW, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_STOP_BIMANUAL_FOLLOW,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_STOP_BIMANUAL_FOLLOW,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t native_runtime_get_bimanual_follow_status(
    YunyiRuntimeCoreState* runtime, ArticoreBimanualFollowStatus* status) {
  return call([&] {
    if (!status || status->struct_size != sizeof(*status)) {
      throw std::invalid_argument(
          "bimanual follow status is null or too small");
    }
    const auto size = status->struct_size;
    *status = checked(runtime).bimanual_follow_status();
    status->struct_size = size;
  });
}

int32_t native_runtime_disable(YunyiRuntimeCoreState* runtime) {
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

int32_t native_runtime_estop(YunyiRuntimeCoreState* runtime) {
  return call([&] { checked(runtime).estop(); });
}

int32_t native_runtime_recover(YunyiRuntimeCoreState* runtime) {
  return call([&] { checked(runtime).recover(); });
}

int32_t native_runtime_get_health(
    YunyiRuntimeCoreState* runtime, ArticoreSafetyHealth* health) {
  if (!health || health->struct_size != sizeof(*health)) {
    g_last_error = "health output is null or too small";
    return -1;
  }
  return call([&] {
    const auto caller_size = health->struct_size;
    *health = checked(runtime).health();
    health->struct_size = caller_size;
  });
}

const char* native_runtime_last_error() { return g_last_error.c_str(); }

void check_result(int32_t result, const char* operation) {
  if (result == ARTICORE_OPERATION_OK) return;
  throw std::runtime_error(
      std::string(operation) + " failed: " + native_runtime_last_error());
}

}  // namespace

namespace articore {

YunyiRuntimeCore::YunyiRuntimeCore(
    ArticoreControlMode mode, const std::string& left_can_interface,
    const std::string& right_can_interface, bool realtime,
    bool lock_memory, int control_cpu, int can_tx_cpu, int can_rx_cpu,
    int control_priority, int can_tx_priority, int can_rx_priority,
    uint32_t feedback_max_age_ms, uint32_t motor_watchdog_ms,
    uint32_t motor_discovery_timeout_ms,
    uint32_t motor_discovery_retries) {
  YunyiRuntimeCoreState* created = nullptr;
  check_result(
      native_runtime_create_yunyi_configured(
          static_cast<int32_t>(mode), left_can_interface.c_str(),
          right_can_interface.c_str(),
          realtime ? 1 : 0, lock_memory ? 1 : 0, control_cpu, can_tx_cpu,
          can_rx_cpu, control_priority, can_tx_priority, can_rx_priority,
          feedback_max_age_ms, motor_watchdog_ms,
          motor_discovery_timeout_ms, motor_discovery_retries, &created),
      "create configured Yunyi Runtime");
  state_.reset(created);
}

YunyiRuntimeCore::~YunyiRuntimeCore() noexcept { release(); }

YunyiRuntimeCore::YunyiRuntimeCore(YunyiRuntimeCore&& other) noexcept =
    default;

YunyiRuntimeCore& YunyiRuntimeCore::operator=(
    YunyiRuntimeCore&& other) noexcept {
  if (this == &other) return *this;
  release();
  state_ = std::move(other.state_);
  return *this;
}

YunyiRuntimeCoreState* YunyiRuntimeCore::checked() const {
  if (!state_) throw std::runtime_error("Yunyi Runtime core is disconnected");
  return state_.get();
}

void YunyiRuntimeCore::release() noexcept {
  if (!state_) return;
  (void)native_runtime_disconnect(state_.get());
  state_.reset();
}

void YunyiRuntimeCore::connect() {
  check_result(native_runtime_connect(checked()), "connect");
}

void YunyiRuntimeCore::disconnect() {
  if (!state_) return;
  const auto result = native_runtime_disconnect(state_.get());
  state_.reset();
  check_result(result, "disconnect");
}

ArticoreControlMode YunyiRuntimeCore::control_mode() const {
  int32_t mode = 0;
  check_result(
      native_runtime_get_control_mode(checked(), &mode), "get_control_mode");
  return static_cast<ArticoreControlMode>(mode);
}

void YunyiRuntimeCore::configure_mode(ArticoreControlMode mode) {
  check_result(
      native_runtime_configure_mode(checked(), static_cast<int32_t>(mode)),
      "configure_mode");
}

void YunyiRuntimeCore::enable() {
  check_result(native_runtime_enable(checked()), "enable");
}

void YunyiRuntimeCore::disable() {
  check_result(native_runtime_disable(checked()), "disable");
}

void YunyiRuntimeCore::set_zero() {
  check_result(native_runtime_set_zero(checked()), "set_zero");
}

void YunyiRuntimeCore::clear_faults() {
  check_result(native_runtime_clear_faults(checked()), "clear_faults");
}

void YunyiRuntimeCore::estop() {
  check_result(native_runtime_estop(checked()), "estop");
}

void YunyiRuntimeCore::recover() {
  check_result(native_runtime_recover(checked()), "recover");
}

bool YunyiRuntimeCore::has_grippers() const {
  int32_t value = 0;
  check_result(native_runtime_has_grippers(checked(), &value), "has_grippers");
  return value != 0;
}

std::array<bool, 2> YunyiRuntimeCore::gripper_presence() const {
  int32_t left = 0;
  int32_t right = 0;
  check_result(
      native_runtime_get_gripper_presence(checked(), &left, &right),
      "get_gripper_presence");
  return {left != 0, right != 0};
}

void YunyiRuntimeCore::set_joint_pv(
    const std::vector<float>& positions, float speed_percent) {
  check_result(
      native_runtime_set_joint_pv(
          checked(), positions.data(), static_cast<uint32_t>(positions.size()),
          speed_percent),
      "set_joint_pv");
}

void YunyiRuntimeCore::set_joint_mit(
    const std::vector<float>& positions,
    const std::vector<float>& velocities,
    const std::vector<float>& kp,
    const std::vector<float>& kd,
    const std::vector<float>& feedforward_torques) {
  if (velocities.size() != positions.size() || kp.size() != positions.size() ||
      kd.size() != positions.size() ||
      feedforward_torques.size() != positions.size()) {
    throw std::invalid_argument("MIT frame arrays must have the same joint count");
  }
  check_result(
      native_runtime_set_joint_mit(
          checked(), positions.data(), velocities.data(), kp.data(), kd.data(),
          feedforward_torques.data(),
          static_cast<uint32_t>(positions.size())),
      "set_joint_mit");
}

void YunyiRuntimeCore::set_joint_mit_fast(
    const std::vector<float>& positions, float speed_percent) {
  check_result(
      native_runtime_set_joint_mit_fast(
          checked(), positions.data(), static_cast<uint32_t>(positions.size()),
          speed_percent),
      "set_joint_mit_fast");
}

void YunyiRuntimeCore::set_speed_percent(float speed_percent) {
  check_result(
      native_runtime_set_speed_percent(checked(), speed_percent),
      "set_speed_percent");
}

float YunyiRuntimeCore::speed_percent() const {
  float value = 0.0f;
  check_result(
      native_runtime_get_speed_percent(checked(), &value),
      "get_speed_percent");
  return value;
}

void YunyiRuntimeCore::set_max_speed(float max_speed_rad_s) {
  check_result(
      native_runtime_set_max_speed(checked(), max_speed_rad_s),
      "set_max_speed");
}

float YunyiRuntimeCore::max_speed() const {
  float value = 0.0f;
  check_result(
      native_runtime_get_max_speed(checked(), &value), "get_max_speed");
  return value;
}

void YunyiRuntimeCore::set_max_acceleration(float max_acceleration_rad_s2) {
  check_result(
      native_runtime_set_max_acceleration(checked(), max_acceleration_rad_s2),
      "set_max_acceleration");
}

float YunyiRuntimeCore::max_acceleration() const {
  float value = 0.0f;
  check_result(
      native_runtime_get_max_acceleration(checked(), &value),
      "get_max_acceleration");
  return value;
}

std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
YunyiRuntimeCore::solve_ik(
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& left_target_pose,
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& right_target_pose)
    const {
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> positions{};
  check_result(
      native_runtime_solve_ik(
          checked(), left_target_pose.data(), right_target_pose.data(),
          positions.data(), static_cast<uint32_t>(positions.size())),
      "solve_ik");
  return positions;
}

void YunyiRuntimeCore::move_pose(
    uint32_t side,
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& target_pose) {
  check_result(
      native_runtime_move_pose(checked(), side, target_pose.data()),
      "move_pose");
}

void YunyiRuntimeCore::move_linear(
    uint32_t side,
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& end_pose) {
  check_result(
      native_runtime_move_linear(checked(), side, nullptr, end_pose.data()),
      "move_linear");
}

void YunyiRuntimeCore::move_circular(
    uint32_t side,
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& start_pose,
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& via_pose,
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& end_pose) {
  check_result(
      native_runtime_move_circular(
          checked(), side, start_pose.data(), via_pose.data(),
          end_pose.data()),
      "move_circular");
}

void YunyiRuntimeCore::stop_motion() {
  check_result(native_runtime_stop_motion(checked()), "stop_motion");
}

void YunyiRuntimeCore::set_grippers(
    float left_opening, float right_opening, int32_t strength,
    ArticoreGripperMode mode) {
  check_result(
      native_runtime_set_grippers(
          checked(), left_opening, right_opening, strength,
          static_cast<int32_t>(mode)),
      "set_grippers");
}

ArticoreProductState YunyiRuntimeCore::state() const {
  ArticoreProductState value{};
  value.struct_size = sizeof(value);
  check_result(native_runtime_get_state(checked(), &value), "get_state");
  return value;
}

ArticoreProductJointAngleVelLimits
YunyiRuntimeCore::joint_angle_vel_limits() const {
  ArticoreProductJointAngleVelLimits value{};
  value.struct_size = sizeof(value);
  check_result(
      native_runtime_get_joint_angle_vel_limits(checked(), &value),
      "get_joint_angle_vel_limits");
  return value;
}

ArticoreProductPose YunyiRuntimeCore::pose(uint32_t side) const {
  ArticoreProductPose value{};
  value.struct_size = sizeof(value);
  check_result(native_runtime_get_pose(checked(), side, &value), "get_pose");
  return value;
}

void YunyiRuntimeCore::set_tcp_offset(
    uint32_t side,
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& values) {
  ArticoreTcpOffset offset{};
  offset.struct_size = sizeof(offset);
  offset.side = side;
  std::copy(values.begin(), values.end(), offset.values);
  check_result(
      native_runtime_set_tcp_offset(checked(), &offset), "set_tcp_offset");
}

ArticoreTcpOffset YunyiRuntimeCore::tcp_offset(uint32_t side) const {
  ArticoreTcpOffset value{};
  value.struct_size = sizeof(value);
  check_result(
      native_runtime_get_tcp_offset(checked(), side, &value),
      "get_tcp_offset");
  return value;
}

void YunyiRuntimeCore::reset_tcp_offset(uint32_t side) {
  check_result(
      native_runtime_reset_tcp_offset(checked(), side), "reset_tcp_offset");
}

ArticoreSafetyHealth YunyiRuntimeCore::health() const {
  ArticoreSafetyHealth value{};
  value.struct_size = sizeof(value);
  check_result(native_runtime_get_health(checked(), &value), "get_health");
  return value;
}

void YunyiRuntimeCore::start_gravity_compensation(uint32_t transition_ms) {
  ArticoreGravityCompensationConfig config{};
  config.struct_size = sizeof(config);
  config.transition_ms = transition_ms;
  check_result(
      native_runtime_start_gravity_compensation(checked(), &config),
      "start_gravity_compensation");
}

void YunyiRuntimeCore::stop_gravity_compensation() {
  check_result(
      native_runtime_stop_gravity_compensation(checked()),
      "stop_gravity_compensation");
}

ArticoreGravityCompensationStatus
YunyiRuntimeCore::gravity_compensation_status() const {
  ArticoreGravityCompensationStatus value{};
  value.struct_size = sizeof(value);
  check_result(
      native_runtime_get_gravity_compensation_status(checked(), &value),
      "get_gravity_compensation_status");
  return value;
}

void YunyiRuntimeCore::start_bimanual_follow(uint32_t leader_side) {
  check_result(
      native_runtime_start_bimanual_follow(checked(), leader_side),
      "start_bimanual_follow");
}

void YunyiRuntimeCore::stop_bimanual_follow() {
  check_result(
      native_runtime_stop_bimanual_follow(checked()),
      "stop_bimanual_follow");
}

ArticoreBimanualFollowStatus
YunyiRuntimeCore::bimanual_follow_status() const {
  ArticoreBimanualFollowStatus value{};
  value.struct_size = sizeof(value);
  check_result(
      native_runtime_get_bimanual_follow_status(checked(), &value),
      "get_bimanual_follow_status");
  return value;
}

}  // namespace articore
