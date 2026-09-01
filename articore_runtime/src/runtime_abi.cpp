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
#include <vector>

#include "articore/runtime_abi.h"
#include "articore/detail/product_cartesian.hpp"
#include "articore/detail/robot_model.hpp"
#include "articore/detail/runtime.hpp"
#include "articore/detail/yunyi_runtime.hpp"

struct ArticoreRuntime {
  explicit ArticoreRuntime(
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

ArticoreRuntime* create_yunyi_runtime_checked(
    int32_t requested_mode, int32_t with_grippers) {
  if (requested_mode != ARTICORE_MODE_PV &&
      requested_mode != ARTICORE_MODE_MIT) {
    throw std::invalid_argument("unsupported Yunyi control mode");
  }
  if (with_grippers != 0 && with_grippers != 1) {
    throw std::invalid_argument("with_grippers must be 0 or 1");
  }
  auto bundle = articore::create_yunyi_runtime(
      static_cast<ArticoreControlMode>(requested_mode), with_grippers != 0);
  return new ArticoreRuntime(
      std::move(bundle.runtime), std::move(bundle.resources), bundle.mode);
}

articore::SafetyRuntime& checked(ArticoreRuntime* runtime) {
  if (!runtime || !runtime->runtime) throw std::invalid_argument("runtime is null");
  return *runtime->runtime;
}

articore::YunyiRuntimeResources& checked_yunyi(ArticoreRuntime* runtime) {
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

CartesianSpeedScale cartesian_speed_scale(ArticoreRuntime* runtime) {
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
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
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
            : "ordinary MIT speed must be finite and within 0..100");
  }
  float selected_reference_velocity = 0.0f;
  std::unique_lock<std::mutex> pv_limits_lock;
  if (runtime->product_mode == ARTICORE_MODE_MIT && !direct_mit) {
    selected_reference_velocity =
        product.mit_fast_follow_reference_velocity * speed_percent / 100.0f;
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
    ArticoreRuntime* runtime, int32_t code, const std::string& error) {
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
    ArticoreRuntime* runtime, const float* left_target_pose,
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

int32_t set_pose_impl(
    ArticoreRuntime* runtime, const float* left_target_pose,
    const float* right_target_pose, float speed_percent) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    const float minimum_speed =
        runtime->product_mode == ARTICORE_MODE_PV ? 1.0f : 0.0f;
    if (!std::isfinite(speed_percent) || speed_percent < minimum_speed ||
        speed_percent > 100.0f) {
      throw std::invalid_argument(
          runtime->product_mode == ARTICORE_MODE_PV
              ? "set_pose PV speed_percent must be finite and within 1..100"
              : "set_pose MIT speed_percent must be finite and within 0..100");
    }
    std::lock_guard<std::mutex> motion_lock(runtime->motion_mutex);
    const auto ik_deadline = std::chrono::steady_clock::now() +
        articore::kYunyiProductIkBudget;
    auto& safety = checked(runtime);
    auto& product = checked_yunyi(runtime);
    const auto joints = articore::product_cartesian_joints(product);
    articore::NativeTrajectorySample reference;
    CommandPlanningScope planning(safety);
    {
      auto transaction = safety.begin_command_transaction();
      planning.begin(transaction);
      reference = safety.planned_arm_sample(joints, transaction);
    }
    if (reference.active) {
      throw std::runtime_error(
          "set_pose cannot start while a linear or circular motion is active");
    }
    const auto target =
        articore::solve_dual_point_to_point_targets_from_reference(
            product, runtime->product_mode, reference,
            left_target_pose, right_target_pose, ik_deadline);
    std::unique_lock<std::mutex> pv_limits_transaction;
    if (runtime->product_mode == ARTICORE_MODE_PV) {
      pv_limits_transaction =
          std::unique_lock<std::mutex>(runtime->product_pv_limits_mutex);
    }
    auto transaction = safety.begin_command_transaction();
    install_product_joint_positions(
        runtime, target.data(), static_cast<uint32_t>(target.size()),
        speed_percent, &transaction, planning.token(),
        runtime->product_mode == ARTICORE_MODE_MIT,
        runtime->product_mode == ARTICORE_MODE_PV
            ? &pv_limits_transaction : nullptr);
    safety.record_operation_result(
        ARTICORE_OPERATION_SET_POSE, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_SET_POSE,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_SET_POSE,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t move_linear_trajectory_impl(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* end_pose, uint64_t* motion_id) {
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
      planning.begin(snapshot, true);
      reference = safety.planned_trajectory_tail_sample(joints, snapshot);
    }
    const auto speed = cartesian_speed_scale(runtime);
    auto plan = articore::build_linear_trajectory_plan_from_reference(
        product, runtime->product_mode, side, reference, start_pose, end_pose,
        speed.reference_acceleration, speed.reference_velocity);

    auto transaction = safety.begin_command_transaction();
    const auto current =
        safety.planned_trajectory_tail_sample(joints, transaction);
    articore::require_unchanged_planned_reference(
        reference, current, "linear");

    const uint64_t new_id = safety.start_trajectory(
        plan.trajectory, 0, &transaction, true, planning.token());
    *motion_id = new_id;
    safety.record_operation_result(
        ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t move_linear_path_trajectory_impl(
    ArticoreRuntime* runtime, uint32_t side, const float* poses,
    uint32_t pose_count, uint64_t* motion_id) {
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
      planning.begin(snapshot, true);
      reference = safety.planned_trajectory_tail_sample(joints, snapshot);
    }
    const auto speed = cartesian_speed_scale(runtime);
    auto plan = articore::build_linear_path_trajectory_plan_from_reference(
        product, runtime->product_mode, side, reference, poses, pose_count,
        speed.reference_acceleration, speed.reference_velocity);

    auto transaction = safety.begin_command_transaction();
    const auto current =
        safety.planned_trajectory_tail_sample(joints, transaction);
    articore::require_unchanged_planned_reference(
        reference, current, "linear path");
    const uint64_t new_id = safety.start_trajectory(
        plan.trajectory, 0, &transaction, true, planning.token());
    *motion_id = new_id;
    safety.record_operation_result(
        ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t move_circular_trajectory_impl(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, uint64_t* motion_id) {
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
      planning.begin(snapshot, true);
      reference = safety.planned_trajectory_tail_sample(joints, snapshot);
    }
    const auto speed = cartesian_speed_scale(runtime);
    auto plan = articore::build_circular_trajectory_plan_from_reference(
        product, runtime->product_mode, side, reference, start_pose,
        via_pose, end_pose,
        speed.reference_acceleration, speed.reference_velocity);

    auto transaction = safety.begin_command_transaction();
    const auto current =
        safety.planned_trajectory_tail_sample(joints, transaction);
    articore::require_unchanged_planned_reference(
        reference, current, "circular");
    const uint64_t new_id = safety.start_trajectory(
        plan.trajectory, 0, &transaction, true, planning.token());

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

articore::RobotModel& checked(ArticoreRobotModel* model) {
  if (!model || !model->model) {
    throw std::invalid_argument("robot model is null");
  }
  return *model->model;
}

template <std::size_t Size>
void copy_abi_text(char (&target)[Size], const std::string& value) {
  const auto count = std::min(value.size(), Size - 1);
  std::memcpy(target, value.data(), count);
  target[count] = '\0';
}

int32_t motor_power_batch(ArticoreRuntime* runtime,
                          const char* const* roles,
                          uint32_t count,
                          bool enabled,
                          ArticoreMotorPowerReport* report) {
  const auto operation = enabled ? ARTICORE_OPERATION_ENABLE
                                 : ARTICORE_OPERATION_DISABLE;
  if (!report) {
    g_last_error = "motor power report is null";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  if (report->struct_size != sizeof(ArticoreMotorPowerReport)) {
    g_last_error = "motor power report struct_size does not match";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  ArticoreMotorPowerReport failure{};
  failure.struct_size = sizeof(failure);
  failure.requested_enabled = enabled ? 1 : 0;
  failure.requested_count = count;
  std::vector<std::string> names;
  try {
    if (count != 0 && !roles) {
      throw std::invalid_argument("motor roles array is null");
    }
    names.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
      if (!roles[i]) {
        throw std::invalid_argument(
            "motor role at index " + std::to_string(i) + " is null");
      }
      names.emplace_back(roles[i]);
    }
    *report = checked(runtime).set_motor_power_batch(names, enabled);
    std::vector<std::string> failed;
    for (uint32_t i = 0; i < report->motor_count; ++i) {
      if (!report->motors[i].confirmed) {
        failed.emplace_back(report->motors[i].role);
      }
    }
    const auto code = report->success ? ARTICORE_OPERATION_OK
                                      : ARTICORE_OPERATION_VERIFICATION;
    checked(runtime).record_operation_result(
        operation, code, report->error, failed);
    g_last_error = report->success ? "ok" : report->error;
    return code;
  } catch (const std::invalid_argument& error) {
    copy_abi_text(failure.error, error.what());
    *report = failure;
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          operation, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what(), names);
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    copy_abi_text(failure.error, error.what());
    *report = failure;
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          operation, ARTICORE_OPERATION_MOTOR_COMMAND, error.what(), names);
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_MOTOR_COMMAND;
  }
}

int32_t set_product_grippers_impl(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
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
    if (!product.with_grippers) {
      checked(runtime).record_operation_result(
          ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
      g_last_error = "ok";
      return 0;
    }
    const float openings[2] = {
        std::clamp(left_opening, 0.0f, 1000.0f),
        std::clamp(right_opening, 0.0f, 1000.0f)};
    ArticoreGripperCommand commands[2]{};
    for (uint32_t side = 0; side < 2; ++side) {
      commands[side].struct_size = sizeof(ArticoreGripperCommand);
      commands[side].motor = product.grippers[side];
      commands[side].opening = openings[side];
      commands[side].speed = 1000.0f;
      commands[side].force_level = strength;
    }
    checked(runtime).set_gripper_commands(commands, 2, mode);
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

extern "C" {

ARTICORE_RUNTIME_API uint32_t articore_runtime_abi_version(void) {
  return (13U << 16);
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_control_mode(
    ArticoreRuntime* runtime, int32_t* mode) {
  return call([&] {
    if (!mode) throw std::invalid_argument("control mode output is null");
    checked(runtime);
    *mode = runtime->yunyi
        ? static_cast<int32_t>(runtime->product_mode)
        : static_cast<int32_t>(runtime->runtime->control_mode());
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_create_yunyi(
    int32_t requested_mode, int32_t with_grippers,
    ArticoreRuntime** runtime) {
  if (!runtime) {
    g_last_error = "runtime output is null";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  *runtime = nullptr;
  try {
    *runtime = create_yunyi_runtime_checked(requested_mode, with_grippers);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  } catch (...) {
    g_last_error = "unknown Yunyi Runtime creation error";
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

ARTICORE_RUNTIME_API void articore_runtime_free(ArticoreRuntime* runtime) {
  delete runtime;
}

ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_disconnect(
    ArticoreRuntime* runtime) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_configure_mode(
    ArticoreRuntime* runtime, int32_t mode) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_clear_faults(
    ArticoreRuntime* runtime) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_set_zero(
    ArticoreRuntime* runtime) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_pv(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
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

ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
    float speed_percent) {
  try {
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_MIT) {
      throw std::runtime_error(
          "MIT joint command requires product MIT mode");
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
        std::string("MIT joint command: ") + error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE,
        std::string("MIT joint command: ") + error.what());
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit_direct(
    ArticoreRuntime* runtime, const float* positions, uint32_t count) {
  try {
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_MIT) {
      throw std::runtime_error(
          "direct MIT joint command requires product MIT mode");
    }
    install_product_joint_positions(
        runtime, positions, count, 0.0f, nullptr, 0, true);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT,
        std::string("direct MIT joint command: ") + error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE,
        std::string("direct MIT joint command: ") + error.what());
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit_fast_follow(
    ArticoreRuntime* runtime, const float* positions, uint32_t count) {
  try {
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_MIT) {
      throw std::runtime_error(
          "fast-follow MIT joint command requires product MIT mode");
    }
    install_product_joint_positions(
        runtime, positions, count, 100.0f);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return 0;
  } catch (const std::invalid_argument& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT,
        std::string("fast-follow MIT joint command: ") + error.what());
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_STATE,
        std::string("fast-follow MIT joint command: ") + error.what());
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_speed_percent(
    ArticoreRuntime* runtime, float speed_percent) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_speed_percent(
    ArticoreRuntime* runtime, float* speed_percent) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_set_max_speed(
    ArticoreRuntime* runtime, float max_speed_rad_s) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_max_speed(
    ArticoreRuntime* runtime, float* max_speed_rad_s) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_set_max_acceleration(
    ArticoreRuntime* runtime, float max_acceleration_rad_s2) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_max_acceleration(
    ArticoreRuntime* runtime, float* max_acceleration_rad_s2) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocities, const float* feedforward_torques,
    const float* kp, const float* kd, uint32_t count) {
  try {
    auto& product = checked_yunyi(runtime);
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_motion_status(
    ArticoreRuntime* runtime, uint64_t motion_id,
    ArticoreMotionStatus* status) {
  if (!status || status->struct_size != sizeof(*status) || motion_id == 0) {
    g_last_error = "motion status requires a non-zero id and exact output";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    const uint32_t caller_size = status->struct_size;
    auto output = checked(runtime).motion_status(motion_id);
    if (output.state == ARTICORE_MOTION_IDLE) {
      throw std::invalid_argument(output.error);
    }
    output.struct_size = caller_size;
    *status = output;
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

ARTICORE_RUNTIME_API int32_t articore_runtime_set_pose(
    ArticoreRuntime* runtime, const float* left_target_pose,
    const float* right_target_pose, float speed_percent) {
  return set_pose_impl(
      runtime, left_target_pose, right_target_pose, speed_percent);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_solve_ik(
    ArticoreRuntime* runtime, const float* left_target_pose,
    const float* right_target_pose, float* positions, uint32_t count) {
  return solve_product_ik_impl(
      runtime, left_target_pose, right_target_pose, positions, count);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_move_linear_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* end_pose, uint64_t* motion_id) {
  return move_linear_trajectory_impl(
      runtime, side, start_pose, end_pose, motion_id);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_move_linear_path_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* poses,
    uint32_t pose_count, uint64_t* motion_id) {
  return move_linear_path_trajectory_impl(
      runtime, side, poses, pose_count, motion_id);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_move_circular_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, uint64_t* motion_id) {
  return move_circular_trajectory_impl(
      runtime, side, start_pose, via_pose, end_pose, motion_id);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_motion(
    ArticoreRuntime* runtime, uint64_t motion_id) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    std::lock_guard<std::mutex> motion_lock(runtime->motion_mutex);
    checked(runtime).cancel_motion(motion_id);
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_CANCEL_MOTION, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_CANCEL_MOTION,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_CANCEL_MOTION,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_all_motions(
    ArticoreRuntime* runtime) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    std::lock_guard<std::mutex> motion_lock(runtime->motion_mutex);
    checked(runtime).cancel_all_motions();
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_CANCEL_ALL_MOTIONS, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_CANCEL_ALL_MOTIONS,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

 ARTICORE_RUNTIME_API int32_t articore_runtime_get_state(
    ArticoreRuntime* runtime, ArticoreProductState* state) {
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
    output.has_grippers = product.with_grippers ? 1 : 0;
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    output.left_gripper_mos_temperature = unavailable;
    output.left_gripper_rotor_temperature = unavailable;
    output.right_gripper_mos_temperature = unavailable;
    output.right_gripper_rotor_temperature = unavailable;
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

    if (product.with_grippers) {
      const auto health = safety.health();
      for (uint32_t side = 0; side < 2; ++side) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_joint_angle_vel_limits(
    ArticoreRuntime* runtime, ArticoreProductJointAngleVelLimits* limits) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_pose(
    ArticoreRuntime* runtime, uint32_t side, ArticoreProductPose* pose) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_set_tcp_offset(
    ArticoreRuntime* runtime, const ArticoreTcpOffset* offset) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_tcp_offset(
    ArticoreRuntime* runtime, uint32_t side, ArticoreTcpOffset* offset) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_reset_tcp_offset(
    ArticoreRuntime* runtime, uint32_t side) {
  try {
    auto& product = checked_yunyi(runtime);
    ArticoreTcpOffset offset{};
    offset.struct_size = sizeof(offset);
    offset.side = side;
    const auto values = articore::default_yunyi_tcp_offset(
        product.with_grippers);
    std::copy(values.begin(), values.end(), offset.values);
    return articore_runtime_set_tcp_offset(runtime, &offset);
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_grippers(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
    int32_t strength, int32_t mode) {
  return set_product_grippers_impl(
      runtime, left_opening, right_opening, strength,
      ARTICORE_GRIPPER_STRENGTH_MIN, mode,
      "gripper strength must be in the range 0..10");
}

ARTICORE_RUNTIME_API int32_t articore_runtime_has_grippers(
    ArticoreRuntime* runtime, int32_t* has_grippers) {
  return call([&] {
    if (!has_grippers) throw std::invalid_argument("has_grippers is null");
    *has_grippers = checked_yunyi(runtime).with_grippers ? 1 : 0;
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_enable(ArticoreRuntime* runtime) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_enable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report) {
  return motor_power_batch(runtime, roles, count, true, report);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_disable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report) {
  return motor_power_batch(runtime, roles, count, false, report);
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
    if (!status || status->struct_size != sizeof(*status)) {
      throw std::invalid_argument(
          "gravity compensation status is null or too small");
    }
    const auto size = status->struct_size;
    *status = checked(runtime).gravity_compensation_status();
    status->struct_size = size;
  });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_start_bimanual_follow(
    ArticoreRuntime* runtime, uint32_t leader_side) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_stop_bimanual_follow(
    ArticoreRuntime* runtime) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_bimanual_follow_status(
    ArticoreRuntime* runtime, ArticoreBimanualFollowStatus* status) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_estop(ArticoreRuntime* runtime) {
  return call([&] { checked(runtime).estop(); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_recover(ArticoreRuntime* runtime) {
  return call([&] { checked(runtime).recover(); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_health(
    ArticoreRuntime* runtime, ArticoreSafetyHealth* health) {
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_mit_torque_limit_stats(
    ArticoreRuntime* runtime, ArticoreMitTorqueLimitStats* stats) {
  return call([&] {
    if (!stats) throw std::invalid_argument("MIT torque limit stats are null");
    if (stats->struct_size != sizeof(ArticoreMitTorqueLimitStats)) {
      throw std::invalid_argument(
          "MIT torque limit stats struct_size does not match");
    }
    *stats = checked(runtime).mit_torque_limit_stats();
  });
}

ARTICORE_RUNTIME_API const char* articore_runtime_last_error(void) {
  return g_last_error.c_str();
}

}  // extern "C"
