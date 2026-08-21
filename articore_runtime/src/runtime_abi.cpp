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
  std::mutex product_speed_mutex;
  float product_max_speed_percent = articore::kYunyiDefaultSpeedPercent;
  std::mutex move_pose_mutex;
  uint64_t move_pose_id = 0;
  uint64_t superseded_move_pose_id = 0;
  uint32_t move_pose_side = ARTICORE_ROBOT_LEFT;
  int32_t cartesian_interpolation = ARTICORE_CARTESIAN_POINT_TO_POINT;
  float move_pose_speed_percent = 0.0f;
  std::array<float, ARTICORE_PRODUCT_POSE_DOF> move_pose_target{};
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

void validate_product_position(
    const articore::YunyiRuntimeResources::Joint& joint, float position,
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

int32_t record_move_pose_error(
    ArticoreRuntime* runtime, int32_t code, const std::string& error) {
  if (runtime && runtime->runtime) {
    runtime->runtime->record_operation_result(
        ARTICORE_OPERATION_MOVE_POSE, code, error);
  }
  g_last_error = error;
  return code;
}

int32_t move_cartesian_impl(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent, ArticoreCartesianInterpolation interpolation,
    uint64_t* motion_id) {
  const auto operation = interpolation == ARTICORE_CARTESIAN_LINEAR
      ? ARTICORE_OPERATION_MOVE_LINEAR
      : ARTICORE_OPERATION_MOVE_POSE;
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    if (!motion_id) throw std::invalid_argument("motion_id output is null");
    std::lock_guard<std::mutex> motion_lock(runtime->move_pose_mutex);
    auto plan = articore::build_cartesian_plan(
        checked(runtime), checked_yunyi(runtime), runtime->product_mode,
        side, target_pose, speed_percent, interpolation);

    uint64_t new_id = 0;
    for (uint32_t attempt = 0; attempt < 12; ++attempt) {
      try {
        new_id = checked(runtime).start_trajectory(
            plan.trajectory, plan.replace_trajectory_id);
        break;
      } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        if (message.find("inside the segment") == std::string::npos ||
            plan.trajectory.waypoints.back().time_s >= 60.0) {
          throw;
        }
        for (auto& waypoint : plan.trajectory.waypoints) {
          waypoint.time_s *= 1.35;
        }
      }
    }
    if (new_id == 0) {
      throw std::invalid_argument(
          "Cartesian motion could not satisfy product limits at any safe duration");
    }

    runtime->superseded_move_pose_id = plan.replace_trajectory_id;
    runtime->move_pose_id = new_id;
    runtime->move_pose_side = side;
    runtime->cartesian_interpolation = interpolation;
    runtime->move_pose_speed_percent = speed_percent;
    std::copy(target_pose, target_pose + ARTICORE_PRODUCT_POSE_DOF,
              runtime->move_pose_target.begin());
    *motion_id = new_id;
    checked(runtime).record_operation_result(operation, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          operation, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          operation, ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t move_circular_impl(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, float speed_percent,
    uint64_t* motion_id) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    if (!motion_id) throw std::invalid_argument("motion_id output is null");
    std::lock_guard<std::mutex> motion_lock(runtime->move_pose_mutex);
    auto plan = articore::build_circular_plan(
        checked(runtime), checked_yunyi(runtime), runtime->product_mode,
        side, start_pose, via_pose, end_pose, speed_percent);

    uint64_t new_id = 0;
    for (uint32_t attempt = 0; attempt < 12; ++attempt) {
      try {
        new_id = checked(runtime).start_trajectory(
            plan.trajectory, plan.replace_trajectory_id);
        break;
      } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        if (message.find("inside the segment") == std::string::npos ||
            plan.trajectory.waypoints.back().time_s >= 60.0) {
          throw;
        }
        for (auto& waypoint : plan.trajectory.waypoints) {
          waypoint.time_s *= 1.35;
        }
      }
    }
    if (new_id == 0) {
      throw std::invalid_argument(
          "circular motion could not satisfy product limits at any safe duration");
    }

    runtime->superseded_move_pose_id = plan.replace_trajectory_id;
    runtime->move_pose_id = new_id;
    runtime->move_pose_side = side;
    runtime->cartesian_interpolation = ARTICORE_CARTESIAN_CIRCULAR;
    runtime->move_pose_speed_percent = speed_percent;
    std::copy(end_pose, end_pose + ARTICORE_PRODUCT_POSE_DOF,
              runtime->move_pose_target.begin());
    *motion_id = new_id;
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_MOVE_CIRCULAR, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_CIRCULAR,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_CIRCULAR,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

int32_t move_circular_v2_impl(
    ArticoreRuntime* runtime, uint32_t side, const float* via_pose,
    const float* end_pose, float speed_percent, uint64_t* motion_id) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    if (!motion_id) throw std::invalid_argument("motion_id output is null");
    std::lock_guard<std::mutex> motion_lock(runtime->move_pose_mutex);
    auto& safety = checked(runtime);
    auto& product = checked_yunyi(runtime);

    // The command transaction is the same barrier used by the native control
    // worker. The circular start is therefore sampled from the current
    // trajectory/mailbox reference and the replacement is installed without
    // allowing another control sample to advance between those two events.
    auto transaction = safety.begin_command_transaction();
    const auto reference = safety.planned_arm_sample(
        articore::product_cartesian_joints(product), transaction);
    auto plan = articore::build_circular_plan_from_reference(
        product, runtime->product_mode, side, reference, via_pose, end_pose,
        speed_percent);

    uint64_t new_id = 0;
    for (uint32_t attempt = 0; attempt < 12; ++attempt) {
      try {
        new_id = safety.start_trajectory(
            plan.trajectory, plan.replace_trajectory_id, &transaction);
        break;
      } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        if (message.find("inside the segment") == std::string::npos ||
            plan.trajectory.waypoints.back().time_s >= 60.0) {
          throw;
        }
        for (auto& waypoint : plan.trajectory.waypoints) {
          waypoint.time_s *= 1.35;
        }
      }
    }
    if (new_id == 0) {
      throw std::invalid_argument(
          "circular motion could not satisfy product limits at any safe duration");
    }

    runtime->superseded_move_pose_id = plan.replace_trajectory_id;
    runtime->move_pose_id = new_id;
    runtime->move_pose_side = side;
    runtime->cartesian_interpolation = ARTICORE_CARTESIAN_CIRCULAR;
    runtime->move_pose_speed_percent = speed_percent;
    std::copy(end_pose, end_pose + ARTICORE_PRODUCT_POSE_DOF,
              runtime->move_pose_target.begin());
    *motion_id = new_id;
    safety.record_operation_result(
        ARTICORE_OPERATION_MOVE_CIRCULAR, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_CIRCULAR,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_MOVE_CIRCULAR,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

articore::RobotModel& checked(ArticoreRobotModel* model) {
  if (!model || !model->model) throw std::invalid_argument("robot model is null");
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
  if (report->struct_size < sizeof(ArticoreMotorPowerReport)) {
    g_last_error = "motor power report struct_size is too small";
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
  return (2U << 16) | 38U;
}

ARTICORE_RUNTIME_API uint64_t articore_runtime_capabilities(void) {
  return ARTICORE_CAP_COMMAND_WATCHDOG |
         ARTICORE_CAP_SAFE_HOLD |
         ARTICORE_CAP_GRIPPER_PROTECTION |
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
         ARTICORE_CAP_BUILTIN_GRIPPER_PRODUCT_PROFILES |
         ARTICORE_CAP_CONNECT_FEEDBACK_BARRIER |
         ARTICORE_CAP_STRUCTURED_CONNECT_REPORT |
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
         ARTICORE_CAP_PRODUCT_POSE |
         ARTICORE_CAP_PARAMETERLESS_ESTOP |
         ARTICORE_CAP_PRODUCT_RECOVERY |
         ARTICORE_CAP_TERMINAL_PRODUCT_DISCONNECT |
         ARTICORE_CAP_YUNYI_DUAL_ARM_RUNTIME |
         ARTICORE_CAP_ATOMIC_MOTOR_POWER_BATCH |
         ARTICORE_CAP_PRODUCT_POWER_STATE_SNAPSHOT |
         ARTICORE_CAP_PRODUCT_QUINTIC_TRAJECTORY |
         ARTICORE_CAP_PRODUCT_GRIPPER_FORCE_10_LEVELS |
         ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE |
         ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE |
         ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10 |
         ARTICORE_CAP_PRODUCT_CARTESIAN_POINT_TO_POINT |
         ARTICORE_CAP_PRODUCT_CARTESIAN_LINEAR |
         ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR |
         ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR_AUTO_START |
         ARTICORE_CAP_PRODUCT_TEMPERATURE_STATE |
         ARTICORE_CAP_LATCHED_ESTOP_POSITION_HOLD |
         ARTICORE_CAP_PRODUCT_JOINT_ANGLE_VEL_LIMITS |
         ARTICORE_CAP_PRODUCT_SPEED_SETTING |
         ARTICORE_CAP_PRODUCT_MAX_SPEED_SETTING |
         ARTICORE_CAP_PRODUCT_TOOL_CENTER_POSE |
         ARTICORE_CAP_PV_MAX_SPEED_ONLY;
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

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create_yunyi(
    int32_t requested_mode, int32_t with_grippers) {
  if (requested_mode != ARTICORE_MODE_PV &&
      requested_mode != ARTICORE_MODE_MIT) {
    g_last_error = "unsupported Yunyi control mode";
    return nullptr;
  }
  if (with_grippers != 0 && with_grippers != 1) {
    g_last_error = "with_grippers must be 0 or 1";
    return nullptr;
  }
  try {
    auto bundle = articore::create_yunyi_runtime(
        static_cast<ArticoreControlMode>(requested_mode), with_grippers != 0);
    g_last_error = "ok";
    return new ArticoreRuntime(
        std::move(bundle.runtime), std::move(bundle.resources), bundle.mode);
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return nullptr;
  }
}

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create_product(
    const char* product_id, int32_t requested_mode, int32_t with_grippers) {
  if (!product_id || std::strcmp(product_id, "yunyi_v1_0") != 0) {
    g_last_error = "only the yunyi_v1_0 dual-arm product is supported";
    return nullptr;
  }
  return articore_runtime_create_yunyi(requested_mode, with_grippers);
}

ARTICORE_RUNTIME_API void articore_runtime_free(ArticoreRuntime* runtime) {
  delete runtime;
}

ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime) {
  try {
    checked(runtime).connect();
    if (runtime->yunyi) {
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
    auto& product = checked_yunyi(runtime);
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

ARTICORE_RUNTIME_API int32_t articore_runtime_set_max_speed(
    ArticoreRuntime* runtime, float max_speed_percent) {
  try {
    if (!std::isfinite(max_speed_percent) || max_speed_percent < 0.0f ||
        max_speed_percent > 100.0f) {
      throw std::invalid_argument(
          "ordinary maximum speed must be finite and within 0..100");
    }
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "maximum speed setting is available only in product PV mode");
    }
    std::lock_guard<std::mutex> lock(runtime->product_speed_mutex);
    checked(runtime).update_joint_position_velocity(
        articore::kYunyiOrdinaryMaximumVelocity *
        max_speed_percent / 100.0f);
    runtime->product_max_speed_percent = max_speed_percent;
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_max_speed(
    ArticoreRuntime* runtime, float* max_speed_percent) {
  if (!max_speed_percent) {
    g_last_error = "max_speed_percent output is null";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    checked_yunyi(runtime);
    if (runtime->product_mode != ARTICORE_MODE_PV) {
      throw std::runtime_error(
          "maximum speed setting is available only in product PV mode");
    }
    std::lock_guard<std::mutex> lock(runtime->product_speed_mutex);
    *max_speed_percent = runtime->product_max_speed_percent;
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_speed(
    ArticoreRuntime* runtime, float speed_percent) {
  try {
    if (!std::isfinite(speed_percent) || speed_percent < 0.0f ||
        speed_percent > 100.0f) {
      throw std::invalid_argument(
          "ordinary speed must be finite and within 0..100");
    }
    checked_yunyi(runtime);
    std::lock_guard<std::mutex> lock(runtime->product_speed_mutex);
    checked(runtime).update_joint_position_velocity(
        articore::kYunyiOrdinaryMaximumVelocity * speed_percent / 100.0f);
    runtime->product_max_speed_percent = speed_percent;
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_speed(
    ArticoreRuntime* runtime, float* speed_percent) {
  if (!speed_percent) {
    g_last_error = "speed_percent output is null";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    checked_yunyi(runtime);
    std::lock_guard<std::mutex> lock(runtime->product_speed_mutex);
    *speed_percent = runtime->product_max_speed_percent;
    g_last_error = "ok";
    return 0;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_positions_v2(
    ArticoreRuntime* runtime, const float* positions, uint32_t count) {
  try {
    checked_yunyi(runtime);
    std::lock_guard<std::mutex> lock(runtime->product_speed_mutex);
    return articore_runtime_set_joint_positions(
        runtime, positions, count, runtime->product_max_speed_percent);
  } catch (const std::exception& error) {
    return record_product_command_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
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

ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pv_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocity_limits, uint32_t count) {
  try {
    auto& product = checked_yunyi(runtime);
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

ARTICORE_RUNTIME_API int32_t articore_runtime_start_trajectory(
    ArticoreRuntime* runtime,
    const ArticoreTrajectoryWaypoint* waypoints,
    uint32_t waypoint_count,
    const ArticoreTrajectoryConfig* config) {
  try {
    auto& product = checked_yunyi(runtime);
    if (!waypoints) {
      throw std::invalid_argument("trajectory waypoints are null");
    }
    if (!config || config->struct_size < sizeof(*config)) {
      throw std::invalid_argument(
          "trajectory config is null or struct_size is too small");
    }
    if (config->interpolation != ARTICORE_TRAJECTORY_QUINTIC) {
      throw std::invalid_argument("only quintic trajectory interpolation is supported");
    }
    if (config->control_mode != runtime->product_mode) {
      throw std::runtime_error(
          "trajectory mode does not match the product Runtime mode");
    }
    if (waypoint_count < 2 ||
        waypoint_count > ARTICORE_MAX_TRAJECTORY_WAYPOINTS) {
      throw std::invalid_argument("trajectory requires 2..10000 waypoints");
    }

    articore::NativeTrajectoryRequest request;
    request.mode = static_cast<ArticoreControlMode>(config->control_mode);
    request.joints.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
    for (uint32_t index = 0;
         index < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++index) {
      const auto& product_joint = product.joints[index];
      articore::NativeTrajectoryJoint joint;
      joint.motor = product_joint.motor;
      joint.direction = product_joint.direction;
      joint.velocity_command_scale = product_joint.velocity_command_scale;
      joint.velocity_feedback_scale = product_joint.velocity_feedback_scale;
      joint.torque_command_scale = product_joint.torque_command_scale;
      joint.lower_position = product_joint.lower;
      joint.upper_position = product_joint.upper;
      joint.velocity_limit = product_joint.velocity_limit;
      joint.acceleration_limit = product_joint.acceleration_limit;
      joint.torque_limit = product_joint.torque_limit;
      if (request.mode == ARTICORE_MODE_MIT) {
        joint.mit_kp = config->mit_kp[index];
        joint.mit_kd = config->mit_kd[index];
        joint.mit_feedforward_torque =
            config->mit_feedforward_torque[index];
      } else {
        joint.pv_velocity_limit = config->pv_velocity_limits[index];
      }
      request.joints.push_back(joint);
    }

    request.waypoints.reserve(waypoint_count);
    for (uint32_t waypoint_index = 0;
         waypoint_index < waypoint_count; ++waypoint_index) {
      const auto& input = waypoints[waypoint_index];
      if (input.struct_size < sizeof(input)) {
        throw std::invalid_argument(
            "trajectory waypoint struct_size is too small at index " +
            std::to_string(waypoint_index));
      }
      articore::NativeTrajectoryWaypoint output;
      output.time_s = input.time_s;
      output.velocity_valid_mask = input.velocity_valid_mask;
      output.acceleration_valid_mask = input.acceleration_valid_mask;
      output.positions.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
      output.velocities.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
      output.accelerations.reserve(ARTICORE_PRODUCT_DUAL_ARM_DOF);
      for (uint32_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
        output.positions.push_back(input.left_positions[joint]);
        output.velocities.push_back(input.left_velocities[joint]);
        output.accelerations.push_back(input.left_accelerations[joint]);
      }
      for (uint32_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
        output.positions.push_back(input.right_positions[joint]);
        output.velocities.push_back(input.right_velocities[joint]);
        output.accelerations.push_back(input.right_accelerations[joint]);
      }
      request.waypoints.push_back(std::move(output));
    }

    checked(runtime).start_trajectory(std::move(request));
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_START_TRAJECTORY, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::invalid_argument& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_START_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_ARGUMENT, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_START_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_trajectory_status(
    ArticoreRuntime* runtime, ArticoreTrajectoryStatus* status) {
  if (!status || status->struct_size < sizeof(*status)) {
    g_last_error = "trajectory status output is null or too small";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  return call([&] { *status = checked(runtime).trajectory_status(); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_trajectory(
    ArticoreRuntime* runtime) {
  try {
    checked(runtime).cancel_trajectory();
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_CANCEL_TRAJECTORY, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_CANCEL_TRAJECTORY,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_move_pose(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent, uint64_t* motion_id) {
  return move_cartesian_impl(
      runtime, side, target_pose, speed_percent,
      ARTICORE_CARTESIAN_POINT_TO_POINT, motion_id);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_move_cartesian(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent, int32_t interpolation, uint64_t* motion_id) {
  if (interpolation != ARTICORE_CARTESIAN_POINT_TO_POINT &&
      interpolation != ARTICORE_CARTESIAN_LINEAR) {
    return record_move_pose_error(
        runtime, ARTICORE_OPERATION_INVALID_ARGUMENT,
        "Cartesian interpolation must be POINT_TO_POINT or LINEAR");
  }
  return move_cartesian_impl(
      runtime, side, target_pose, speed_percent,
      static_cast<ArticoreCartesianInterpolation>(interpolation), motion_id);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_move_linear(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent, uint64_t* motion_id) {
  return move_cartesian_impl(
      runtime, side, target_pose, speed_percent,
      ARTICORE_CARTESIAN_LINEAR, motion_id);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_move_circular(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, float speed_percent,
    uint64_t* motion_id) {
  return move_circular_impl(
      runtime, side, start_pose, via_pose, end_pose, speed_percent,
      motion_id);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_move_circular_v2(
    ArticoreRuntime* runtime, uint32_t side, const float* via_pose,
    const float* end_pose, float speed_percent, uint64_t* motion_id) {
  return move_circular_v2_impl(
      runtime, side, via_pose, end_pose, speed_percent, motion_id);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_move_pose_status(
    ArticoreRuntime* runtime, ArticoreMovePoseStatus* status) {
  if (!status || status->struct_size < sizeof(*status)) {
    g_last_error = "move_pose status output is null or too small";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    std::lock_guard<std::mutex> motion_lock(runtime->move_pose_mutex);
    const uint32_t caller_size = status->struct_size;
    ArticoreMovePoseStatus output{};
    output.struct_size = caller_size;
    output.state = ARTICORE_TRAJECTORY_IDLE;
    output.motion_id = runtime->move_pose_id;
    output.superseded_motion_id = runtime->superseded_move_pose_id;
    output.side = runtime->move_pose_side;
    output.speed_percent = runtime->move_pose_speed_percent;
    std::copy(runtime->move_pose_target.begin(),
              runtime->move_pose_target.end(), output.target_pose);
    if (runtime->move_pose_id != 0) {
      const auto trajectory = checked(runtime).trajectory_status();
      if (trajectory.trajectory_id == runtime->move_pose_id) {
        output.state = trajectory.state;
        output.elapsed_s = trajectory.elapsed_s;
        output.duration_s = trajectory.duration_s;
        output.progress = trajectory.progress;
        copy_abi_text(output.error, trajectory.error);
      } else {
        output.state = ARTICORE_TRAJECTORY_CANCELLED;
        copy_abi_text(
            output.error,
            "point-to-point motion was replaced by another Runtime command");
      }
    }
    *status = output;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_cartesian_motion_status(
    ArticoreRuntime* runtime, ArticoreCartesianMotionStatus* status) {
  if (!status || status->struct_size < sizeof(*status)) {
    g_last_error = "Cartesian motion status output is null or too small";
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    std::lock_guard<std::mutex> motion_lock(runtime->move_pose_mutex);
    const uint32_t caller_size = status->struct_size;
    ArticoreCartesianMotionStatus output{};
    output.struct_size = caller_size;
    output.state = ARTICORE_TRAJECTORY_IDLE;
    output.motion_id = runtime->move_pose_id;
    output.superseded_motion_id = runtime->superseded_move_pose_id;
    output.side = runtime->move_pose_side;
    output.interpolation = runtime->cartesian_interpolation;
    output.speed_percent = runtime->move_pose_speed_percent;
    std::copy(runtime->move_pose_target.begin(),
              runtime->move_pose_target.end(), output.target_pose);
    if (runtime->move_pose_id != 0) {
      const auto trajectory = checked(runtime).trajectory_status();
      if (trajectory.trajectory_id == runtime->move_pose_id) {
        output.state = trajectory.state;
        output.elapsed_s = trajectory.elapsed_s;
        output.duration_s = trajectory.duration_s;
        output.progress = trajectory.progress;
        copy_abi_text(output.error, trajectory.error);
      } else {
        output.state = ARTICORE_TRAJECTORY_CANCELLED;
        copy_abi_text(
            output.error,
            "Cartesian motion was replaced by another Runtime command");
      }
    }
    *status = output;
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_move_pose(
    ArticoreRuntime* runtime) {
  try {
    if (!runtime) throw std::invalid_argument("runtime is null");
    std::lock_guard<std::mutex> motion_lock(runtime->move_pose_mutex);
    if (runtime->move_pose_id != 0) {
      const auto trajectory = checked(runtime).trajectory_status();
      if (trajectory.trajectory_id == runtime->move_pose_id &&
          trajectory.state == ARTICORE_TRAJECTORY_RUNNING) {
        checked(runtime).cancel_trajectory();
      }
    }
    checked(runtime).record_operation_result(
        ARTICORE_OPERATION_CANCEL_MOVE_POSE, ARTICORE_OPERATION_OK);
    g_last_error = "ok";
    return ARTICORE_OPERATION_OK;
  } catch (const std::exception& error) {
    if (runtime && runtime->runtime) {
      runtime->runtime->record_operation_result(
          ARTICORE_OPERATION_CANCEL_MOVE_POSE,
          ARTICORE_OPERATION_INVALID_STATE, error.what());
    }
    g_last_error = error.what();
    return ARTICORE_OPERATION_INVALID_STATE;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_cartesian_motion(
    ArticoreRuntime* runtime) {
  return articore_runtime_cancel_move_pose(runtime);
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_state(
    ArticoreRuntime* runtime, ArticoreProductState* state) {
  if (!state || state->struct_size < sizeof(*state)) {
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
      const auto health = safety.health();
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
          output.left_gripper_level = safety.gripper_force_level(side);
        } else {
          output.right_gripper_available = 1;
          output.right_gripper_level = safety.gripper_force_level(side);
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_state_v2(
    ArticoreRuntime* runtime, ArticoreProductStateV2* state) {
  if (!state || state->struct_size < sizeof(*state)) {
    g_last_error = "product state v2 output is null or too small";
    return -1;
  }
  try {
    auto& product = checked_yunyi(runtime);
    auto& safety = checked(runtime);
    const uint32_t caller_size = state->struct_size;
    ArticoreProductStateV2 output{};
    output.struct_size = caller_size;
    output.has_grippers = product.with_grippers ? 1 : 0;
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    uint64_t maximum_age = 0;
    uint64_t sequence = std::numeric_limits<uint64_t>::max();
    bool complete_timing = true;
    const uint64_t fresh_limit = safety.feedback_max_age_ns();

    for (uint32_t i = 0; i < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++i) {
      const auto& joint = product.joints[i];
      MotorState motor{};
      MotorFeedbackStats stats{};
      const bool cached = motor_handle_get_state_snapshot(
          joint.motor, &motor, &stats) == 0;
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
      const bool power_valid = feedback_present && stats.age_ns <= fresh_limit &&
                               motor.status_code <= 1;
      if (power_valid) {
        arm.enabled_valid_mask |= bit;
        if (motor.status_code == 1) arm.enabled_mask |= bit;
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
        MotorState motor{};
        MotorFeedbackStats stats{};
        const bool cached = motor_handle_get_state_snapshot(
            product.grippers[side], &motor, &stats) == 0;
        const bool feedback_present =
            cached && motor.has_value && stats.has_feedback;
        const bool power_valid = feedback_present &&
            stats.age_ns <= fresh_limit && motor.status_code <= 1;
        if (side == 0) {
          output.left_gripper_available = 1;
          output.left_gripper_level = safety.gripper_force_level(side);
          output.left_gripper_enabled_valid = power_valid ? 1 : 0;
          output.left_gripper_enabled =
              power_valid && motor.status_code == 1 ? 1 : 0;
        } else {
          output.right_gripper_available = 1;
          output.right_gripper_level = safety.gripper_force_level(side);
          output.right_gripper_enabled_valid = power_valid ? 1 : 0;
          output.right_gripper_enabled =
              power_valid && motor.status_code == 1 ? 1 : 0;
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_state_v3(
    ArticoreRuntime* runtime, ArticoreProductStateV3* state) {
  if (!state || state->struct_size < sizeof(*state)) {
    g_last_error = "product state v3 output is null or too small";
    return -1;
  }
  try {
    auto& product = checked_yunyi(runtime);
    auto& safety = checked(runtime);
    const uint32_t caller_size = state->struct_size;
    ArticoreProductStateV3 output{};
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
      MotorState motor{};
      MotorFeedbackStats stats{};
      const bool cached = motor_handle_get_state_snapshot(
          joint.motor, &motor, &stats) == 0;
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
        MotorState motor{};
        MotorFeedbackStats stats{};
        const bool cached = motor_handle_get_state_snapshot(
            product.grippers[side], &motor, &stats) == 0;
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
  if (!limits || limits->struct_size < sizeof(*limits)) {
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
  if (!pose || pose->struct_size < sizeof(*pose)) {
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
  return set_product_grippers_impl(
      runtime, left_opening, right_opening, gripper_level,
      ARTICORE_GRIPPER_FORCE_MIN, ARTICORE_GRIPPER_MODE_PROTECTED,
      "gripper_level must be in the range 1..10");
}

ARTICORE_RUNTIME_API int32_t articore_runtime_set_grippers_v2(
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
        value.enable(runtime->yunyi ? runtime->product_mode
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

ARTICORE_RUNTIME_API int32_t articore_runtime_estop(ArticoreRuntime* runtime) {
  return call([&] { checked(runtime).estop(); });
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
  if (runtime && runtime->yunyi_owned) {
    return articore_runtime_disconnect(runtime);
  }
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
