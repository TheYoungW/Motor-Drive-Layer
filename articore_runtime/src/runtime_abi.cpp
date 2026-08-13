#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "articore/runtime_abi.h"
#include "runtime.hpp"

struct ArticoreRuntime {
  explicit ArticoreRuntime(std::unique_ptr<articore::SafetyRuntime> value)
      : runtime(std::move(value)) {}
  std::unique_ptr<articore::SafetyRuntime> runtime;
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

}  // namespace

extern "C" {

ARTICORE_RUNTIME_API uint32_t articore_runtime_abi_version(void) {
  return (1U << 16) | 6U;
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
         ARTICORE_CAP_JOINT_TRAJECTORY |
         ARTICORE_CAP_ATOMIC_ENABLE |
         ARTICORE_CAP_COMMAND_LIFETIME |
         ARTICORE_CAP_NONPREEMPTIVE_TRAJECTORY |
         ARTICORE_CAP_PROTECTIVE_FAULT_HOLD |
         ARTICORE_CAP_DETERMINISTIC_DISABLE;
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
  if (!config || !motor_api || (!motors && motor_count > 0)) {
    g_last_error = "invalid Articore runtime creation arguments";
    return nullptr;
  }
  try {
    std::vector<ArticoreMotorDescriptor> descriptors(motors, motors + motor_count);
    auto value = std::make_unique<articore::SafetyRuntime>(
        *config, *motor_api, controller_group, left_controller, right_controller,
        std::move(descriptors), controller_enable_all, motor_enable);
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

ARTICORE_RUNTIME_API void articore_runtime_free(ArticoreRuntime* runtime) {
  delete runtime;
}

ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime) {
  return call([&] { checked(runtime).connect(); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_enable(ArticoreRuntime* runtime,
                                                     int32_t mode) {
  return call([&] {
    checked(runtime).enable(static_cast<ArticoreControlMode>(mode));
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

ARTICORE_RUNTIME_API uint64_t articore_runtime_start_joint_trajectory(
    ArticoreRuntime* runtime,
    const ArticoreJointTrajectoryTarget* targets,
    uint32_t target_count,
    int32_t profile) {
  try {
    const auto id = checked(runtime).start_joint_trajectory(
        targets, target_count, static_cast<ArticoreTrajectoryProfile>(profile));
    g_last_error = "ok";
    return id;
  } catch (const std::exception& error) {
    g_last_error = error.what();
    return 0;
  } catch (...) {
    g_last_error = "unknown Articore runtime exception";
    return 0;
  }
}

ARTICORE_RUNTIME_API int32_t articore_runtime_get_trajectory(
    ArticoreRuntime* runtime,
    uint64_t trajectory_id,
    ArticoreTrajectoryInfo* info) {
  if (!info || info->struct_size < sizeof(ArticoreTrajectoryInfo)) {
    g_last_error = "trajectory info is null or has an incompatible struct_size";
    return -1;
  }
  return call([&] { *info = checked(runtime).trajectory_info(trajectory_id); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_wait_trajectory(
    ArticoreRuntime* runtime,
    uint64_t trajectory_id,
    uint32_t timeout_ms,
    ArticoreTrajectoryInfo* info) {
  if (!info || info->struct_size < sizeof(ArticoreTrajectoryInfo)) {
    g_last_error = "trajectory info is null or has an incompatible struct_size";
    return -1;
  }
  return call([&] {
    *info = checked(runtime).wait_trajectory(
        trajectory_id, std::chrono::milliseconds(timeout_ms));
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
  return call([&] { checked(runtime).disable(); });
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
  return call([&] { checked(runtime).close(); });
}

ARTICORE_RUNTIME_API const char* articore_runtime_last_error(void) {
  return g_last_error.c_str();
}

}  // extern "C"
