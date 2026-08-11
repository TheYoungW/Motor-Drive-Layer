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
  return (1U << 16) | 1U;
}

ARTICORE_RUNTIME_API uint64_t articore_runtime_capabilities(void) {
  return ARTICORE_CAP_COMMAND_WATCHDOG |
         ARTICORE_CAP_SAFE_HOLD |
         ARTICORE_CAP_GRIPPER_PROTECTION |
         ARTICORE_CAP_SINGLE_CHANNEL |
         ARTICORE_CAP_DUAL_CHANNEL |
         ARTICORE_CAP_TRANSPORT_HEALTH |
         ARTICORE_CAP_CURRENT_POSITION_HOLD;
}

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create(
    const ArticoreRuntimeConfig* config,
    const ArticoreMotorApi* motor_api,
    void* controller_group,
    void* left_controller,
    void* right_controller,
    const ArticoreMotorDescriptor* motors,
    uint32_t motor_count) {
  if (!config || !motor_api || (!motors && motor_count > 0)) {
    g_last_error = "invalid Articore runtime creation arguments";
    return nullptr;
  }
  try {
    std::vector<ArticoreMotorDescriptor> descriptors(motors, motors + motor_count);
    auto value = std::make_unique<articore::SafetyRuntime>(
        *config, *motor_api, controller_group, left_controller, right_controller,
        std::move(descriptors));
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

ARTICORE_RUNTIME_API int32_t articore_runtime_get_health(
    ArticoreRuntime* runtime, ArticoreSafetyHealth* health) {
  if (!health) {
    g_last_error = "health output is null";
    return -1;
  }
  return call([&] { *health = checked(runtime).health(); });
}

ARTICORE_RUNTIME_API int32_t articore_runtime_close(ArticoreRuntime* runtime) {
  return call([&] { checked(runtime).close(); });
}

ARTICORE_RUNTIME_API const char* articore_runtime_last_error(void) {
  return g_last_error.c_str();
}

}  // extern "C"
