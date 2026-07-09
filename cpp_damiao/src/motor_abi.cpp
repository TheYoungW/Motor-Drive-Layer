#include "motor_abi.h"

#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "damiao/dm_device_bus.hpp"
#include "damiao/dm_serial_bus.hpp"
#include "damiao/runtime.hpp"
#include "damiao/socketcan_bus.hpp"

namespace {

thread_local std::string last_error = "ok";

void set_last_error(const std::string& message) {
  last_error = message.empty() ? "error" : message;
}

int32_t fail(const std::string& message) {
  set_last_error(message);
  return -1;
}

template <typename Fn>
int32_t ffi_call(Fn&& fn) {
  try {
    fn();
    set_last_error("ok");
    return 0;
  } catch (const std::exception& err) {
    return fail(err.what());
  } catch (...) {
    return fail("unknown C++ exception");
  }
}

uint8_t param_to_rid(uint16_t param_id) {
  if (param_id > 255) {
    throw std::invalid_argument("Damiao parameter/register id must be in 0..=255");
  }
  return static_cast<uint8_t>(param_id);
}

}  // namespace

struct MotorController {
  explicit MotorController(std::unique_ptr<damiao::Controller> c) : controller(std::move(c)) {}
  std::unique_ptr<damiao::Controller> controller;
  std::mutex mutex;
};

struct MotorHandle {
  explicit MotorHandle(std::shared_ptr<damiao::MotorHandle> m) : motor(std::move(m)) {}
  std::shared_ptr<damiao::MotorHandle> motor;
  std::mutex mutex;
};

extern "C" {

const char* motor_last_error_message(void) {
  return last_error.c_str();
}

const char* motor_abi_version(void) {
  return "0.2.0-cpp";
}

const char* motor_abi_capabilities_json(void) {
  return R"({"schema":1,"abi":{"name":"motor_abi","version":"0.2.0-cpp"},"transports":["socketcan","socketcanfd","dm-serial","dm-device"],"vendors":["damiao"],"features":{"state_cache":true,"background_polling":true,"tx_pacing":true,"controller_lifecycle":["shutdown","close_bus","poll_feedback_once","enable_all","disable_all"],"control_modes":["mit","pos-vel","vel","force-pos"],"damiao":["dm-serial","dm-device","register_u32","register_f32","param_u32","param_f32","set_can_timeout_ms"]}})";
}

MotorController* motor_controller_new_socketcan(const char* channel) {
  if (channel == nullptr) {
    set_last_error("channel is null");
    return nullptr;
  }
  try {
    auto bus = damiao::SocketCanBus::open(channel);
    return new MotorController(std::make_unique<damiao::Controller>(bus));
  } catch (const std::exception& err) {
    set_last_error(err.what());
    return nullptr;
  }
}

MotorController* motor_controller_new_socketcanfd(const char* channel) {
  if (channel == nullptr) {
    set_last_error("channel is null");
    return nullptr;
  }
  try {
    const bool enable_brs = false;
    auto bus = damiao::SocketCanFdBus::open(channel, enable_brs);
    return new MotorController(std::make_unique<damiao::Controller>(bus));
  } catch (const std::exception& err) {
    set_last_error(err.what());
    return nullptr;
  }
}

MotorController* motor_controller_new_dm_serial(const char* serial_port, uint32_t baud) {
  if (serial_port == nullptr) {
    set_last_error("serial_port is null");
    return nullptr;
  }
  try {
    auto bus = damiao::DmSerialBus::open(serial_port, baud);
    return new MotorController(std::make_unique<damiao::Controller>(bus));
  } catch (const std::exception& err) {
    set_last_error(err.what());
    return nullptr;
  }
}

MotorController* motor_controller_new_dm_device(const char* dm_device_type,
                                                const char* dm_channel) {
  if (dm_device_type == nullptr || dm_channel == nullptr) {
    set_last_error("dm_device_type or dm_channel is null");
    return nullptr;
  }
  try {
    auto bus = damiao::DmDeviceBus::open(damiao::parse_dm_device_type(dm_device_type),
                                         dm_channel);
    return new MotorController(std::make_unique<damiao::Controller>(bus));
  } catch (const std::exception& err) {
    set_last_error(err.what());
    return nullptr;
  }
}

void motor_controller_free(MotorController* controller) {
  delete controller;
}

int32_t motor_controller_poll_feedback_once(MotorController* controller) {
  if (controller == nullptr) return fail("controller is null");
  std::lock_guard<std::mutex> lock(controller->mutex);
  return ffi_call([&] { controller->controller->poll_feedback_once(); });
}

int32_t motor_controller_enable_all(MotorController* controller) {
  if (controller == nullptr) return fail("controller is null");
  std::lock_guard<std::mutex> lock(controller->mutex);
  return ffi_call([&] { controller->controller->enable_all(); });
}

int32_t motor_controller_disable_all(MotorController* controller) {
  if (controller == nullptr) return fail("controller is null");
  std::lock_guard<std::mutex> lock(controller->mutex);
  return ffi_call([&] { controller->controller->disable_all(); });
}

int32_t motor_controller_shutdown(MotorController* controller) {
  if (controller == nullptr) return fail("controller is null");
  std::lock_guard<std::mutex> lock(controller->mutex);
  return ffi_call([&] { controller->controller->shutdown(); });
}

int32_t motor_controller_close_bus(MotorController* controller) {
  if (controller == nullptr) return fail("controller is null");
  std::lock_guard<std::mutex> lock(controller->mutex);
  return ffi_call([&] { controller->controller->close_bus(); });
}

MotorHandle* motor_controller_add_damiao_motor(MotorController* controller,
                                               uint16_t motor_id,
                                               uint16_t feedback_id,
                                               const char* model) {
  if (controller == nullptr) {
    set_last_error("controller is null");
    return nullptr;
  }
  if (model == nullptr) {
    set_last_error("model is null");
    return nullptr;
  }
  try {
    std::lock_guard<std::mutex> lock(controller->mutex);
    return new MotorHandle(
        controller->controller->add_damiao_motor(motor_id, feedback_id, model));
  } catch (const std::exception& err) {
    set_last_error(err.what());
    return nullptr;
  }
}

void motor_handle_free(MotorHandle* motor) {
  delete motor;
}

int32_t motor_handle_enable(MotorHandle* motor) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->enable(); });
}

int32_t motor_handle_disable(MotorHandle* motor) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->disable(); });
}

int32_t motor_handle_clear_error(MotorHandle* motor) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->clear_error(); });
}

int32_t motor_handle_set_zero_position(MotorHandle* motor) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->set_zero_position(); });
}

int32_t motor_handle_ensure_mode(MotorHandle* motor, uint32_t mode, uint32_t timeout_ms) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call(
      [&] { motor->motor->ensure_mode(mode, std::chrono::milliseconds(timeout_ms)); });
}

int32_t motor_handle_send_mit(MotorHandle* motor,
                              float target_position,
                              float target_velocity,
                              float stiffness,
                              float damping,
                              float feedforward_torque) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] {
    motor->motor->send_mit(target_position, target_velocity, stiffness, damping,
                           feedforward_torque);
  });
}

int32_t motor_handle_send_pos_vel(MotorHandle* motor,
                                  float target_position,
                                  float velocity_limit) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->send_pos_vel(target_position, velocity_limit); });
}

int32_t motor_handle_send_vel(MotorHandle* motor, float target_velocity) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->send_vel(target_velocity); });
}

int32_t motor_handle_send_force_pos(MotorHandle* motor,
                                    float target_position,
                                    float velocity_limit,
                                    float torque_limit_ratio) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call(
      [&] { motor->motor->send_force_pos(target_position, velocity_limit, torque_limit_ratio); });
}

int32_t motor_handle_store_parameters(MotorHandle* motor) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->store_parameters(); });
}

int32_t motor_handle_request_feedback(MotorHandle* motor) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->request_feedback(); });
}

int32_t motor_handle_set_can_timeout_ms(MotorHandle* motor, uint32_t timeout_ms) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->set_can_timeout_ms(timeout_ms); });
}

int32_t motor_handle_write_register_f32(MotorHandle* motor, uint8_t rid, float value) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->write_register_f32(rid, value); });
}

int32_t motor_handle_write_register_u32(MotorHandle* motor, uint8_t rid, uint32_t value) {
  if (motor == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] { motor->motor->write_register_u32(rid, value); });
}

int32_t motor_handle_get_register_f32(MotorHandle* motor,
                                      uint8_t rid,
                                      uint32_t timeout_ms,
                                      float* out_value) {
  if (motor == nullptr || out_value == nullptr) return fail("motor or out_value is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] {
    *out_value = motor->motor->get_register_f32(rid, std::chrono::milliseconds(timeout_ms));
  });
}

int32_t motor_handle_get_register_u32(MotorHandle* motor,
                                      uint8_t rid,
                                      uint32_t timeout_ms,
                                      uint32_t* out_value) {
  if (motor == nullptr || out_value == nullptr) return fail("motor or out_value is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] {
    *out_value = motor->motor->get_register_u32(rid, std::chrono::milliseconds(timeout_ms));
  });
}

int32_t motor_handle_get_state(MotorHandle* motor, MotorState* out_state) {
  if (motor == nullptr || out_state == nullptr) return fail("motor or out_state is null");
  std::lock_guard<std::mutex> lock(motor->mutex);
  return ffi_call([&] {
    std::memset(out_state, 0, sizeof(MotorState));
    const auto state = motor->motor->latest_state();
    if (!state.has_value()) {
      return;
    }
    out_state->has_value = 1;
    out_state->can_id = state->can_id;
    out_state->arbitration_id = state->arbitration_id;
    out_state->status_code = state->status_code;
    out_state->pos = state->pos;
    out_state->vel = state->vel;
    out_state->torq = state->torq;
    out_state->t_mos = state->t_mos;
    out_state->t_rotor = state->t_rotor;
  });
}

int32_t motor_handle_damiao_get_param_f32(MotorHandle* motor,
                                          uint16_t param_id,
                                          uint32_t timeout_ms,
                                          float* out_value) {
  try {
    return motor_handle_get_register_f32(motor, param_to_rid(param_id), timeout_ms, out_value);
  } catch (const std::exception& err) {
    return fail(err.what());
  }
}

int32_t motor_handle_damiao_get_param_u32(MotorHandle* motor,
                                          uint16_t param_id,
                                          uint32_t timeout_ms,
                                          uint32_t* out_value) {
  try {
    return motor_handle_get_register_u32(motor, param_to_rid(param_id), timeout_ms, out_value);
  } catch (const std::exception& err) {
    return fail(err.what());
  }
}

int32_t motor_handle_damiao_write_param_f32(MotorHandle* motor,
                                            uint16_t param_id,
                                            float value) {
  try {
    return motor_handle_write_register_f32(motor, param_to_rid(param_id), value);
  } catch (const std::exception& err) {
    return fail(err.what());
  }
}

int32_t motor_handle_damiao_write_param_u32(MotorHandle* motor,
                                            uint16_t param_id,
                                            uint32_t value) {
  try {
    return motor_handle_write_register_u32(motor, param_to_rid(param_id), value);
  } catch (const std::exception& err) {
    return fail(err.what());
  }
}

}  // extern "C"
