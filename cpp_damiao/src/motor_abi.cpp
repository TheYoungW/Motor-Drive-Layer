#include "motor_abi.h"

#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "damiao/dm_device_bus.hpp"
#include "damiao/dm_serial_bus.hpp"
#include "damiao/registers.hpp"
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

void copy_motor_state(const damiao::MotorState& state, ::MotorState* out_state) {
  out_state->has_value = 1;
  out_state->can_id = state.can_id;
  out_state->arbitration_id = state.arbitration_id;
  out_state->status_code = state.status_code;
  out_state->pos = state.pos;
  out_state->vel = state.vel;
  out_state->torq = state.torq;
  out_state->t_mos = state.t_mos;
  out_state->t_rotor = state.t_rotor;
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
  return "0.5.0-cpp";
}

const char* motor_abi_capabilities_json(void) {
  return R"({"schema":1,"abi":{"name":"motor_abi","version":"0.5.0-cpp"},"transports":["socketcan","socketcanfd","dm-serial","dm-device"],"vendors":["damiao"],"features":{"state_cache":true,"background_polling":true,"tx_pacing":true,"feedback_stats":true,"fresh_feedback_wait":true,"register_metadata":true,"controller_lifecycle":["shutdown","close_bus","poll_feedback_once","request_feedback_all","enable_all","disable_all","set_tx_gap_us"],"control_modes":["mit","pos-vel","vel","force-pos"],"damiao":["dm-serial","dm-device","register_u32","register_f32","param_u32","param_f32","set_can_timeout_ms"]}})";
}

int32_t motor_damiao_register_info(uint8_t rid, MotorRegisterInfo* out_info) {
  if (out_info == nullptr) return fail("out_info is null");
  return ffi_call([&] {
    std::memset(out_info, 0, sizeof(MotorRegisterInfo));
    const auto info = damiao::register_info(rid);
    if (!info.has_value()) return;
    out_info->has_value = 1;
    out_info->rid = info->rid;
    out_info->access = info->access == damiao::RegisterAccess::ReadWrite
                           ? MOTOR_REGISTER_ACCESS_READ_WRITE
                           : MOTOR_REGISTER_ACCESS_READ_ONLY;
    out_info->data_type = info->data_type == damiao::RegisterDataType::UInt32
                              ? MOTOR_REGISTER_DATA_UINT32
                              : MOTOR_REGISTER_DATA_FLOAT;
  });
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

int32_t motor_controller_request_feedback_all(MotorController* controller,
                                              uint32_t timeout_ms) {
  if (controller == nullptr) return fail("controller is null");
  std::lock_guard<std::mutex> lock(controller->mutex);
  return ffi_call([&] {
    controller->controller->request_feedback_all(std::chrono::milliseconds(timeout_ms));
  });
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

int32_t motor_controller_set_tx_gap_us(MotorController* controller, uint32_t gap_us) {
  if (controller == nullptr) return fail("controller is null");
  std::lock_guard<std::mutex> lock(controller->mutex);
  return ffi_call([&] {
    controller->controller->set_tx_gap(std::chrono::microseconds(gap_us));
  });
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

void motor_handle_free(MotorHandle* handle) {
  delete handle;
}

int32_t motor_handle_enable(MotorHandle* handle) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->enable(); });
}

int32_t motor_handle_disable(MotorHandle* handle) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->disable(); });
}

int32_t motor_handle_clear_error(MotorHandle* handle) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->clear_error(); });
}

int32_t motor_handle_set_zero_position(MotorHandle* handle) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->set_zero_position(); });
}

int32_t motor_handle_ensure_mode(MotorHandle* handle, uint32_t mode, uint32_t timeout_ms) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call(
      [&] { handle->motor->ensure_mode(mode, std::chrono::milliseconds(timeout_ms)); });
}

int32_t motor_handle_send_mit(MotorHandle* handle,
                              float target_position,
                              float target_velocity,
                              float stiffness,
                              float damping,
                              float feedforward_torque) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] {
    handle->motor->send_mit(target_position, target_velocity, stiffness, damping,
                           feedforward_torque);
  });
}

int32_t motor_handle_send_pos_vel(MotorHandle* handle,
                                  float target_position,
                                  float velocity_limit) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->send_pos_vel(target_position, velocity_limit); });
}

int32_t motor_handle_send_vel(MotorHandle* handle, float target_velocity) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->send_vel(target_velocity); });
}

int32_t motor_handle_send_force_pos(MotorHandle* handle,
                                    float target_position,
                                    float velocity_limit,
                                    float torque_limit_ratio) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call(
      [&] { handle->motor->send_force_pos(target_position, velocity_limit, torque_limit_ratio); });
}

int32_t motor_handle_store_parameters(MotorHandle* handle) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->store_parameters(); });
}

int32_t motor_handle_request_feedback(MotorHandle* handle) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->request_feedback(); });
}

int32_t motor_handle_request_fresh_state(MotorHandle* handle,
                                         uint32_t timeout_ms,
                                         MotorState* out_state) {
  if (handle == nullptr || out_state == nullptr) return fail("motor or out_state is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] {
    std::memset(out_state, 0, sizeof(MotorState));
    const auto state =
        handle->motor->request_fresh_state(std::chrono::milliseconds(timeout_ms));
    if (!state.has_value()) {
      throw std::runtime_error("fresh feedback timed out for motor ID " +
                               std::to_string(handle->motor->motor_id()));
    }
    copy_motor_state(*state, out_state);
  });
}

int32_t motor_handle_set_can_timeout_ms(MotorHandle* handle, uint32_t timeout_ms) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->set_can_timeout_ms(timeout_ms); });
}

int32_t motor_handle_write_register_f32(MotorHandle* handle, uint8_t rid, float value) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->write_register_f32(rid, value); });
}

int32_t motor_handle_write_register_u32(MotorHandle* handle, uint8_t rid, uint32_t value) {
  if (handle == nullptr) return fail("motor is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] { handle->motor->write_register_u32(rid, value); });
}

int32_t motor_handle_get_register_f32(MotorHandle* handle,
                                      uint8_t rid,
                                      uint32_t timeout_ms,
                                      float* out_value) {
  if (handle == nullptr || out_value == nullptr) return fail("motor or out_value is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] {
    *out_value = handle->motor->get_register_f32(rid, std::chrono::milliseconds(timeout_ms));
  });
}

int32_t motor_handle_get_register_u32(MotorHandle* handle,
                                      uint8_t rid,
                                      uint32_t timeout_ms,
                                      uint32_t* out_value) {
  if (handle == nullptr || out_value == nullptr) return fail("motor or out_value is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] {
    *out_value = handle->motor->get_register_u32(rid, std::chrono::milliseconds(timeout_ms));
  });
}

int32_t motor_handle_get_state(MotorHandle* handle, MotorState* out_state) {
  if (handle == nullptr || out_state == nullptr) return fail("motor or out_state is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] {
    std::memset(out_state, 0, sizeof(MotorState));
    const auto state = handle->motor->latest_state();
    if (!state.has_value()) {
      return;
    }
    copy_motor_state(*state, out_state);
  });
}

int32_t motor_handle_get_feedback_stats(MotorHandle* handle,
                                        MotorFeedbackStats* out_stats) {
  if (handle == nullptr || out_stats == nullptr) return fail("motor or out_stats is null");
  std::lock_guard<std::mutex> lock(handle->mutex);
  return ffi_call([&] {
    std::memset(out_stats, 0, sizeof(MotorFeedbackStats));
    const auto stats = handle->motor->feedback_stats();
    out_stats->has_feedback = stats.has_feedback ? 1 : 0;
    out_stats->update_count = stats.update_count;
    out_stats->age_ns = stats.age.count() < 0 ? 0 : static_cast<uint64_t>(stats.age.count());
  });
}

int32_t motor_handle_damiao_get_param_f32(MotorHandle* handle,
                                          uint16_t param_id,
                                          uint32_t timeout_ms,
                                          float* out_value) {
  try {
    return motor_handle_get_register_f32(handle, param_to_rid(param_id), timeout_ms, out_value);
  } catch (const std::exception& err) {
    return fail(err.what());
  }
}

int32_t motor_handle_damiao_get_param_u32(MotorHandle* handle,
                                          uint16_t param_id,
                                          uint32_t timeout_ms,
                                          uint32_t* out_value) {
  try {
    return motor_handle_get_register_u32(handle, param_to_rid(param_id), timeout_ms, out_value);
  } catch (const std::exception& err) {
    return fail(err.what());
  }
}

int32_t motor_handle_damiao_write_param_f32(MotorHandle* handle,
                                            uint16_t param_id,
                                            float value) {
  try {
    return motor_handle_write_register_f32(handle, param_to_rid(param_id), value);
  } catch (const std::exception& err) {
    return fail(err.what());
  }
}

int32_t motor_handle_damiao_write_param_u32(MotorHandle* handle,
                                            uint16_t param_id,
                                            uint32_t value) {
  try {
    return motor_handle_write_register_u32(handle, param_to_rid(param_id), value);
  } catch (const std::exception& err) {
    return fail(err.what());
  }
}

}  // extern "C"
