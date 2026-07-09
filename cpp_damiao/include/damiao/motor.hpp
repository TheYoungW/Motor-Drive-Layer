#pragma once

#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>

#include "damiao/can_bus.hpp"
#include "damiao/protocol.hpp"

namespace damiao {

inline Limits model_limits(const std::string& model) {
  if (model == "3507") return {-12.566f, 12.566f, -50.0f, 50.0f, -5.0f, 5.0f};
  if (model == "4310") return {-12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f};
  if (model == "4310P") return {-12.5f, 12.5f, -50.0f, 50.0f, -10.0f, 10.0f};
  if (model == "4340" || model == "4340P") {
    return {-12.5f, 12.5f, -10.0f, 10.0f, -28.0f, 28.0f};
  }
  if (model == "4340_v20") return {-12.5f, 12.5f, -20.0f, 20.0f, -28.0f, 28.0f};
  if (model == "6006") return {-12.5f, 12.5f, -45.0f, 45.0f, -20.0f, 20.0f};
  if (model == "8006") return {-12.5f, 12.5f, -45.0f, 45.0f, -40.0f, 40.0f};
  if (model == "8009") return {-12.5f, 12.5f, -45.0f, 45.0f, -54.0f, 54.0f};
  if (model == "10010L") return {-12.5f, 12.5f, -25.0f, 25.0f, -200.0f, 200.0f};
  if (model == "10010") return {-12.5f, 12.5f, -20.0f, 20.0f, -200.0f, 200.0f};
  if (model == "H3510") return {-12.5f, 12.5f, -280.0f, 280.0f, -1.0f, 1.0f};
  if (model == "G6215") return {-12.5f, 12.5f, -45.0f, 45.0f, -10.0f, 10.0f};
  if (model == "H6220") return {-12.5f, 12.5f, -45.0f, 45.0f, -10.0f, 10.0f};
  if (model == "JH11") return {-12.5f, 12.5f, -10.0f, 10.0f, -12.0f, 12.0f};
  if (model == "6248P") return {-12.566f, 12.566f, -20.0f, 20.0f, -120.0f, 120.0f};
  throw std::invalid_argument("unknown Damiao model: " + model);
}

class Motor {
 public:
  Motor(CanBus& bus, uint16_t motor_id, uint16_t feedback_id, Limits limits)
      : bus_(bus), motor_id_(motor_id), feedback_id_(feedback_id), limits_(limits) {}

  uint16_t motor_id() const { return motor_id_; }
  uint16_t feedback_id() const { return feedback_id_; }
  Limits limits() const { return limits_; }

  void enable() { send_to_motor(encode_enable_command()); }
  void disable() { send_to_motor(encode_disable_command()); }
  void set_zero_position() { send_to_motor(encode_set_zero_command()); }
  void clear_error() { send_to_motor(encode_clear_error_command()); }
  void request_feedback() { send_to_motor(encode_feedback_request_command(motor_id_)); }
  void store_parameters() { send_to_motor(encode_store_parameters_command(motor_id_)); }

  void send_mit(float pos, float vel, float kp, float kd, float tau) {
    send_to_motor(encode_mit_command(pos, vel, tau, kp, kd, limits_));
  }

  void send_position_velocity(float pos, float velocity_limit) {
    send_mode_frame(0x100, encode_position_velocity_command(pos, velocity_limit));
  }

  void send_velocity(float velocity) {
    send_mode_frame(0x200, encode_velocity_command(velocity));
  }

  void send_force_position(float pos, float velocity_limit, float torque_limit_ratio) {
    send_mode_frame(0x300, encode_force_position_command(pos, velocity_limit, torque_limit_ratio));
  }

  std::optional<SensorFeedback> receive_feedback(std::chrono::milliseconds timeout) {
    const auto frame = bus_.receive_for(timeout);
    if (!frame.has_value() || frame->id != feedback_id_) {
      return std::nullopt;
    }
    return decode_sensor_feedback(frame->data, limits_);
  }

 private:
  void send_to_motor(std::array<uint8_t, 8> data) {
    bus_.send(CanFrame{motor_id_, data});
  }

  void send_mode_frame(uint32_t base_id, std::array<uint8_t, 8> data) {
    bus_.send(CanFrame{base_id + motor_id_, data});
  }

  CanBus& bus_;
  uint16_t motor_id_;
  uint16_t feedback_id_;
  Limits limits_;
};

}  // namespace damiao
