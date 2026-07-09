#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace damiao {

inline constexpr float kKpMin = 0.0f;
inline constexpr float kKpMax = 500.0f;
inline constexpr float kKdMin = 0.0f;
inline constexpr float kKdMax = 5.0f;

struct Limits {
  float p_min;
  float p_max;
  float v_min;
  float v_max;
  float t_min;
  float t_max;
};

struct SensorFeedback {
  uint8_t can_id;
  uint8_t status_code;
  float pos;
  float vel;
  float torq;
  float t_mos;
  float t_rotor;
};

struct RegisterValue {
  uint8_t rid;
  std::array<uint8_t, 4> data;
};

inline const char* status_name(uint8_t status) {
  switch (status) {
    case 0x0:
      return "DISABLED";
    case 0x1:
      return "ENABLED";
    case 0x8:
      return "OVER_VOLTAGE";
    case 0x9:
      return "UNDER_VOLTAGE";
    case 0xA:
      return "OVER_CURRENT";
    case 0xB:
      return "MOS_OVER_TEMP";
    case 0xC:
      return "ROTOR_OVER_TEMP";
    case 0xD:
      return "LOST_COMM";
    case 0xE:
      return "OVERLOAD";
    default:
      return "UNKNOWN";
  }
}

inline uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
  const float span = x_max - x_min;
  const float clipped = std::min(std::max(x, x_min), x_max);
  return static_cast<uint32_t>(
      (clipped - x_min) * static_cast<float>((uint32_t{1} << bits) - 1) / span);
}

inline float uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits) {
  const float span = x_max - x_min;
  return static_cast<float>(x) * span / static_cast<float>((uint32_t{1} << bits) - 1) +
         x_min;
}

inline std::array<uint8_t, 8> encode_mit_command(float pos,
                                                  float vel,
                                                  float torq,
                                                  float kp,
                                                  float kd,
                                                  Limits limits) {
  const uint32_t pos_u = float_to_uint(pos, limits.p_min, limits.p_max, 16);
  const uint32_t vel_u = float_to_uint(vel, limits.v_min, limits.v_max, 12);
  const uint32_t kp_u = float_to_uint(kp, kKpMin, kKpMax, 12);
  const uint32_t kd_u = float_to_uint(kd, kKdMin, kKdMax, 12);
  const uint32_t torq_u = float_to_uint(torq, limits.t_min, limits.t_max, 12);

  return {
      static_cast<uint8_t>((pos_u >> 8) & 0xFF),
      static_cast<uint8_t>(pos_u & 0xFF),
      static_cast<uint8_t>((vel_u >> 4) & 0xFF),
      static_cast<uint8_t>(((vel_u & 0xF) << 4) | ((kp_u >> 8) & 0xF)),
      static_cast<uint8_t>(kp_u & 0xFF),
      static_cast<uint8_t>((kd_u >> 4) & 0xFF),
      static_cast<uint8_t>(((kd_u & 0xF) << 4) | ((torq_u >> 8) & 0xF)),
      static_cast<uint8_t>(torq_u & 0xFF),
  };
}

inline void write_f32_le(std::array<uint8_t, 8>& out, std::size_t offset, float value) {
  static_assert(sizeof(float) == 4, "Damiao frame packing requires 32-bit float");
  uint8_t raw[4] = {};
  std::memcpy(raw, &value, sizeof(raw));
  out[offset] = raw[0];
  out[offset + 1] = raw[1];
  out[offset + 2] = raw[2];
  out[offset + 3] = raw[3];
}

inline std::array<uint8_t, 8> encode_position_velocity_command(float target_position,
                                                               float velocity_limit) {
  std::array<uint8_t, 8> out{};
  write_f32_le(out, 0, target_position);
  write_f32_le(out, 4, velocity_limit);
  return out;
}

inline std::array<uint8_t, 8> encode_velocity_command(float target_velocity) {
  std::array<uint8_t, 8> out{};
  write_f32_le(out, 0, target_velocity);
  return out;
}

inline void write_u16_le(std::array<uint8_t, 8>& out, std::size_t offset, uint16_t value) {
  out[offset] = static_cast<uint8_t>(value & 0xFF);
  out[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline std::array<uint8_t, 8> encode_force_position_command(float target_position,
                                                            float velocity_limit,
                                                            float torque_limit_ratio) {
  const auto v_des =
      static_cast<uint16_t>(std::min(std::max(velocity_limit, 0.0f), 100.0f) * 100.0f);
  const auto i_des = static_cast<uint16_t>(
      std::min(std::max(torque_limit_ratio, 0.0f), 1.0f) * 10000.0f);
  std::array<uint8_t, 8> out{};
  write_f32_le(out, 0, target_position);
  write_u16_le(out, 4, std::min<uint16_t>(v_des, 10000));
  write_u16_le(out, 6, std::min<uint16_t>(i_des, 10000));
  return out;
}

inline SensorFeedback decode_sensor_feedback(std::array<uint8_t, 8> data, Limits limits) {
  const uint8_t can_id = data[0] & 0x0F;
  const uint8_t status = data[0] >> 4;
  const uint32_t pos_int = (static_cast<uint32_t>(data[1]) << 8) | data[2];
  const uint32_t vel_int = (static_cast<uint32_t>(data[3]) << 4) |
                           (static_cast<uint32_t>(data[4]) >> 4);
  const uint32_t torq_int = ((static_cast<uint32_t>(data[4]) & 0x0F) << 8) | data[5];

  return SensorFeedback{
      can_id,
      status,
      uint_to_float(pos_int, limits.p_min, limits.p_max, 16),
      uint_to_float(vel_int, limits.v_min, limits.v_max, 12),
      uint_to_float(torq_int, limits.t_min, limits.t_max, 12),
      static_cast<float>(data[6]),
      static_cast<float>(data[7]),
  };
}

inline bool is_known_register(uint8_t rid) {
  return (rid <= 36) || (rid >= 50 && rid <= 56) || (rid >= 59 && rid <= 65) ||
         rid == 80 || rid == 81;
}

inline bool is_register_frame(const std::array<uint8_t, 8>& data, uint8_t marker) {
  return data[1] <= 0x0F && data[2] == marker && is_known_register(data[3]);
}

inline bool is_register_reply(const std::array<uint8_t, 8>& data) {
  return is_register_frame(data, 0x33);
}

inline bool is_register_write_ack(const std::array<uint8_t, 8>& data) {
  return is_register_frame(data, 0x55);
}

inline RegisterValue decode_register_value(std::array<uint8_t, 8> data) {
  if (!is_register_reply(data)) {
    throw std::runtime_error("not a register reply frame");
  }
  return RegisterValue{data[3], {data[4], data[5], data[6], data[7]}};
}

inline std::array<uint8_t, 8> encode_enable_command() {
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
}

inline std::array<uint8_t, 8> encode_disable_command() {
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
}

inline std::array<uint8_t, 8> encode_set_zero_command() {
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
}

inline std::array<uint8_t, 8> encode_clear_error_command() {
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
}

inline std::array<uint8_t, 8> encode_register_read_command(uint16_t motor_id, uint8_t rid) {
  return {static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          0x33,
          rid,
          0,
          0,
          0,
          0};
}

inline std::array<uint8_t, 8> encode_register_write_command(uint16_t motor_id,
                                                            uint8_t rid,
                                                            std::array<uint8_t, 4> data) {
  return {static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          0x55,
          rid,
          data[0],
          data[1],
          data[2],
          data[3]};
}

inline std::array<uint8_t, 8> encode_store_parameters_command(uint16_t motor_id) {
  return {static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          0xAA,
          0x01,
          0,
          0,
          0,
          0};
}

inline std::array<uint8_t, 8> encode_feedback_request_command(uint16_t motor_id) {
  return {static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          0xCC,
          0,
          0,
          0,
          0,
          0};
}

}  // namespace damiao
