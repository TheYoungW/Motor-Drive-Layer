#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "damiao/can_bus.hpp"
#include "damiao/motor.hpp"
#include "damiao/protocol.hpp"

namespace damiao {

struct MotorState {
  uint8_t can_id = 0;
  uint32_t arbitration_id = 0;
  uint8_t status_code = 0;
  float pos = 0.0f;
  float vel = 0.0f;
  float torq = 0.0f;
  float t_mos = 0.0f;
  float t_rotor = 0.0f;
};

class MotorHandle {
 public:
  MotorHandle(std::shared_ptr<CanBus> bus, uint16_t motor_id, uint16_t feedback_id,
              std::string model);

  uint16_t motor_id() const { return motor_id_; }
  uint16_t feedback_id() const { return feedback_id_; }
  const std::string& model() const { return model_; }

  void enable();
  void disable();
  void clear_error();
  void set_zero_position();
  void send_mit(float pos, float vel, float kp, float kd, float tau);
  void send_pos_vel(float pos, float velocity_limit);
  void send_vel(float velocity);
  void send_force_pos(float pos, float velocity_limit, float torque_limit_ratio);
  void request_feedback();
  void store_parameters();
  void write_register_f32(uint8_t rid, float value);
  void write_register_u32(uint8_t rid, uint32_t value);
  float get_register_f32(uint8_t rid, std::chrono::milliseconds timeout);
  uint32_t get_register_u32(uint8_t rid, std::chrono::milliseconds timeout);
  void ensure_mode(uint32_t mode, std::chrono::milliseconds timeout);
  void set_can_timeout_ms(uint32_t timeout_ms);

  bool accepts_frame(const CanFrame& frame) const;
  void process_feedback_frame(const CanFrame& frame);
  std::optional<MotorState> latest_state() const;

 private:
  void send_raw(uint32_t arbitration_id, std::array<uint8_t, 8> data);
  void send_to_motor(std::array<uint8_t, 8> data);
  void send_mode_frame(uint32_t base_id, std::array<uint8_t, 8> data);
  void write_register_raw(uint8_t rid, std::array<uint8_t, 4> data);
  std::array<uint8_t, 4> wait_for_register(uint8_t rid, std::chrono::milliseconds timeout);
  void wait_for_write_ack(uint8_t rid, std::array<uint8_t, 4> expected,
                          std::chrono::milliseconds timeout);

  std::shared_ptr<CanBus> bus_;
  uint16_t motor_id_;
  uint16_t feedback_id_;
  std::string model_;
  Limits limits_;
  mutable std::mutex state_mutex_;
  std::optional<MotorState> state_;
  mutable std::mutex register_mutex_;
  std::map<uint8_t, std::pair<std::array<uint8_t, 4>, std::chrono::steady_clock::time_point>>
      register_replies_;
  std::map<uint8_t, std::pair<std::array<uint8_t, 4>, std::chrono::steady_clock::time_point>>
      register_acks_;
};

class Controller {
 public:
  explicit Controller(std::shared_ptr<CanBus> bus);
  ~Controller();

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  std::shared_ptr<MotorHandle> add_damiao_motor(uint16_t motor_id, uint16_t feedback_id,
                                                const std::string& model);
  void poll_feedback_once();
  void enable_all();
  void disable_all();
  void shutdown();
  void close_bus();
  void set_tx_gap(std::chrono::microseconds gap);

 private:
  void send_with_pacing(const CanFrame& frame);
  void start_polling();
  void polling_loop();
  std::vector<std::shared_ptr<MotorHandle>> sorted_motors() const;

  class PacingBus;

  std::shared_ptr<PacingBus> bus_;
  mutable std::mutex motors_mutex_;
  std::map<uint16_t, std::shared_ptr<MotorHandle>> motors_;
  std::atomic<bool> polling_active_{false};
  std::thread polling_thread_;
  mutable std::mutex recv_mutex_;
};

}  // namespace damiao
