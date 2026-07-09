#include "damiao/runtime.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace damiao {

class Controller::PacingBus final : public CanBus {
 public:
  explicit PacingBus(std::shared_ptr<CanBus> inner) : inner_(std::move(inner)) {}

  void send(const CanFrame& frame) override {
    const auto gap = tx_gap_.load(std::memory_order_acquire);
    if (gap > 0) {
      const auto min_gap = std::chrono::microseconds(gap);
      std::unique_lock<std::mutex> lock(send_mutex_);
      if (last_send_.has_value()) {
        const auto elapsed = std::chrono::steady_clock::now() - *last_send_;
        if (elapsed < min_gap) {
          lock.unlock();
          std::this_thread::sleep_for(min_gap - elapsed);
          lock.lock();
        }
      }
      last_send_ = std::chrono::steady_clock::now();
    }
    inner_->send(frame);
  }

  std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) override {
    return inner_->receive_for(timeout);
  }

  void shutdown() override { inner_->shutdown(); }

  void set_tx_gap(std::chrono::microseconds gap) {
    tx_gap_.store(static_cast<uint64_t>(gap.count()), std::memory_order_release);
  }

 private:
  std::shared_ptr<CanBus> inner_;
  std::atomic<uint64_t> tx_gap_{0};
  std::mutex send_mutex_;
  std::optional<std::chrono::steady_clock::time_point> last_send_;
};

MotorHandle::MotorHandle(std::shared_ptr<CanBus> bus, uint16_t motor_id, uint16_t feedback_id,
                         std::string model)
    : bus_(std::move(bus)),
      motor_id_(motor_id),
      feedback_id_(feedback_id),
      model_(std::move(model)),
      limits_(model_limits(model_)) {}

void MotorHandle::send_raw(uint32_t arbitration_id, std::array<uint8_t, 8> data) {
  bus_->send(CanFrame{arbitration_id, data});
}

void MotorHandle::send_to_motor(std::array<uint8_t, 8> data) {
  send_raw(motor_id_, data);
}

void MotorHandle::send_mode_frame(uint32_t base_id, std::array<uint8_t, 8> data) {
  send_raw(base_id + motor_id_, data);
}

void MotorHandle::enable() {
  send_to_motor(encode_enable_command());
}

void MotorHandle::disable() {
  send_to_motor(encode_disable_command());
}

void MotorHandle::clear_error() {
  send_to_motor(encode_clear_error_command());
}

void MotorHandle::set_zero_position() {
  send_to_motor(encode_set_zero_command());
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void MotorHandle::send_mit(float pos, float vel, float kp, float kd, float tau) {
  send_to_motor(encode_mit_command(pos, vel, tau, kp, kd, limits_));
}

void MotorHandle::send_pos_vel(float pos, float velocity_limit) {
  send_mode_frame(0x100, encode_position_velocity_command(pos, velocity_limit));
}

void MotorHandle::send_vel(float velocity) {
  send_mode_frame(0x200, encode_velocity_command(velocity));
}

void MotorHandle::send_force_pos(float pos, float velocity_limit, float torque_limit_ratio) {
  send_mode_frame(0x300,
                  encode_force_position_command(pos, velocity_limit, torque_limit_ratio));
}

void MotorHandle::request_feedback() {
  send_raw(0x7FF, encode_feedback_request_command(motor_id_));
}

void MotorHandle::store_parameters() {
  send_raw(0x7FF, encode_store_parameters_command(motor_id_));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void MotorHandle::write_register_raw(uint8_t rid, std::array<uint8_t, 4> data) {
  send_raw(0x7FF, encode_register_write_command(motor_id_, rid, data));
}

void MotorHandle::write_register_f32(uint8_t rid, float value) {
  std::array<uint8_t, 4> data{};
  std::memcpy(data.data(), &value, sizeof(value));
  std::runtime_error last_error("register write ack not received");
  for (int attempt = 0; attempt < 3; ++attempt) {
    write_register_raw(rid, data);
    try {
      wait_for_write_ack(rid, data, std::chrono::milliseconds(120));
      return;
    } catch (const std::runtime_error& err) {
      last_error = err;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  throw last_error;
}

void MotorHandle::write_register_u32(uint8_t rid, uint32_t value) {
  std::array<uint8_t, 4> data{
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>((value >> 16) & 0xFF),
      static_cast<uint8_t>((value >> 24) & 0xFF),
  };
  std::runtime_error last_error("register write ack not received");
  for (int attempt = 0; attempt < 3; ++attempt) {
    write_register_raw(rid, data);
    try {
      wait_for_write_ack(rid, data, std::chrono::milliseconds(120));
      return;
    } catch (const std::runtime_error& err) {
      last_error = err;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  throw last_error;
}

std::array<uint8_t, 4> MotorHandle::wait_for_register(uint8_t rid,
                                                       std::chrono::milliseconds timeout) {
  const auto request_at = std::chrono::steady_clock::now();
  send_raw(0x7FF, encode_register_read_command(motor_id_, rid));
  const auto deadline = request_at + timeout;
  for (;;) {
    {
      std::lock_guard<std::mutex> lock(register_mutex_);
      const auto found = register_replies_.find(rid);
      if (found != register_replies_.end() && found->second.second >= request_at) {
        return found->second.first;
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("register read timed out");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void MotorHandle::wait_for_write_ack(uint8_t rid,
                                     std::array<uint8_t, 4> expected,
                                     std::chrono::milliseconds timeout) {
  const auto request_at = std::chrono::steady_clock::now();
  const auto deadline = request_at + timeout;
  for (;;) {
    {
      std::lock_guard<std::mutex> lock(register_mutex_);
      const auto found = register_acks_.find(rid);
      if (found != register_acks_.end() && found->second.second >= request_at) {
        if (found->second.first == expected) {
          return;
        }
        throw std::runtime_error("register write ack mismatched expected value");
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("register write ack timed out");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

float MotorHandle::get_register_f32(uint8_t rid, std::chrono::milliseconds timeout) {
  const auto raw = wait_for_register(rid, timeout);
  float value = 0.0f;
  std::memcpy(&value, raw.data(), sizeof(value));
  return value;
}

uint32_t MotorHandle::get_register_u32(uint8_t rid, std::chrono::milliseconds timeout) {
  const auto raw = wait_for_register(rid, timeout);
  return static_cast<uint32_t>(raw[0]) | (static_cast<uint32_t>(raw[1]) << 8) |
         (static_cast<uint32_t>(raw[2]) << 16) | (static_cast<uint32_t>(raw[3]) << 24);
}

void MotorHandle::ensure_mode(uint32_t mode, std::chrono::milliseconds timeout) {
  if (mode < 1 || mode > 4) {
    throw std::invalid_argument("Damiao mode must be 1(MIT) / 2(POS_VEL) / 3(VEL) / 4(FORCE_POS)");
  }
  const auto current = get_register_u32(10, std::min(timeout, std::chrono::milliseconds(250)));
  if (current == mode) {
    return;
  }
  write_register_u32(10, mode);
  switch (mode) {
    case 1:
      send_mit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      break;
    case 2:
      send_pos_vel(0.0f, 0.0f);
      break;
    case 3:
      send_vel(0.0f);
      break;
    case 4:
      send_force_pos(0.0f, 0.0f, 0.0f);
      break;
  }
}

void MotorHandle::set_can_timeout_ms(uint32_t timeout_ms) {
  write_register_u32(11, timeout_ms);
}

bool MotorHandle::accepts_frame(const CanFrame& frame) const {
  return frame.id == feedback_id_ ||
         ((frame.data[0] & 0x0F) == static_cast<uint8_t>(motor_id_ & 0x0F));
}

void MotorHandle::process_feedback_frame(const CanFrame& frame) {
  if (is_register_reply(frame.data)) {
    const auto value = decode_register_value(frame.data);
    std::lock_guard<std::mutex> lock(register_mutex_);
    register_replies_[value.rid] = {value.data, std::chrono::steady_clock::now()};
    return;
  }
  if (is_register_write_ack(frame.data)) {
    const uint8_t rid = frame.data[3];
    std::array<uint8_t, 4> data{frame.data[4], frame.data[5], frame.data[6], frame.data[7]};
    std::lock_guard<std::mutex> lock(register_mutex_);
    register_acks_[rid] = {data, std::chrono::steady_clock::now()};
    return;
  }

  const auto decoded = decode_sensor_feedback(frame.data, limits_);
  MotorState state;
  state.can_id = decoded.can_id;
  state.arbitration_id = frame.id;
  state.status_code = decoded.status_code;
  state.pos = decoded.pos;
  state.vel = decoded.vel;
  state.torq = decoded.torq;
  state.t_mos = decoded.t_mos;
  state.t_rotor = decoded.t_rotor;

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = state;
}

std::optional<MotorState> MotorHandle::latest_state() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

Controller::Controller(std::shared_ptr<CanBus> bus) : bus_(std::make_shared<PacingBus>(bus)) {}

Controller::~Controller() {
  try {
    shutdown();
  } catch (...) {
  }
}

std::shared_ptr<MotorHandle> Controller::add_damiao_motor(uint16_t motor_id,
                                                          uint16_t feedback_id,
                                                          const std::string& model) {
  auto motor = std::make_shared<MotorHandle>(bus_, motor_id, feedback_id, model);
  {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    if (motors_.find(motor_id) != motors_.end()) {
      throw std::invalid_argument("device with motor_id already exists");
    }
    motors_[motor_id] = motor;
    if (motors_.size() >= 2) {
      bus_->set_tx_gap(std::chrono::microseconds(120));
    }
  }
  start_polling();
  return motor;
}

std::vector<std::shared_ptr<MotorHandle>> Controller::sorted_motors() const {
  std::vector<std::shared_ptr<MotorHandle>> motors;
  std::lock_guard<std::mutex> lock(motors_mutex_);
  for (const auto& kv : motors_) {
    motors.push_back(kv.second);
  }
  return motors;
}

void Controller::poll_feedback_once() {
  std::lock_guard<std::mutex> recv_lock(recv_mutex_);
  for (;;) {
    const auto frame = bus_->receive_for(std::chrono::milliseconds(0));
    if (!frame.has_value()) {
      return;
    }
    const auto motors = sorted_motors();
    for (const auto& motor : motors) {
      if (motor->accepts_frame(*frame)) {
        motor->process_feedback_frame(*frame);
        break;
      }
    }
  }
}

void Controller::enable_all() {
  const auto motors = sorted_motors();
  for (std::size_t i = 0; i < motors.size(); ++i) {
    motors[i]->enable();
    poll_feedback_once();
    if (i + 1 < motors.size()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

void Controller::disable_all() {
  const auto motors = sorted_motors();
  for (std::size_t i = 0; i < motors.size(); ++i) {
    motors[i]->disable();
    poll_feedback_once();
    if (i + 1 < motors.size()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

void Controller::start_polling() {
  bool expected = false;
  if (!polling_active_.compare_exchange_strong(expected, true)) {
    return;
  }
  polling_thread_ = std::thread([this] { polling_loop(); });
}

void Controller::polling_loop() {
  while (polling_active_.load(std::memory_order_acquire)) {
    bool had_frame = false;
    {
      std::lock_guard<std::mutex> recv_lock(recv_mutex_);
      const auto frame = bus_->receive_for(std::chrono::milliseconds(0));
      if (frame.has_value()) {
        had_frame = true;
        const auto motors = sorted_motors();
        for (const auto& motor : motors) {
          if (motor->accepts_frame(*frame)) {
            motor->process_feedback_frame(*frame);
            break;
          }
        }
      }
    }
    if (!had_frame) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  }
}

void Controller::shutdown() {
  polling_active_.store(false, std::memory_order_release);
  if (polling_thread_.joinable()) {
    polling_thread_.join();
  }
  bus_->shutdown();
}

void Controller::close_bus() {
  shutdown();
}

void Controller::set_tx_gap(std::chrono::microseconds gap) {
  bus_->set_tx_gap(gap);
}

}  // namespace damiao
