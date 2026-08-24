#include "damiao/runtime.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace damiao {
namespace {

constexpr auto kRegisterWriteAckTimeout = std::chrono::milliseconds(50);
constexpr auto kRegisterWriteRetryGap = std::chrono::milliseconds(20);
constexpr auto kBulkFeedbackRetryDelay = std::chrono::milliseconds(5);
constexpr auto kDefaultMultiMotorTxGap = std::chrono::microseconds(120);

bool matches_feedback_arbitration_id(uint32_t arbitration_id,
                                     uint16_t configured_feedback_id,
                                     uint16_t motor_id) {
  const auto compact_id = static_cast<uint16_t>(motor_id & 0x0FU);
  const auto configured_feedback = static_cast<uint16_t>(0x10U | compact_id);
  // DM-USB2FDCAN with gs_usb exposes arm feedback as 0x200 | motor_id on
  // SocketCAN-FD, while the gripper and some firmware revisions use the
  // configured 0x10 | motor_id form. This is a wire-protocol identity alias;
  // it does not depend on or call the removed DM Device SDK.
  const auto socketcanfd_feedback = static_cast<uint32_t>(0x200U | compact_id);
  return arbitration_id == configured_feedback_id ||
         (configured_feedback_id == configured_feedback &&
          arbitration_id == socketcanfd_feedback);
}

void validate_register(uint8_t rid, RegisterDataType expected_type, bool writing) {
  const auto info = register_info(rid);
  if (!info.has_value()) {
    throw std::invalid_argument("unknown Damiao register: " + std::to_string(rid));
  }
  if (writing && info->access != RegisterAccess::ReadWrite) {
    throw std::invalid_argument("Damiao register is read-only: " + std::to_string(rid));
  }
  if (info->data_type != expected_type) {
    throw std::invalid_argument("Damiao register has a different data type: " +
                                std::to_string(rid));
  }
}

}  // namespace

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
          std::this_thread::sleep_for(min_gap - elapsed);
        }
      }
      last_send_ = std::chrono::steady_clock::now();
    }
    try {
      inner_->send(frame);
      tx_frames_.fetch_add(1, std::memory_order_relaxed);
      send_healthy_.store(true, std::memory_order_release);
      std::lock_guard<std::mutex> lock(health_mutex_);
      last_tx_ = std::chrono::steady_clock::now();
    } catch (const std::exception& error) {
      record_error(error.what(), true);
      throw;
    } catch (...) {
      record_error("unknown transport send error", true);
      throw;
    }
  }

  std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) override {
    try {
      auto frame = inner_->receive_for(timeout);
      if (frame.has_value()) {
        rx_frames_.fetch_add(1, std::memory_order_relaxed);
        receive_healthy_.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lock(health_mutex_);
        last_rx_ = std::chrono::steady_clock::now();
      }
      return frame;
    } catch (const std::exception& error) {
      record_error(error.what(), false);
      throw;
    } catch (...) {
      record_error("unknown transport receive error", false);
      throw;
    }
  }

  void shutdown() override {
    try {
      inner_->shutdown();
    } catch (const std::exception& error) {
      record_error(error.what(), false);
      connected_.store(false, std::memory_order_release);
      throw;
    } catch (...) {
      record_error("unknown transport shutdown error", false);
      connected_.store(false, std::memory_order_release);
      throw;
    }
    connected_.store(false, std::memory_order_release);
  }

  TransportCapabilities capabilities() const override { return inner_->capabilities(); }

  TransportHealth health() const override {
    TransportHealth value;
    value.connected = connected_.load(std::memory_order_acquire);
    value.healthy = value.connected &&
                    send_healthy_.load(std::memory_order_acquire) &&
                    receive_healthy_.load(std::memory_order_acquire);
    value.tx_frames = tx_frames_.load(std::memory_order_relaxed);
    value.rx_frames = rx_frames_.load(std::memory_order_relaxed);
    value.send_errors = send_errors_.load(std::memory_order_relaxed);
    value.receive_errors = receive_errors_.load(std::memory_order_relaxed);
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(health_mutex_);
    if (last_tx_.has_value()) value.last_tx_age = now - *last_tx_;
    if (last_rx_.has_value()) value.last_rx_age = now - *last_rx_;
    value.last_error = last_error_;
    return value;
  }

  void set_tx_gap(std::chrono::microseconds gap) {
    tx_gap_.store(static_cast<uint64_t>(gap.count()), std::memory_order_release);
  }

 private:
  void record_error(const std::string& error, bool sending) {
    if (sending) {
      send_errors_.fetch_add(1, std::memory_order_relaxed);
      send_healthy_.store(false, std::memory_order_release);
    } else {
      receive_errors_.fetch_add(1, std::memory_order_relaxed);
      receive_healthy_.store(false, std::memory_order_release);
    }
    std::lock_guard<std::mutex> lock(health_mutex_);
    last_error_ = error;
  }

  std::shared_ptr<CanBus> inner_;
  std::atomic<uint64_t> tx_gap_{0};
  std::mutex send_mutex_;
  std::optional<std::chrono::steady_clock::time_point> last_send_;
  std::atomic<bool> connected_{true};
  std::atomic<bool> send_healthy_{true};
  std::atomic<bool> receive_healthy_{true};
  std::atomic<uint64_t> tx_frames_{0};
  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> send_errors_{0};
  std::atomic<uint64_t> receive_errors_{0};
  mutable std::mutex health_mutex_;
  std::optional<std::chrono::steady_clock::time_point> last_tx_;
  std::optional<std::chrono::steady_clock::time_point> last_rx_;
  std::string last_error_;
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
  disabled_hint_.store(false, std::memory_order_release);
}

void MotorHandle::disable() {
  send_to_motor(encode_disable_command());
  disabled_hint_.store(true, std::memory_order_release);
}

void MotorHandle::clear_error() {
  send_to_motor(encode_clear_error_command());
}

void MotorHandle::set_zero_position() {
  if (!disabled_hint_.load(std::memory_order_acquire)) {
    throw std::invalid_argument("motor is not disabled; call disable() before set_zero_position()");
  }
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
  const auto state = request_fresh_state(std::chrono::milliseconds(50));
  if (!state.has_value() || state->status_code != 0x0) {
    disable();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  send_raw(0x7FF, encode_store_parameters_command(motor_id_));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

std::optional<MotorState> MotorHandle::request_fresh_state(std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  uint64_t previous_count = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    previous_count = feedback_update_count_;
  }
  request_feedback();
  if (!wait_for_feedback_after(previous_count, deadline)) return std::nullopt;
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

bool MotorHandle::wait_for_feedback_after(
    uint64_t previous_count, std::chrono::steady_clock::time_point deadline) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  return state_cv_.wait_until(
      lock, deadline, [&] { return feedback_update_count_ > previous_count; });
}

uint64_t MotorHandle::write_register_raw(uint8_t rid,
                                         std::array<uint8_t, 4> data) {
  uint64_t previous_update_count = 0;
  {
    std::lock_guard<std::mutex> lock(register_mutex_);
    previous_update_count = register_ack_update_counts_[rid];
  }
  send_raw(0x7FF, encode_register_write_command(motor_id_, rid, data));
  return previous_update_count;
}

void MotorHandle::write_register_f32(uint8_t rid, float value) {
  validate_register(rid, RegisterDataType::Float, true);
  std::array<uint8_t, 4> data{};
  std::memcpy(data.data(), &value, sizeof(value));
  std::runtime_error last_error("register write ack not received");
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto previous_update_count = write_register_raw(rid, data);
    try {
      wait_for_write_ack(
          rid, data, previous_update_count, kRegisterWriteAckTimeout);
      return;
    } catch (const std::runtime_error& err) {
      last_error = err;
      if (attempt + 1 < 3) std::this_thread::sleep_for(kRegisterWriteRetryGap);
    }
  }
  throw last_error;
}

void MotorHandle::write_register_u32(uint8_t rid, uint32_t value) {
  validate_register(rid, RegisterDataType::UInt32, true);
  std::array<uint8_t, 4> data{
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>((value >> 16) & 0xFF),
      static_cast<uint8_t>((value >> 24) & 0xFF),
  };
  std::runtime_error last_error("register write ack not received");
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto previous_update_count = write_register_raw(rid, data);
    try {
      wait_for_write_ack(
          rid, data, previous_update_count, kRegisterWriteAckTimeout);
      return;
    } catch (const std::runtime_error& err) {
      last_error = err;
      if (attempt + 1 < 3) std::this_thread::sleep_for(kRegisterWriteRetryGap);
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
                                     uint64_t previous_update_count,
                                     std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;;) {
    {
      std::lock_guard<std::mutex> lock(register_mutex_);
      const auto update = register_ack_update_counts_.find(rid);
      const auto found = register_acks_.find(rid);
      if (update != register_ack_update_counts_.end() &&
          update->second > previous_update_count &&
          found != register_acks_.end()) {
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
  validate_register(rid, RegisterDataType::Float, false);
  const auto raw = wait_for_register(rid, timeout);
  float value = 0.0f;
  std::memcpy(&value, raw.data(), sizeof(value));
  return value;
}

uint32_t MotorHandle::get_register_u32(uint8_t rid, std::chrono::milliseconds timeout) {
  validate_register(rid, RegisterDataType::UInt32, false);
  const auto raw = wait_for_register(rid, timeout);
  return static_cast<uint32_t>(raw[0]) | (static_cast<uint32_t>(raw[1]) << 8) |
         (static_cast<uint32_t>(raw[2]) << 16) | (static_cast<uint32_t>(raw[3]) << 24);
}

void MotorHandle::ensure_mode(uint32_t mode, std::chrono::milliseconds timeout) {
  if (mode < 1 || mode > 4) {
    throw std::invalid_argument("Damiao mode must be 1(MIT) / 2(POS_VEL) / 3(VEL) / 4(FORCE_POS)");
  }
  if (timeout <= std::chrono::milliseconds::zero()) {
    throw std::runtime_error("control mode switch timed out");
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const auto mode_read_cap =
      std::min(std::max(timeout / 4, std::chrono::milliseconds(2)),
               std::chrono::milliseconds(100));
  const auto hold_read_cap =
      std::min(std::max(timeout / 6, std::chrono::milliseconds(2)),
               std::chrono::milliseconds(30));
  std::string last_error = "control mode verify failed";

  const auto remaining = [&]() -> std::optional<std::chrono::milliseconds> {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return std::nullopt;
    return std::max(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now),
                    std::chrono::milliseconds(1));
  };
  const auto capped_remaining = [&](std::chrono::milliseconds cap)
      -> std::optional<std::chrono::milliseconds> {
    const auto value = remaining();
    if (!value.has_value()) return std::nullopt;
    return std::min(*value, cap);
  };
  const auto sleep_reserving = [&](std::chrono::milliseconds duration,
                                   std::chrono::milliseconds reserve) {
    const auto value = remaining();
    if (value.has_value() && *value > reserve) {
      std::this_thread::sleep_for(std::min(duration, *value - reserve));
    }
  };
  const auto is_timeout = [](const std::runtime_error& error) {
    return std::string(error.what()).find("timed out") != std::string::npos;
  };

  if (const auto read_timeout = capped_remaining(mode_read_cap)) {
    try {
      const auto current = get_register_u32(10, *read_timeout);
      if (current == mode) return;
      last_error = "control mode verify failed: expected " + std::to_string(mode) +
                   ", got " + std::to_string(current);
    } catch (const std::runtime_error& error) {
      if (!is_timeout(error)) throw;
      last_error = error.what();
    }
  } else {
    throw std::runtime_error("control mode switch timed out before register 10 could be read");
  }

  std::optional<float> hold_position;
  if (mode != 3) {
    if (const auto read_timeout = capped_remaining(hold_read_cap)) {
      try {
        hold_position = get_register_f32(80, *read_timeout);
      } catch (const std::runtime_error&) {
      }
    }
  }
  const float hold = hold_position.value_or(0.0f);
  try {
    send_vel(0.0f);
  } catch (...) {
  }
  try {
    send_pos_vel(hold, 0.0f);
  } catch (...) {
  }
  try {
    send_force_pos(hold, 0.0f, 0.0f);
  } catch (...) {
  }
  try {
    send_mit(hold, 0.0f, 0.0f, 0.0f, 0.0f);
  } catch (...) {
  }

  const auto finish_switch = [&]() {
    switch (mode) {
      case 1:
        send_mit(hold, 0.0f, 0.0f, 0.0f, 0.0f);
        break;
      case 2:
        if (hold_position.has_value()) send_pos_vel(*hold_position, 0.0f);
        break;
      case 3:
        send_vel(0.0f);
        break;
      case 4:
        if (hold_position.has_value()) send_force_pos(*hold_position, 0.0f, 0.0f);
        break;
    }
  };

  write_register_u32(10, mode);
  sleep_reserving(std::chrono::milliseconds(20), mode_read_cap);
  finish_switch();

  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto read_timeout = capped_remaining(mode_read_cap);
    if (!read_timeout.has_value()) break;
    try {
      const auto current = get_register_u32(10, *read_timeout);
      if (current == mode) return;
      last_error = "control mode verify failed: expected " + std::to_string(mode) +
                   ", got " + std::to_string(current);
    } catch (const std::runtime_error& error) {
      if (!is_timeout(error)) throw;
      last_error = error.what();
    }
    if (attempt + 1 < 3) {
      write_register_u32(10, mode);
      sleep_reserving(std::chrono::milliseconds(20), mode_read_cap);
      finish_switch();
    }
  }

  throw std::runtime_error(last_error);
}

void MotorHandle::set_can_timeout_ms(uint32_t timeout_ms) {
  const auto ticks = std::min<uint64_t>(static_cast<uint64_t>(timeout_ms) * 20,
                                        std::numeric_limits<uint32_t>::max());
  write_register_u32(9, static_cast<uint32_t>(ticks));
}

bool MotorHandle::accepts_frame(const CanFrame& frame) const {
  if (frame.is_extended ||
      !matches_feedback_arbitration_id(frame.id, feedback_id_, motor_id_)) {
    return false;
  }
  if (frame.dlc != 8) {
    record_feedback_rejection(
        FeedbackRejectionReason::ShortFrame, frame, 0, 0.0f, 0.0f, 0.0f,
        "feedback frame is shorter than the required 8-byte Damiao payload");
    return false;
  }
  const bool register_frame =
      is_register_reply(frame.data) || is_register_write_ack(frame.data);
  const uint16_t decoded_can_id = register_frame
      ? static_cast<uint16_t>(frame.data[0]) |
            (static_cast<uint16_t>(frame.data[1]) << 8)
      : static_cast<uint16_t>(frame.data[0] & 0x0F);
  const uint16_t expected_can_id = register_frame
      ? motor_id_
      : static_cast<uint16_t>(motor_id_ & 0x0F);
  if (decoded_can_id != expected_can_id) {
    record_feedback_rejection(
        FeedbackRejectionReason::IdentityMismatch, frame, decoded_can_id,
        0.0f, 0.0f, 0.0f,
        "feedback arbitration ID matched but payload CAN ID did not match the MotorHandle");
    return false;
  }
  return true;
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
    ++register_ack_update_counts_[rid];
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

  const auto now = std::chrono::steady_clock::now();
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    if (state_.has_value() && state_time_.has_value()) {
      const float elapsed = std::max(
          0.0f,
          std::chrono::duration<float>(now - *state_time_).count());
      const float observed_velocity = std::max(
          std::fabs(state_->vel), std::fabs(decoded.vel));
      const float velocity_bound = std::max(
          observed_velocity * 4.0f, std::fabs(limits_.v_max) * 1.5f);
      const float allowed_delta = std::max(
          0.15f, 0.03f + velocity_bound * elapsed);
      const float actual_delta = std::fabs(decoded.pos - state_->pos);
      if (actual_delta > allowed_delta) {
        const float previous_position = state_->pos;
        lock.unlock();
        std::ostringstream message;
        message << "rejected implausible feedback position jump: channel="
                << static_cast<unsigned>(frame.channel)
                << ", arbitration_id=0x" << std::hex << frame.id << std::dec
                << ", can_id=" << static_cast<unsigned>(decoded.can_id)
                << ", previous_position=" << previous_position
                << ", position=" << decoded.pos
                << ", actual_delta=" << actual_delta
                << ", allowed_delta=" << allowed_delta
                << ", elapsed_s=" << elapsed;
        record_feedback_rejection(
            FeedbackRejectionReason::ImplausiblePositionJump, frame,
            decoded.can_id, decoded.pos, previous_position, allowed_delta,
            message.str());
        return;
      }
    }
    state_ = state;
    state_time_ = now;
    ++feedback_update_count_;
  }
  state_cv_.notify_all();
}

void MotorHandle::record_feedback_rejection(
    FeedbackRejectionReason reason, const CanFrame& frame,
    uint16_t decoded_can_id, float position, float previous_position,
    float allowed_position_delta, const std::string& error) const {
  std::lock_guard<std::mutex> lock(integrity_mutex_);
  ++integrity_stats_.rejected_frame_count;
  if (reason == FeedbackRejectionReason::ShortFrame) {
    ++integrity_stats_.short_frame_count;
  } else if (reason == FeedbackRejectionReason::IdentityMismatch) {
    ++integrity_stats_.identity_mismatch_count;
  } else if (reason == FeedbackRejectionReason::ImplausiblePositionJump) {
    ++integrity_stats_.implausible_position_jump_count;
  }
  integrity_stats_.last_reason = reason;
  integrity_stats_.channel = frame.channel;
  integrity_stats_.arbitration_id = frame.id;
  integrity_stats_.expected_arbitration_id = feedback_id_;
  integrity_stats_.decoded_can_id = decoded_can_id;
  integrity_stats_.expected_can_id = motor_id_;
  integrity_stats_.position = position;
  integrity_stats_.previous_position = previous_position;
  integrity_stats_.allowed_position_delta = allowed_position_delta;
  integrity_stats_.error = error;
}

FeedbackIntegrityStats MotorHandle::feedback_integrity_stats() const {
  std::lock_guard<std::mutex> lock(integrity_mutex_);
  return integrity_stats_;
}

std::optional<MotorState> MotorHandle::latest_state() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

MotorStateSnapshot MotorHandle::state_snapshot() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  MotorStateSnapshot snapshot;
  snapshot.state = state_;
  snapshot.feedback.update_count = feedback_update_count_;
  if (state_time_.has_value()) {
    snapshot.feedback.has_feedback = true;
    snapshot.feedback.age = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - *state_time_);
  }
  return snapshot;
}

FeedbackStats MotorHandle::feedback_stats() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  FeedbackStats stats;
  stats.update_count = feedback_update_count_;
  if (state_time_.has_value()) {
    stats.has_feedback = true;
    stats.age = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - *state_time_);
  }
  return stats;
}

Controller::Controller(std::shared_ptr<CanBus> bus, std::string endpoint_label)
    : bus_(std::make_shared<PacingBus>(bus)),
      endpoint_label_(std::move(endpoint_label)) {
  if (const char* raw = std::getenv("MOTOR_DRIVE_LAYER_TX_GAP_US")) {
    uint64_t gap_us = 0;
    const auto* end = raw + std::strlen(raw);
    const auto parsed = std::from_chars(raw, end, gap_us);
    if (parsed.ec == std::errc{} && parsed.ptr == end) {
      const auto capped = std::min<uint64_t>(gap_us, std::numeric_limits<int64_t>::max());
      bus_->set_tx_gap(std::chrono::microseconds(static_cast<int64_t>(capped)));
      tx_gap_env_override_ = true;
    }
  }
  if (const char* raw = std::getenv("MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS")) {
    uint64_t gap_ms = 0;
    const auto* end = raw + std::strlen(raw);
    const auto parsed = std::from_chars(raw, end, gap_ms);
    if (parsed.ec == std::errc{} && parsed.ptr == end) {
      const auto capped = std::min<uint64_t>(gap_ms, std::numeric_limits<int64_t>::max());
      bulk_op_gap_ = std::chrono::milliseconds(static_cast<int64_t>(capped));
    }
  }
}

Controller::~Controller() {
  try {
    close_bus();
  } catch (...) {
  }
}

TransportCapabilities Controller::transport_capabilities() const {
  return bus_->capabilities();
}

TransportHealth Controller::transport_health() const {
  return bus_->health();
}

std::shared_ptr<MotorHandle> Controller::add_damiao_motor(uint16_t motor_id,
                                                          uint16_t feedback_id,
                                                          const std::string& model) {
  auto motor = std::make_shared<MotorHandle>(bus_, motor_id, feedback_id, model);
  {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    if (discovery_finalized_) {
      throw std::logic_error(
          "controller motor set is fixed for this connection after discovery");
    }
    if (motors_.find(motor_id) != motors_.end()) {
      throw std::invalid_argument("device with motor_id already exists");
    }
    motors_[motor_id] = motor;
    if (!tx_gap_env_override_ && motors_.size() >= 2) {
      bus_->set_tx_gap(kDefaultMultiMotorTxGap);
    }
  }
  start_polling();
  return motor;
}

std::vector<MotorDiscoveryResult> Controller::discover_damiao_motors(
    const std::vector<MotorCandidate>& candidates,
    std::chrono::milliseconds timeout,
    uint32_t retry_count) {
  if (timeout <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("motor discovery timeout must be positive");
  }
  if (retry_count > 10) {
    throw std::invalid_argument("motor discovery retry_count must be in 0..=10");
  }

  std::set<std::string> roles;
  std::set<uint16_t> motor_ids;
  std::set<uint16_t> feedback_ids;
  for (const auto& candidate : candidates) {
    if (candidate.role.empty()) {
      throw std::invalid_argument("motor discovery candidate role is empty");
    }
    if (!roles.insert(candidate.role).second) {
      throw std::invalid_argument("duplicate motor discovery role: " + candidate.role);
    }
    if (candidate.policy != PresencePolicy::Required &&
        candidate.policy != PresencePolicy::Optional &&
        candidate.policy != PresencePolicy::Disabled) {
      throw std::invalid_argument("invalid presence policy for role " + candidate.role);
    }
    if (candidate.policy == PresencePolicy::Disabled) continue;
    if (candidate.model.empty()) {
      throw std::invalid_argument("motor model is empty for role " + candidate.role);
    }
    if (!motor_ids.insert(candidate.motor_id).second) {
      throw std::invalid_argument("duplicate active motor_id in discovery: " +
                                  std::to_string(candidate.motor_id));
    }
    if (!feedback_ids.insert(candidate.feedback_id).second) {
      throw std::invalid_argument("duplicate active feedback_id in discovery: " +
                                  std::to_string(candidate.feedback_id));
    }
  }

  {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    if (discovery_finalized_) {
      throw std::logic_error("motor discovery is already finalized for this connection");
    }
    if (!motors_.empty()) {
      throw std::logic_error(
          "motor discovery must run before adding individual motors");
    }
  }

  std::vector<MotorDiscoveryResult> results;
  results.reserve(candidates.size());
  std::vector<std::size_t> pending;
  std::vector<uint64_t> previous_counts(candidates.size(), 0);
  try {
    {
      std::lock_guard<std::mutex> lock(motors_mutex_);
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        MotorDiscoveryResult result;
        result.candidate = candidates[i];
        if (candidates[i].policy == PresencePolicy::Disabled) {
          result.reason = "disabled by policy";
        } else {
          result.motor = std::make_shared<MotorHandle>(
              bus_, candidates[i].motor_id, candidates[i].feedback_id,
              candidates[i].model);
          motors_.emplace(candidates[i].motor_id, result.motor);
          previous_counts[i] = result.motor->feedback_stats().update_count;
          pending.push_back(i);
        }
        results.push_back(std::move(result));
      }
      if (!tx_gap_env_override_ && motors_.size() >= 2) {
        bus_->set_tx_gap(kDefaultMultiMotorTxGap);
      }
    }
    if (!pending.empty()) start_polling();

    for (uint32_t attempt = 0; attempt <= retry_count && !pending.empty(); ++attempt) {
      const auto deadline = std::chrono::steady_clock::now() + timeout;
      for (const auto index : pending) {
        results[index].motor->request_feedback();
      }

      std::vector<std::size_t> still_pending;
      for (const auto index : pending) {
        auto& result = results[index];
        if (!result.motor->wait_for_feedback_after(previous_counts[index], deadline)) {
          result.reason = "fresh feedback timed out on attempt " +
                          std::to_string(attempt + 1) + " of " +
                          std::to_string(retry_count + 1);
          still_pending.push_back(index);
          continue;
        }

        const auto state = result.motor->latest_state();
        const auto expected_can_id =
            static_cast<uint8_t>(result.candidate.motor_id & 0x0F);
        // DM-USB2FDCAN Dual reports motor feedback as 0x200 | motor_id,
        // while the same motor's configured MST_ID is conventionally
        // 0x10 | motor_id. Other transports report the configured feedback
        // ID directly. Accept those two protocol-defined representations,
        // but still reject arbitrary arbitration IDs that merely contain a
        // matching motor nibble in the payload.
        const bool arbitration_id_matches =
            state.has_value() &&
            matches_feedback_arbitration_id(
                state->arbitration_id, result.candidate.feedback_id,
                result.candidate.motor_id);
        if (!state.has_value() || state->can_id != expected_can_id ||
            !arbitration_id_matches) {
          std::ostringstream reason;
          reason << "received feedback with mismatched identity; expected motor_id="
                 << result.candidate.motor_id << " feedback_id="
                 << result.candidate.feedback_id;
          if (state.has_value()) {
            reason << ", got can_id=" << static_cast<unsigned>(state->can_id)
                   << " arbitration_id=" << state->arbitration_id;
          }
          result.reason = reason.str();
          previous_counts[index] = result.motor->feedback_stats().update_count;
          still_pending.push_back(index);
          continue;
        }

        result.state = PresenceState::Present;
        result.reason.clear();
      }
      pending = std::move(still_pending);
    }

    std::vector<std::size_t> missing_required;
    {
      std::lock_guard<std::mutex> lock(motors_mutex_);
      for (const auto index : pending) {
        const auto& candidate = results[index].candidate;
        if (candidate.policy == PresencePolicy::Required) {
          missing_required.push_back(index);
        } else {
          motors_.erase(candidate.motor_id);
          results[index].motor.reset();
        }
      }
      if (!missing_required.empty()) {
        for (const auto& candidate : candidates) {
          if (candidate.policy != PresencePolicy::Disabled) {
            motors_.erase(candidate.motor_id);
          }
        }
      } else {
        discovery_finalized_ = true;
      }
    }

    if (!missing_required.empty()) {
      std::ostringstream message;
      message << "motor discovery failed on " << endpoint_label_
              << "; missing required motors:";
      for (const auto index : missing_required) {
        const auto& result = results[index];
        message << " " << result.candidate.role
                << " (motor_id=" << result.candidate.motor_id
                << ", feedback_id=" << result.candidate.feedback_id
                << ", reason=" << result.reason << ")";
      }
      throw std::runtime_error(message.str());
    }
    return results;
  } catch (...) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    if (!discovery_finalized_) {
      for (const auto& candidate : candidates) {
        if (candidate.policy != PresencePolicy::Disabled) {
          motors_.erase(candidate.motor_id);
        }
      }
    }
    throw;
  }
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

FeedbackBatchReport Controller::request_feedback_all_report(
    std::chrono::milliseconds timeout) {
  if (timeout <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("feedback timeout must be positive");
  }
  const auto motors = sorted_motors();
  FeedbackBatchReport report;
  report.timeout = timeout;
  report.expected_count = static_cast<uint32_t>(motors.size());
  if (motors.empty()) return report;

  std::vector<uint64_t> previous_counts;
  previous_counts.reserve(motors.size());
  for (const auto& motor : motors) {
    previous_counts.push_back(motor->feedback_stats().update_count);
  }
  const auto capture_current_counts = [&] {
    report.missing_motor_ids.clear();
    for (std::size_t i = 0; i < motors.size(); ++i) {
      if (motors[i]->feedback_stats().update_count <= previous_counts[i]) {
        report.missing_motor_ids.push_back(motors[i]->motor_id());
      }
    }
    report.received_count = report.expected_count -
                            static_cast<uint32_t>(report.missing_motor_ids.size());
  };

  const auto health_before = bus_->health();
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  try {
    for (const auto& motor : motors) {
      motor->request_feedback();
    }
  } catch (const std::exception& error) {
    report.status = FeedbackBatchStatus::TransportError;
    report.error = error.what();
    capture_current_counts();
    return report;
  }

  // Some USB-CAN firmware revisions occasionally drop the final request or
  // response in a burst. Give the normal batch time to complete, then retry
  // only the motors that are still missing while preserving the caller's one
  // shared deadline.
  const auto retry_at =
      std::min(deadline, std::chrono::steady_clock::now() + kBulkFeedbackRetryDelay);
  std::vector<std::size_t> missing;
  for (std::size_t i = 0; i < motors.size(); ++i) {
    if (!motors[i]->wait_for_feedback_after(previous_counts[i], retry_at)) {
      missing.push_back(i);
    }
  }

  if (!missing.empty() && retry_at < deadline) {
    try {
      for (const auto i : missing) {
        if (std::chrono::steady_clock::now() >= deadline) break;
        motors[i]->request_feedback();
      }
    } catch (const std::exception& error) {
      report.status = FeedbackBatchStatus::TransportError;
      report.error = error.what();
      capture_current_counts();
      return report;
    }

    std::vector<std::size_t> still_missing;
    for (const auto i : missing) {
      if (!motors[i]->wait_for_feedback_after(previous_counts[i], deadline)) {
        still_missing.push_back(i);
      }
    }
    missing = std::move(still_missing);
  }

  for (const auto i : missing) {
    report.missing_motor_ids.push_back(motors[i]->motor_id());
  }
  report.received_count = report.expected_count -
                          static_cast<uint32_t>(report.missing_motor_ids.size());

  const auto health_after = bus_->health();
  if (!health_after.connected ||
      health_after.send_errors > health_before.send_errors ||
      health_after.receive_errors > health_before.receive_errors) {
    report.status = FeedbackBatchStatus::TransportError;
    report.error = health_after.last_error.empty()
        ? "transport disconnected or reported an I/O error during feedback request"
        : health_after.last_error;
  } else if (report.missing_motor_ids.empty()) {
    report.status = FeedbackBatchStatus::Ok;
  } else if (report.received_count == 0) {
    report.status = FeedbackBatchStatus::Timeout;
  } else {
    report.status = FeedbackBatchStatus::Incomplete;
  }
  return report;
}

void Controller::request_feedback_all(std::chrono::milliseconds timeout) {
  const auto report = request_feedback_all_report(timeout);
  if (report.status == FeedbackBatchStatus::Ok) return;
  if (report.status == FeedbackBatchStatus::TransportError) {
    throw std::runtime_error("feedback transport failed: " + report.error);
  }
  std::string message = "fresh feedback timed out; missing motor IDs:";
  for (const auto motor_id : report.missing_motor_ids) {
    message += " " + std::to_string(motor_id);
  }
  throw std::runtime_error(message);
}

void Controller::enable_all() {
  const auto motors = sorted_motors();
  for (std::size_t i = 0; i < motors.size(); ++i) {
    motors[i]->enable();
    try {
      poll_feedback_once();
    } catch (...) {
    }
    if (i + 1 < motors.size() && bulk_op_gap_ > std::chrono::milliseconds::zero()) {
      std::this_thread::sleep_for(bulk_op_gap_);
    }
  }
}

void Controller::disable_all() {
  const auto motors = sorted_motors();
  for (std::size_t i = 0; i < motors.size(); ++i) {
    motors[i]->disable();
    try {
      poll_feedback_once();
    } catch (...) {
    }
    if (i + 1 < motors.size() && bulk_op_gap_ > std::chrono::milliseconds::zero()) {
      std::this_thread::sleep_for(bulk_op_gap_);
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
  constexpr std::size_t kMaximumFramesPerDrain = 64;
  while (polling_active_.load(std::memory_order_acquire)) {
    bool had_frame = false;
    try {
      {
        std::lock_guard<std::mutex> recv_lock(recv_mutex_);
        const auto motors = sorted_motors();
        for (std::size_t drained = 0; drained < kMaximumFramesPerDrain;
             ++drained) {
          const auto frame = bus_->receive_for(std::chrono::milliseconds(0));
          if (!frame.has_value()) break;
          had_frame = true;
          for (const auto& motor : motors) {
            if (motor->accepts_frame(*frame)) {
              motor->process_feedback_frame(*frame);
              break;
            }
          }
        }
      }
    } catch (...) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    if (!had_frame) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  }
}

void Controller::stop_polling() {
  polling_active_.store(false, std::memory_order_release);
  if (polling_thread_.joinable()) {
    polling_thread_.join();
  }
}

void Controller::shutdown() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (bus_closed_) {
    return;
  }
  stop_polling();
  try {
    disable_all();
  } catch (...) {
  }
  bus_->shutdown();
  bus_closed_ = true;
}

void Controller::close_bus() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (bus_closed_) {
    return;
  }
  stop_polling();
  bus_->shutdown();
  bus_closed_ = true;
}

void Controller::set_tx_gap(std::chrono::microseconds gap) {
  bus_->set_tx_gap(gap);
}

bool Controller::owns_motor(const std::shared_ptr<MotorHandle>& motor) const {
  if (!motor) return false;
  std::lock_guard<std::mutex> lock(motors_mutex_);
  const auto found = motors_.find(motor->motor_id());
  return found != motors_.end() && found->second == motor;
}

void Controller::send_mit_batch(const std::vector<MitBatchCommand>& commands) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (bus_closed_) throw std::runtime_error(endpoint_label_ + " is closed");
  for (const auto& command : commands) {
    if (!command.motor || !owns_motor(command.motor)) {
      throw std::invalid_argument(endpoint_label_ + " received a motor it does not own");
    }
    try {
      command.motor->send_mit(command.pos, command.vel, command.kp, command.kd,
                              command.tau);
    } catch (const std::exception& err) {
      throw std::runtime_error(endpoint_label_ + ", motor ID " +
                               std::to_string(command.motor->motor_id()) + ": " + err.what());
    }
  }
}

void Controller::send_pos_vel_batch(const std::vector<PosVelBatchCommand>& commands) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (bus_closed_) throw std::runtime_error(endpoint_label_ + " is closed");
  for (const auto& command : commands) {
    if (!command.motor || !owns_motor(command.motor)) {
      throw std::invalid_argument(endpoint_label_ + " received a motor it does not own");
    }
    try {
      command.motor->send_pos_vel(command.pos, command.velocity_limit);
    } catch (const std::exception& err) {
      throw std::runtime_error(endpoint_label_ + ", motor ID " +
                               std::to_string(command.motor->motor_id()) + ": " + err.what());
    }
  }
}

class ControllerGroup::Impl {
 public:
  explicit Impl(std::vector<Controller*> controllers) : controllers_(std::move(controllers)) {
    if (controllers_.empty()) {
      throw std::invalid_argument("controller group requires at least one controller");
    }
    std::set<Controller*> unique;
    for (auto* controller : controllers_) {
      if (!controller) throw std::invalid_argument("controller group contains a null controller");
      if (!unique.insert(controller).second) {
        throw std::invalid_argument("controller group contains a duplicate controller");
      }
    }
    mit_batches_.resize(controllers_.size());
    pos_vel_batches_.resize(controllers_.size());
    errors_.resize(controllers_.size());
    try {
      for (std::size_t i = 0; i < controllers_.size(); ++i) {
        workers_.emplace_back([this, i] { worker_loop(i); });
      }
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stopping_ = true;
      }
      start_cv_.notify_all();
      for (auto& worker : workers_) {
        if (worker.joinable()) worker.join();
      }
      throw;
    }
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stopping_ = true;
    }
    start_cv_.notify_all();
    for (auto& worker : workers_) {
      if (worker.joinable()) worker.join();
    }
  }

  const std::vector<Controller*>& controllers() const { return controllers_; }

  void dispatch_mit(std::vector<std::vector<MitBatchCommand>> batches) {
    std::lock_guard<std::mutex> call_lock(call_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      mit_batches_ = std::move(batches);
      for (auto& batch : pos_vel_batches_) batch.clear();
      task_kind_ = TaskKind::Mit;
      completed_ = 0;
      std::fill(errors_.begin(), errors_.end(), std::string{});
      ++generation_;
    }
    start_cv_.notify_all();
    wait_and_raise();
  }

  void dispatch_pos_vel(std::vector<std::vector<PosVelBatchCommand>> batches) {
    std::lock_guard<std::mutex> call_lock(call_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      pos_vel_batches_ = std::move(batches);
      for (auto& batch : mit_batches_) batch.clear();
      task_kind_ = TaskKind::PosVel;
      completed_ = 0;
      std::fill(errors_.begin(), errors_.end(), std::string{});
      ++generation_;
    }
    start_cv_.notify_all();
    wait_and_raise();
  }

 private:
  enum class TaskKind { Mit, PosVel };

  void worker_loop(std::size_t index) {
    uint64_t seen_generation = 0;
    for (;;) {
      TaskKind kind = TaskKind::Mit;
      std::vector<MitBatchCommand> mit;
      std::vector<PosVelBatchCommand> pos_vel;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        start_cv_.wait(lock, [&] { return stopping_ || generation_ != seen_generation; });
        if (stopping_) return;
        seen_generation = generation_;
        kind = task_kind_;
        if (kind == TaskKind::Mit) {
          mit = mit_batches_[index];
        } else {
          pos_vel = pos_vel_batches_[index];
        }
      }

      std::string error;
      try {
        if (kind == TaskKind::Mit) {
          controllers_[index]->send_mit_batch(mit);
        } else {
          controllers_[index]->send_pos_vel_batch(pos_vel);
        }
      } catch (const std::exception& err) {
        error = err.what();
      } catch (...) {
        error = "unknown native exception";
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        errors_[index] = std::move(error);
        ++completed_;
      }
      done_cv_.notify_one();
    }
  }

  void wait_and_raise() {
    std::vector<std::string> errors;
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      done_cv_.wait(lock, [&] { return completed_ == controllers_.size(); });
      errors = errors_;
    }
    std::ostringstream message;
    bool failed = false;
    for (std::size_t i = 0; i < errors.size(); ++i) {
      if (errors[i].empty()) continue;
      if (!failed) message << "controller group send failed";
      failed = true;
      message << "; controller index " << i << ": " << errors[i];
    }
    if (failed) throw std::runtime_error(message.str());
  }

  std::vector<Controller*> controllers_;
  std::vector<std::thread> workers_;
  std::mutex call_mutex_;
  std::mutex state_mutex_;
  std::condition_variable start_cv_;
  std::condition_variable done_cv_;
  bool stopping_ = false;
  uint64_t generation_ = 0;
  std::size_t completed_ = 0;
  TaskKind task_kind_ = TaskKind::Mit;
  std::vector<std::vector<MitBatchCommand>> mit_batches_;
  std::vector<std::vector<PosVelBatchCommand>> pos_vel_batches_;
  std::vector<std::string> errors_;
};

ControllerGroup::ControllerGroup(std::vector<Controller*> controllers)
    : impl_(std::make_unique<Impl>(std::move(controllers))) {}

ControllerGroup::~ControllerGroup() = default;

void ControllerGroup::send_mit(const std::vector<MitBatchCommand>& commands) {
  std::vector<std::vector<MitBatchCommand>> batches(impl_->controllers().size());
  std::set<const MotorHandle*> unique;
  for (const auto& command : commands) {
    if (!command.motor) throw std::invalid_argument("MIT batch contains a null motor");
    if (!unique.insert(command.motor.get()).second) {
      throw std::invalid_argument("MIT batch contains duplicate motor ID " +
                                  std::to_string(command.motor->motor_id()));
    }
    bool found = false;
    for (std::size_t i = 0; i < impl_->controllers().size(); ++i) {
      if (impl_->controllers()[i]->owns_motor(command.motor)) {
        batches[i].push_back(command);
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::invalid_argument("MIT batch motor ID " +
                                  std::to_string(command.motor->motor_id()) +
                                  " does not belong to this controller group");
    }
  }
  impl_->dispatch_mit(std::move(batches));
}

void ControllerGroup::send_pos_vel(const std::vector<PosVelBatchCommand>& commands) {
  std::vector<std::vector<PosVelBatchCommand>> batches(impl_->controllers().size());
  std::set<const MotorHandle*> unique;
  for (const auto& command : commands) {
    if (!command.motor) throw std::invalid_argument("POS_VEL batch contains a null motor");
    if (!unique.insert(command.motor.get()).second) {
      throw std::invalid_argument("POS_VEL batch contains duplicate motor ID " +
                                  std::to_string(command.motor->motor_id()));
    }
    bool found = false;
    for (std::size_t i = 0; i < impl_->controllers().size(); ++i) {
      if (impl_->controllers()[i]->owns_motor(command.motor)) {
        batches[i].push_back(command);
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::invalid_argument("POS_VEL batch motor ID " +
                                  std::to_string(command.motor->motor_id()) +
                                  " does not belong to this controller group");
    }
  }
  impl_->dispatch_pos_vel(std::move(batches));
}

}  // namespace damiao
