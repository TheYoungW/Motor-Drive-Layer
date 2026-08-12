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

class MotorHandle;

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

struct FeedbackStats {
  bool has_feedback = false;
  uint64_t update_count = 0;
  std::chrono::nanoseconds age{0};
};

struct MitBatchCommand {
  std::shared_ptr<MotorHandle> motor;
  float pos = 0.0f;
  float vel = 0.0f;
  float kp = 0.0f;
  float kd = 0.0f;
  float tau = 0.0f;
};

struct PosVelBatchCommand {
  std::shared_ptr<MotorHandle> motor;
  float pos = 0.0f;
  float velocity_limit = 0.0f;
};

enum class PresencePolicy : uint8_t {
  Required = 1,
  Optional = 2,
  Disabled = 3,
};

enum class PresenceState : uint8_t {
  NotInstalled = 1,
  Present = 2,
  Faulted = 3,
};

struct MotorCandidate {
  std::string role;
  uint16_t motor_id = 0;
  uint16_t feedback_id = 0;
  std::string model;
  PresencePolicy policy = PresencePolicy::Required;
};

struct MotorDiscoveryResult {
  MotorCandidate candidate;
  PresenceState state = PresenceState::NotInstalled;
  std::shared_ptr<MotorHandle> motor;
  std::string reason;
};

enum class FeedbackBatchStatus : uint8_t {
  Ok = 0,
  Timeout = 1,
  Incomplete = 2,
  TransportError = 3,
};

struct FeedbackBatchReport {
  FeedbackBatchStatus status = FeedbackBatchStatus::Ok;
  std::chrono::milliseconds timeout{0};
  uint32_t expected_count = 0;
  uint32_t received_count = 0;
  std::vector<uint16_t> missing_motor_ids;
  std::string error;
};

class MotorHandle {
 public:
  MotorHandle(std::shared_ptr<CanBus> bus, uint16_t motor_id, uint16_t feedback_id,
              std::string model);

  MotorHandle(const MotorHandle&) = delete;
  MotorHandle& operator=(const MotorHandle&) = delete;
  MotorHandle(MotorHandle&&) = delete;
  MotorHandle& operator=(MotorHandle&&) = delete;

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
  std::optional<MotorState> request_fresh_state(std::chrono::milliseconds timeout);
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
  FeedbackStats feedback_stats() const;

 private:
  friend class Controller;

  void send_raw(uint32_t arbitration_id, std::array<uint8_t, 8> data);
  void send_to_motor(std::array<uint8_t, 8> data);
  void send_mode_frame(uint32_t base_id, std::array<uint8_t, 8> data);
  void write_register_raw(uint8_t rid, std::array<uint8_t, 4> data);
  bool wait_for_feedback_after(uint64_t previous_count,
                               std::chrono::steady_clock::time_point deadline);
  std::array<uint8_t, 4> wait_for_register(uint8_t rid, std::chrono::milliseconds timeout);
  void wait_for_write_ack(uint8_t rid, std::array<uint8_t, 4> expected,
                          std::chrono::milliseconds timeout);

  std::shared_ptr<CanBus> bus_;
  uint16_t motor_id_;
  uint16_t feedback_id_;
  std::string model_;
  Limits limits_;
  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  std::optional<MotorState> state_;
  std::optional<std::chrono::steady_clock::time_point> state_time_;
  uint64_t feedback_update_count_ = 0;
  std::atomic<bool> disabled_hint_{true};
  mutable std::mutex register_mutex_;
  std::map<uint8_t, std::pair<std::array<uint8_t, 4>, std::chrono::steady_clock::time_point>>
      register_replies_;
  std::map<uint8_t, std::pair<std::array<uint8_t, 4>, std::chrono::steady_clock::time_point>>
      register_acks_;
};

class Controller {
 public:
  explicit Controller(std::shared_ptr<CanBus> bus,
                      std::string endpoint_label = "controller");
  ~Controller();

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  std::shared_ptr<MotorHandle> add_damiao_motor(uint16_t motor_id, uint16_t feedback_id,
                                                const std::string& model);
  std::vector<MotorDiscoveryResult> discover_damiao_motors(
      const std::vector<MotorCandidate>& candidates,
      std::chrono::milliseconds timeout,
      uint32_t retry_count);
  void poll_feedback_once();
  FeedbackBatchReport request_feedback_all_report(
      std::chrono::milliseconds timeout);
  void request_feedback_all(std::chrono::milliseconds timeout);
  void enable_all();
  void disable_all();
  void shutdown();
  void close_bus();
  void set_tx_gap(std::chrono::microseconds gap);
  void send_mit_batch(const std::vector<MitBatchCommand>& commands);
  void send_pos_vel_batch(const std::vector<PosVelBatchCommand>& commands);
  const std::string& endpoint_label() const { return endpoint_label_; }
  TransportCapabilities transport_capabilities() const;
  TransportHealth transport_health() const;

 private:
  friend class ControllerGroup;

  void start_polling();
  void stop_polling();
  void polling_loop();
  std::vector<std::shared_ptr<MotorHandle>> sorted_motors() const;
  bool owns_motor(const std::shared_ptr<MotorHandle>& motor) const;

  class PacingBus;

  std::shared_ptr<PacingBus> bus_;
  mutable std::mutex motors_mutex_;
  std::map<uint16_t, std::shared_ptr<MotorHandle>> motors_;
  std::atomic<bool> polling_active_{false};
  std::thread polling_thread_;
  mutable std::mutex recv_mutex_;
  std::mutex lifecycle_mutex_;
  bool bus_closed_ = false;
  bool tx_gap_env_override_ = false;
  bool discovery_finalized_ = false;
  std::chrono::milliseconds bulk_op_gap_{2};
  std::string endpoint_label_;
};

// Coordinates one persistent native worker per Controller. Each dispatch wakes
// all workers from one generation, sends each controller's commands in order,
// and returns only after every controller completes. Controllers and motor
// handles must outlive the group and must not be freed concurrently with it.
class ControllerGroup {
 public:
  explicit ControllerGroup(std::vector<Controller*> controllers);
  ~ControllerGroup();

  ControllerGroup(const ControllerGroup&) = delete;
  ControllerGroup& operator=(const ControllerGroup&) = delete;
  ControllerGroup(ControllerGroup&&) = delete;
  ControllerGroup& operator=(ControllerGroup&&) = delete;

  void send_mit(const std::vector<MitBatchCommand>& commands);
  void send_pos_vel(const std::vector<PosVelBatchCommand>& commands);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace damiao
