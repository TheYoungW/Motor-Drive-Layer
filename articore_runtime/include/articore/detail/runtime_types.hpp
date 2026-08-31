#pragma once

#include "articore/runtime_abi.h"

// Native Runtime implementation types. None of these structures cross the
// Yunyi product C ABI or are assembled by an SDK.

enum ArticoreConnectErrorCode {
  ARTICORE_CONNECT_OK = 0,
  ARTICORE_CONNECT_CONFIGURATION = 1,
  ARTICORE_CONNECT_TRANSPORT = 2,
  ARTICORE_CONNECT_FEEDBACK_TIMEOUT = 3,
  ARTICORE_CONNECT_FEEDBACK_INCOMPLETE = 4,
  ARTICORE_CONNECT_FEEDBACK_INVALID = 5,
};

enum ArticorePresenceState {
  ARTICORE_NOT_INSTALLED = 1,
  ARTICORE_PRESENT = 2,
  ARTICORE_FAULTED = 3,
};

enum ArticoreMotorPowerState {
  ARTICORE_MOTOR_POWER_UNKNOWN = 0,
  ARTICORE_MOTOR_POWER_DISABLED = 1,
  ARTICORE_MOTOR_POWER_ENABLED = 2,
  ARTICORE_MOTOR_POWER_MIXED = 3,
};

enum ArticoreGripperFaultAction {
  ARTICORE_GRIPPER_FAULT_HOLD = 1,
  ARTICORE_GRIPPER_FAULT_DISABLE = 2,
};

enum ArticoreGripperForceLevel {
  ARTICORE_GRIPPER_FORCE_LEVEL_1 = 1,
  ARTICORE_GRIPPER_FORCE_LEVEL_2 = 2,
  ARTICORE_GRIPPER_FORCE_LEVEL_3 = 3,
  ARTICORE_GRIPPER_FORCE_LEVEL_4 = 4,
  ARTICORE_GRIPPER_FORCE_LEVEL_5 = 5,
  ARTICORE_GRIPPER_FORCE_LEVEL_6 = 6,
  ARTICORE_GRIPPER_FORCE_LEVEL_7 = 7,
  ARTICORE_GRIPPER_FORCE_LEVEL_8 = 8,
  ARTICORE_GRIPPER_FORCE_LEVEL_9 = 9,
  ARTICORE_GRIPPER_FORCE_LEVEL_10 = 10,
  ARTICORE_GRIPPER_FORCE_MIN = ARTICORE_GRIPPER_FORCE_LEVEL_1,
  ARTICORE_GRIPPER_FORCE_DEFAULT = ARTICORE_GRIPPER_FORCE_LEVEL_5,
  ARTICORE_GRIPPER_FORCE_MAX = ARTICORE_GRIPPER_FORCE_LEVEL_10,
  ARTICORE_GRIPPER_FORCE_LOW = ARTICORE_GRIPPER_FORCE_MIN,
  ARTICORE_GRIPPER_FORCE_NORMAL = ARTICORE_GRIPPER_FORCE_DEFAULT,
  ARTICORE_GRIPPER_FORCE_HIGH = ARTICORE_GRIPPER_FORCE_MAX,
};

enum ArticoreCommandLifetime {
  ARTICORE_COMMAND_STREAMING = 1,
  ARTICORE_COMMAND_HOLD_UNTIL_REPLACED = 2,
};

typedef struct ArticoreGravityProductBinding {
  uint32_t struct_size;
  uint32_t runtime_side;
  uint32_t robot_side;
  char product_id[64];
} ArticoreGravityProductBinding;

typedef struct ArticorePosVelCommand {
  void* motor;
  float target_position;
  float velocity_limit;
} ArticorePosVelCommand;

typedef struct ArticoreMitCommand {
  void* motor;
  float target_position;
  float target_velocity;
  float stiffness;
  float damping;
  float feedforward_torque;
} ArticoreMitCommand;

typedef struct ArticoreJointMitTarget {
  uint32_t struct_size;
  void* motor;
  float target_position;
} ArticoreJointMitTarget;

typedef struct ArticoreJointPvTarget {
  uint32_t struct_size;
  void* motor;
  float target_position;
} ArticoreJointPvTarget;

typedef struct ArticoreGripperTarget {
  void* motor;
  float opening;
} ArticoreGripperTarget;

typedef struct ArticoreGripperCommand {
  uint32_t struct_size;
  void* motor;
  float opening;
  float speed;
  int32_t force_level;
} ArticoreGripperCommand;

typedef struct ArticoreGripperForceProfile {
  uint32_t struct_size;
  void* motor;
  int32_t force_level;
  float contact_torque;
  float overload_torque;
  float moving_kp;
  float moving_kd;
  float hold_kp;
  float hold_kd;
} ArticoreGripperForceProfile;

typedef struct ArticoreGripperProductBinding {
  uint32_t struct_size;
  void* motor;
  char profile_id[64];
} ArticoreGripperProductBinding;

typedef struct ArticoreJointControlConfig {
  void* motor;
  float lower_position;
  float upper_position;
  float velocity_limit;
  float torque_limit;
  float mit_kp;
  float mit_kd;
  float mit_feedforward_torque;
  float mit_fast_follow_kp;
  float mit_fast_follow_kd;
} ArticoreJointControlConfig;

typedef struct ArticoreJointSafetyLimits {
  uint32_t struct_size;
  void* motor;
  float hard_lower_position;
  float hard_upper_position;
  float soft_lower_position;
  float soft_upper_position;
  float soft_limit_braking_zone;
  float braking_acceleration;
} ArticoreJointSafetyLimits;

typedef struct ArticoreMotorState {
  int32_t has_value;
  uint8_t can_id;
  uint32_t arbitration_id;
  uint8_t status_code;
  float pos;
  float vel;
  float torq;
  float t_mos;
  float t_rotor;
} ArticoreMotorState;

typedef struct ArticoreFeedbackStats {
  int32_t has_feedback;
  uint64_t update_count;
  uint64_t age_ns;
} ArticoreFeedbackStats;

typedef struct ArticoreFeedbackReport {
  uint32_t struct_size;
  uint32_t timeout_ms;
  uint32_t expected_count;
  uint32_t received_count;
  uint32_t missing_count;
} ArticoreFeedbackReport;

typedef struct ArticoreMotorIdentity {
  uint32_t struct_size;
  void* motor;
  uint32_t can_id;
} ArticoreMotorIdentity;

typedef struct ArticoreConnectChannelResult {
  uint8_t side;
  uint8_t active;
  int32_t request_code;
  uint32_t expected_count;
  uint32_t received_count;
  uint32_t missing_count;
  uint32_t missing_motor_ids[32];
  char error[256];
} ArticoreConnectChannelResult;

typedef struct ArticoreConnectMotorResult {
  uint8_t side;
  uint8_t has_feedback;
  uint8_t feedback_fresh;
  uint8_t feedback_valid;
  uint32_t configured_can_id;
  uint32_t reported_can_id;
  uint64_t update_count;
  uint64_t feedback_age_ns;
  char name[64];
  char error[256];
} ArticoreConnectMotorResult;

typedef struct ArticoreConnectReport {
  uint32_t struct_size;
  int32_t success;
  int32_t error_code;
  uint32_t expected_count;
  uint32_t received_count;
  uint32_t missing_count;
  uint32_t failure_count;
  uint32_t channel_count;
  ArticoreConnectChannelResult channels[2];
  uint32_t motor_count;
  ArticoreConnectMotorResult motors[32];
  char error[512];
} ArticoreConnectReport;

typedef struct ArticoreEnableMotorResult {
  uint8_t side;
  uint8_t can_id;
  uint8_t status_code;
  uint8_t has_feedback;
  uint8_t feedback_fresh;
  uint8_t enabled;
  char name[64];
} ArticoreEnableMotorResult;

typedef struct ArticoreEnableReport {
  uint32_t struct_size;
  int32_t success;
  int32_t disable_confirmed;
  uint32_t expected_count;
  uint32_t enabled_count;
  uint32_t missing_count;
  uint32_t failure_count;
  uint8_t missing_motor_sides[32];
  uint32_t missing_motor_ids[32];
  uint32_t motor_count;
  ArticoreEnableMotorResult motors[32];
  char error[512];
} ArticoreEnableReport;

typedef struct ArticoreDisableMotorResult {
  uint8_t side;
  uint8_t can_id;
  uint8_t status_code;
  uint8_t has_feedback;
  uint8_t feedback_fresh;
  uint8_t disabled;
  uint8_t disable_sent;
  uint8_t retry_sent;
  char name[64];
} ArticoreDisableMotorResult;

typedef struct ArticoreDisableReport {
  uint32_t struct_size;
  int32_t success;
  int32_t barrier_confirmed;
  uint32_t expected_count;
  uint32_t disabled_count;
  uint32_t missing_count;
  uint32_t failure_count;
  uint32_t retry_count;
  uint8_t missing_motor_sides[32];
  uint32_t missing_motor_ids[32];
  uint32_t motor_count;
  ArticoreDisableMotorResult motors[32];
  char error[512];
} ArticoreDisableReport;

typedef struct ArticoreDriverTransportHealth {
  int32_t connected;
  int32_t healthy;
  uint64_t tx_frames;
  uint64_t rx_frames;
  uint64_t send_errors;
  uint64_t receive_errors;
  uint64_t last_tx_age_ns;
  uint64_t last_rx_age_ns;
  char last_error[256];
} ArticoreDriverTransportHealth;

typedef struct ArticoreRuntimeConfig {
  uint32_t command_timeout_ms;
  uint32_t enable_grace_ms;
  uint32_t safe_hold_hz;
  uint32_t feedback_check_hz;
  uint32_t feedback_failure_threshold;
  uint32_t feedback_max_age_ms;
  uint32_t safe_hold_failure_threshold;
  uint32_t disable_feedback_timeout_ms;
  float safe_pv_velocity_limit;
  int32_t gripper_fault_action;
} ArticoreRuntimeConfig;

typedef struct ArticoreMotorDescriptor {
  void* motor;
  uint8_t side;
  uint8_t is_gripper;
  char name[64];
  float safe_kp;
  float safe_kd;
  float overload_torque;
  float retreat_distance;
  float contact_torque;
  uint32_t motion_window_ms;
  float stall_movement;
  float min_position_error;
  uint32_t contact_hold_ms;
  uint32_t overload_hold_ms;
  float hold_offset;
  uint32_t retreat_retry_ms;
  float open_position;
  float closed_position;
  float normal_kp;
  float normal_kd;
  float close_speed;
  uint32_t max_step_interval_ms;
  float closing_direction;
  float lower_position;
  float upper_position;
} ArticoreMotorDescriptor;
