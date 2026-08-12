#ifndef ARTICORE_RUNTIME_ABI_H
#define ARTICORE_RUNTIME_ABI_H

#include <stdint.h>

#if defined(_WIN32)
#if defined(ARTICORE_RUNTIME_BUILDING_LIBRARY)
#define ARTICORE_RUNTIME_API __declspec(dllexport)
#else
#define ARTICORE_RUNTIME_API __declspec(dllimport)
#endif
#else
#define ARTICORE_RUNTIME_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ArticoreRuntime ArticoreRuntime;

enum ArticoreRuntimeCapability {
  ARTICORE_CAP_COMMAND_WATCHDOG = 1ULL << 0,
  ARTICORE_CAP_SAFE_HOLD = 1ULL << 1,
  ARTICORE_CAP_GRIPPER_PROTECTION = 1ULL << 2,
  ARTICORE_CAP_SINGLE_CHANNEL = 1ULL << 3,
  ARTICORE_CAP_DUAL_CHANNEL = 1ULL << 4,
  ARTICORE_CAP_TRANSPORT_HEALTH = 1ULL << 5,
  ARTICORE_CAP_CURRENT_POSITION_HOLD = 1ULL << 6,
  ARTICORE_CAP_MOTOR_PRESENCE = 1ULL << 7,
};

enum ArticorePresenceState {
  ARTICORE_NOT_INSTALLED = 1,
  ARTICORE_PRESENT = 2,
  ARTICORE_FAULTED = 3,
};

enum ArticoreActiveCapability {
  ARTICORE_ACTIVE_ARM_SIDE_0 = 1ULL << 0,
  ARTICORE_ACTIVE_ARM_SIDE_1 = 1ULL << 1,
  ARTICORE_ACTIVE_GRIPPER_SIDE_0 = 1ULL << 2,
  ARTICORE_ACTIVE_GRIPPER_SIDE_1 = 1ULL << 3,
};

enum ArticoreSafetyState {
  ARTICORE_DISCONNECTED = 0,
  ARTICORE_READY = 1,
  ARTICORE_ENABLED = 2,
  ARTICORE_RUNNING = 3,
  ARTICORE_SAFE_HOLD = 4,
  ARTICORE_FAULT = 5,
};

enum ArticoreControlMode {
  ARTICORE_MODE_PV = 1,
  ARTICORE_MODE_MIT = 2,
};

enum ArticoreGripperControlState {
  ARTICORE_GRIPPER_DISABLED = 0,
  ARTICORE_GRIPPER_IDLE = 1,
  ARTICORE_GRIPPER_MOVING = 2,
  ARTICORE_GRIPPER_CONTACT = 3,
  ARTICORE_GRIPPER_HOLDING = 4,
  ARTICORE_GRIPPER_OVERLOAD_RETREAT = 5,
  ARTICORE_GRIPPER_FAULT = 6,
};

enum ArticoreGripperFaultAction {
  ARTICORE_GRIPPER_FAULT_HOLD = 1,
  ARTICORE_GRIPPER_FAULT_DISABLE = 2,
};

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

typedef struct ArticoreGripperTarget {
  void* motor;
  float opening;
} ArticoreGripperTarget;

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

typedef int32_t (*ArticoreGroupSendPosVelFn)(
    void*, const ArticorePosVelCommand*, uint32_t);
typedef int32_t (*ArticoreGroupSendMitFn)(
    void*, const ArticoreMitCommand*, uint32_t);
typedef int32_t (*ArticoreControllerCallFn)(void*);
typedef int32_t (*ArticoreControllerFeedbackFn)(void*, uint32_t);
typedef int32_t (*ArticoreMotorGetStateFn)(void*, ArticoreMotorState*);
typedef int32_t (*ArticoreMotorGetFeedbackStatsFn)(void*, ArticoreFeedbackStats*);
typedef int32_t (*ArticoreControllerTransportHealthFn)(
    void*, ArticoreDriverTransportHealth*);
typedef const char* (*ArticoreLastErrorFn)(void);

typedef struct ArticoreMotorApi {
  ArticoreGroupSendPosVelFn group_send_pos_vel;
  ArticoreGroupSendMitFn group_send_mit;
  ArticoreControllerCallFn controller_disable_all;
  ArticoreControllerFeedbackFn controller_request_feedback_all;
  ArticoreMotorGetStateFn motor_get_state;
  ArticoreMotorGetFeedbackStatsFn motor_get_feedback_stats;
  ArticoreLastErrorFn last_error_message;
  ArticoreControllerTransportHealthFn controller_get_transport_health;
  ArticoreControllerCallFn motor_disable;
} ArticoreMotorApi;

typedef struct ArticoreRuntimeConfig {
  uint32_t control_hz;
  uint32_t command_timeout_ms;
  uint32_t enable_grace_ms;
  uint32_t safe_hold_hz;
  uint32_t feedback_check_hz;
  uint32_t feedback_failure_threshold;
  uint32_t feedback_max_age_ms;
  uint32_t safe_hold_failure_threshold;
  uint32_t disable_feedback_timeout_ms;
  float safe_pv_velocity_limit;
  uint32_t gripper_control_hz;
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

typedef struct ArticoreTransportHealth {
  int32_t connected;
  int32_t healthy;
  uint32_t consecutive_send_failures;
  uint32_t consecutive_feedback_failures;
  uint64_t last_feedback_age_ns;
  uint64_t tx_frames;
  uint64_t rx_frames;
  uint64_t send_errors;
  uint64_t receive_errors;
  uint64_t last_tx_age_ns;
  uint64_t last_rx_age_ns;
  char last_error[256];
} ArticoreTransportHealth;

typedef struct ArticoreGripperHealth {
  int32_t available;
  uint8_t side;
  int32_t control_state;
  float opening;
  float motor_position;
  float torque;
  int32_t contact_detected;
  int32_t stalled;
  int32_t overload;
  int32_t has_hold_target;
  float hold_target;
  uint64_t feedback_age_ns;
  char name[64];
  char fault_reason[256];
} ArticoreGripperHealth;

typedef struct ArticoreSafetyHealth {
  int32_t state;
  int32_t safe_holding;
  int32_t disable_confirmed;
  uint64_t last_successful_command_age_ns;
  uint64_t last_fresh_feedback_age_ns;
  uint32_t consecutive_send_failures;
  uint32_t consecutive_feedback_failures;
  ArticoreTransportHealth left_transport;
  ArticoreTransportHealth right_transport;
  uint32_t gripper_count;
  ArticoreGripperHealth grippers[2];
  uint32_t motor_fault_count;
  char motor_faults[32][64];
  uint32_t unconfirmed_disable_count;
  char unconfirmed_disable[32][64];
  char fault_reason[512];
} ArticoreSafetyHealth;

// Packed as 0xMMMMmmmm: major in the high 16 bits, minor in the low 16 bits.
ARTICORE_RUNTIME_API uint32_t articore_runtime_abi_version(void);
ARTICORE_RUNTIME_API uint64_t articore_runtime_capabilities(void);

ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create(
    const ArticoreRuntimeConfig* config,
    const ArticoreMotorApi* motor_api,
    void* controller_group,
    void* left_controller,
    void* right_controller,
    const ArticoreMotorDescriptor* motors,
    uint32_t motor_count);
ARTICORE_RUNTIME_API void articore_runtime_free(ArticoreRuntime* runtime);

ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_enable(
    ArticoreRuntime* runtime, int32_t mode);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pos_vel(
    ArticoreRuntime* runtime,
    const ArticorePosVelCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_gripper_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_gripper_openings(
    ArticoreRuntime* runtime,
    const ArticoreGripperTarget* targets,
    uint32_t target_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_report_feedback_failure(
    ArticoreRuntime* runtime, uint8_t side, const char* reason);
ARTICORE_RUNTIME_API int32_t articore_runtime_disable(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_estop(
    ArticoreRuntime* runtime, const char* reason);
ARTICORE_RUNTIME_API int32_t articore_runtime_recover(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_health(
    ArticoreRuntime* runtime, ArticoreSafetyHealth* health);
// Declares roles omitted from the active motor descriptor set. This is only
// valid while DISCONNECTED and is intended for Optional/Disabled model roles.
// Active descriptor names are registered as PRESENT automatically.
ARTICORE_RUNTIME_API int32_t articore_runtime_declare_motor_presence(
    ArticoreRuntime* runtime, const char* motor_role, int32_t state);
ARTICORE_RUNTIME_API int32_t articore_runtime_motor_presence(
    ArticoreRuntime* runtime, const char* motor_role, int32_t* state);
// Returns ArticoreActiveCapability bits for the fixed motor set discovered for
// this connection. Faulted motors remain active; NotInstalled motors do not.
ARTICORE_RUNTIME_API uint64_t articore_runtime_active_capabilities(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_close(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API const char* articore_runtime_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
