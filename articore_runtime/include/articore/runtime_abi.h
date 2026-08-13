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
  ARTICORE_CAP_REALTIME_JOINT_MAILBOX = 1ULL << 8,
  ARTICORE_CAP_JOINT_TRAJECTORY = 1ULL << 9,
  ARTICORE_CAP_ATOMIC_ENABLE = 1ULL << 10,
  ARTICORE_CAP_COMMAND_LIFETIME = 1ULL << 11,
  ARTICORE_CAP_NONPREEMPTIVE_TRAJECTORY = 1ULL << 12,
  // Operational faults latch FAULT and stop trajectories, but keep sending
  // protective holds to motors/channels that remain controllable. Only an
  // explicit disable or product estop policy requests torque-off.
  ARTICORE_CAP_PROTECTIVE_FAULT_HOLD = 1ULL << 13,
  // disable() and close() share a bounded transaction that drains prior
  // control traffic, disables active channels in parallel, confirms fresh
  // disabled feedback, and retries only unconfirmed motors once.
  ARTICORE_CAP_DETERMINISTIC_DISABLE = 1ULL << 14,
  // ABI 1.7 adds explicit cancellation and opt-in velocity-continuous
  // replacement while preserving the legacy reject-if-busy entry point.
  ARTICORE_CAP_TRAJECTORY_MANAGEMENT = 1ULL << 15,
  // ABI 1.8 separates reference generation from measured convergence and
  // monitors sustained per-joint following error during trajectory execution.
  ARTICORE_CAP_TRAJECTORY_SETTLING = 1ULL << 16,
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

// A persistent setpoint is retransmitted at the control rate until another
// command, trajectory, disable, or fault replaces it. A streaming command
// must be refreshed before command_timeout_ms expires.
enum ArticoreCommandLifetime {
  ARTICORE_COMMAND_STREAMING = 1,
  ARTICORE_COMMAND_HOLD_UNTIL_REPLACED = 2,
};

enum ArticoreTrajectoryProfile {
  ARTICORE_TRAJECTORY_MIN_JERK = 1,
  ARTICORE_TRAJECTORY_LINEAR = 2,
};

enum ArticoreTrajectoryStatus {
  ARTICORE_TRAJECTORY_RUNNING = 1,
  ARTICORE_TRAJECTORY_COMPLETED = 2,
  ARTICORE_TRAJECTORY_PREEMPTED = 3,
  ARTICORE_TRAJECTORY_FAILED = 4,
  ARTICORE_TRAJECTORY_CANCELED = 5,
};

enum ArticoreTrajectoryReplacePolicy {
  ARTICORE_TRAJECTORY_REJECT_IF_BUSY = 0,
  ARTICORE_TRAJECTORY_SMOOTH_REPLACE = 1,
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

typedef struct ArticoreJointControlConfig {
  void* motor;
  float lower_position;
  float upper_position;
  float velocity_limit;
  float torque_limit;
  float mit_kp;
  float mit_kd;
  float mit_feedforward_torque;
} ArticoreJointControlConfig;

typedef struct ArticoreJointTrajectoryTarget {
  void* motor;
  float target_position;
  float velocity_limit;
} ArticoreJointTrajectoryTarget;

typedef struct ArticoreTrajectoryExecutionConfig {
  // Caller initializes this to sizeof(ArticoreTrajectoryExecutionConfig).
  uint32_t struct_size;
  float position_tolerance;
  float velocity_tolerance;
  float following_error_limit;
  uint32_t settling_stable_ms;
  uint32_t settling_timeout_ms;
  uint32_t following_error_timeout_ms;
} ArticoreTrajectoryExecutionConfig;

typedef struct ArticoreTrajectoryInfo {
  uint32_t struct_size;
  uint64_t trajectory_id;
  int32_t status;
  int32_t profile;
  uint64_t duration_ns;
  uint64_t elapsed_ns;
  char error[256];
} ArticoreTrajectoryInfo;

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
  // Caller initializes this to sizeof(ArticoreFeedbackReport).
  uint32_t struct_size;
  uint32_t timeout_ms;
  uint32_t expected_count;
  uint32_t received_count;
  uint32_t missing_count;
} ArticoreFeedbackReport;

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
  // Caller initializes this to sizeof(ArticoreEnableReport).
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
  // Caller initializes this to sizeof(ArticoreDisableReport).
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

typedef int32_t (*ArticoreGroupSendPosVelFn)(
    void*, const ArticorePosVelCommand*, uint32_t);
typedef int32_t (*ArticoreGroupSendMitFn)(
    void*, const ArticoreMitCommand*, uint32_t);
typedef int32_t (*ArticoreControllerCallFn)(void*);
// Returns the stable MotorErrorCode value used by libmotor_abi's structured
// feedback entry point. The runtime never parses an error string for policy.
typedef int32_t (*ArticoreControllerFeedbackFn)(
    void*, uint32_t, ArticoreFeedbackReport*, uint32_t*, uint32_t);
typedef int32_t (*ArticoreMotorGetStateFn)(void*, ArticoreMotorState*);
typedef int32_t (*ArticoreMotorGetFeedbackStatsFn)(void*, ArticoreFeedbackStats*);
typedef int32_t (*ArticoreControllerTransportHealthFn)(
    void*, ArticoreDriverTransportHealth*);
typedef const char* (*ArticoreLastErrorFn)(void);

typedef struct ArticoreMotorApi {
  ArticoreGroupSendPosVelFn group_send_pos_vel;
  ArticoreGroupSendMitFn group_send_mit;
  ArticoreControllerCallFn controller_disable_all;
  ArticoreControllerFeedbackFn controller_request_feedback_all_ex;
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
  // Retained for ABI compatibility. Normal gripper control follows
  // control_hz; safe gripper holding follows safe_hold_hz.
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
// ABI 1.4 entry point. The two additive callbacks let the product runtime own
// native enable without creating a shared-library dependency on libmotor_abi.
// Language bindings pass motor_controller_enable_all and motor_handle_enable.
ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create_ex(
    const ArticoreRuntimeConfig* config,
    const ArticoreMotorApi* motor_api,
    void* controller_group,
    void* left_controller,
    void* right_controller,
    const ArticoreMotorDescriptor* motors,
    uint32_t motor_count,
    ArticoreControllerCallFn controller_enable_all,
    ArticoreControllerCallFn motor_enable);
ARTICORE_RUNTIME_API void articore_runtime_free(ArticoreRuntime* runtime);

ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime);
// With ARTICORE_CAP_ATOMIC_ENABLE this owns the complete native transaction:
// fresh disabled-state position capture, parallel controller enable, immediate
// current-position hold, parallel enabled-feedback confirmation, and linked
// disable rollback on every failure. SDKs must not enable motors beforehand.
ARTICORE_RUNTIME_API int32_t articore_runtime_enable(
    ArticoreRuntime* runtime, int32_t mode);
// Returns the stable result of the most recent native enable transaction.
// This remains available after enable fails and rolls every motor back to the
// disabled state.
ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_enable_report(
    ArticoreRuntime* runtime, ArticoreEnableReport* report);
// Configures every active arm joint before connect. Trajectories require this
// configuration so position, velocity and torque limits are explicit.
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_joints(
    ArticoreRuntime* runtime,
    const ArticoreJointControlConfig* configs,
    uint32_t config_count);
// Optional ABI 1.8 trajectory execution policy. It must be configured before
// connect. If omitted, conservative native defaults are used. The following
// error limit is checked independently for every arm joint.
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_trajectory_execution(
    ArticoreRuntime* runtime,
    const ArticoreTrajectoryExecutionConfig* config);
// Direct arm commands are validated and atomically overwrite a capacity-one
// mailbox. Success means accepted; the persistent control thread transmits the
// latest accepted value on the next control tick. These ABI 1.0 entry points
// retain their original STREAMING watchdog behavior.
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pos_vel(
    ArticoreRuntime* runtime,
    const ArticorePosVelCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count);
// ABI 1.5 entry points. HOLD_UNTIL_REPLACED is intended for one-shot position
// setpoints whose physical motion may legitimately exceed command_timeout_ms.
// Persistent MIT setpoints require target_velocity=0 and
// feedforward_torque=0; ongoing velocity/feedforward control must be STREAMING.
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pos_vel_ex(
    ArticoreRuntime* runtime,
    const ArticorePosVelCommand* commands,
    uint32_t command_count,
    int32_t lifetime);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit_ex(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count,
    int32_t lifetime);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_gripper_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_gripper_openings(
    ArticoreRuntime* runtime,
    const ArticoreGripperTarget* targets,
    uint32_t target_count);
// Starts one time-parameterized trajectory. Exactly one trajectory may be
// active: another trajectory or a direct submit_pos_vel/submit_mit command is
// rejected until it completes. Disable, estop, close, and safety faults may
// still cancel/fail it. Returns 0 on failure and sets
// articore_runtime_last_error().
ARTICORE_RUNTIME_API uint64_t articore_runtime_start_joint_trajectory(
    ArticoreRuntime* runtime,
    const ArticoreJointTrajectoryTarget* targets,
    uint32_t target_count,
    int32_t profile);
// ABI 1.7 opt-in trajectory replacement. SMOOTH_REPLACE atomically marks the
// old trajectory PREEMPTED and starts the new minimum-jerk polynomial from the
// old trajectory's position, velocity, and acceleration at one steady-clock
// instant. The legacy entry point above is equivalent to REJECT_IF_BUSY.
ARTICORE_RUNTIME_API uint64_t articore_runtime_start_joint_trajectory_ex(
    ArticoreRuntime* runtime,
    const ArticoreJointTrajectoryTarget* targets,
    uint32_t target_count,
    int32_t profile,
    int32_t replace_policy);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_trajectory(
    ArticoreRuntime* runtime,
    uint64_t trajectory_id,
    ArticoreTrajectoryInfo* info);
ARTICORE_RUNTIME_API int32_t articore_runtime_wait_trajectory(
    ArticoreRuntime* runtime,
    uint64_t trajectory_id,
    uint32_t timeout_ms,
    ArticoreTrajectoryInfo* info);
// Cancels exactly the active trajectory with this ID. The complete arm layout
// atomically enters a current-position internal hold; no partial-side cancel
// is possible. The trajectory becomes CANCELED and remains queryable.
ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_trajectory(
    ArticoreRuntime* runtime, uint64_t trajectory_id);
ARTICORE_RUNTIME_API int32_t articore_runtime_report_feedback_failure(
    ArticoreRuntime* runtime, uint8_t side, const char* reason);
// Valid in every connected state, including latched FAULT. A successful call
// confirms physical disable but intentionally does not clear a FAULT latch.
ARTICORE_RUNTIME_API int32_t articore_runtime_disable(ArticoreRuntime* runtime);
// Returns the structured result of the most recent deterministic disable or
// close transaction, including channel/motor identity for every motor whose
// fresh disabled state could not be confirmed.
ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_disable_report(
    ArticoreRuntime* runtime, ArticoreDisableReport* report);
// Latches FAULT and torque-disables all arm joints. Grippers follow the
// configured product estop action (hold or disable).
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
