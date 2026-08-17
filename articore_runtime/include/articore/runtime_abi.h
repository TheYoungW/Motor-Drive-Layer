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
  ARTICORE_CAP_ATOMIC_ENABLE = 1ULL << 10,
  ARTICORE_CAP_COMMAND_LIFETIME = 1ULL << 11,
  // Operational faults latch FAULT, but keep sending
  // protective holds to motors/channels that remain controllable. Only an
  // explicit disable or product estop policy requests torque-off.
  ARTICORE_CAP_PROTECTIVE_FAULT_HOLD = 1ULL << 13,
  // disable() and close() share a bounded transaction that drains prior
  // control traffic, disables active channels in parallel, confirms fresh
  // disabled feedback, and retries only unconfirmed motors once.
  ARTICORE_CAP_DETERMINISTIC_DISABLE = 1ULL << 14,
  // Outgoing-command hard and soft position limits are configured independently.
  // Feedback measurements are not compared with these command limits.
  ARTICORE_CAP_LAYERED_JOINT_LIMITS = 1ULL << 18,
  // ABI 1.10 adds atomic per-command gripper speed/force selection and
  // product-owned force calibration profiles.
  ARTICORE_CAP_GRIPPER_COMMAND_PROFILES = 1ULL << 19,
  // ABI 1.11 expands the product-independent gripper force selector to ten
  // calibrated levels. Level 1 is lightest, level 10 strongest, and level 5
  // is the default used by the legacy opening-only API.
  ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS = 1ULL << 20,
  // ABI 1.12 adds a one-shot ordinary MIT position command. The Runtime
  // advances the complete arm reference at one shared rad/s limit while the
  // raw MIT submission APIs retain their direct q/dq/kp/kd/tau semantics.
  ARTICORE_CAP_JOINT_MIT_POSITION = 1ULL << 21,
  // ABI 1.13 adds the symmetric ordinary PV position command while retaining
  // raw submit_pos_vel[_ex]() as an internal direct-control capability.
  ARTICORE_CAP_JOINT_PV_POSITION = 1ULL << 22,
  // ABI 2.1 exposes the effective native control rate. Dual-arm runtimes cap
  // the requested rate at 400 Hz for the verified shared-adapter envelope.
  ARTICORE_CAP_EFFECTIVE_CONTROL_RATE = 1ULL << 23,
  // ABI 2.2 moves product gripper calibration and safety policy into named,
  // immutable motor-layer profiles selected before connect.
  ARTICORE_CAP_BUILTIN_GRIPPER_PRODUCT_PROFILES = 1ULL << 24,
  // ABI 2.3 makes connect() a feedback barrier. Success guarantees that every
  // configured arm joint and installed gripper has a fresh cached state. READY
  // then refreshes the complete cache at a bounded low rate.
  ARTICORE_CAP_CONNECT_FEEDBACK_BARRIER = 1ULL << 25,
  // ABI 2.4 adds immutable pre-connect motor CAN identities and a structured
  // report for every connect transaction. Language bindings no longer need
  // to parse error text or infer the identity of a motor with no prior cache.
  ARTICORE_CAP_STRUCTURED_CONNECT_REPORT = 1ULL << 26,
};

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
// command, disable, or fault replaces it. A streaming command
// must be refreshed before command_timeout_ms expires.
enum ArticoreCommandLifetime {
  ARTICORE_COMMAND_STREAMING = 1,
  ARTICORE_COMMAND_HOLD_UNTIL_REPLACED = 2,
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
  // Source-level convenience aliases. New bindings should expose 1..10.
  ARTICORE_GRIPPER_FORCE_LOW = ARTICORE_GRIPPER_FORCE_MIN,
  ARTICORE_GRIPPER_FORCE_NORMAL = ARTICORE_GRIPPER_FORCE_DEFAULT,
  ARTICORE_GRIPPER_FORCE_HIGH = ARTICORE_GRIPPER_FORCE_MAX,
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

typedef struct ArticoreJointMitTarget {
  // Caller initializes this to sizeof(ArticoreJointMitTarget).
  uint32_t struct_size;
  void* motor;
  float target_position;
} ArticoreJointMitTarget;

typedef struct ArticoreJointPvTarget {
  // Caller initializes this to sizeof(ArticoreJointPvTarget).
  uint32_t struct_size;
  void* motor;
  float target_position;
} ArticoreJointPvTarget;

typedef struct ArticoreGripperTarget {
  void* motor;
  float opening;
} ArticoreGripperTarget;

typedef struct ArticoreGripperCommand {
  // Caller initializes this to sizeof(ArticoreGripperCommand).
  uint32_t struct_size;
  void* motor;
  // Product-independent opening scale: 0=closed, 1000=open.
  float opening;
  // Product-independent speed scale: (0, 1000], where 1000 is the calibrated
  // maximum speed in the motor descriptor.
  float speed;
  int32_t force_level;
} ArticoreGripperCommand;

typedef struct ArticoreGripperForceProfile {
  // Caller initializes this to sizeof(ArticoreGripperForceProfile).
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
  // Caller initializes this to sizeof(ArticoreGripperProductBinding).
  uint32_t struct_size;
  void* motor;
  // NUL-terminated built-in profile identifier. Runtime copies this value;
  // no caller-owned string lifetime is retained.
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
} ArticoreJointControlConfig;

typedef struct ArticoreJointSafetyLimits {
  // Caller initializes this to sizeof(ArticoreJointSafetyLimits).
  uint32_t struct_size;
  void* motor;
  float hard_lower_position;
  float hard_upper_position;
  float soft_lower_position;
  float soft_upper_position;
  float soft_limit_braking_zone;
  // Positive magnitude used by d_stop = velocity^2 / (2 * acceleration).
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
  // Caller initializes this to sizeof(ArticoreFeedbackReport).
  uint32_t struct_size;
  uint32_t timeout_ms;
  uint32_t expected_count;
  uint32_t received_count;
  uint32_t missing_count;
} ArticoreFeedbackReport;

typedef struct ArticoreMotorIdentity {
  // Caller initializes this to sizeof(ArticoreMotorIdentity).
  uint32_t struct_size;
  void* motor;
  // Damiao motor identity carried by decoded feedback (0..255).
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
  // Caller initializes this to sizeof(ArticoreConnectReport).
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
// Returns the immutable rate actually used by the native worker. This can be
// lower than the requested config rate when a dual-arm runtime is capped to
// the verified shared-adapter limit.
ARTICORE_RUNTIME_API int32_t articore_runtime_get_control_hz(
    ArticoreRuntime* runtime, uint32_t* control_hz);

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

// Configures the immutable CAN identity of every Runtime motor. The complete
// active motor set must be supplied before connect(); IDs must be unique within
// a channel. This additive call preserves compatibility for legacy embedders,
// while official bindings always configure identities.
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_motor_identities(
    ArticoreRuntime* runtime,
    const ArticoreMotorIdentity* identities,
    uint32_t identity_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_connect_report(
    ArticoreRuntime* runtime, ArticoreConnectReport* report);
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
// Configures every active arm joint before connect so ordinary and raw Runtime
// commands use explicit position, velocity and torque limits.
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_joints(
    ArticoreRuntime* runtime,
    const ArticoreJointControlConfig* configs,
    uint32_t config_count);
// ABI 1.9 layered joint limits. This additive configuration keeps the legacy
// ArticoreJointControlConfig layout stable. It must cover every active arm
// joint and be called after configure_joints(), before connect().
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_joint_safety_limits(
    ArticoreRuntime* runtime,
    const ArticoreJointSafetyLimits* limits,
    uint32_t limit_count);
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
// ABI 1.13 ordinary PV position setting. Semantics match set_joint_mit(): one
// complete arm batch, one shared rad/s reference speed, capacity-one latest
// value replacement, and feedback initialization only for the first ordinary
// PV command after enable/reconnect/recover. The generated PV position advances
// by at most max_reference_velocity/control_hz per native cycle.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_pv(
    ArticoreRuntime* runtime,
    const ArticoreJointPvTarget* targets,
    uint32_t target_count,
    float max_reference_velocity);
// ABI 1.12 ordinary MIT position setting. The target array must contain every
// active arm joint in one atomic batch. max_reference_velocity is one shared
// positive rad/s limit used only to advance q; transmitted dq and tau are zero
// and kp/kd come from the configured product joint parameters. The Runtime
// keeps transmitting the final position until an explicit control transaction
// replaces it. Raw submit_mit[_ex]() remains unmodified and un-ramped.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit(
    ArticoreRuntime* runtime,
    const ArticoreJointMitTarget* targets,
    uint32_t target_count,
    float max_reference_velocity);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_gripper_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_gripper_openings(
    ArticoreRuntime* runtime,
    const ArticoreGripperTarget* targets,
    uint32_t target_count);
// ABI 2.2 product binding. Every active gripper must be covered exactly once
// before connect. No binding is required when the runtime has no grippers.
// The built-in yunyi_gripper_v1 profile owns opening conversion, motion ramp,
// contact/stall/overload protection, retreat behavior, ten force levels, gains,
// and fault action.
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gripper_products(
    ArticoreRuntime* runtime,
    const ArticoreGripperProductBinding* bindings,
    uint32_t binding_count);
// ABI 1.11 product force calibration. A complete level 1..10 profile set for
// every active gripper is fixed before connect. This remains available as an
// advanced/test override after a built-in product profile has been bound; it
// is not required for normal product integration.
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gripper_force_profiles(
    ArticoreRuntime* runtime,
    const ArticoreGripperForceProfile* profiles,
    uint32_t profile_count);
// Atomically replaces the complete active gripper command set. Opening and
// closing both use a bounded Runtime-generated position ramp. Changing speed
// or force level takes effect together on the next native control cycle.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_gripper_commands(
    ArticoreRuntime* runtime,
    const ArticoreGripperCommand* commands,
    uint32_t command_count);
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
