#ifndef ARTICORE_DETAIL_RUNTIME_BRIDGE_HPP
#define ARTICORE_DETAIL_RUNTIME_BRIDGE_HPP

#include <stdint.h>

#define ARTICORE_RUNTIME_API

typedef struct ArticoreRuntime ArticoreRuntime;
typedef struct ArticoreRobotModel ArticoreRobotModel;

/* ABI conventions:
 * - ABI versions must match exactly; this header defines the only supported
 *   Yunyi product contract.
 * - Callers set struct_size to sizeof(the structure) before output calls.
 * - Product joint arrays are ordered left J1..J7, then right J1..J7.
 * - Angles use radians, distances use metres, and timestamps use CLOCK_MONOTONIC.
 * - Functions return 0 on success; details for failures are available from
 *   runtime_bridge_last_error() and Runtime health.
 */

enum {
  ARTICORE_PRODUCT_ARM_DOF = 7,
  ARTICORE_PRODUCT_DUAL_ARM_DOF = 14,
  ARTICORE_PRODUCT_POSE_DOF = 6,
};

enum ArticoreRobotSide {
  ARTICORE_ROBOT_LEFT = 0,
  ARTICORE_ROBOT_RIGHT = 1,
};

enum ArticoreJacobianReference {
  ARTICORE_JACOBIAN_LOCAL = 0,
  ARTICORE_JACOBIAN_WORLD = 1,
  ARTICORE_JACOBIAN_LOCAL_WORLD_ALIGNED = 2,
};

enum { ARTICORE_MAX_ROBOT_DOF = 16 };

typedef struct ArticoreRobotModelInfo {
  uint32_t struct_size;
  uint32_t dof;
  uint32_t side;
  char product_id[64];
  char end_effector_frame[64];
  char joint_names[ARTICORE_MAX_ROBOT_DOF][64];
  double lower_limits[ARTICORE_MAX_ROBOT_DOF];
  double upper_limits[ARTICORE_MAX_ROBOT_DOF];
} ArticoreRobotModelInfo;

typedef struct ArticoreRobotPose {
  uint32_t struct_size;
  double position[3];
  // Row-major 3x3 rotation matrix.
  double rotation[9];
  // Row-major 4x4 homogeneous transform.
  double homogeneous[16];
} ArticoreRobotPose;

typedef struct ArticoreIkOptions {
  // Zero-valued options select Runtime defaults.
  uint32_t struct_size;
  uint32_t max_iterations;
  uint32_t max_retries;
  double tolerance;
  double step_size;
  double damping;
  uint64_t random_seed;
} ArticoreIkOptions;

typedef struct ArticoreIkResult {
  uint32_t struct_size;
  int32_t success;
  uint32_t iterations;
  uint32_t dof;
  double error_norm;
  double q[ARTICORE_MAX_ROBOT_DOF];
} ArticoreIkResult;

enum ArticoreSafetyState {
  ARTICORE_DISCONNECTED = 0,
  ARTICORE_READY = 1,
  ARTICORE_ENABLED = 2,
  ARTICORE_RUNNING = 3,
  ARTICORE_SAFE_HOLD = 4,
  ARTICORE_FAULT = 5,
  ARTICORE_DEGRADED = 6,
  ARTICORE_SAFE_STOP = 7,
  ARTICORE_PARTIALLY_ENABLED = 8,
};

enum ArticoreControlMode {
  ARTICORE_MODE_PV = 1,
  ARTICORE_MODE_MIT = 2,
};

enum ArticoreRuntimeOperation {
  ARTICORE_OPERATION_NONE = 0,
  ARTICORE_OPERATION_CONNECT = 1,
  ARTICORE_OPERATION_ENABLE = 2,
  ARTICORE_OPERATION_DISABLE = 3,
  ARTICORE_OPERATION_CONFIGURE_MODE = 4,
  ARTICORE_OPERATION_CLEAR_FAULTS = 5,
  ARTICORE_OPERATION_SET_ZERO = 6,
  ARTICORE_OPERATION_DISCONNECT = 7,
  ARTICORE_OPERATION_COMMAND = 8,
  ARTICORE_OPERATION_RECOVER = 9,
  ARTICORE_OPERATION_CANCEL_MOTION = 11,
  ARTICORE_OPERATION_MOVE_POSE = 12,
  ARTICORE_OPERATION_STOP_MOTION = 13,
  ARTICORE_OPERATION_MOVE_LINEAR = 14,
  ARTICORE_OPERATION_MOVE_CIRCULAR = 15,
  ARTICORE_OPERATION_START_BIMANUAL_FOLLOW = 16,
  ARTICORE_OPERATION_STOP_BIMANUAL_FOLLOW = 17,
  ARTICORE_OPERATION_SET_TCP_OFFSET = 18,
};

/* Source aliases for the legacy Motion-ID ABI. */
#define ARTICORE_OPERATION_CANCEL_ALL_MOTIONS ARTICORE_OPERATION_STOP_MOTION
#define ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY ARTICORE_OPERATION_MOVE_LINEAR
#define ARTICORE_OPERATION_MOVE_CIRCULAR_TRAJECTORY ARTICORE_OPERATION_MOVE_CIRCULAR

enum ArticoreMotionState {
  ARTICORE_MOTION_IDLE = 0,
  ARTICORE_MOTION_RUNNING = 1,
  // The planned clock has finished and fresh physical feedback has remained
  // within the native product position/velocity arrival window. A progress
  // value of 1.0 while state is still RUNNING means final-setpoint settling,
  // not physical completion.
  ARTICORE_MOTION_COMPLETED = 2,
  ARTICORE_MOTION_CANCELLED = 3,
  ARTICORE_MOTION_FAULT = 4,
  /* Accepted and fully planned, waiting for earlier FIFO motions to finish. */
  ARTICORE_MOTION_QUEUED = 5,
};

enum ArticoreMotionType {
  ARTICORE_MOTION_CARTESIAN_LINEAR = 2,
  ARTICORE_MOTION_CARTESIAN_CIRCULAR = 3,
};

enum ArticoreOperationError {
  ARTICORE_OPERATION_OK = 0,
  ARTICORE_OPERATION_INVALID_ARGUMENT = 1,
  ARTICORE_OPERATION_INVALID_STATE = 2,
  ARTICORE_OPERATION_TRANSPORT = 3,
  ARTICORE_OPERATION_FEEDBACK = 4,
  ARTICORE_OPERATION_NOT_DISABLED = 5,
  ARTICORE_OPERATION_NOT_STATIONARY = 6,
  ARTICORE_OPERATION_MOTOR_COMMAND = 7,
  ARTICORE_OPERATION_VERIFICATION = 8,
  ARTICORE_OPERATION_UNSUPPORTED = 9,
};

/* Per-Motor feedback diagnostics. Multiple issue bits may be set at once so
 * stale cached state never hides a reported Motor status or invalid payload. */
enum ArticoreMotorFeedbackIssue {
  ARTICORE_FEEDBACK_ISSUE_NONE = 0,
  ARTICORE_FEEDBACK_ISSUE_MISSING = 1U << 0,
  ARTICORE_FEEDBACK_ISSUE_STALE = 1U << 1,
  ARTICORE_FEEDBACK_ISSUE_STATE_UNAVAILABLE = 1U << 2,
  ARTICORE_FEEDBACK_ISSUE_NONFINITE = 1U << 3,
  ARTICORE_FEEDBACK_ISSUE_MOTOR_FAULT = 1U << 4,
  ARTICORE_FEEDBACK_ISSUE_UNEXPECTED_POWER_STATE = 1U << 5,
};

enum ArticoreFeedbackIssueScope {
  ARTICORE_FEEDBACK_SCOPE_NONE = 0,
  ARTICORE_FEEDBACK_SCOPE_SINGLE_MOTOR = 1,
  ARTICORE_FEEDBACK_SCOPE_MULTIPLE_MOTORS = 2,
  ARTICORE_FEEDBACK_SCOPE_LEFT_CHANNEL = 3,
  ARTICORE_FEEDBACK_SCOPE_RIGHT_CHANNEL = 4,
  ARTICORE_FEEDBACK_SCOPE_BOTH_CHANNELS = 5,
};

enum ArticoreGravityCompensationPhase {
  ARTICORE_GRAVITY_INACTIVE = 0,
  ARTICORE_GRAVITY_ENTERING = 1,
  ARTICORE_GRAVITY_ACTIVE = 2,
  ARTICORE_GRAVITY_EXITING = 3,
};

typedef struct ArticoreGravityCompensationConfig {
  uint32_t struct_size;
  // Zero selects the product default of 500 ms.
  uint32_t transition_ms;
} ArticoreGravityCompensationConfig;

typedef struct ArticoreGravityCompensationStatus {
  uint32_t struct_size;
  int32_t phase;
  int32_t active;
  float transition_progress;
  uint64_t control_cycles;
  // Always 14; values use fixed product joint order.
  uint32_t joint_count;
  // Final motor-domain gravity feedforward from the latest sent cycle.
  float gravity_feedforward_torque[ARTICORE_PRODUCT_DUAL_ARM_DOF];
} ArticoreGravityCompensationStatus;

enum ArticoreBimanualFollowPhase {
  ARTICORE_BIMANUAL_FOLLOW_INACTIVE = 0,
  ARTICORE_BIMANUAL_FOLLOW_ENTERING = 1,
  ARTICORE_BIMANUAL_FOLLOW_ACTIVE = 2,
  ARTICORE_BIMANUAL_FOLLOW_EXITING = 3,
};

typedef struct ArticoreBimanualFollowStatus {
  uint32_t struct_size;
  int32_t phase;
  int32_t active;
  uint32_t leader_side;
  uint32_t follower_side;
  float transition_progress;
  uint64_t control_cycles;
  float leader_positions[ARTICORE_PRODUCT_ARM_DOF];
  float follower_target_positions[ARTICORE_PRODUCT_ARM_DOF];
  float max_tracking_error;
  char error[512];
} ArticoreBimanualFollowStatus;

enum ArticoreGripperControlState {
  ARTICORE_GRIPPER_DISABLED = 0,
  ARTICORE_GRIPPER_IDLE = 1,
  ARTICORE_GRIPPER_MOVING = 2,
  ARTICORE_GRIPPER_CONTACT = 3,
  ARTICORE_GRIPPER_HOLDING = 4,
  ARTICORE_GRIPPER_OVERLOAD_RETREAT = 5,
  ARTICORE_GRIPPER_FAULT = 6,
};

enum ArticoreGripperMode {
  // Default behavior: detect sustained contact/stall, switch to calibrated
  // holding gains, and retreat after sustained overload.
  ARTICORE_GRIPPER_MODE_PROTECTED = 0,
  // Direct behavior: continue tracking the requested opening with the
  // selected strength and do not run contact/stall or overload-retreat logic.
  ARTICORE_GRIPPER_MODE_DIRECT = 1,
};

enum {
  ARTICORE_GRIPPER_STRENGTH_MIN = 0,
  ARTICORE_GRIPPER_STRENGTH_DEFAULT = 5,
  ARTICORE_GRIPPER_STRENGTH_MAX = 10,
};

typedef struct ArticoreMotionStatus {
  uint32_t struct_size;
  uint64_t motion_id;
  int32_t motion_type;
  int32_t state;
  uint32_t active_segment;
  uint32_t waypoint_count;
  double elapsed_s;
  double duration_s;
  float progress;
  char error[512];
} ArticoreMotionStatus;

typedef struct ArticoreProductArmState {
  float positions[ARTICORE_PRODUCT_ARM_DOF];
  float velocities[ARTICORE_PRODUCT_ARM_DOF];
  float torques[ARTICORE_PRODUCT_ARM_DOF];
  float mos_temperatures[ARTICORE_PRODUCT_ARM_DOF];
  float rotor_temperatures[ARTICORE_PRODUCT_ARM_DOF];
  // Bit i is set only when fresh cached feedback reports status_code == 1.
  uint32_t enabled_mask;
  // A clear bit means the power state is unknown.
  uint32_t enabled_valid_mask;
  // A clear bit means the corresponding temperature values are NaN.
  uint32_t temperature_valid_mask;
} ArticoreProductArmState;

typedef struct ArticoreProductState {
  uint32_t struct_size;
  int32_t has_grippers;
  ArticoreProductArmState left;
  ArticoreProductArmState right;
  int32_t left_gripper_available;
  int32_t right_gripper_available;
  float left_gripper_opening;
  float right_gripper_opening;
  int32_t left_gripper_level;
  int32_t right_gripper_level;
  int32_t left_gripper_enabled;
  int32_t right_gripper_enabled;
  int32_t left_gripper_enabled_valid;
  int32_t right_gripper_enabled_valid;
  float left_gripper_mos_temperature;
  float left_gripper_rotor_temperature;
  float right_gripper_mos_temperature;
  float right_gripper_rotor_temperature;
  int32_t left_gripper_temperature_valid;
  int32_t right_gripper_temperature_valid;
  // Global finite Cartesian-motion arrival bit. One means the most recently
  // accepted move_pose/move_linear/move_circular command has physically
  // completed and no finite Cartesian motion remains active. Zero means the
  // command is running, settling, cancelled, faulted, or otherwise not known
  // to have arrived. This is not a progress value and does not describe raw
  // latest-target PV/MIT streaming commands.
  int32_t motion_arrived;
  // CLOCK_MONOTONIC-compatible timestamp and slowest update sequence across
  // every installed motor represented by this single cached snapshot.
  uint64_t timestamp_ns;
  uint64_t sequence;
} ArticoreProductState;

typedef struct ArticoreProductJointAngleVelLimits {
  uint32_t struct_size;
  // Always ARTICORE_PRODUCT_DUAL_ARM_DOF. Array order is fixed as
  // left/l-joint1..7 followed by right/r-joint1..7; grippers are excluded.
  uint32_t joint_count;
  // Logical product coordinates from the same product table used by Runtime
  // command validation. Angles are radians and velocities are radians/second.
  float lower_angles[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float upper_angles[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float velocity_limits[ARTICORE_PRODUCT_DUAL_ARM_DOF];
} ArticoreProductJointAngleVelLimits;

typedef struct ArticoreProductPose {
  uint32_t struct_size;
  uint32_t side;
  // [x, y, z, roll, pitch, yaw], using metres and radians. RPY follows the
  // conventional Rz(yaw) * Ry(pitch) * Rx(roll) decomposition.
  float values[ARTICORE_PRODUCT_POSE_DOF];
  // Timestamp and sequence describe the oldest joint feedback used by this
  // coherent seven-joint pose sample.
  uint64_t timestamp_ns;
  uint64_t sequence;
} ArticoreProductPose;

typedef struct ArticoreTcpOffset {
  uint32_t struct_size;
  uint32_t side;
  // Transform from the product flange (link7) to the active TCP, expressed
  // as [x, y, z, roll, pitch, yaw] in metres and radians.
  float values[ARTICORE_PRODUCT_POSE_DOF];
} ArticoreTcpOffset;

typedef struct ArticoreMotorPowerResult {
  uint8_t side;
  uint8_t can_id;
  uint8_t requested_enabled;
  uint8_t command_sent;
  uint8_t rollback_sent;
  uint8_t has_feedback;
  uint8_t feedback_fresh;
  uint8_t status_code;
  uint8_t confirmed;
  char role[64];
  char error[256];
} ArticoreMotorPowerResult;

typedef struct ArticoreMotorPowerReport {
  uint32_t struct_size;
  int32_t success;
  int32_t requested_enabled;
  int32_t rollback_attempted;
  int32_t rollback_confirmed;
  uint32_t requested_count;
  uint32_t command_sent_count;
  uint32_t confirmed_count;
  uint32_t failure_count;
  uint32_t motor_count;
  ArticoreMotorPowerResult motors[32];
  char error[512];
} ArticoreMotorPowerReport;

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

typedef struct ArticoreMotorFeedbackHealth {
  uint32_t side;
  uint32_t can_id;
  uint32_t can_id_valid;
  uint32_t is_gripper;
  uint32_t has_feedback;
  uint32_t fresh;
  uint32_t has_state;
  uint32_t values_finite;
  uint32_t status_code;
  uint32_t issues;
  float position;
  float velocity;
  float torque;
  uint64_t feedback_age_ns;
  uint64_t update_count;
  char role[64];
} ArticoreMotorFeedbackHealth;

typedef struct ArticoreSafetyHealth {
  uint32_t struct_size;
  int32_t state;
  int32_t safe_holding;
  int32_t disable_confirmed;
  uint64_t last_successful_command_age_ns;
  uint64_t last_fresh_feedback_age_ns;
  uint32_t consecutive_send_failures;
  uint32_t consecutive_feedback_failures;
  ArticoreTransportHealth left_transport;
  ArticoreTransportHealth right_transport;
  /* Complete product-order feedback diagnostics. feedback_issue_scope
   * distinguishes an isolated Motor from a stalled left/right/both channel. */
  uint32_t motor_feedback_count;
  uint32_t feedback_issue_count;
  int32_t feedback_issue_scope;
  ArticoreMotorFeedbackHealth motor_feedback[32];
  uint32_t gripper_count;
  ArticoreGripperHealth grippers[2];
  uint32_t motor_fault_count;
  char motor_faults[32][64];
  uint32_t unconfirmed_disable_count;
  char unconfirmed_disable[32][64];
  char fault_reason[512];
  // One diagnostic surface for every Runtime lifecycle or maintenance action.
  int32_t last_operation;
  int32_t last_operation_code;
  uint32_t operation_failed_motor_count;
  char operation_failed_motors[32][64];
  char last_operation_error[512];
  // fault_reason is for confirmed FAULT; transient quality is reported here.
  int32_t degraded;
  int32_t safe_stopped;
  int32_t requires_resynchronization;
  float command_scale;
  char safety_reason[512];
  uint64_t control_ticks;
  uint64_t control_overruns;
  uint64_t maximum_control_period_ns;
  uint64_t maximum_send_time_ns;
} ArticoreSafetyHealth;

typedef struct ArticoreMitTorqueLimitStats {
  uint32_t struct_size;
  // Number of successfully transmitted native control cycles in which at
  // least one arm joint required limiting.
  uint64_t torque_limit_activation_count;
  // Bit i describes product joint i from the most recently transmitted MIT
  // arm batch. A set bit means that joint was limited in that cycle.
  uint64_t torque_limited_joint_mask;
  // Values use fixed product joint order and joint_count is always 14 for
  // product commands.
  uint32_t joint_count;
  // Values use the motor command/feedback torque domain used by the native
  // MIT protocol after any product range mapping performed by the caller.
  float requested_resultant_torque[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float applied_scale[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float applied_resultant_torque[ARTICORE_PRODUCT_DUAL_ARM_DOF];
} ArticoreMitTorqueLimitStats;

/* Private bridge for the Runtime implementation. This file is not installed. */
ARTICORE_RUNTIME_API ArticoreRobotModel* robot_model_bridge_create(
    const char* product_id, uint32_t side);
ARTICORE_RUNTIME_API void robot_model_bridge_free(ArticoreRobotModel* model);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_get_info(
    ArticoreRobotModel* model, ArticoreRobotModelInfo* info);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_fk(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    ArticoreRobotPose* pose);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_jacobian(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    uint32_t reference, double* jacobian, uint32_t jacobian_count);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_gravity(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    double* torque, uint32_t torque_count);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_mass_matrix(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    double* matrix, uint32_t matrix_count);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_coriolis_matrix(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, double* matrix,
    uint32_t matrix_count);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_nonlinear_effects(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, double* torque,
    uint32_t torque_count);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_rnea(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, const double* ddq,
    uint32_t ddq_count, double* torque, uint32_t torque_count);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_aba(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, const double* torque,
    uint32_t torque_count, double* ddq, uint32_t ddq_count);
ARTICORE_RUNTIME_API int32_t robot_model_bridge_ik(
    ArticoreRobotModel* model, const ArticoreRobotPose* target,
    const double* initial_q, uint32_t initial_q_count,
    const ArticoreIkOptions* options, ArticoreIkResult* result);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_control_mode(
    ArticoreRuntime* runtime, int32_t* mode);

/* Product lifecycle. Runtime owns both SocketCAN-FD channels and all Motors. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_create_yunyi(
    int32_t mode, int32_t with_grippers, ArticoreRuntime** runtime);
/* Private 1.0 facade bridge. This header is never installed or linked as a
 * shared ABI; the function exists only inside articore_runtime.a. */
int32_t runtime_bridge_create_yunyi_configured(
    int32_t mode, int32_t with_grippers, const char* left_can_interface,
    const char* right_can_interface, int32_t realtime, int32_t lock_memory,
    int32_t control_cpu, int32_t can_tx_cpu, int32_t can_rx_cpu,
    int32_t control_priority, int32_t can_tx_priority,
    int32_t can_rx_priority, uint32_t feedback_max_age_ms,
    uint32_t motor_watchdog_ms, ArticoreRuntime** runtime);
ARTICORE_RUNTIME_API void runtime_bridge_free(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t runtime_bridge_connect(ArticoreRuntime* runtime);
/* Idempotent safe shutdown; call free() afterward to release the opaque handle. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_disconnect(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t runtime_bridge_configure_mode(
    ArticoreRuntime* runtime, int32_t mode);
/* Clear recoverable Motor faults without motion. A faulted product remains
 * connected; success also reapplies mode/watchdog and verifies disable. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_clear_faults(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_zero(
    ArticoreRuntime* runtime);

/* Shared product speed percentage. The default is 100. Values within 1..100
 * immediately update an active ordinary-PV velocity/acceleration envelope and
 * are snapshotted by each subsequently submitted Linear/Circular trajectory.
 * Cartesian reference velocity is multiplied by s and acceleration by s^2,
 * where s=speed_percent/100; automatic time parameterization then selects the
 * safe duration. Existing queued or running Cartesian plans retain the
 * percentage captured at submission.
 * Legacy per-command PV speed_percent arguments also update this shared value
 * for subsequent commands. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_speed_percent(
    ArticoreRuntime* runtime, float speed_percent);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_speed_percent(
    ArticoreRuntime* runtime, float* speed_percent);

/* Ordinary product joint commands use fixed left J1..J7, right J1..J7 order.
 * PV is public latest-target-wins endpoint control, not a complete trajectory
 * task. Runtime sends final P on the first 500 Hz frame and shapes only Motor
 * V from the speed ceiling, acceleration limit, and physical feedback
 * distance. Distance-based braking and a confirmed V=0 hold complete
 * arrival without creating a finite point list.
 * speed_percent accepts 1..100. Unless the optional maximum-speed or
 * maximum-acceleration override below is configured, at 100 percent the
 * J1..J7 velocity limits are
 * [180,180,180,225,225,225,225] deg/s and acceleration hard limits are
 * [450,450,900,900,900,900,900] deg/s^2. Runtime time-scales velocity by s
 * and acceleration by s^2. A configured user value replaces the corresponding
 * per-joint 100-percent base. Runtime selects POS_VEL V every cycle from the
 * acceleration ramp and physical feedback-distance braking limit.
 */
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_joint_pv(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
    float speed_percent);
/* Standard MIT control. Callers provide one complete atomic q/dq/kp/kd/tau_ff
 * frame in fixed left J1..J7, right J1..J7 order. A newer frame replaces the
 * previous frame. This is a watchdog-protected streaming command: it performs
 * no interpolation and creates no finite motion task.
 */
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_joint_mit(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocities, const float* kp, const float* kd,
    const float* feedforward_torques, uint32_t count);
/* Fast MIT endpoint control. Callers provide the newest complete joint angles
 * and speed_percent within 0..100. Runtime owns dq=0, tau_ff=0 and fixed fast
 * gains. 100 selects the 5 rad/s reference-step base, 50 selects 2.5 rad/s and
 * 0 keeps the current reference while retaining the latest endpoint. This is a
 * latest-target-wins online reference step, not a finite trajectory. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_joint_mit_fast(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
    float speed_percent);
/* Pure product inverse kinematics. Both poses use
 * [x,y,z,roll,pitch,yaw] in metres/radians. positions receives exactly 14
 * logical joint angles in fixed left J1..J7, right J1..J7 order. Runtime uses
 * one coherent planned reference, or fresh connected feedback before enable,
 * as the nearest-branch seed and applies the active TCP and product limits.
 * This call never enables Motors,
 * sends a command, or changes the motion queue. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_solve_ik(
    ArticoreRuntime* runtime, const float* left_target_pose,
    const float* right_target_pose, float* positions, uint32_t count);
/* Optional persistent ordinary-PV joint limits in physical units. A positive
 * value becomes the 100-percent base limit for every arm joint; 0 clears that
 * user override and restores the fixed per-joint product defaults. The getter
 * returns 0 when the corresponding override is not configured.
 *
 * For speed_percent=s, Runtime applies max_speed*s and
 * max_acceleration*s^2. Values use 0.01 physical-unit resolution and values
 * above the most restrictive J1..J7 safety limit are rejected. These base
 * settings affect ordinary PV only. Finite Cartesian motions retain
 * their own internal base limits but share the Runtime speed percentage.
 */
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_max_speed(
    ArticoreRuntime* runtime, float max_speed_rad_s);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_max_speed(
    ArticoreRuntime* runtime, float* max_speed_rad_s);
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_max_acceleration(
    ArticoreRuntime* runtime, float max_acceleration_rad_s2);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_max_acceleration(
    ArticoreRuntime* runtime, float* max_acceleration_rad_s2);
/* Simple non-blocking Cartesian motion API. These calls return only whether
 * the complete plan was accepted. Runtime owns execution and physical-arrival
 * detection; callers read ArticoreProductState.motion_arrived and health.
 * Only one finite Cartesian motion may be active through this surface. */
/* Pose-to-Pose starts at the current planned pose captured by Runtime in the
 * submission transaction. It has the same implicit-start behavior as passing
 * NULL start_pose to runtime_bridge_move_linear(). */
ARTICORE_RUNTIME_API int32_t runtime_bridge_move_pose(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose);
/* Unified finite Linear entry point. If start_pose is NULL, the Cartesian line
 * begins at the current planned pose. Otherwise Runtime applies the same
 * deterministic multi-seed endpoint-IK policy as ordinary Cartesian PTP,
 * orders the reachable start branches by distance from the current planned
 * joints, and selects the first branch that can continuously complete the
 * whole Linear path. Runtime preplans the PTP approach and line before it
 * installs either segment, then executes them under one motion task with a
 * physical-feedback convergence barrier at start_pose. No public move_pose
 * call or SDK-side command sequence is involved. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_move_linear(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* end_pose);
ARTICORE_RUNTIME_API int32_t runtime_bridge_move_linear_path(
    ArticoreRuntime* runtime, uint32_t side, const float* poses,
    uint32_t pose_count);
ARTICORE_RUNTIME_API int32_t runtime_bridge_move_circular(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose);
ARTICORE_RUNTIME_API int32_t runtime_bridge_stop_motion(
    ArticoreRuntime* runtime);
/* Legacy Motion-ID/FIFO ABI retained for binary compatibility. New SDKs use
 * the simple nonblocking move_* surface above. Linear interpolates XYZ on a
 * Cartesian line and uses shortest-path quaternion SLERP for orientation.
 * For an implicit start, the first path pose is the current planned pose. For
 * an explicit start, Runtime first searches the ordinary PTP endpoint branches
 * and jointly validates each candidate against the complete later path. After
 * a start branch is selected, every later pose is solved only from the
 * preceding joint solution. Later path IK never uses fallback, random, or
 * extrapolated posture seeds. The null-space objective stays near that same
 * seed, while path IK prioritizes XYZ and permits a
 * bounded orientation residual. One required reversal is valid;
 * repeated small +/- joint-direction chatter and true branch jumps are rejected
 * before execution. Geometry is sampled at 2 mm / 0.1 rad or better.
 * A quintic time law generates adaptive 4..50 ms internal trajectory-PV knots,
 * additionally bounded by joint step and linearization error.
 * Runtime automatically selects the shortest safe duration from the shared
 * Runtime speed percentage and internal Cartesian speed/acceleration limits.
 * Runtime linearly resamples adjacent knots and transmits the resulting
 * reference on its 500 Hz command clock, without applying the ordinary-PV
 * endpoint step generator.
 * Physical arrival may be later than the planned reference duration. Linear
 * and Circular require PV mode. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_move_linear_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* end_pose, uint64_t* motion_id);
/* Atomic multi-segment Linear path. poses is pose_count contiguous
 * [x,y,z,roll,pitch,yaw] records. Every declared internal pose is preserved;
 * no Cartesian fillet is inserted. Each sharp segment boundary uses its own
 * rest-to-rest quintic law so the path reaches the corner without a non-zero
 * velocity direction discontinuity. poses[0] is an explicit start and uses
 * the same atomic PTP-approach and full-path branch-selection semantics as
 * move_linear. Runtime automatically parameterizes the complete path using
 * the shared speed percentage. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_move_linear_path_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* poses,
    uint32_t pose_count, uint64_t* motion_id);
/* Standard directed circular path through start/via/end. Position is sampled
 * at 2 mm or better, orientation uses shortest-path SLERP through the via
 * orientation, and start is an explicit PTP-approached pose. Runtime tests
 * endpoint IK branches in nearest-current order and accepts only one that can
 * continuously complete the whole arc. After selecting that start branch,
 * seed-only sequential IK preserves it without random retries or posture
 * extrapolation, and one global
 * quintic time law uses the shared Runtime speed percentage to select an
 * automatic safe duration and produces adaptive 4..50 ms internal trajectory-PV
 * knots, which Runtime linearly resamples on its 500 Hz command clock. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_move_circular_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, uint64_t* motion_id);
/* Legacy Motion-ID status and cancellation ABI.
 * Terminal states remain queryable by id in the bounded Runtime history.
 * Cancelling a queued id removes only that item and inserts a validated native
 * approach into its immediate successor so the FIFO cannot jump. Cancelling
 * the running id holds the last safe reference and cancels its queue tail
 * because those plans were generated from the running task's final reference.
 */
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_motion_status(
    ArticoreRuntime* runtime, uint64_t motion_id,
    ArticoreMotionStatus* status);
ARTICORE_RUNTIME_API int32_t runtime_bridge_cancel_motion(
    ArticoreRuntime* runtime, uint64_t motion_id);
ARTICORE_RUNTIME_API int32_t runtime_bridge_cancel_all_motions(
    ArticoreRuntime* runtime);

/* Grippers: opening 0..1000, strength 0..10, protected/direct mode.
 * Gripperless products return success without sending a command. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_grippers(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
    int32_t strength, int32_t mode);
ARTICORE_RUNTIME_API int32_t runtime_bridge_has_grippers(
    ArticoreRuntime* runtime, int32_t* has_grippers);

/* Coherent cached product state. These calls perform no CAN I/O. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_state(
    ArticoreRuntime* runtime, ArticoreProductState* state);
/* Static product metadata. This call performs no CAN I/O and is valid before
 * connect() and independently of the enabled state. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_joint_angle_vel_limits(
    ArticoreRuntime* runtime, ArticoreProductJointAngleVelLimits* limits);
/* Cached active-TCP pose. The default is tool0 with grippers and link7
 * without grippers. A custom offset affects FK and every Cartesian IK path. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_pose(
    ArticoreRuntime* runtime, uint32_t side, ArticoreProductPose* pose);
ARTICORE_RUNTIME_API int32_t runtime_bridge_set_tcp_offset(
    ArticoreRuntime* runtime, const ArticoreTcpOffset* offset);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_tcp_offset(
    ArticoreRuntime* runtime, uint32_t side, ArticoreTcpOffset* offset);
ARTICORE_RUNTIME_API int32_t runtime_bridge_reset_tcp_offset(
    ArticoreRuntime* runtime, uint32_t side);

/* Power operations verify fresh feedback; batch enable rolls back on failure. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_enable(ArticoreRuntime* runtime);
/* roles use l/r-joint1..7 and optional l/r-gripper names. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_enable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report);
ARTICORE_RUNTIME_API int32_t runtime_bridge_disable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report);

/* Runtime-owned MIT gravity compensation; null config uses product defaults. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_start_gravity_compensation(
    ArticoreRuntime* runtime,
    const ArticoreGravityCompensationConfig* config);
ARTICORE_RUNTIME_API int32_t runtime_bridge_stop_gravity_compensation(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_gravity_compensation_status(
    ArticoreRuntime* runtime,
    ArticoreGravityCompensationStatus* status);

/* Current-mode relative joint-space leader/follower control. Runtime captures
 * both arm positions atomically. Subsequent ordinary PV/MIT commands control
 * the selected leader; the opposite arm follows its seven relative joint
 * displacements in the same complete fourteen-axis product frame. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_start_bimanual_follow(
    ArticoreRuntime* runtime, uint32_t leader_side);
ARTICORE_RUNTIME_API int32_t runtime_bridge_stop_bimanual_follow(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_bimanual_follow_status(
    ArticoreRuntime* runtime, ArticoreBimanualFollowStatus* status);

/* Safety and diagnostics. disable() never clears a latched fault. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_disable(ArticoreRuntime* runtime);
/* Idempotent, latched current-position hold; only recover() clears it. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_estop(ArticoreRuntime* runtime);
/* From any live Runtime state: disable, clear recoverable faults, return to
 * calibrated zero, then finish physically disabled. */
ARTICORE_RUNTIME_API int32_t runtime_bridge_recover(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_health(
    ArticoreRuntime* runtime, ArticoreSafetyHealth* health);
ARTICORE_RUNTIME_API int32_t runtime_bridge_get_mit_torque_limit_stats(
    ArticoreRuntime* runtime, ArticoreMitTorqueLimitStats* stats);
ARTICORE_RUNTIME_API const char* runtime_bridge_last_error(void);

#endif
