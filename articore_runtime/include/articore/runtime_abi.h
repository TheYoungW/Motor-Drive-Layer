#ifndef ARTICORE_RUNTIME_ABI_H
#define ARTICORE_RUNTIME_ABI_H

#include <stdint.h>

#define ARTICORE_RUNTIME_API __attribute__((visibility("default")))

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ArticoreRuntime ArticoreRuntime;
typedef struct ArticoreRobotModel ArticoreRobotModel;

/* ABI conventions:
 * - ABI versions must match exactly; this header defines the only supported
 *   Yunyi product contract.
 * - Callers set struct_size to sizeof(the structure) before output calls.
 * - Product joint arrays are ordered left J1..J7, then right J1..J7.
 * - Angles use radians, distances use metres, and timestamps use CLOCK_MONOTONIC.
 * - Functions return 0 on success; details for failures are available from
 *   articore_runtime_last_error() and Runtime health.
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
  ARTICORE_OPERATION_MOVE_JOINT_TRAJECTORY = 10,
  ARTICORE_OPERATION_CANCEL_MOTION = 11,
  ARTICORE_OPERATION_SET_POSE = 12,
  ARTICORE_OPERATION_CANCEL_ALL_MOTIONS = 13,
  ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY = 14,
  ARTICORE_OPERATION_MOVE_CIRCULAR_TRAJECTORY = 15,
  ARTICORE_OPERATION_START_BIMANUAL_FOLLOW = 16,
  ARTICORE_OPERATION_STOP_BIMANUAL_FOLLOW = 17,
  ARTICORE_OPERATION_SET_TCP_OFFSET = 18,
};

enum ArticoreTrajectoryInterpolation {
  ARTICORE_TRAJECTORY_QUINTIC = 1,
};

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
  ARTICORE_MOTION_JOINT_TRAJECTORY = 1,
  ARTICORE_MOTION_CARTESIAN_LINEAR = 2,
  ARTICORE_MOTION_CARTESIAN_CIRCULAR = 3,
};

enum { ARTICORE_MAX_TRAJECTORY_WAYPOINTS = 30000 };

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

typedef struct ArticoreTrajectoryWaypoint {
  uint32_t struct_size;
  double time_s;
  float left_positions[ARTICORE_PRODUCT_ARM_DOF];
  float right_positions[ARTICORE_PRODUCT_ARM_DOF];
  /* Reserved for ABI layout compatibility; callers must zero these fields.
   * Runtime derives all trajectory velocity, acceleration and jerk from joint
   * positions and timestamps. */
  float left_velocities[ARTICORE_PRODUCT_ARM_DOF];
  float right_velocities[ARTICORE_PRODUCT_ARM_DOF];
  float left_accelerations[ARTICORE_PRODUCT_ARM_DOF];
  float right_accelerations[ARTICORE_PRODUCT_ARM_DOF];
  uint32_t velocity_valid_mask;
  uint32_t acceleration_valid_mask;
} ArticoreTrajectoryWaypoint;

typedef struct ArticoreTrajectoryConfig {
  uint32_t struct_size;
  int32_t interpolation;
  int32_t control_mode;
  float mit_kp[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float mit_kd[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float mit_feedforward_torque[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  /* Reserved for ABI layout compatibility; callers must zero this array.
   * Runtime owns trajectory PV drive limits. */
  float pv_velocity_limits[ARTICORE_PRODUCT_DUAL_ARM_DOF];
} ArticoreTrajectoryConfig;

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

/* Version and robot model. ABI version is packed as 0xMMMMmmmm and must match
 * the SDK exactly. */
ARTICORE_RUNTIME_API uint32_t articore_runtime_abi_version(void);

/* Seven-axis Yunyi model. Matrices are row-major; no C++ types cross the ABI. */
ARTICORE_RUNTIME_API ArticoreRobotModel* articore_robot_model_create(
    const char* product_id, uint32_t side);
ARTICORE_RUNTIME_API void articore_robot_model_free(ArticoreRobotModel* model);
ARTICORE_RUNTIME_API int32_t articore_robot_model_get_info(
    ArticoreRobotModel* model, ArticoreRobotModelInfo* info);
ARTICORE_RUNTIME_API int32_t articore_robot_model_fk(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    ArticoreRobotPose* pose);
ARTICORE_RUNTIME_API int32_t articore_robot_model_jacobian(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    uint32_t reference, double* jacobian, uint32_t jacobian_count);
ARTICORE_RUNTIME_API int32_t articore_robot_model_gravity(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    double* torque, uint32_t torque_count);
ARTICORE_RUNTIME_API int32_t articore_robot_model_mass_matrix(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    double* matrix, uint32_t matrix_count);
ARTICORE_RUNTIME_API int32_t articore_robot_model_coriolis_matrix(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, double* matrix,
    uint32_t matrix_count);
ARTICORE_RUNTIME_API int32_t articore_robot_model_nonlinear_effects(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, double* torque,
    uint32_t torque_count);
ARTICORE_RUNTIME_API int32_t articore_robot_model_rnea(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, const double* ddq,
    uint32_t ddq_count, double* torque, uint32_t torque_count);
ARTICORE_RUNTIME_API int32_t articore_robot_model_aba(
    ArticoreRobotModel* model, const double* q, uint32_t q_count,
    const double* dq, uint32_t dq_count, const double* torque,
    uint32_t torque_count, double* ddq, uint32_t ddq_count);
ARTICORE_RUNTIME_API int32_t articore_robot_model_ik(
    ArticoreRobotModel* model, const ArticoreRobotPose* target,
    const double* initial_q, uint32_t initial_q_count,
    const ArticoreIkOptions* options, ArticoreIkResult* result);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_control_mode(
    ArticoreRuntime* runtime, int32_t* mode);

/* Product lifecycle. Runtime owns both SocketCAN-FD channels and all Motors. */
ARTICORE_RUNTIME_API int32_t articore_runtime_create_yunyi(
    int32_t mode, int32_t with_grippers, ArticoreRuntime** runtime);
ARTICORE_RUNTIME_API void articore_runtime_free(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime);
/* Idempotent safe shutdown; call free() afterward to release the opaque handle. */
ARTICORE_RUNTIME_API int32_t articore_runtime_disconnect(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_mode(
    ArticoreRuntime* runtime, int32_t mode);
/* Clear recoverable Motor faults without motion. A faulted product remains
 * connected; success also reapplies mode/watchdog and verifies disable. */
ARTICORE_RUNTIME_API int32_t articore_runtime_clear_faults(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_zero(
    ArticoreRuntime* runtime);

/* Ordinary product joint commands use fixed left J1..J7, right J1..J7 order.
 * PV is public latest-target-wins endpoint control, not a complete trajectory
 * task. Runtime sends the final P directly and preserves the current Motor-V
 * speed envelope when a target is replaced. Its 500 Hz worker refreshes the
 * ordinary POS_VEL command and shapes V; it does not generate intermediate P.
 * speed_percent is the only public PV motion parameter and accepts 1..100.
 * At 100 percent, J1..J7 velocity hard limits are
 * [180,180,180,225,225,225,225] deg/s and acceleration hard limits are
 * [450,450,900,900,900,900,900] deg/s^2. Runtime time-scales velocity by s
 * and acceleration by s^2, and derives the POS_VEL V field every cycle from
 * the reference velocity and measured tracking error.
 */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_pv(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
    float speed_percent);
/* Ordinary MIT endpoint control. Each complete command immediately replaces
 * the prior target without an intermediate position-reference step. Runtime
 * owns the fixed product gains and refreshes the persistent command at 500 Hz.
 */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit_direct(
    ArticoreRuntime* runtime, const float* positions, uint32_t count);
/* High-frequency teleoperation endpoint control. Callers provide only the
 * newest complete joint angles. Runtime applies the fast-follow gains and the
 * fixed internal 100-percent (5 rad/s) position-reference step limit.
 */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit_fast_follow(
    ArticoreRuntime* runtime, const float* positions, uint32_t count);
/* Deprecated ABI-compatibility entry point for the former user-selectable MIT
 * step speed. New SDKs must expose direct and fast-follow MIT instead.
 */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
    float speed_percent);
/* Pure product inverse kinematics. Both poses use
 * [x,y,z,roll,pitch,yaw] in metres/radians. positions receives exactly 14
 * logical joint angles in fixed left J1..J7, right J1..J7 order. Runtime uses
 * one coherent planned reference, or fresh connected feedback before enable,
 * as the nearest-branch seed and applies the active TCP and product limits.
 * This call never enables Motors,
 * sends a command, or changes the motion queue. */
ARTICORE_RUNTIME_API int32_t articore_runtime_solve_ik(
    ArticoreRuntime* runtime, const float* left_target_pose,
    const float* right_target_pose, float* positions, uint32_t count);
/* Deprecated ABI-compatibility symbols. Ordinary product PV acceleration is
 * fixed per joint and speed_percent is its only public motion parameter, so a
 * valid product Runtime returns ARTICORE_OPERATION_INVALID_STATE here. SDKs
 * should no longer expose these methods. Complete Joint/Linear/Circular
 * trajectories remain independent and are not affected by this deprecation.
 */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_max_acceleration(
    ArticoreRuntime* runtime, float max_acceleration_rad_s2);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_max_acceleration(
    ArticoreRuntime* runtime, float* max_acceleration_rad_s2);
/* Advanced streaming MIT frame with explicit q, dq, torque and gains. */
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocities, const float* feedforward_torques,
    const float* kp, const float* kd, uint32_t count);

/* Asynchronous dual-arm trajectory. Inputs are copied before return and the
 * task joins the same FIFO and motion-id namespace as Linear/Circular. In PV
 * mode Runtime converts the finite plan into internal 100 Hz real-time-PV
 * knots and linearly resamples them on its 500 Hz command clock. Product
 * callers provide joint positions and timestamps only;
 * waypoint derivative fields and PV limit fields are reserved and must be
 * zero. Runtime owns velocity, acceleration and jerk planning. There is no
 * public raw/streaming PV command. MIT mode samples the validated quintic
 * directly on the worker clock. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_joint_trajectory(
    ArticoreRuntime* runtime,
    const ArticoreTrajectoryWaypoint* waypoints,
    uint32_t waypoint_count,
    const ArticoreTrajectoryConfig* config,
    uint64_t* motion_id);

/* ABI compatibility convenience. This entry solves IK once, then atomically
 * installs that joint endpoint through the Runtime's current ordinary PV or
 * direct ordinary MIT mode. It does not plan a Cartesian path and has no
 * motion ID. The speed percentage is ignored in MIT mode. */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_pose(
    ArticoreRuntime* runtime, const float* left_target_pose,
    const float* right_target_pose, float speed_percent);
/* Asynchronous FIFO Cartesian trajectories. Linear interpolates XYZ on a
 * Cartesian line, uses shortest-path quaternion SLERP for orientation, and
 * solves the first path pose only from the current planned joints, then solves
 * every later pose only from the preceding joint solution. Path IK never uses
 * fallback or random seeds. The preceding joint step predicts a preferred
 * posture for the next null-space solve, biasing the result away from +/- IK
 * chatter without rejecting a kinematically required reversal. Only a true
 * per-sample branch jump remains a hard failure. Geometry is sampled at
 * 2 mm / 0.1 rad or better.
 * One global quintic time law generates internal 100 Hz real-time-PV knots.
 * Runtime linearly resamples adjacent knots and transmits the resulting
 * reference on its 500 Hz command clock, without applying the ordinary-PV
 * endpoint step generator.
 * Runtime stretches an undersized duration to satisfy the discrete PV
 * speed/acceleration limits. Physical arrival may be later than the nominal
 * duration. Linear and Circular require PV mode. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_linear_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* end_pose, double duration_s, uint64_t* motion_id);
/* Atomic multi-segment Linear path. poses is pose_count contiguous
 * [x,y,z,roll,pitch,yaw] records. Runtime applies a default 10 mm Cartesian
 * fillet at every valid internal corner, reducing the radius automatically on
 * short adjacent segments. segment_duration_s retains the two-pose duration
 * meaning, so total nominal time is (pose_count - 1) * segment_duration_s. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_linear_path_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* poses,
    uint32_t pose_count, double segment_duration_s, uint64_t* motion_id);
/* ABI 11.1 compatibility entry. point_count is ignored; duration_s owns the
 * planning density. New callers should use move_linear_trajectory(). */
ARTICORE_RUNTIME_API int32_t
articore_runtime_move_linear_trajectory_with_point_count(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* end_pose, double duration_s, uint32_t point_count,
    uint64_t* motion_id);
/* Standard directed circular path through start/via/end. Position is sampled
 * at 2 mm or better, orientation uses shortest-path SLERP through the via
 * orientation, seed-only sequential IK plus joint-step posture prediction
 * preserve the current local branch without random retries, and one global
 * quintic time law produces 100 Hz internal real-time-PV knots, which Runtime
 * linearly resamples on its 500 Hz command clock. Runtime stretches an
 * undersized duration to satisfy joint speed and acceleration. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_circular_trajectory(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, double duration_s,
    uint64_t* motion_id);
/* Unified status and cancellation for joint, Linear, and Circular motions.
 * Terminal states remain queryable by id in the bounded Runtime history.
 * Cancelling a queued id removes only that item and inserts a validated native
 * approach into its immediate successor so the FIFO cannot jump. Cancelling
 * the running id holds the last safe reference and cancels its queue tail
 * because those plans were generated from the running task's final reference.
 */
ARTICORE_RUNTIME_API int32_t articore_runtime_get_motion_status(
    ArticoreRuntime* runtime, uint64_t motion_id,
    ArticoreMotionStatus* status);
ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_motion(
    ArticoreRuntime* runtime, uint64_t motion_id);
ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_all_motions(
    ArticoreRuntime* runtime);

/* Grippers: opening 0..1000, strength 0..10, protected/direct mode.
 * Gripperless products return success without sending a command. */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_grippers(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
    int32_t strength, int32_t mode);
ARTICORE_RUNTIME_API int32_t articore_runtime_has_grippers(
    ArticoreRuntime* runtime, int32_t* has_grippers);

/* Coherent cached product state. These calls perform no CAN I/O. */
ARTICORE_RUNTIME_API int32_t articore_runtime_get_state(
    ArticoreRuntime* runtime, ArticoreProductState* state);
/* Static product metadata. This call performs no CAN I/O and is valid before
 * connect() and independently of the enabled state. */
ARTICORE_RUNTIME_API int32_t articore_runtime_get_joint_angle_vel_limits(
    ArticoreRuntime* runtime, ArticoreProductJointAngleVelLimits* limits);
/* Cached active-TCP pose. The default is tool0 with grippers and link7
 * without grippers. A custom offset affects FK and every Cartesian IK path. */
ARTICORE_RUNTIME_API int32_t articore_runtime_get_pose(
    ArticoreRuntime* runtime, uint32_t side, ArticoreProductPose* pose);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_tcp_offset(
    ArticoreRuntime* runtime, const ArticoreTcpOffset* offset);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_tcp_offset(
    ArticoreRuntime* runtime, uint32_t side, ArticoreTcpOffset* offset);
ARTICORE_RUNTIME_API int32_t articore_runtime_reset_tcp_offset(
    ArticoreRuntime* runtime, uint32_t side);

/* Power operations verify fresh feedback; batch enable rolls back on failure. */
ARTICORE_RUNTIME_API int32_t articore_runtime_enable(ArticoreRuntime* runtime);
/* roles use l/r-joint1..7 and optional l/r-gripper names. */
ARTICORE_RUNTIME_API int32_t articore_runtime_enable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report);
ARTICORE_RUNTIME_API int32_t articore_runtime_disable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report);

/* Runtime-owned MIT gravity compensation; null config uses product defaults. */
ARTICORE_RUNTIME_API int32_t articore_runtime_start_gravity_compensation(
    ArticoreRuntime* runtime,
    const ArticoreGravityCompensationConfig* config);
ARTICORE_RUNTIME_API int32_t articore_runtime_stop_gravity_compensation(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_gravity_compensation_status(
    ArticoreRuntime* runtime,
    ArticoreGravityCompensationStatus* status);

/* Current-mode relative joint-space leader/follower control. Runtime captures
 * both arm positions atomically. Subsequent ordinary PV/MIT commands control
 * the selected leader; the opposite arm follows its seven relative joint
 * displacements in the same complete fourteen-axis product frame. */
ARTICORE_RUNTIME_API int32_t articore_runtime_start_bimanual_follow(
    ArticoreRuntime* runtime, uint32_t leader_side);
ARTICORE_RUNTIME_API int32_t articore_runtime_stop_bimanual_follow(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_bimanual_follow_status(
    ArticoreRuntime* runtime, ArticoreBimanualFollowStatus* status);

/* Safety and diagnostics. disable() never clears a latched fault. */
ARTICORE_RUNTIME_API int32_t articore_runtime_disable(ArticoreRuntime* runtime);
/* Idempotent, latched current-position hold; only recover() clears it. */
ARTICORE_RUNTIME_API int32_t articore_runtime_estop(ArticoreRuntime* runtime);
/* From any live Runtime state: disable, clear recoverable faults, return to
 * calibrated zero, then finish physically disabled. */
ARTICORE_RUNTIME_API int32_t articore_runtime_recover(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_health(
    ArticoreRuntime* runtime, ArticoreSafetyHealth* health);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_mit_torque_limit_stats(
    ArticoreRuntime* runtime, ArticoreMitTorqueLimitStats* stats);
ARTICORE_RUNTIME_API const char* articore_runtime_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
