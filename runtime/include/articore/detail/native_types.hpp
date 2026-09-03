#pragma once

#include <stdint.h>

// Internal Runtime data model. These types stay inside the single C++ service
// process; they are not installed and do not form a binary interface.

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

// Internal trajectory-operation aliases retained by the control state machine.
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
  // Opening feedback is valid only when a real cached sample is fresh.
  int32_t left_gripper_feedback_valid;
  int32_t right_gripper_feedback_valid;
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
