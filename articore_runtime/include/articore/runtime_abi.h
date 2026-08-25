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
 * - Callers set struct_size to sizeof(the structure) before output calls.
 * - Product joint arrays are ordered left J1..J7, then right J1..J7.
 * - Angles use radians, distances use metres, and timestamps use CLOCK_MONOTONIC.
 * - Functions return 0 on success; details for failures are available from
 *   articore_runtime_last_error() and Runtime health.
 */
enum ArticoreRuntimeCapability {
  /* Safety and lifecycle. */
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
  ARTICORE_CAP_PROTECTIVE_FAULT_HOLD = 1ULL << 13,
  ARTICORE_CAP_DETERMINISTIC_DISABLE = 1ULL << 14,
  ARTICORE_CAP_LAYERED_JOINT_LIMITS = 1ULL << 18,

  /* Joint, gripper and model control. */
  ARTICORE_CAP_GRIPPER_COMMAND_PROFILES = 1ULL << 19,
  ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS = 1ULL << 20,
  ARTICORE_CAP_JOINT_MIT_POSITION = 1ULL << 21,
  ARTICORE_CAP_JOINT_PV_POSITION = 1ULL << 22,
  ARTICORE_CAP_BUILTIN_GRIPPER_PRODUCT_PROFILES = 1ULL << 24,

  /* Product connection and diagnostics. */
  ARTICORE_CAP_CONNECT_FEEDBACK_BARRIER = 1ULL << 25,
  ARTICORE_CAP_STRUCTURED_CONNECT_REPORT = 1ULL << 26,
  ARTICORE_CAP_PER_CYCLE_MIT_TORQUE_LIMIT = 1ULL << 28,
  ARTICORE_CAP_NATIVE_ROBOT_MODEL = 1ULL << 29,
  ARTICORE_CAP_NATIVE_GRAVITY_COMPENSATION = 1ULL << 30,
  ARTICORE_CAP_RUNTIME_MAINTENANCE = 1ULL << 31,

  /* Yunyi product Runtime. */
  ARTICORE_CAP_PRODUCT_RUNTIME_FACTORY = 1ULL << 32,
  ARTICORE_CAP_UNIFIED_OPERATION_HEALTH = 1ULL << 33,
  ARTICORE_CAP_PRODUCT_COMMAND_FRAMES = 1ULL << 34,
  ARTICORE_CAP_PRODUCT_STATE = 1ULL << 35,
  ARTICORE_CAP_OPTIONAL_PRODUCT_GRIPPERS = 1ULL << 36,
  ARTICORE_CAP_GRADED_FEEDBACK_SAFETY = 1ULL << 37,
  ARTICORE_CAP_NORMALIZED_ORDINARY_SPEED = 1ULL << 38,
  ARTICORE_CAP_RUNTIME_MOTOR_POWER = 1ULL << 39,
  ARTICORE_CAP_PRODUCT_POSE = 1ULL << 40,
  ARTICORE_CAP_PARAMETERLESS_ESTOP = 1ULL << 41,
  ARTICORE_CAP_PRODUCT_RECOVERY = 1ULL << 42,
  ARTICORE_CAP_TERMINAL_PRODUCT_DISCONNECT = 1ULL << 43,
  ARTICORE_CAP_YUNYI_DUAL_ARM_RUNTIME = 1ULL << 44,
  ARTICORE_CAP_ATOMIC_MOTOR_POWER_BATCH = 1ULL << 45,
  ARTICORE_CAP_PRODUCT_POWER_STATE_SNAPSHOT = 1ULL << 46,

  /* Native trajectory and Cartesian motion. */
  ARTICORE_CAP_PRODUCT_QUINTIC_TRAJECTORY = 1ULL << 47,
  ARTICORE_CAP_PRODUCT_GRIPPER_FORCE_10_LEVELS = 1ULL << 48,
  ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE = 1ULL << 49,
  ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE = 1ULL << 50,
  ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10 = 1ULL << 51,
  ARTICORE_CAP_PRODUCT_CARTESIAN_POINT_TO_POINT = 1ULL << 52,
  ARTICORE_CAP_PRODUCT_CARTESIAN_LINEAR = 1ULL << 53,
  ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR = 1ULL << 54,
  ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR_AUTO_START = 1ULL << 55,

  /* Product state and finalized control semantics. */
  ARTICORE_CAP_PRODUCT_TEMPERATURE_STATE = 1ULL << 56,
  ARTICORE_CAP_LATCHED_ESTOP_POSITION_HOLD = 1ULL << 57,
  ARTICORE_CAP_PRODUCT_JOINT_ANGLE_VEL_LIMITS = 1ULL << 58,
  ARTICORE_CAP_PRODUCT_SPEED_SETTING = 1ULL << 59,
  ARTICORE_CAP_PRODUCT_MAX_SPEED_SETTING = 1ULL << 60,
  ARTICORE_CAP_PRODUCT_TOOL_CENTER_POSE = 1ULL << 61,
  ARTICORE_CAP_PV_MAX_SPEED_ONLY = 1ULL << 62,
  ARTICORE_CAP_DIRECT_CPP_MOTOR_CORE = 1ULL << 63,
};

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
  ARTICORE_DEGRADED = 6,
  ARTICORE_SAFE_STOP = 7,
  ARTICORE_PARTIALLY_ENABLED = 8,
};

enum ArticoreMotorPowerState {
  ARTICORE_MOTOR_POWER_UNKNOWN = 0,
  ARTICORE_MOTOR_POWER_DISABLED = 1,
  ARTICORE_MOTOR_POWER_ENABLED = 2,
  // Returned only for a whole-product query.
  ARTICORE_MOTOR_POWER_MIXED = 3,
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
  ARTICORE_OPERATION_CLOSE = 7,
  ARTICORE_OPERATION_DISCONNECT = 8,
  ARTICORE_OPERATION_COMMAND = 9,
  ARTICORE_OPERATION_RECOVER = 10,
  ARTICORE_OPERATION_START_TRAJECTORY = 11,
  ARTICORE_OPERATION_CANCEL_TRAJECTORY = 12,
  ARTICORE_OPERATION_MOVE_POSE = 13,
  ARTICORE_OPERATION_CANCEL_CARTESIAN_MOTION = 14,
  ARTICORE_OPERATION_MOVE_LINEAR = 15,
  ARTICORE_OPERATION_MOVE_CIRCULAR = 16,
};

enum ArticoreTrajectoryInterpolation {
  ARTICORE_TRAJECTORY_QUINTIC = 1,
};

enum ArticoreTrajectoryState {
  ARTICORE_TRAJECTORY_IDLE = 0,
  ARTICORE_TRAJECTORY_RUNNING = 1,
  // The planned clock has finished and fresh physical feedback has remained
  // within the native product position/velocity arrival window. A progress
  // value of 1.0 while state is still RUNNING means final-setpoint settling,
  // not physical completion.
  ARTICORE_TRAJECTORY_COMPLETED = 2,
  ARTICORE_TRAJECTORY_CANCELLED = 3,
  ARTICORE_TRAJECTORY_FAULT = 4,
  /* Accepted and fully planned, waiting for earlier FIFO motions to finish. */
  ARTICORE_TRAJECTORY_QUEUED = 5,
};

enum ArticoreCartesianInterpolation {
  ARTICORE_CARTESIAN_POINT_TO_POINT = 1,
  ARTICORE_CARTESIAN_LINEAR = 2,
  ARTICORE_CARTESIAN_CIRCULAR = 3,
};

enum { ARTICORE_MAX_TRAJECTORY_WAYPOINTS = 10000 };

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

enum ArticoreGravityCompensationPhase {
  ARTICORE_GRAVITY_INACTIVE = 0,
  ARTICORE_GRAVITY_ENTERING = 1,
  ARTICORE_GRAVITY_ACTIVE = 2,
  ARTICORE_GRAVITY_EXITING = 3,
};

enum { ARTICORE_MAX_MIT_TORQUE_LIMIT_JOINTS = 32 };

typedef struct ArticoreGravityProductBinding {
  uint32_t struct_size;
  // Runtime transport side whose seven arm joints use this model.
  uint32_t runtime_side;
  // Physical product side: ARTICORE_ROBOT_LEFT or ARTICORE_ROBOT_RIGHT.
  uint32_t robot_side;
  char product_id[64];
} ArticoreGravityProductBinding;

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
  uint32_t joint_count;
  void* joints[ARTICORE_MAX_MIT_TORQUE_LIMIT_JOINTS];
  // Final motor-domain gravity feedforward from the latest sent cycle.
  float gravity_feedforward_torque[ARTICORE_MAX_MIT_TORQUE_LIMIT_JOINTS];
} ArticoreGravityCompensationStatus;

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

enum ArticoreGripperMode {
  // Default behavior: detect sustained contact/stall, switch to calibrated
  // holding gains, and retreat after sustained overload.
  ARTICORE_GRIPPER_MODE_PROTECTED = 0,
  // Direct behavior: continue tracking the requested opening with the
  // selected strength and do not run contact/stall or overload-retreat logic.
  ARTICORE_GRIPPER_MODE_DIRECT = 1,
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
  // Convenience aliases; the public scale remains 1..10.
  ARTICORE_GRIPPER_FORCE_LOW = ARTICORE_GRIPPER_FORCE_MIN,
  ARTICORE_GRIPPER_FORCE_NORMAL = ARTICORE_GRIPPER_FORCE_DEFAULT,
  ARTICORE_GRIPPER_FORCE_HIGH = ARTICORE_GRIPPER_FORCE_MAX,
};

enum {
  ARTICORE_GRIPPER_STRENGTH_MIN = 0,
  ARTICORE_GRIPPER_STRENGTH_MAX = 10,
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

typedef struct ArticoreTrajectoryWaypoint {
  uint32_t struct_size;
  double time_s;
  float left_positions[ARTICORE_PRODUCT_ARM_DOF];
  float right_positions[ARTICORE_PRODUCT_ARM_DOF];
  float left_velocities[ARTICORE_PRODUCT_ARM_DOF];
  float right_velocities[ARTICORE_PRODUCT_ARM_DOF];
  float left_accelerations[ARTICORE_PRODUCT_ARM_DOF];
  float right_accelerations[ARTICORE_PRODUCT_ARM_DOF];
  // Bits 0..6 describe the left arm and bits 7..13 the right arm. Missing
  // endpoint derivatives default to zero. Missing intermediate derivatives
  // are computed once by the native planner and shared by adjacent segments.
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
  float pv_velocity_limits[ARTICORE_PRODUCT_DUAL_ARM_DOF];
} ArticoreTrajectoryConfig;

typedef struct ArticoreTrajectoryStatus {
  uint32_t struct_size;
  int32_t state;
  uint64_t trajectory_id;
  uint32_t active_segment;
  uint32_t waypoint_count;
  double elapsed_s;
  double duration_s;
  float progress;
  char error[512];
} ArticoreTrajectoryStatus;

typedef struct ArticoreCartesianMotionStatus {
  uint32_t struct_size;
  int32_t state;
  uint64_t motion_id;
  uint64_t superseded_motion_id;
  uint32_t side;
  int32_t interpolation;
  float speed_percent;
  double elapsed_s;
  double duration_s;
  float progress;
  float target_pose[ARTICORE_PRODUCT_POSE_DOF];
  char error[512];
} ArticoreCartesianMotionStatus;

typedef struct ArticoreProductArmState {
  float positions[ARTICORE_PRODUCT_ARM_DOF];
  float velocities[ARTICORE_PRODUCT_ARM_DOF];
  float torques[ARTICORE_PRODUCT_ARM_DOF];
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
  // CLOCK_MONOTONIC-compatible timestamp of the oldest feedback represented
  // by this complete snapshot. sequence advances with the slowest installed
  // Motor, so it cannot hide one frozen side behind updates from the other.
  uint64_t timestamp_ns;
  uint64_t sequence;
} ArticoreProductState;

typedef struct ArticoreProductArmStateV2 {
  float positions[ARTICORE_PRODUCT_ARM_DOF];
  float velocities[ARTICORE_PRODUCT_ARM_DOF];
  float torques[ARTICORE_PRODUCT_ARM_DOF];
  // Bit i is set only when fresh cached feedback reports status_code == 1.
  uint32_t enabled_mask;
  // Bit i is set when fresh cached feedback reports a recognized enabled or
  // disabled status. A clear bit means the power state is unknown.
  uint32_t enabled_valid_mask;
} ArticoreProductArmStateV2;

typedef struct ArticoreProductStateV2 {
  uint32_t struct_size;
  int32_t has_grippers;
  ArticoreProductArmStateV2 left;
  ArticoreProductArmStateV2 right;
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
  // CLOCK_MONOTONIC-compatible timestamp and slowest update sequence across
  // every installed motor represented by this single cached snapshot.
  uint64_t timestamp_ns;
  uint64_t sequence;
} ArticoreProductStateV2;

typedef struct ArticoreProductArmStateV3 {
  float positions[ARTICORE_PRODUCT_ARM_DOF];
  float velocities[ARTICORE_PRODUCT_ARM_DOF];
  float torques[ARTICORE_PRODUCT_ARM_DOF];
  float mos_temperatures[ARTICORE_PRODUCT_ARM_DOF];
  float rotor_temperatures[ARTICORE_PRODUCT_ARM_DOF];
  uint32_t enabled_mask;
  uint32_t enabled_valid_mask;
  // Bit i is set when both temperatures come from finite, fresh cached
  // feedback. A clear bit means the corresponding values are NaN.
  uint32_t temperature_valid_mask;
} ArticoreProductArmStateV3;

typedef struct ArticoreProductStateV3 {
  uint32_t struct_size;
  int32_t has_grippers;
  ArticoreProductArmStateV3 left;
  ArticoreProductArmStateV3 right;
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
} ArticoreProductStateV3;

typedef struct ArticoreProductJointAngleVelLimits {
  uint32_t struct_size;
  uint32_t joint_count;
  // Logical product coordinates in radians and radians/second.
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
  // Product-independent opening scale: 0=closed, 1000=open.
  float opening;
  // Product-independent speed scale: (0, 1000], where 1000 is the calibrated
  // maximum speed in the motor descriptor.
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
  uint32_t struct_size;
  uint32_t timeout_ms;
  uint32_t expected_count;
  uint32_t received_count;
  uint32_t missing_count;
} ArticoreFeedbackReport;

typedef struct ArticoreMotorIdentity {
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

typedef struct ArticoreRuntimeTransportCapabilities {
  uint32_t struct_size;
  uint32_t side;
  int32_t can_fd;
  int32_t can_fd_brs;
  char transport[32];
} ArticoreRuntimeTransportCapabilities;

typedef struct ArticoreRuntimeConfig {
  // Reserved; Runtime scheduling is internal.
  uint32_t reserved_control_rate;
  uint32_t command_timeout_ms;
  uint32_t enable_grace_ms;
  uint32_t safe_hold_hz;
  uint32_t feedback_check_hz;
  uint32_t feedback_failure_threshold;
  uint32_t feedback_max_age_ms;
  uint32_t safe_hold_failure_threshold;
  uint32_t disable_feedback_timeout_ms;
  float safe_pv_velocity_limit;
  // Reserved; gripper scheduling is internal.
  uint32_t reserved_gripper_control_rate;
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

typedef struct ArticoreSafetyHealthV2 {
  uint32_t struct_size;
  ArticoreSafetyHealth health;
  // One generic diagnostic surface for every Runtime lifecycle or maintenance
  // operation. Native code may retain richer private transaction data, but
  // language SDKs should expose these fields instead of adding per-operation
  // report APIs.
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
} ArticoreSafetyHealthV2;

typedef struct ArticoreMitTorqueLimitStats {
  uint32_t struct_size;
  // Number of successfully transmitted native control cycles in which at
  // least one arm joint required limiting.
  uint64_t torque_limit_activation_count;
  // Bit i describes joints[i] from the most recently transmitted MIT arm
  // batch. A set bit means that joint was limited in that cycle.
  uint64_t torque_limited_joint_mask;
  uint32_t joint_count;
  void* joints[ARTICORE_MAX_MIT_TORQUE_LIMIT_JOINTS];
  // Values use the motor command/feedback torque domain used by the native
  // MIT protocol after any product range mapping performed by the caller.
  float requested_resultant_torque[ARTICORE_MAX_MIT_TORQUE_LIMIT_JOINTS];
  float applied_scale[ARTICORE_MAX_MIT_TORQUE_LIMIT_JOINTS];
  float applied_resultant_torque[ARTICORE_MAX_MIT_TORQUE_LIMIT_JOINTS];
} ArticoreMitTorqueLimitStats;

/* Version and robot model. ABI version is packed as 0xMMMMmmmm. */
ARTICORE_RUNTIME_API uint32_t articore_runtime_abi_version(void);
ARTICORE_RUNTIME_API uint64_t articore_runtime_capabilities(void);

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

/* Pre-connect only; identities must cover every installed Motor. */
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_motor_identities(
    ArticoreRuntime* runtime,
    const ArticoreMotorIdentity* identities,
    uint32_t identity_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_connect(ArticoreRuntime* runtime);
/* Idempotent safe shutdown; call free() afterward to release the opaque handle. */
ARTICORE_RUNTIME_API int32_t articore_runtime_disconnect(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_mode(
    ArticoreRuntime* runtime, int32_t mode);
ARTICORE_RUNTIME_API int32_t articore_runtime_clear_faults(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_zero(
    ArticoreRuntime* runtime);

/* Product joint commands. Prefer set_max_speed() plus set_joint_positions_v2(). */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_positions(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
    float speed_percent);
/* Compatibility names for the persistent ordinary-motion speed percentage. */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_speed(
    ArticoreRuntime* runtime, float speed_percent);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_speed(
    ArticoreRuntime* runtime, float* speed_percent);
/* PV reference speed: 0..100 maps linearly to 0..2 rad/s; default 50. */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_max_speed(
    ArticoreRuntime* runtime, float max_speed_percent);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_max_speed(
    ArticoreRuntime* runtime, float* max_speed_percent);
/* Canonical ordinary position command using the persistent maximum speed. */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_positions_v2(
    ArticoreRuntime* runtime, const float* positions, uint32_t count);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocities, const float* feedforward_torques,
    const float* kp, const float* kd, uint32_t count);
/* Low-level PV compatibility entry point; product SDKs should not expose it. */
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pv_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocity_limits, uint32_t count);

/* Asynchronous dual-arm quintic trajectory. Inputs are copied before return. */
ARTICORE_RUNTIME_API int32_t articore_runtime_start_trajectory(
    ArticoreRuntime* runtime,
    const ArticoreTrajectoryWaypoint* waypoints,
    uint32_t waypoint_count,
    const ArticoreTrajectoryConfig* config);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_trajectory_status(
    ArticoreRuntime* runtime, ArticoreTrajectoryStatus* status);
ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_trajectory(
    ArticoreRuntime* runtime);

/* Point-to-point command. Runtime performs endpoint IK, then uses the ordinary
 * 500 Hz PV reference step. It has no motion ID, status, or cancellation API. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_pose(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent);
/* Asynchronous FIFO Cartesian trajectories. New linear and circular plans
 * are appended after the current queue tail and never replace it.
 * Linear orientation uses shortest-path quaternion SLERP. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_linear(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent, uint64_t* motion_id);
/* Standard linear path with an explicit, validated geometric start pose. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_linear_v2(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* end_pose, float speed_percent, uint64_t* motion_id);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_cartesian_motion_status(
    ArticoreRuntime* runtime, ArticoreCartesianMotionStatus* status);
/* Query one returned motion ID; unlike the legacy getter, this is independent
 * of later FIFO submissions. */
ARTICORE_RUNTIME_API int32_t articore_runtime_get_cartesian_motion_status_v2(
    ArticoreRuntime* runtime, uint64_t motion_id,
    ArticoreCartesianMotionStatus* status);
ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_cartesian_motion(
    ArticoreRuntime* runtime);
/* Standard circular path with an explicit, validated geometric start pose. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_circular(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, float speed_percent,
    uint64_t* motion_id);
/* Compatibility auto-start circular motion. */
ARTICORE_RUNTIME_API int32_t articore_runtime_move_circular_v2(
    ArticoreRuntime* runtime, uint32_t side, const float* via_pose,
    const float* end_pose, float speed_percent, uint64_t* motion_id);

/* Grippers: opening 0..1000, force 1..10; gripperless products return success. */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_grippers(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
    int32_t gripper_level);
/* v2 accepts strength 0..10 and protected/direct gripper modes. */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_grippers_v2(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
    int32_t strength, int32_t mode);
ARTICORE_RUNTIME_API int32_t articore_runtime_has_grippers(
    ArticoreRuntime* runtime, int32_t* has_grippers);

/* Coherent cached product state. These calls perform no CAN I/O. */
ARTICORE_RUNTIME_API int32_t articore_runtime_get_state(
    ArticoreRuntime* runtime, ArticoreProductState* state);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_state_v2(
    ArticoreRuntime* runtime, ArticoreProductStateV2* state);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_state_v3(
    ArticoreRuntime* runtime, ArticoreProductStateV3* state);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_joint_angle_vel_limits(
    ArticoreRuntime* runtime, ArticoreProductJointAngleVelLimits* limits);
/* Cached tool0 pose with grippers, otherwise link7 pose. */
ARTICORE_RUNTIME_API int32_t articore_runtime_get_pose(
    ArticoreRuntime* runtime, uint32_t side, ArticoreProductPose* pose);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_connect_report(
    ArticoreRuntime* runtime, ArticoreConnectReport* report);

/* Power operations verify fresh feedback; batch enable rolls back on failure. */
ARTICORE_RUNTIME_API int32_t articore_runtime_enable(
    ArticoreRuntime* runtime, int32_t mode);
/* roles use l/r-joint1..7 and optional l/r-gripper names. */
ARTICORE_RUNTIME_API int32_t articore_runtime_enable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report);
ARTICORE_RUNTIME_API int32_t articore_runtime_disable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report);
/* Compatibility single-Motor power API; null/empty name selects all Motors. */
ARTICORE_RUNTIME_API int32_t articore_runtime_set_motor_power(
    ArticoreRuntime* runtime, const char* motor_name, int32_t enabled,
    int32_t* confirmed_state);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_motor_power(
    ArticoreRuntime* runtime, const char* motor_name, int32_t* state);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_enable_report(
    ArticoreRuntime* runtime, ArticoreEnableReport* report);

/* Pre-connect compatibility configuration. Fixed Yunyi products configure this internally. */
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_joints(
    ArticoreRuntime* runtime,
    const ArticoreJointControlConfig* configs,
    uint32_t config_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_joint_safety_limits(
    ArticoreRuntime* runtime,
    const ArticoreJointSafetyLimits* limits,
    uint32_t limit_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gravity_products(
    ArticoreRuntime* runtime,
    const ArticoreGravityProductBinding* bindings,
    uint32_t binding_count);

/* Runtime-owned MIT gravity compensation; null config uses product defaults. */
ARTICORE_RUNTIME_API int32_t articore_runtime_start_gravity_compensation(
    ArticoreRuntime* runtime,
    const ArticoreGravityCompensationConfig* config);
ARTICORE_RUNTIME_API int32_t articore_runtime_stop_gravity_compensation(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_gravity_compensation_status(
    ArticoreRuntime* runtime,
    ArticoreGravityCompensationStatus* status);

/* Low-level compatibility commands. Accepted frames replace a one-slot mailbox. */
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pos_vel(
    ArticoreRuntime* runtime,
    const ArticorePosVelCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count);
/* Persistent MIT requires zero target velocity and feedforward torque. */
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
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_pv(
    ArticoreRuntime* runtime,
    const ArticoreJointPvTarget* targets,
    uint32_t target_count,
    float speed_percent);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_mit(
    ArticoreRuntime* runtime,
    const ArticoreJointMitTarget* targets,
    uint32_t target_count,
    float speed_percent);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_gripper_mit(
    ArticoreRuntime* runtime,
    const ArticoreMitCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_gripper_openings(
    ArticoreRuntime* runtime,
    const ArticoreGripperTarget* targets,
    uint32_t target_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gripper_products(
    ArticoreRuntime* runtime,
    const ArticoreGripperProductBinding* bindings,
    uint32_t binding_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gripper_force_profiles(
    ArticoreRuntime* runtime,
    const ArticoreGripperForceProfile* profiles,
    uint32_t profile_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_gripper_commands(
    ArticoreRuntime* runtime,
    const ArticoreGripperCommand* commands,
    uint32_t command_count);
ARTICORE_RUNTIME_API int32_t articore_runtime_report_feedback_failure(
    ArticoreRuntime* runtime, uint8_t side, const char* reason);

/* Safety and diagnostics. disable() never clears a latched fault. */
ARTICORE_RUNTIME_API int32_t articore_runtime_disable(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_disable_report(
    ArticoreRuntime* runtime, ArticoreDisableReport* report);
/* Idempotent, latched current-position hold; only recover() clears it. */
ARTICORE_RUNTIME_API int32_t articore_runtime_estop(ArticoreRuntime* runtime);
/* From any live Runtime state: disable, clear recoverable faults, return to
 * calibrated zero, then finish physically disabled. */
ARTICORE_RUNTIME_API int32_t articore_runtime_recover(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_health(
    ArticoreRuntime* runtime, ArticoreSafetyHealth* health);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_health_v2(
    ArticoreRuntime* runtime, ArticoreSafetyHealthV2* health);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_mit_torque_limit_stats(
    ArticoreRuntime* runtime, ArticoreMitTorqueLimitStats* stats);
/* Presence declarations are valid only while disconnected. */
ARTICORE_RUNTIME_API int32_t articore_runtime_declare_motor_presence(
    ArticoreRuntime* runtime, const char* motor_role, int32_t state);
ARTICORE_RUNTIME_API int32_t articore_runtime_motor_presence(
    ArticoreRuntime* runtime, const char* motor_role, int32_t* state);
ARTICORE_RUNTIME_API uint64_t articore_runtime_active_capabilities(
    ArticoreRuntime* runtime);
/* Compatibility alias for disconnect(). */
ARTICORE_RUNTIME_API int32_t articore_runtime_close(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API const char* articore_runtime_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
