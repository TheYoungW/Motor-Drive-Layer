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
typedef struct ArticoreRobotModel ArticoreRobotModel;

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
  // explicit disable or emergency stop requests torque-off.
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
  // ABI 2.6 recomputes complete raw-MIT P + D + feedforward output from the
  // newest native feedback on every actual control tick, including ticks
  // that repeat the latest mailbox target. Per-joint output is bounded to
  // its configured torque limit by scaling Kp, Kd and feedforward together.
  ARTICORE_CAP_PER_CYCLE_MIT_TORQUE_LIMIT = 1ULL << 28,
  // ABI 2.7 exposes product-owned whole-arm kinematics and rigid-body
  // dynamics through a Pinocchio-independent C ABI. Exact model parameters
  // and the numerical implementation remain private to the Runtime library.
  ARTICORE_CAP_NATIVE_ROBOT_MODEL = 1ULL << 29,
  // ABI 2.8 adds a Runtime-owned fixed-rate gravity-compensation controller.
  // It exclusively owns arm MIT output while active and provides a manual
  // hand-guiding mode with zero stiffness and product gravity feedforward.
  ARTICORE_CAP_NATIVE_GRAVITY_COMPENSATION = 1ULL << 30,
  // ABI 2.9 adds owner-safe whole-runtime mode configuration, fault clear,
  // and product-profile zeroing transactions. They reuse the Runtime's
  // existing Motor lease and publish one unified operation result in health.
  ARTICORE_CAP_RUNTIME_MAINTENANCE = 1ULL << 31,
  // ABI 2.9 can construct the complete yunyi_v1_0 dual-arm product, including
  // transports, controllers, group, motors, mappings, and their lifetimes.
  ARTICORE_CAP_PRODUCT_RUNTIME_FACTORY = 1ULL << 32,
  ARTICORE_CAP_UNIFIED_OPERATION_HEALTH = 1ULL << 33,
  // ABI 2.10 exposes fixed-layout whole-product command frames. Callers no
  // longer resolve or pass Motor handles for normal dual-arm control.
  ARTICORE_CAP_PRODUCT_COMMAND_FRAMES = 1ULL << 34,
  // ABI 2.10 exposes one coherent logical-coordinate product state and a
  // reconnectable disconnect operation while the Runtime object remains alive.
  ARTICORE_CAP_PRODUCT_STATE = 1ULL << 35,
  // ABI 2.11 lets a built-in product Runtime select its installed gripper
  // topology at creation. A gripperless product owns only the 14 arm Motors;
  // whole-product gripper commands are then successful no-ops.
  ARTICORE_CAP_OPTIONAL_PRODUCT_GRIPPERS = 1ULL << 36,
  // ABI 2.12 separates communication quality from confirmed hardware faults.
  // Brief feedback jitter is tolerated, sustained delay is derated, and a
  // longer outage enters a recoverable position-holding SAFE_STOP. FAULT is
  // reserved for confirmed motor/transport faults and never changes a motor's
  // protocol mode by itself.
  ARTICORE_CAP_GRADED_FEEDBACK_SAFETY = 1ULL << 37,
  // ABI 2.13 expresses the ordinary MIT/PV position-control pace as an
  // intuitive 0..100 product percentage. Raw MIT/PV frames keep physical
  // velocity units.
  ARTICORE_CAP_NORMALIZED_ORDINARY_SPEED = 1ULL << 38,
  // ABI 2.14 adds Runtime-owned whole-product and single-motor power control.
  // Every operation confirms the resulting state from fresh feedback; a
  // Runtime with only some motors enabled is explicitly non-commandable.
  ARTICORE_CAP_RUNTIME_MOTOR_POWER = 1ULL << 39,
  // ABI 2.15 exposes a coherent product-arm pose computed in C++ from the
  // latest native feedback cache. ABI 2.37 defines its active tool/link frame.
  ARTICORE_CAP_PRODUCT_POSE = 1ULL << 40,
  // ABI 2.16 makes emergency stop a parameterless Runtime-owned transaction.
  // It records a standard health reason, is idempotent, and can only be
  // unlatched by recover(). ABI 2.33 defines the current hold behavior.
  ARTICORE_CAP_PARAMETERLESS_ESTOP = 1ULL << 41,
  // ABI 2.17 defines recover() as a native whole-product transaction: clear
  // recoverable faults, validate both arms, return every arm joint to its
  // calibrated zero at fixed low speed, and finish physically disabled.
  ARTICORE_CAP_PRODUCT_RECOVERY = 1ULL << 42,
  // ABI 2.19 makes product disconnect a terminal, idempotent safety shutdown
  // that stops the worker and releases all product-owned native resources.
  ARTICORE_CAP_TERMINAL_PRODUCT_DISCONNECT = 1ULL << 43,
  // ABI 2.20 defines the supported product surface as the fixed Yunyi
  // dual-arm Runtime. Product clients no longer submit product identifiers,
  // Controllers, Motor handles, mappings, profiles, or scheduler settings.
  ARTICORE_CAP_YUNYI_DUAL_ARM_RUNTIME = 1ULL << 44,
  // ABI 2.21 adds atomic product-level subset power transactions. A failed
  // subset enable rolls back motors changed by that call, subset disable is
  // available from abnormal states, and PARTIALLY_ENABLED control filters
  // intentionally disabled motors from otherwise complete product frames.
  ARTICORE_CAP_ATOMIC_MOTOR_POWER_BATCH = 1ULL << 45,
  // ABI 2.22 returns feedback-confirmed per-motor power state in one cached
  // product snapshot, without issuing bus reads or requiring 14 SDK calls.
  ARTICORE_CAP_PRODUCT_POWER_STATE_SNAPSHOT = 1ULL << 46,
  // ABI 2.23 adds one native, product-level dual-arm trajectory transaction.
  // Waypoints are copied at submission, quintic segments are precomputed and
  // validated in C++, and the existing native worker evaluates and dispatches
  // complete Raw MIT/PV frames at its private control rate.
  ARTICORE_CAP_PRODUCT_QUINTIC_TRAJECTORY = 1ULL << 47,
  // ABI 2.24 makes the public Yunyi whole-product gripper selector a direct
  // 1..10 scale. Unlike ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS, this bit is
  // specific to articore_runtime_set_grippers() and therefore lets SDKs
  // reject older product runtimes that still expose only the compressed
  // 1..5 selector.
  ARTICORE_CAP_PRODUCT_GRIPPER_FORCE_10_LEVELS = 1ULL << 48,
  // ABI 2.25 adds a product gripper command mode that bypasses contact/stall
  // detection and overload retreat while retaining Runtime feedback, motor
  // fault, transport, estop, and disconnect safety. The v2 command accepts
  // strength 0..10; zero produces no active gripper stiffness.
  ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE = 1ULL << 49,
  // ABI 2.26 guarantees that the selected product mode applies only to arm
  // joints. Installed Yunyi grippers are configured and commanded as MIT in
  // both PV-arm and MIT-arm products.
  ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE = 1ULL << 50,
  // ABI 2.27 applies a fixed 10x multiplier to both Kp and Kd in product
  // DIRECT gripper mode. PROTECTED calibration and strength zero are unchanged.
  ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10 = 1ULL << 51,
  // ABI 2.28 adds native, non-blocking PV Cartesian point-to-point motion. A
  // validated newer pose atomically supersedes the current point target and
  // is replanned from the active quintic state by the private 500 Hz worker.
  ARTICORE_CAP_PRODUCT_CARTESIAN_POINT_TO_POINT = 1ULL << 52,
  // ABI 2.29 adds PV Cartesian-linear translation with shortest-path quaternion
  // SLERP orientation. The path is prevalidated by sequential native IK and
  // executed from precomputed joint polynomials; no IK runs in the 500 Hz loop.
  ARTICORE_CAP_PRODUCT_CARTESIAN_LINEAR = 1ULL << 53,
  // ABI 2.30 adds a PV-only three-pose circular arc. XYZ start/via/end define the
  // unique traversed arc; orientation passes through all three poses using
  // piecewise shortest-path quaternion SLERP.
  ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR = 1ULL << 54,
  // ABI 2.31 removes the public circular start pose. Runtime snapshots the
  // current planned joint reference and installs the replacement under one
  // native command transaction, avoiding feedback lag and SDK round trips.
  ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR_AUTO_START = 1ULL << 55,
  // ABI 2.32 exposes MOS and rotor temperature for every installed product
  // Motor in the coherent cached state snapshot. State reads never issue CAN
  // requests; freshness is reported explicitly for each arm joint/gripper.
  ARTICORE_CAP_PRODUCT_TEMPERATURE_STATE = 1ULL << 56,
  // ABI 2.33 defines product estop as a latched current-position hold. It
  // atomically supersedes user motion, keeps enabled Motors enabled, and
  // continuously transmits native PV/MIT safety holds until recover().
  ARTICORE_CAP_LATCHED_ESTOP_POSITION_HOLD = 1ULL << 57,
  // ABI 2.34 returns the fixed logical-coordinate angle and velocity limits
  // for all 14 Yunyi arm joints in one product snapshot. Grippers are omitted.
  ARTICORE_CAP_PRODUCT_JOINT_ANGLE_VEL_LIMITS = 1ULL << 58,
  // ABI 2.35 adds one persistent 0..100 ordinary-motion speed setting. The
  // default is 70 and 100 maps to a shared 5 rad/s cap for all 14 arm joints.
  // Updating it also changes an active ordinary MIT/PV position reference.
  ARTICORE_CAP_PRODUCT_SPEED_SETTING = 1ULL << 59,
  // ABI 2.36 gives the setting its precise public meaning and preferred API
  // name: it is the maximum ordinary-motion speed percentage, not an
  // instantaneous target velocity. ABI 2.35 symbols remain compatibility
  // aliases.
  ARTICORE_CAP_PRODUCT_MAX_SPEED_SETTING = 1ULL << 60,
  // ABI 2.37 defines the single public product pose as the active physical
  // control point. A Yunyi Runtime with grippers uses the fixed l/r-tool0
  // gripper-center frame; a gripperless Runtime uses l/r-link7. FK, IK, PTP,
  // linear and circular motion all share this exact frame selection.
  ARTICORE_CAP_PRODUCT_TOOL_CENTER_POSE = 1ULL << 61,
  // ABI 2.38 defines one public ordinary-PV path: set_max_speed(0..100)
  // configures the persistent product-owned step limit and the position
  // command carries positions only. Explicit-speed and raw-PV C symbols stay
  // exported solely for binary compatibility and must not be exposed by new
  // product SDKs. MIT ordinary and raw command semantics are unchanged.
  ARTICORE_CAP_PV_MAX_SPEED_ONLY = 1ULL << 62,
  // ABI 2.39 removes the caller-assembled generic Runtime factory. The fixed
  // Yunyi factory directly owns a C++ MotorBackend and SocketCAN-FD resources;
  // no Motor C ABI or second shared library exists in the product path.
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
  // Caller initializes this to sizeof(ArticoreRobotModelInfo).
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
  // Caller initializes this to sizeof(ArticoreRobotPose).
  uint32_t struct_size;
  double position[3];
  // Row-major 3x3 rotation matrix.
  double rotation[9];
  // Row-major 4x4 homogeneous transform.
  double homogeneous[16];
} ArticoreRobotPose;

typedef struct ArticoreIkOptions {
  // Caller initializes this to sizeof(ArticoreIkOptions). A zero-valued field
  // selects the documented Runtime default for that field.
  uint32_t struct_size;
  uint32_t max_iterations;
  uint32_t max_retries;
  double tolerance;
  double step_size;
  double damping;
  uint64_t random_seed;
} ArticoreIkOptions;

typedef struct ArticoreIkResult {
  // Caller initializes this to sizeof(ArticoreIkResult).
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
  ARTICORE_OPERATION_CANCEL_MOVE_POSE = 14,
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
};

enum ArticoreCartesianInterpolation {
  ARTICORE_CARTESIAN_POINT_TO_POINT = 1,
  ARTICORE_CARTESIAN_LINEAR = 2,
  ARTICORE_CARTESIAN_CIRCULAR = 3,
};

enum { ARTICORE_MAX_TRAJECTORY_WAYPOINTS = 10000 };

// Stable result codes returned by the ABI 2.9 maintenance entry points.
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
  // Caller initializes this to sizeof(ArticoreGravityProductBinding).
  uint32_t struct_size;
  // Runtime transport side whose seven arm joints use this model.
  uint32_t runtime_side;
  // Physical product side: ARTICORE_ROBOT_LEFT or ARTICORE_ROBOT_RIGHT.
  uint32_t robot_side;
  char product_id[64];
} ArticoreGravityProductBinding;

typedef struct ArticoreGravityCompensationConfig {
  // Caller initializes this to sizeof(ArticoreGravityCompensationConfig).
  uint32_t struct_size;
  // Zero selects the product default of 500 ms.
  uint32_t transition_ms;
} ArticoreGravityCompensationConfig;

typedef struct ArticoreGravityCompensationStatus {
  // Caller initializes this to sizeof(ArticoreGravityCompensationStatus).
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
  // Source-level convenience aliases. New bindings should expose 1..10.
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
  // Caller initializes this to sizeof(ArticoreTrajectoryWaypoint). Runtime
  // copies every waypoint before returning from start_trajectory().
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
  // Caller initializes this to sizeof(ArticoreTrajectoryConfig).
  uint32_t struct_size;
  int32_t interpolation;
  int32_t control_mode;
  float mit_kp[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float mit_kd[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float mit_feedforward_torque[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float pv_velocity_limits[ARTICORE_PRODUCT_DUAL_ARM_DOF];
} ArticoreTrajectoryConfig;

typedef struct ArticoreTrajectoryStatus {
  // Caller initializes this to sizeof(ArticoreTrajectoryStatus).
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

typedef struct ArticoreMovePoseStatus {
  // Caller initializes this to sizeof(ArticoreMovePoseStatus). state uses
  // ArticoreTrajectoryState because point-to-point motion is executed by the
  // same native quintic engine.
  uint32_t struct_size;
  int32_t state;
  uint64_t motion_id;
  // Non-zero after a newer valid pose replaced a running point target.
  uint64_t superseded_motion_id;
  uint32_t side;
  float speed_percent;
  double elapsed_s;
  double duration_s;
  float progress;
  float target_pose[ARTICORE_PRODUCT_POSE_DOF];
  char error[512];
} ArticoreMovePoseStatus;

typedef struct ArticoreCartesianMotionStatus {
  // Caller initializes this to sizeof(ArticoreCartesianMotionStatus).
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
  // Caller initializes this to sizeof(ArticoreProductState).
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
  // Caller initializes this to sizeof(ArticoreProductStateV2).
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
  // Caller initializes this to sizeof(ArticoreProductStateV3).
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
  // Caller initializes this to sizeof(ArticoreProductJointAngleVelLimits).
  // Array order is left joint1..7, then right joint1..7.
  uint32_t struct_size;
  uint32_t joint_count;
  // Logical product coordinates in radians and radians/second.
  float lower_angles[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float upper_angles[ARTICORE_PRODUCT_DUAL_ARM_DOF];
  float velocity_limits[ARTICORE_PRODUCT_DUAL_ARM_DOF];
} ArticoreProductJointAngleVelLimits;

typedef struct ArticoreProductPose {
  // Caller initializes this to sizeof(ArticoreProductPose).
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
  // Caller initializes this to sizeof(ArticoreMotorPowerReport).
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
  // Caller initializes this to sizeof(ArticoreRuntimeTransportCapabilities).
  uint32_t struct_size;
  uint32_t side;
  int32_t can_fd;
  int32_t can_fd_brs;
  char transport[32];
} ArticoreRuntimeTransportCapabilities;

typedef struct ArticoreRuntimeConfig {
  // ABI layout placeholder. Runtime scheduling is selected internally and
  // this caller-provided value is ignored.
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
  // ABI layout placeholder. Normal gripper scheduling is internal; safe
  // holding continues to use the independent safety cadence above.
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
  // Caller initializes this to sizeof(ArticoreSafetyHealthV2). The embedded
  // ABI 1.x health layout remains byte-for-byte stable for legacy callers.
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
  // ABI 2.12 graded-safety diagnostics. fault_reason remains reserved for a
  // confirmed ARTICORE_FAULT; transient communication quality is described
  // here instead.
  int32_t degraded;
  int32_t safe_stopped;
  int32_t requires_resynchronization;
  float command_scale;
  char safety_reason[512];
} ArticoreSafetyHealthV2;

typedef struct ArticoreMitTorqueLimitStats {
  // Caller initializes this to sizeof(ArticoreMitTorqueLimitStats).
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

// Packed as 0xMMMMmmmm: major in the high 16 bits, minor in the low 16 bits.
ARTICORE_RUNTIME_API uint32_t articore_runtime_abi_version(void);
ARTICORE_RUNTIME_API uint64_t articore_runtime_capabilities(void);

// Product-owned whole-arm model API. No Pinocchio or Eigen types cross this
// boundary. product_id currently accepts "yunyi_v1_0"; side selects the
// corresponding reduced seven-axis arm model. Returned matrix storage is
// row-major and callers provide exact element counts.
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

// ABI 2.20 fixed Yunyi dual-arm factory. It owns can-left/can-right,
// SocketCAN-FD+BRS, both Controllers, the ControllerGroup, and every product
// mapping. with_grippers=1 creates 16 Motors and requires both grippers;
// with_grippers=0 creates only the 14 arm Motors. The returned Runtime starts
// DISCONNECTED; connect() performs the normal complete feedback barrier.
ARTICORE_RUNTIME_API ArticoreRuntime* articore_runtime_create_yunyi(
    int32_t mode, int32_t with_grippers);
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
// ABI 2.19 terminal product shutdown. For a product-owned Runtime this stops
// command production, disables and verifies every installed motor, joins the
// worker, closes both CAN channels, and releases Controllers, ControllerGroup,
// Motors, models, and product resources. The opaque allocation remains as an
// idempotent tombstone until articore_runtime_free(); language bindings should
// free it automatically after success.
ARTICORE_RUNTIME_API int32_t articore_runtime_disconnect(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_mode(
    ArticoreRuntime* runtime, int32_t mode);
ARTICORE_RUNTIME_API int32_t articore_runtime_clear_faults(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_set_zero(
    ArticoreRuntime* runtime);
// ABI 2.10 compatibility entry point. All arrays use fixed product
// order: left joint1..7, then right joint1..7. Direction conversion, product
// limits, Motor mapping, and per-Motor command construction remain native.
// speed_percent is inclusive 0..100. 0 pauses reference advancement and 100
// selects the product's maximum ordinary speed for the active control mode.
// ABI 2.38 product bindings must use set_max_speed() plus the v2 position
// command instead of exposing this per-call speed parameter.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_positions(
    ArticoreRuntime* runtime, const float* positions, uint32_t count,
    float speed_percent);
// ABI 2.35 persistent whole-product ordinary-motion speed. The setting is
// inclusive 0..100 and defaults to 70. It applies to ordinary MIT/PV joint
// position references only; raw frames, trajectories, and Cartesian motions
// keep their explicit physical limits or speed percentages. set_speed() also
// updates an active ordinary position reference without replacing its target.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_speed(
    ArticoreRuntime* runtime, float speed_percent);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_speed(
    ArticoreRuntime* runtime, float* speed_percent);
// ABI 2.38 PV-only names. max_speed_percent is the persistent upper bound used
// while advancing ordinary PV position references. The inclusive 0..100
// scale, default 70, and 5 rad/s physical maximum are unchanged. MIT continues
// to use its per-command ordinary speed or explicit raw frame parameters.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_max_speed(
    ArticoreRuntime* runtime, float max_speed_percent);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_max_speed(
    ArticoreRuntime* runtime, float* max_speed_percent);
// Canonical ordinary MIT/PV position command. It always uses the current
// persistent max-speed setting and advances the reference inside Runtime.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_positions_v2(
    ArticoreRuntime* runtime, const float* positions, uint32_t count);
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_mit_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocities, const float* feedforward_torques,
    const float* kp, const float* kd, uint32_t count);
// ABI compatibility only. ABI 2.38 product bindings must not expose raw PV;
// native trajectory and Cartesian implementations may still use raw PV
// internally after completing their own validation and interpolation.
ARTICORE_RUNTIME_API int32_t articore_runtime_submit_pv_frame(
    ArticoreRuntime* runtime, const float* positions,
    const float* velocity_limits, uint32_t count);
// ABI 2.23 product-level asynchronous trajectory transaction. The input is
// copied synchronously; no caller-owned pointer is retained. The fixed Yunyi
// order is left joint1..7 followed by right joint1..7. Execution uses the same
// native worker and Raw MIT/PV ControllerGroup path as direct product frames.
// COMPLETED is reported only after the final reference has been sent and fresh
// enabled feedback confirms position/velocity settling for consecutive native
// samples. A native arrival timeout changes only the motion to FAULT, preserves
// the final hold, and records the failed Motor and error in Runtime health.
ARTICORE_RUNTIME_API int32_t articore_runtime_start_trajectory(
    ArticoreRuntime* runtime,
    const ArticoreTrajectoryWaypoint* waypoints,
    uint32_t waypoint_count,
    const ArticoreTrajectoryConfig* config);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_trajectory_status(
    ArticoreRuntime* runtime, ArticoreTrajectoryStatus* status);
ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_trajectory(
    ArticoreRuntime* runtime);
// ABI 2.28 asynchronous PV Cartesian point-to-point motion. target_pose is
// [x,y,z,roll,pitch,yaw] in metres/radians. speed_percent is (0,100]. The
// command returns after native IK, limit/extrema validation and plan install;
// execution continues in the Runtime worker. A valid newer target atomically
// replaces a running point target. A rejected target leaves the old motion
// untouched. This is joint-space point-to-point motion, not a Cartesian line.
ARTICORE_RUNTIME_API int32_t articore_runtime_move_pose(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent, uint64_t* motion_id);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_move_pose_status(
    ArticoreRuntime* runtime, ArticoreMovePoseStatus* status);
ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_move_pose(
    ArticoreRuntime* runtime);
// ABI 2.29 unified PV Cartesian motion entry point. POINT_TO_POINT performs one
// endpoint IK and follows a joint-space quintic. LINEAR interpolates XYZ on a
// straight segment and orientation with shortest-path quaternion SLERP, solves
// and validates sequential IK samples before installation, then executes the
// precomputed joint path without running IK in the realtime worker.
ARTICORE_RUNTIME_API int32_t articore_runtime_move_cartesian(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent, int32_t interpolation, uint64_t* motion_id);
ARTICORE_RUNTIME_API int32_t articore_runtime_move_linear(
    ArticoreRuntime* runtime, uint32_t side, const float* target_pose,
    float speed_percent, uint64_t* motion_id);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_cartesian_motion_status(
    ArticoreRuntime* runtime, ArticoreCartesianMotionStatus* status);
ARTICORE_RUNTIME_API int32_t articore_runtime_cancel_cartesian_motion(
    ArticoreRuntime* runtime);
// ABI 2.30 PV-only three-pose circular motion. Every pose is
// [x,y,z,roll,pitch,yaw]. XYZ defines the unique arc from start through via to
// end. The declared start must match the current planned end-effector pose; it is
// never used as a teleport target. Orientation uses shortest-path quaternion
// SLERP from start to via and from via to end.
ARTICORE_RUNTIME_API int32_t articore_runtime_move_circular(
    ArticoreRuntime* runtime, uint32_t side, const float* start_pose,
    const float* via_pose, const float* end_pose, float speed_percent,
    uint64_t* motion_id);
// ABI 2.31 circular motion whose start is the Runtime-owned planned end-effector
// pose. Reading that reference and installing the replacement are one native
// command transaction. The old three-pose entry point remains ABI-compatible.
ARTICORE_RUNTIME_API int32_t articore_runtime_move_circular_v2(
    ArticoreRuntime* runtime, uint32_t side, const float* via_pose,
    const float* end_pose, float speed_percent, uint64_t* motion_id);
// Whole-product gripper command. Openings use 0=closed and 1000=open and are
// clamped when finite. gripper_level directly selects one of the built-in
// yunyi_gripper_v1 levels 1..10; 1 is lightest and 10 is strongest.
// For a Runtime created without grippers, valid commands return success without
// sending traffic or changing safety state.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_grippers(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
    int32_t gripper_level);
// ABI 2.25 product gripper command. strength uses 0..10: zero sends no active
// stiffness and levels 1..10 select the existing calibrated force profiles.
// DIRECT mode bypasses only gripper contact/stall and overload-retreat logic;
// Runtime feedback checks, hard motor faults, transport safety, estop, and
// disconnect remain active. The legacy function above is equivalent to this
// function with mode=PROTECTED and continues to require level 1..10.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_grippers_v2(
    ArticoreRuntime* runtime, float left_opening, float right_opening,
    int32_t strength, int32_t mode);
ARTICORE_RUNTIME_API int32_t articore_runtime_has_grippers(
    ArticoreRuntime* runtime, int32_t* has_grippers);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_state(
    ArticoreRuntime* runtime, ArticoreProductState* state);
// ABI 2.22 coherent cached product state including actual per-motor power
// feedback. This function sends no CAN request. The legacy get_state symbol
// and structure remain unchanged for existing binary clients.
ARTICORE_RUNTIME_API int32_t articore_runtime_get_state_v2(
    ArticoreRuntime* runtime, ArticoreProductStateV2* state);
// ABI 2.32 cached product state including finite, fresh MOS and rotor
// temperatures. This function performs no CAN I/O. V1/V2 remain unchanged.
ARTICORE_RUNTIME_API int32_t articore_runtime_get_state_v3(
    ArticoreRuntime* runtime, ArticoreProductStateV3* state);
// ABI 2.34 immutable product limits for exactly 14 arm joints. This performs
// no CAN I/O, does not require a connected Runtime, and never includes grippers.
ARTICORE_RUNTIME_API int32_t articore_runtime_get_joint_angle_vel_limits(
    ArticoreRuntime* runtime, ArticoreProductJointAngleVelLimits* limits);
// Computes one arm's active product-control pose from the latest complete
// native feedback cache. With grippers this is l/r-tool0 at the gripper
// center; without grippers it is l/r-link7. This call performs no CAN I/O and
// is suitable for high-rate reads.
ARTICORE_RUNTIME_API int32_t articore_runtime_get_pose(
    ArticoreRuntime* runtime, uint32_t side, ArticoreProductPose* pose);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_last_connect_report(
    ArticoreRuntime* runtime, ArticoreConnectReport* report);
// With ARTICORE_CAP_ATOMIC_ENABLE this owns the complete native transaction:
// fresh disabled-state position capture, parallel controller enable, immediate
// current-position hold, parallel enabled-feedback confirmation, and linked
// disable rollback on every failure. SDKs must not enable motors beforehand.
ARTICORE_RUNTIME_API int32_t articore_runtime_enable(
    ArticoreRuntime* runtime, int32_t mode);
// ABI 2.21 atomic subset transactions. roles must contain unique stable Yunyi
// names (l-joint1..7, r-joint1..7, l-gripper, r-gripper). Empty lists are
// rejected. Failed enable rolls back and verifies every motor newly enabled by
// that call. Disable remains available in abnormal states and verifies the
// selected motors. Complete product command frames remain required; the
// Runtime filters intentionally disabled motors during native dispatch.
ARTICORE_RUNTIME_API int32_t articore_runtime_enable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report);
ARTICORE_RUNTIME_API int32_t articore_runtime_disable_motors(
    ArticoreRuntime* runtime, const char* const* roles, uint32_t count,
    ArticoreMotorPowerReport* report);
// ABI 2.14 Runtime-owned motor power API. motor_name accepts a stable product
// role such as "left/joint1" or "right/gripper". A null or empty name selects
// the complete product. Single-motor writes are valid only in READY or
// PARTIALLY_ENABLED. ABI 2.21 also accepts the stable short product names and
// filters intentionally disabled motors from complete control frames.
// confirmed_state is optional for writes and required for queries.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_motor_power(
    ArticoreRuntime* runtime, const char* motor_name, int32_t enabled,
    int32_t* confirmed_state);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_motor_power(
    ArticoreRuntime* runtime, const char* motor_name, int32_t* state);
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
// Binds each active seven-axis arm transport side to its built-in rigid-body
// model. Bindings are immutable after connect and must cover every arm side.
ARTICORE_RUNTIME_API int32_t articore_runtime_configure_gravity_products(
    ArticoreRuntime* runtime,
    const ArticoreGravityProductBinding* bindings,
    uint32_t binding_count);
// Starts a Runtime-owned MIT hand-guiding mode. While active, ordinary arm
// commands are rejected and every native control cycle sends zero-stiffness,
// zero-damping gravity feedforward. A null config selects the 500 ms default.
ARTICORE_RUNTIME_API int32_t articore_runtime_start_gravity_compensation(
    ArticoreRuntime* runtime,
    const ArticoreGravityCompensationConfig* config);
// Smoothly hands gravity feedforward back to a current-position MIT hold.
ARTICORE_RUNTIME_API int32_t articore_runtime_stop_gravity_compensation(
    ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_gravity_compensation_status(
    ArticoreRuntime* runtime,
    ArticoreGravityCompensationStatus* status);
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
// complete arm batch, one shared 0..100 speed percentage, capacity-one latest
// value replacement, and feedback initialization only for the first ordinary
// PV command after enable/reconnect/recover. The generated PV position advances
// by the corresponding scaled product reference step per native cycle.
ARTICORE_RUNTIME_API int32_t articore_runtime_set_joint_pv(
    ArticoreRuntime* runtime,
    const ArticoreJointPvTarget* targets,
    uint32_t target_count,
    float speed_percent);
// ABI 1.12 ordinary MIT position setting. The target array must contain every
// active arm joint in one atomic batch. speed_percent is one shared inclusive
// 0..100 pace used only to advance q; transmitted dq and tau are zero
// and kp/kd come from the configured product joint parameters. The Runtime
// keeps transmitting the final position until an explicit control transaction
// replaces it. Raw submit_mit[_ex]() remains unmodified and un-ramped.
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
// Immediately supersedes Runtime motion with a latched current-position hold.
// Enabled Motors remain enabled and receive continuous native PV/MIT safety
// frames; an already-disabled product is never re-enabled. The Runtime records
// a standard emergency-stop reason in health, clears every superseded user
// command/trajectory, and accepts no new motion. This call is idempotent and
// the latch can only be cleared by articore_runtime_recover().
ARTICORE_RUNTIME_API int32_t articore_runtime_estop(ArticoreRuntime* runtime);
// ABI 2.17 whole-product recovery transaction. This clears recoverable motor
// faults, validates both transports and all installed feedback, enables the
// product only long enough to return every arm joint to its already-calibrated
// zero at a Runtime-owned low speed, then disables and verifies the complete
// product. Any failed stage attempts a full disable and records its stage,
// error code, message, and affected motors in health v2. This does not change
// calibration; use articore_runtime_set_zero() to define the current position
// as zero, or articore_runtime_clear_faults() for a non-moving clear-only call.
ARTICORE_RUNTIME_API int32_t articore_runtime_recover(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_health(
    ArticoreRuntime* runtime, ArticoreSafetyHealth* health);
ARTICORE_RUNTIME_API int32_t articore_runtime_get_health_v2(
    ArticoreRuntime* runtime, ArticoreSafetyHealthV2* health);
// Returns the cumulative activation counter and the latest successfully sent
// MIT arm cycle. Statistics are updated for both newly consumed mailbox
// targets and native cycles that repeat the previous target.
ARTICORE_RUNTIME_API int32_t articore_runtime_get_mit_torque_limit_stats(
    ArticoreRuntime* runtime, ArticoreMitTorqueLimitStats* stats);
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
// Compatibility alias retained for existing ABI users. New product bindings
// expose only disconnect() and call free internally.
ARTICORE_RUNTIME_API int32_t articore_runtime_close(ArticoreRuntime* runtime);
ARTICORE_RUNTIME_API const char* articore_runtime_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
