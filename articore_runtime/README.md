# Articore native safety runtime

This directory contains the native Yunyi V1.0 dual-arm product Runtime. It directly links and owns
the repository's layered C++ Motor core; no intermediate Motor C ABI or second shared library is
used. SDK users do not assemble Controllers, Motors, mappings, profiles, or product identifiers.

The public runtime remains one `libarticore_runtime` library and one
`SafetyRuntime` state owner. Its implementation is split by responsibility:

- `runtime.cpp`: lifetime, connection, presence, and shutdown facade.
- `runtime_commands.cpp`: command validation, latest-value mailbox, and the arm send cycle.
- `runtime_enable.cpp`: atomic enable, explicit disable, estop, and recovery transactions.
- `runtime_gripper.cpp`: gripper command mapping, contact/stall detection, hold, and overload retreat.
- `runtime_joint_position.cpp`: ordinary PV/MIT position targets and constant-speed reference advancement.
- `runtime_safety.cpp`: feedback/transport supervision, protective hold, fault policy, and health snapshots.
- `runtime_worker.cpp`: persistent absolute-deadline scheduler and safety event dispatch.
- `yunyi_runtime.cpp`: the single product factory, fixed dual-channel topology, calibration,
  joint mapping, gripper profiles, models, and ownership of all native resources.

The split does not create additional shared libraries or independent state machines; it only gives
the existing state owner smaller compilation units with a single lock order and one state owner.

Source layout is also separated by file type: `src/` contains implementation `.cpp` files only,
`include/articore/` contains the installed public C/C++ API, and
`include/articore/detail/` contains build-only internal `.hpp` files. The `detail` tree is excluded
from installation and is not a supported SDK surface.

The runtime owns one persistent worker thread. Its arm loop uses `steady_clock` absolute deadlines
at an internal product cadence, skips missed periods, and never
replays expired frames. Complete PV or MIT commands atomically overwrite a capacity-one
latest-value mailbox; if A, B, and C arrive before the next tick, only C is sent. The native
control path reads the mailbox storage directly without copying a command queue or allocating a
per-tick snapshot, and keeps transmitting the latest valid target through `ControllerGroup`. Streaming
commands must be refreshed by the caller and are covered by the native watchdog; explicit
persistent setpoints remain active until replaced. The worker
independently performs command timeout handling,
feedback and transport-health checks, safe-hold transmission, fault latching, protective fault
hold, and explicit-disable confirmation while Python is blocked or has stopped running.

Runtime ABI 1.4 makes enable a native all-or-nothing transaction. It refreshes both channels in
parallel while motors are disabled, captures current positions, enables CH0 and CH1 concurrently,
sends the first arm and gripper hold frames before waiting for confirmation, and continues the
hold from the persistent worker. Both channels are refreshed in parallel until every motor reports
fresh `ENABLED` feedback within `enable_grace_ms`. Any failure latches `FAULT`, attempts every
motor disable, confirms physical disable, and remains available through a structured per-channel,
per-motor enable report. `disable()` is valid in `FAULT` and never clears the latch; `recover()`
performs the ABI 2.17 whole-product recovery transaction described below. SDKs must call
the runtime enable entry point directly instead of enabling individual arms first. If a complete
fresh confirmation shows one motor is still `DISABLED`, the runtime retries only that motor once;
there is no unbounded enable retry loop.
Current SDK bindings call only `articore_runtime_create_yunyi()`. Legacy generic creation symbols
remain exported for binary compatibility, but they are not part of the product SDK or the normal
C++ wrapper path.

Feedback health is deliberately separate from command validation. Runtime command limits apply to
user commands before transmission, but are not applied to measured
position, velocity, or torque. Feedback monitoring checks finite values, freshness, transport
health, motor status, and unexpected disable; mechanical feedback protection requires separately
defined thresholds, tolerance, and persistence rather than reusing URDF command limits.
Missing or stale feedback remains visible through per-motor age, side health, error text, and
consecutive-failure counters, but it is diagnostic-only and does not transition a running Runtime
to `SAFE_HOLD` or `FAULT`. Motor fault status, unexpected disable, non-finite feedback, and
transport disconnect remain actionable faults, but a hard fault does not automatically torque off
unrelated healthy motors.

Runtime ABI 1.5 separates command update lifetime from physical motion duration. The legacy direct
submission entry points remain `STREAMING`: callers must refresh them before `command_timeout_ms`.
The new `_ex` entry points accept either `ARTICORE_COMMAND_STREAMING` or
`ARTICORE_COMMAND_HOLD_UNTIL_REPLACED`. Product SDK one-shot position APIs use the latter, so a slow
move may take longer than the watchdog timeout while the native control thread keeps transmitting
its latest setpoint. Real-time servo loops use `STREAMING`, so a stalled caller still enters
`SAFE_HOLD`. Persistent MIT commands require zero target velocity and zero feedforward torque;
advanced dynamic MIT control should use the raw streaming interface.

Runtime ABI 1.12 adds `articore_runtime_set_joint_mit()` for ordinary one-shot MIT position
setting. One call supplies the complete active arm layout, final joint positions, and one shared
speed value. ABI 2.13 defines that value as 0..100 and converts it to the product's physical
reference velocity inside C++. On the first command after enable or recovery, the Runtime
requires complete fresh enabled feedback and initializes every `current_target` from measured
position. At each native control tick it advances each reference by a bounded
step derived from the product-owned scheduler, transmits `dq=0` and `tau=0`,
and uses the product-configured MIT Kp/Kd. Different travel distances may finish at different times; this interface deliberately
does not synchronize different joint arrival times.

The ordinary MIT target is a capacity-one latest-value mailbox. A new complete dual-arm command
atomically discards the previous `final_target` and shared velocity while preserving the currently
transmitted `current_target`; the next tick therefore reverses or changes speed without a position
jump. The final target remains active and is retransmitted until another explicit control
transaction, disable, close, or estop replaces it. This persistent behavior does not require a
Python refresh loop. The original `articore_runtime_submit_mit_ex()` remains a raw advanced
control path: q, dq, Kp, Kd, and feedforward torque are sent exactly as provided and receive no
ordinary-position ramping.

Runtime ABI 1.13 adds the symmetric `articore_runtime_set_joint_pv()` ordinary position path. It
uses the same complete-arm latest-value mailbox, fresh-feedback initialization, shared rad/s speed,
and a scheduler-owned bounded position step. Generated PV frames use the current native
reference as position and the same shared speed as their protocol velocity limit. A new PV target
atomically replaces only the final positions and shared speed; the currently transmitted reference
remains continuous. The raw `articore_runtime_submit_pos_vel_ex()` entry point remains unchanged for
internal advanced controllers.

This was the ABI 1.13 product-binding rule. ABI 2.38 supersedes the PV half:
product SDKs expose persistent `set_max_speed(0..100)` plus a positions-only
`set_joint_pv`; raw PV and per-command PV speed are no longer public. MIT keeps
its existing ordinary and advanced raw controls.

Runtime ABI 1.6 makes normal torque-off a checked transaction instead of relying on the motor
communication watchdog. `disable()` first rejects new commands and waits for any in-flight
ControllerGroup batch. It then submits an empty group barrier followed by parallel fresh-feedback
markers on CH0/CH1; receipt proves that previously accepted Runtime motion frames have passed the
USB/CAN queue boundary. Both channels are disabled in parallel and confirmed with fresh feedback.
If confirmation is incomplete, only the unconfirmed motors receive one directed disable retry,
followed by one final parallel confirmation. There is no unbounded retry loop.
Terminal shutdown reuses the same transaction. If physical disable cannot be confirmed, the
Runtime still stops and joins its worker before reporting failure, so no background sender survives.
`articore_runtime_get_last_disable_report()` exposes stable
per-channel/per-motor status, missing IDs, barrier status, and whether the one-shot retry ran.
Legacy `articore_runtime_free()` remains a void best-effort destructor for ABI compatibility;
ABI 2.20 product bindings call checked `articore_runtime_disconnect()` and then free the opaque
tombstone internally.

The additive `articore_runtime_configure_joint_safety_limits()` call keeps the original joint
configuration ABI stable while separating mechanical hard limits from normal-operation soft
limits. A first ordinary-position command initializes its continuous reference from fresh feedback
even when that measured position is outside the configured command limits; an in-range target can
therefore move back into the configured region without resetting or jumping the reference.
Ordinary PV/MIT targets remain bounded by the configured hard and soft position limits. Feedback
position, velocity, and torque are not compared with command limits and cannot trigger a limit
FAULT. Any future mechanical feedback protection must use separate product thresholds, tolerances,
and persistence rules instead of reusing command validation.

Runtime ABI 2.2 adds built-in gripper product calibration and advertises
`builtin_gripper_product_profiles`. Before `connect()`, a production Runtime binds every installed
gripper with `articore_runtime_configure_gripper_products()`. The built-in
`yunyi_gripper_v1` profile owns the complete mapping, maximum speed, contact/stall/overload
thresholds and timing, hold/retreat behavior, moving/holding MIT gains, all ten force levels, and
the fault action. The Runtime copies `profile_id`; it never depends on caller string lifetime.
An unknown profile, a partial active-gripper binding, or a post-connect configuration attempt is
rejected. A Runtime with no active grippers needs no binding.

Runtime ABI 2.4 advertises both `connect_feedback_barrier` and
`structured_connect_report`. Official bindings configure the complete immutable motor-to-CAN-ID
mapping before connect. `connect()` performs one parallel structured feedback transaction on every
active channel and validates fresh finite cache state for every configured joint and installed
gripper before exposing READY. `ArticoreConnectReport` classifies configuration, transport,
zero-feedback timeout, incomplete feedback, and invalid-cache failures; it also carries channel
counts and per-motor identity/age/error data. While the Runtime remains READY, its native worker
repeats this full-cache refresh at no more than 10 Hz, so SDKs may read cached state before enable
without issuing their own transport request.

Runtime ABI 1.10 adds `articore_runtime_set_gripper_commands()`. Each complete active-gripper
transaction contains only `opening`, normalized `speed`, and a stable `force_level`; all fields
become visible to the persistent worker under one command lock. Runtime ABI 1.11 defines the public
force selector as ten calibrated integer levels: 1 is lightest, 10 is strongest, and 5 is the
compatibility default. Speed
uses the same product-independent scale as opening: 1000 means the maximum gripper speed calibrated
by its bound product profile. Both opening and closing advance from the previous native command
position through the same bounded ramp, so neither direction jumps directly to its endpoint.

`articore_runtime_configure_gripper_force_profiles()` remains available as an advanced/test
override after a built-in product has been bound; ordinary SDK product setup does not need it.
Changing speed or force level during motion is atomic; a
force change resets only threshold-dependent contact evidence while motion continues. Existing
contact detection, low-gain holding, overload retreat, feedback supervision, and whole-Runtime
safety-state integration remain active. The legacy opening-only call is preserved and now uses the
same bidirectional ramp with maximum speed and force level 5.

For ABI migration, Runtime 1.11 also accepts the former three profile values `1/2/3` as
light/normal/strong calibration anchors and deterministically interpolates them to levels 1..10.
For a Runtime configured through that fallback, legacy command values `1/2/3` retain their
light/normal/strong meanings (mapped to new levels `1/5/10`); values `4..10` address the expanded
levels directly. This removes semantic surprises for an already-built ABI 1.10 SDK.
The old three/ten-level override remains ABI-compatible for advanced tests. Production C ABI
runtimes now require the built-in product binding instead of requiring SDKs to reproduce the
calibration table.

When an arm enters safe hold, the runtime snapshots every arm motor's current position from the
non-blocking feedback cache. PV safe hold uses the captured positions with a dedicated low velocity limit.
MIT safe hold uses the captured positions, zeros velocity and feedforward torque, and substitutes
product safety Kp/Kd. Grippers keep their independently generated low-stiffness safety targets.
The same persistent worker owns each configured product gripper's
`IDLE -> MOVING -> CONTACT -> HOLDING -> OVERLOAD_RETREAT` state machine. It maps public 0..1000
opening targets to motor position, ramps closing motion with the normal MIT gains, detects contact
from torque plus a position-motion window and target error, then switches to a low-gain hold with
zero feedforward torque. Sustained overload produces a rate-limited bounded retreat. No Python
gripper control loop is involved in the native dual-arm path. During `ENABLED` and `RUNNING`, the
gripper state machine and MIT output run on the same internal scheduler as the arm.
Only `SAFE_HOLD` and protective `FAULT` holding use their independent safety cadence. Legacy
rate slots remain in the ABI struct layout as ignored placeholders and do not configure normal
arm or gripper control.

Operational faults use protective fault hold rather than linked torque-off. One missing feedback
sample only increments the failure counters: arms continue their current control-rate output and a
gripper retransmits its last successful safe output. At the configured consecutive-failure
threshold, both arms enter protection. The runtime captures a fresh
current position where feedback remains usable, falls back to the last successfully transmitted
position for a motor whose feedback is missing, and excludes a motor that is confirmed disabled or
faulted. A gripper with missing feedback keeps its last safe low-gain target; a confirmed gripper
fault is latched and is never automatically re-enabled.

Fault holds are dispatched separately per channel. A disconnected or failing channel therefore
does not prevent a still-controllable channel from continuing its hold, and a hold-send failure
never automatically disables another channel. `FAULT` remains latched and rejects ordinary motion.
An explicit `disable()` always attempts every motor, records any motor whose disabled state was not
confirmed, and does not clear the latch. `recover()` returns to `READY` only after fresh disabled
feedback and transport health are confirmed. With ABI 2.33, parameterless `estop()` immediately
supersedes Runtime motion with a latched current-position hold instead of torque-off. It keeps
enabled arm and gripper Motors enabled, continuously transmits native safety frames, records the
standard `emergency stop requested` health reason, is idempotent, and can only be unlatched by
`recover()`.

`runtime_abi.h` is the stable boundary used by Articore-SDK's private native binding. The optional generic
motor-drive-layer transport-health callback is used when available; the wrapper remains compatible
with the motor function-table ABI and falls back to motor feedback health when that callback is not
present. ABI version and capability functions make mismatches fail during runtime creation rather
than during motion.

Python language bindings live in Articore-SDK rather than this native repository. C++17 users include
`articore/runtime.hpp` and link the installed `motorbridge::articore_runtime_cpp` CMake target.
The move-only `articore::Runtime` RAII object delegates every operation to this C ABI and never
duplicates worker, watchdog, safety, or gripper behavior.

The controller feedback callback introduced in runtime ABI 1.3 matches
`motor_controller_request_feedback_all_ex()`: it returns a stable motor error code and fills the
expected/received/missing counts plus missing motor IDs. Disable confirmation and safety diagnostics
consume those fields directly; diagnostic error text is retained for logs but is never parsed to
choose safety behavior.

Runtime ABI 2.5 adds immutable per-side transport capabilities to runtime creation so the native
implementation can select a safe scheduler for the installed product. Raw submissions replace a
separate capacity-one pending mailbox and do not wait for an in-flight physical batch; the worker
consumes only the newest complete generation at its next tick. Runtime ABI 2.18 removes the former
control-rate getter and capability. Rate fields retained in the configuration struct are ignored
ABI-layout placeholders. Articore-SDK and applications neither set nor observe the Runtime control
frequency; they submit business commands and inspect state, timestamps, health, and operation
results instead.

Runtime ABI 2.6 advertises `ARTICORE_CAP_PER_CYCLE_MIT_TORQUE_LIMIT`. Before every native MIT arm
send—including cycles that repeat the latest capacity-one mailbox target—the worker reads the
newest native position and velocity feedback and computes
`Kp * (q_target - q_feedback) + Kd * (dq_target - dq_feedback) + tau_ff`. Each joint is bounded to
its configured `torque_limit`; when limiting is necessary, Kp, Kd, and feedforward torque
are multiplied by the same scale so their relative contributions are preserved. The complete arm
batch is rejected before transmission if the cached feedback is disabled, unavailable, or
non-finite, which then enters the existing protective fault-hold path. A stale-but-valid cached
sample remains usable while its age is reported diagnostically, so feedback timeout alone does not
stop the native sender. This keeps protection inside every native product cycle even when a
publisher updates at a different application cadence. The cumulative
activation count and latest per-joint requested torque, applied scale, and applied torque are
available through `articore_runtime_get_mit_torque_limit_stats()`.

Runtime ABI 2.7 advertises `ARTICORE_CAP_NATIVE_ROBOT_MODEL`. The opaque
`ArticoreRobotModel` handle owns the exact product model and provides FK, IK, three Jacobian
reference conventions, gravity, mass and Coriolis matrices, nonlinear effects, RNEA and ABA. The C
ABI contains only fixed-size metadata and caller-owned row-major `double` arrays; Pinocchio and
Eigen types never cross the boundary. The Yunyi model is constructed from private calibrated
parameters in the shared library, while the Python SDK receives only results and joint limits.

Runtime ABI 2.8 advertises `ARTICORE_CAP_NATIVE_GRAVITY_COMPENSATION`. Before `connect()`, bind
every active seven-axis arm side with `articore_runtime_configure_gravity_products()`. After
enabling MIT mode, `articore_runtime_start_gravity_compensation()` transfers exclusive arm output
ownership to the native worker. Each control cycle reads q, evaluates the product gravity model,
and sends MIT gravity feedforward; the ACTIVE phase uses `kp=kd=0`. Entry and exit use a 500 ms
default blend, and exit lands in a current-position MIT hold. The normal per-cycle resultant-torque
limiter remains the final gate before transmission.

Runtime ABI 2.9 moves product maintenance behind the Runtime ownership boundary.
`articore_runtime_configure_mode()`, `articore_runtime_clear_faults()`, and
`articore_runtime_set_zero()` reuse the already-owned Motor handles and execute behind the same
worker/transport barrier without releasing the ControllerGroup or rebuilding the Runtime. The
zeroing transaction requires READY, fresh feedback, healthy transports, confirmed physical
disable, and a fixed 0.05 rad/s stationary threshold. Both channels run in parallel; every motor
is re-read and checked for disabled state, position within 0.02 rad, and stationary velocity.
Partial completion returns a stable error and is recorded in `ArticoreSafetyHealthV2`; it is never
reported as success. `articore_runtime_create_yunyi(...)` constructs and owns the
fixed can-left/can-right SocketCAN-FD+BRS dual-arm product with all 14 joints and two grippers.
True all-or-none zeroing still requires a future firmware prepare/commit protocol.

Runtime ABI 2.10 makes `yunyi_v1_0` a complete product-owned Runtime instead of
a generic container assembled by a language binding. The product factory owns
both SocketCAN-FD/BRS channels, all 16 Motors, direction and range conversion,
joint limits, default MIT gains, grippers, gravity models, the ControllerGroup,
leases, workers, and resource lifetimes. Fixed 14-joint logical-coordinate
position, raw MIT, and raw PV frames are validated and converted natively;
`articore_runtime_get_state()` returns one left/right/gripper snapshot.

Runtime ABI 2.11 adds the immutable `with_grippers` product topology. The true
variant creates and validates 16 Motors; the false variant creates only the 14
arm Motors and never waits for or faults on absent grippers. The simplified
whole-product gripper command accepts only left/right 0..1000 openings and one
1..10 force level, and is a successful no-op for a gripperless Runtime. Product
state exposes only gripper availability, logical opening, and force level.

Runtime ABI 2.12 adds graded feedback safety. One or two missed feedback checks
are tolerated. Sustained delay enters `DEGRADED` and applies a native 0.25
velocity/torque scale; three times the configured failure threshold enters
`SAFE_STOP`, rejects new motion, and continuously holds the latest safe
position without disabling motors. `fault_reason` is reserved for confirmed
motor/transport faults; communication quality is exposed as `safety_reason`.
Recovered communication never resumes an old target. Explicit `recover()`
clears recoverable faults, validates both arms, returns all arm joints to the
already-calibrated zero at low speed, and finishes in confirmed-disabled
`READY`; it never restores the previous command.

Runtime ABI 2.13 normalizes the shared pace of ordinary MIT/PV position
commands to an inclusive 0..100 product scale. Zero pauses reference
advancement and 100 selects the active product/mode maximum. The C++ Runtime
owns conversion to rad/s, per-cycle stepping, and absolute joint-limit checks;
raw MIT/PV frames remain in physical units.

Runtime ABI 2.14 adds confirmed whole-product and single-motor power control.
`articore_runtime_set_motor_power()` accepts stable roles such as
`left/joint1` and `right/gripper`; a null role selects every installed motor.
`articore_runtime_get_motor_power()` returns `DISABLED`, `ENABLED`, `MIXED`, or
`UNKNOWN`. Single-motor changes are restricted to non-motion states and enter
`PARTIALLY_ENABLED`, where all motion APIs stay blocked until the normal atomic
whole-product `enable()` transaction establishes an initial position hold.

Runtime ABI 2.15 adds `articore_runtime_get_pose()`. It reads one coherent
seven-joint sample from the native feedback cache and computes the selected
left/right product pose with the product-owned Pinocchio model. The fixed output
is `[x, y, z, roll, pitch, yaw]` in metres/radians plus the oldest contributing
feedback timestamp and sequence. The getter sends no CAN frames. ABI 2.37
defines whether this pose is the gripper-center tool frame or link7.

Runtime ABI 2.16 makes `articore_runtime_estop()` parameterless, records the
standard emergency-stop reason in health, and latches until an explicit
`recover()`. ABI 2.33 supersedes the earlier whole-product torque-off behavior
with continuous current-position holding.

Runtime ABI 2.17 defines `articore_runtime_recover()` as a complete native
whole-product transaction: stop old commands and disable, clear recoverable
faults on both channels, validate transports and fresh disabled feedback,
enable only for a fixed low-speed return of all 14 arm joints to their
previously calibrated zero, then disable and verify every installed motor.
Every failed stage attempts the same full-product disable and records the
stage, stable operation code, error text, and affected motor names in health.
`clear_faults()` remains clear-only and never moves; `set_zero()` still changes
calibration by defining the current physical position as zero.

Runtime ABI 2.19 makes product `disconnect()` the single terminal shutdown
operation. It rejects new commands, performs and verifies whole-product
disable, joins the native worker, closes both CAN channels, and releases all
product-owned Controllers, Motors, models, and group resources. Calls are
idempotent. The old `close/free` C symbols remain as compatibility aliases;
language bindings free the small opaque tombstone internally and expose no
separate close operation.

Runtime ABI 2.20 removes product selection from the supported binding surface. Yunyi V1.0 dual arm
is the only product: `articore_runtime_create_yunyi(mode, with_grippers)` owns both CAN channels,
the fixed 14-joint mapping, optional paired grippers, models, calibration and all resource
lifetimes. C++ and Python product APIs no longer expose generic construction or any
`configure_joints/configure_gripper_products/configure_gravity_products` step. The generic
`create_product/create_ex*` C constructors are removed in the new major package line so callers
cannot reassemble Motor, Controller, or product resources above the native product runtime.

Runtime ABI 2.21 adds atomic product-level subset power transactions with stable Yunyi roles
(`l-joint1..7`, `r-joint1..7`, `l-gripper`, `r-gripper`). A failed subset enable rolls back and
verifies motors changed by that call; subset disable remains available from abnormal states and
verifies each requested motor. `PARTIALLY_ENABLED` is a supported control state: clients keep
submitting complete 14-axis frames while native dispatch filters intentionally disabled motors.
Per-motor command, feedback, confirmation and error details are returned in
`ArticoreMotorPowerReport` and mirrored into unified operation health.

Runtime ABI 2.22 adds `articore_runtime_get_state_v2()`. It reads the complete product from the
existing low-rate Motor feedback caches and never emits a feedback request. Each arm carries
`enabled_mask` plus `enabled_valid_mask`; grippers carry the same value/validity semantics.
State, update sequence and feedback age for each Motor are captured under one cache lock. Missing,
stale or unrecognized feedback remains unknown instead of being inferred from Runtime intent. The
legacy state structure and `articore_runtime_get_state()` symbol remain binary compatible.

Runtime ABI 2.23 adds one product-level native dual-arm quintic trajectory transaction.
`articore_runtime_start_trajectory()` copies every waypoint before returning, resolves shared
intermediate velocity/acceleration boundary values, precomputes each quintic segment, and checks
timestamp, finite-value, product position, velocity, acceleration, torque and polynomial interior
extrema constraints. The existing product worker samples the plan from an absolute monotonic
clock at its private 500 Hz schedule and feeds the existing complete-frame Raw MIT/PV
ControllerGroup path; no Python loop or second realtime thread exists. MIT frames carry explicit
target velocity, Kp, Kd and feedforward torque, while PV frames carry explicit per-axis velocity
limits. Partial power filters intentionally disabled Motors. Cancellation is idempotent and turns
the in-flight MIT sample into a stationary hold; disable, estop, disconnect, safe stop and
transport/send faults terminate the active plan. Status and errors remain native and are mirrored
into unified operation health. Reaching the nominal plan duration does not by itself produce
`COMPLETED`: the worker keeps transmitting the final hold and requires consecutive fresh Motor
feedback samples inside native position and velocity arrival windows. `progress == 1` with
`state == RUNNING` therefore means physical settling. An internal arrival deadline reports the
motion as `FAULT`, preserves the final hold without putting Motors into a fault mode, and records
the failed Motor plus measured error in unified health.

Runtime ABI 2.24 adds `product_gripper_force_10_levels`. The public Yunyi
`articore_runtime_set_grippers()` selector now addresses the ten immutable
`yunyi_gripper_v1` calibrations directly: 1 is lightest, 10 is strongest, and
5 is the default. Product state reports this same selected level without a
1..5 compatibility compression. Bindings must require the product-specific
capability instead of inferring support from the older generic ten-level
gripper capability.

Package 0.10.27 doubles the command strength of all ten `yunyi_gripper_v1`
calibrations. Moving/holding stiffness and contact/overload torque thresholds
are doubled; damping, motion speed, opening conversion, stall timing, retreat
distance, public APIs, and Runtime ABI remain unchanged.

Runtime ABI 2.25 adds `articore_runtime_set_grippers_v2()` and
`ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE`. The default `PROTECTED` mode keeps
contact/stall detection, calibrated low-gain holding, and sustained-overload
retreat. `DIRECT` continuously tracks the requested opening with the selected
moving gains and bypasses those three behaviors. The v2 strength scale is
0..10: zero applies no active gripper stiffness and 1..10 reuse the immutable
calibrations. Motor fault, feedback/transport safety, estop, and disconnect
remain active in both modes. The legacy product function remains protected and
retains its 1..10 contract.

Runtime ABI 2.26 adds `ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE`. Product control
mode now applies only to the fourteen arm joints: PV arms are configured as PV
and MIT arms as MIT, while both installed grippers are always configured and
commanded as MIT. Mode configuration, enable-time current-position hold,
normal gripper control, and recovery holding therefore use one consistent
gripper protocol and never place a Yunyi gripper in PV mode.

Runtime ABI 2.27 adds `ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10`. Product `DIRECT`
mode scales the selected gains by ten and caps MIT damping at the protocol
maximum of 5: default level 5 becomes Kp=80/Kd=5 and level 10 becomes
Kp=120/Kd=5. `PROTECTED` calibration is unchanged, strength zero remains
Kp=Kd=0, and Runtime DEGRADED scaling still reduces the resulting gains to
25 percent.

Runtime ABI 2.28 adds
`ARTICORE_CAP_PRODUCT_CARTESIAN_POINT_TO_POINT` and the native
`articore_runtime_move_pose()` transaction. The target is
`[x, y, z, roll, pitch, yaw]` in metres/radians and execution is asynchronous.
The C++ Runtime performs IK, product-limit and quintic-extrema validation, then
executes at its private control rate. A valid newer point target atomically
replaces the running point target from its current polynomial state; an invalid
replacement leaves the old target running. Explicit multi-waypoint
trajectories remain strict and are never replaced by this API. This is
joint-space point-to-point motion, not Cartesian-linear interpolation.
All product Cartesian motion entry points are PV-only; an MIT product Runtime
rejects them before planning and leaves any active motion unchanged.

Runtime ABI 2.29 adds `ARTICORE_CAP_PRODUCT_CARTESIAN_LINEAR`,
`articore_runtime_move_cartesian()` and the `articore_runtime_move_linear()`
convenience entry point. Linear mode keeps XYZ on the start-to-target segment
and interpolates orientation using shortest-path quaternion SLERP. The Runtime
solves sequential IK samples at no more than 5 mm / 0.035 rad spacing, rejects
unreachable, discontinuous or over-limit paths before installation, and then
executes only precomputed joint polynomials in the realtime worker. PTP and
linear single-target motions may atomically replace each other; explicit
multi-waypoint trajectories remain isolated.

Runtime ABI 2.30 adds `ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR` and
`articore_runtime_move_circular()`. Callers provide three complete
`[x,y,z,roll,pitch,yaw]` poses. Their XYZ values define the unique arc from
start through via to end; duplicate or collinear points are rejected. The
declared start must match the current planned end-effector pose within 5 mm and
0.035 rad and is never treated as a teleport target. Orientation passes through
all three declared attitudes using shortest-path quaternion SLERP on the two
arc portions. The complete sampled arc receives the same sequential IK,
branch-continuity, product-limit and polynomial-extrema checks as linear motion
before it may atomically replace another single-target Cartesian motion.

Runtime ABI 2.31 adds
`ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR_AUTO_START` and
`articore_runtime_move_circular_v2()`. Callers provide only via and end poses.
Runtime derives the start from its current planned joint reference—not delayed
motor feedback—and performs that snapshot plus trajectory installation inside
one native command transaction. This removes the SDK `get_pose()` round trip
and its race. The legacy three-pose function remains available for ABI
compatibility. Non-PV Runtime instances reject the v2 call before any existing
motion is replaced.

Motor-Drive-Layer 0.10.33 strengthens the ABI 2.31 Cartesian planner without
changing its public surface. Point-to-point endpoints use a deterministic
fixed-seed 1000-retry global IK search. Linear planning no longer performs an
unnecessary isolated endpoint solve before constructing the continuous path;
its intermediate samples stay on the current local IK branch and its final
sample receives the same global fallback. Circular endpoints follow the same
policy. Any unreachable, discontinuous or over-limit result is still rejected
before the active motion is replaced.

Runtime ABI 2.32 adds `ARTICORE_CAP_PRODUCT_TEMPERATURE_STATE` and
`articore_runtime_get_state_v3()`. The V3 snapshot extends V2 with MOS and
rotor temperatures in degrees Celsius for all 14 arm joints and installed
grippers, plus explicit per-Motor validity. Values are copied from the existing
coherent Motor feedback cache; the getter performs no CAN I/O. Missing, stale,
or non-finite temperature feedback is returned as NaN with its validity bit
clear. The V1/V2 structures and symbols remain binary compatible.

Runtime ABI 2.33 adds `ARTICORE_CAP_LATCHED_ESTOP_POSITION_HOLD`. Product
`estop()` atomically terminates the superseded trajectory/user mailbox,
captures the current arm reference from fresh native feedback, and continuously
dispatches PV/MIT safety holds. Enabled Motors remain enabled; a product that
was already disabled is never re-enabled. Feedback and hold-send failures stay
visible in health while the estop latch rejects all new motion. Repeated calls
are idempotent and only `recover()` may clear the latch.

Runtime ABI 2.34 adds `ARTICORE_CAP_PRODUCT_JOINT_ANGLE_VEL_LIMITS` and
`articore_runtime_get_joint_angle_vel_limits()`. One immutable snapshot returns
lower angle, upper angle, and product velocity limit for exactly 14 arm joints
in left J1..J7 then right J1..J7 order. Units are radians and radians/second.
Values come directly from the built-in Yunyi product configuration, require no
CAN traffic or connected state, and never include grippers.

Runtime ABI 2.35 adds `ARTICORE_CAP_PRODUCT_SPEED_SETTING`,
`articore_runtime_set_speed()`, and `articore_runtime_get_speed()`. A product
Runtime owns one persistent ordinary-joint-motion setting in the inclusive
range 0..100, defaulting to 70. For all 14 arm joints, 100 maps to a shared
5 rad/s reference cap, so the default maps to 3.5 rad/s. Updating the setting
also updates an active ordinary MIT/PV position reference. The additive
`articore_runtime_set_joint_positions_v2()` uses the stored value, while the
legacy explicit-speed symbol remains available. Raw frames, native
trajectories, and Cartesian motions retain their independent explicit limits.

Runtime ABI 2.36 corrects the preferred public terminology to an ordinary
motion maximum-speed setting. `ARTICORE_CAP_PRODUCT_MAX_SPEED_SETTING`,
`articore_runtime_set_max_speed()`, and `articore_runtime_get_max_speed()` are
the canonical API. The 0..100 scale, default 70, 5 rad/s maximum, and live
ordinary-reference update semantics are unchanged. ABI 2.35
`set_speed/get_speed` remain exported compatibility aliases; new bindings
should expose only the max-speed names.

Runtime ABI 2.37 adds `ARTICORE_CAP_PRODUCT_TOOL_CENTER_POSE` and makes the
existing product pose the single active Cartesian control point. A Yunyi
Runtime created with grippers uses `l-tool0` / `r-tool0`, fixed at
`[-0.004, 0, -0.178]` metres from link7; a gripperless Runtime uses link7
directly. `get_pose()`, endpoint IK, point-to-point, linear and circular motion
all use the same selection. No additional public flange-pose getter is added.

Runtime ABI 2.38 adds `ARTICORE_CAP_PV_MAX_SPEED_ONLY`. Product SDKs expose
one ordinary PV path: `set_max_speed(0..100)` configures the persistent limit
(default 70), and position commands contain positions only. Runtime advances
the reference incrementally at its private rate. Per-command ordinary speed
and raw direct-PV entry points are no longer product SDK APIs; their existing C
symbols remain exported only for binary compatibility. The max-speed names are
PV-only from ABI 2.38 onward; MIT retains its existing per-command ordinary
speed, raw targets, gains, and feedforward behavior unchanged. The ABI 2.35
`set_speed/get_speed` compatibility symbols keep their historical semantics.

Runtime ABI 2.40 adds the status-returning factory
`articore_runtime_create_yunyi_v2(mode, with_grippers, runtime_out)`. The ABI
2.39 `articore_runtime_create_yunyi(mode, with_grippers)` entry point remains a
two-argument function returning `ArticoreRuntime*`; its signature is frozen for
binary compatibility. SDKs must not infer a factory calling convention from
`ARTICORE_CAP_DIRECT_CPP_MOTOR_CORE`: that bit only identifies the direct C++
Motor implementation. Bind v2 only when ABI is at least 2.40 and the symbol is
present.

motor-drive-layer 0.10.39 hardens native PV Cartesian completion and final
hold. The public 0.02 rad / 0.05 rad/s arrival window remains compatible, but
the Runtime first performs low-speed settling and native FK endpoint checks at
2.5 mm / 0.01 rad. It then installs a continuously refreshed zero-speed final
position frame and verifies 200 ms of fresh stable feedback. Completed holds
remain supervised and can return to settling if instability persists. This
does not rewrite motor flash, zero offsets, or firmware PV gains, and it does
not layer the ordinary position stepper on top of native quintic execution.

Runtime ABI 1.2 adds fixed-connection motor presence. Active descriptor names begin as `PRESENT`;
omitted optional roles can be declared `NOT_INSTALLED` before `connect()`. Presence declarations
are rejected after connect, and a present motor that loses fresh feedback, reports a motor fault,
or belongs to a disconnected transport becomes `FAULTED` rather than disappearing from the
layout. `articore_runtime_motor_presence()` and
`articore_runtime_active_capabilities()` expose those decisions to language bindings.

Build and run the native tests with:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
