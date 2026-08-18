# Articore native safety runtime

This directory contains single- and dual-arm Articore product policy. It is a separate native
library in the Motor-Drive-Layer repository and depends on the generic motor C ABI without adding
Yunyi or Articore concepts to `libmotor_abi`.

The public runtime remains one `libarticore_runtime` library and one
`SafetyRuntime` state owner. Its implementation is split by responsibility:

- `runtime.cpp`: lifetime, connection, presence, and shutdown facade.
- `runtime_commands.cpp`: command validation, latest-value mailbox, and the arm send cycle.
- `runtime_enable.cpp`: atomic enable, explicit disable, estop, and recovery transactions.
- `runtime_gripper.cpp`: gripper command mapping, contact/stall detection, hold, and overload retreat.
- `runtime_joint_position.cpp`: ordinary PV/MIT position targets and constant-speed reference advancement.
- `runtime_safety.cpp`: feedback/transport supervision, protective hold, fault policy, and health snapshots.
- `runtime_worker.cpp`: persistent absolute-deadline scheduler and safety event dispatch.

The split does not create additional shared libraries or independent state machines; it only gives
the existing state owner smaller compilation units with a single lock order and one state owner.

The runtime owns one persistent worker thread. Its arm loop uses `steady_clock` absolute deadlines
at the effective control rate, skips missed periods, and never
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
only clears it after disabled feedback and transport health have been confirmed. SDKs must call
the runtime enable entry point directly instead of enabling individual arms first. If a complete
fresh confirmation shows one motor is still `DISABLED`, the runtime retries only that motor once;
there is no unbounded enable retry loop.
SDK bindings create ABI 1.4-or-newer runtimes with `articore_runtime_create_ex()` and pass the generic
`motor_controller_enable_all` and `motor_handle_enable` function pointers. This preserves the
separate-library boundary and avoids a direct DLL/dylib dependency between the product runtime and
`libmotor_abi`; the original `articore_runtime_create()` remains available for ABI 1.3 callers.

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
reference velocity in rad/s. On the first command after enable, reconnect, or recovery, the Runtime
requires complete fresh enabled feedback and initializes every `current_target` from measured
position. At each native control tick it advances each reference by at most
`max_reference_velocity / control_hz`, transmits `dq=0` and `tau=0`, and uses the product-configured
MIT Kp/Kd. Different travel distances may finish at different times; this interface deliberately
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
and `max_reference_velocity / control_hz` position step. Generated PV frames use the current native
reference as position and the same shared speed as their protocol velocity limit. A new PV target
atomically replaces only the final positions and shared speed; the currently transmitted reference
remains continuous. The raw `articore_runtime_submit_pos_vel_ex()` entry point remains unchanged for
internal advanced controllers.

Product SDKs should expose only one ordinary position method for the selected mode—`set_joint_mit`
or `set_joint_pv`—with final positions and one velocity argument. Raw PV/MIT structures, gains,
feedforward terms, and streaming lifetimes remain internal SDK/runtime integration capabilities and
should not appear in ordinary user examples.

Runtime ABI 1.6 makes normal torque-off a checked transaction instead of relying on the motor
communication watchdog. `disable()` first rejects new commands and waits for any in-flight
ControllerGroup batch. It then submits an empty group barrier followed by parallel fresh-feedback
markers on CH0/CH1; receipt proves that previously accepted Runtime motion frames have passed the
USB/CAN queue boundary. Both channels are disabled in parallel and confirmed with fresh feedback.
If confirmation is incomplete, only the unconfirmed motors receive one directed disable retry,
followed by one final parallel confirmation. There is no unbounded retry loop.
`close()` and the native destructor reuse the same transaction. A checked close does not enter
`DISCONNECTED` when physical disable cannot be confirmed, so the caller must not release its
Controller or Transport handles. `articore_runtime_get_last_disable_report()` exposes stable
per-channel/per-motor status, missing IDs, barrier status, and whether the one-shot retry ran.
Legacy `articore_runtime_free()` remains a void best-effort destructor for ABI compatibility;
bindings must call checked `articore_runtime_close()` first.

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
gripper state machine and MIT output run at the same effective `control_hz` as the arm.
Only `SAFE_HOLD` and protective `FAULT` holding use `safe_hold_hz` (normally 100 Hz). The legacy
`gripper_control_hz` config field remains in the ABI layout but no longer down-samples normal
gripper control.

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
feedback and transport health are confirmed. `estop()` currently applies the explicit product
policy: arm joints are torque-disabled, while `gripper_fault_action` selects low-gain gripper hold
or gripper disable. A later explicit `disable()` always disables held grippers as well.

`runtime_abi.h` is the stable boundary used by `arx_d_can.sdk.native_safety`. The optional generic
motor-drive-layer transport-health callback is used when available; the wrapper remains compatible
with the motor function-table ABI and falls back to motor feedback health when that callback is not
present. ABI version and capability functions make mismatches fail during runtime creation rather
than during motion.

Language bindings live in this repository rather than product SDKs. Python users import the typed
`motor_drive_layer.ArticoreRuntime` wrapper; ctypes definitions stay private. C++17 users include
`articore/runtime.hpp` and link the installed `motorbridge::articore_runtime_cpp` CMake target.
The move-only `articore::Runtime` RAII object delegates every operation to this C ABI and never
duplicates worker, watchdog, safety, or gripper behavior.

The controller feedback callback introduced in runtime ABI 1.3 matches
`motor_controller_request_feedback_all_ex()`: it returns a stable motor error code and fills the
expected/received/missing counts plus missing motor IDs. Disable confirmation and safety diagnostics
consume those fields directly; diagnostic error text is retained for logs but is never parsed to
choose safety behavior.

Runtime ABI 2.1 exposes `articore_runtime_get_control_hz()` and advertises
`ARTICORE_CAP_EFFECTIVE_CONTROL_RATE`. Runtime ABI 2.5 adds immutable per-side transport
capabilities to runtime creation. A dual runtime preserves a requested rate up to 500 Hz only when
both sides report `socketcanfd`, CAN-FD, and active BRS. That path sustained approximately 499 Hz
feedback for all 16 motors in a 30-second pure C++ streaming raw-MIT hardware test. Raw submissions
replace a separate capacity-one pending mailbox and do not wait for an in-flight physical batch;
the worker consumes only the newest complete generation at its next tick. Legacy callers, mixed
transports, and DM Device dual runtimes remain capped at 400 Hz. Single-side runtimes keep their
explicitly requested rate. Product SDKs should query and report the effective value instead of
assuming the requested rate was accepted unchanged.

Runtime ABI 2.6 advertises `ARTICORE_CAP_PER_CYCLE_MIT_TORQUE_LIMIT`. Before every native MIT arm
send—including cycles that repeat the latest capacity-one mailbox target—the worker reads the
newest native position and velocity feedback and computes
`Kp * (q_target - q_feedback) + Kd * (dq_target - dq_feedback) + tau_ff`. Each joint is bounded to
its configured `torque_limit`; when limiting is necessary, Kp, Kd, and feedforward torque
are multiplied by the same scale so their relative contributions are preserved. The complete arm
batch is rejected before transmission if the cached feedback is disabled, unavailable, or
non-finite, which then enters the existing protective fault-hold path. A stale-but-valid cached
sample remains usable while its age is reported diagnostically, so feedback timeout alone does not
stop the native sender. This keeps the protection at
the actual native control rate even when a publisher updates at only 100--500 Hz. The cumulative
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
