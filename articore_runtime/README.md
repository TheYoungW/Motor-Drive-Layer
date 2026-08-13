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
- `runtime_safety.cpp`: feedback/transport supervision, protective hold, fault policy, and health snapshots.
- `runtime_trajectory.cpp`: single-slot trajectory lifecycle and time-parameterized profiles.
- `runtime_worker.cpp`: persistent absolute-deadline scheduler and safety event dispatch.

The split does not create additional shared libraries or independent state machines; it only gives
the existing state owner smaller compilation units while preserving its ABI and lock ordering.

The runtime owns one persistent worker thread. Its arm loop uses `steady_clock` absolute deadlines
at the configured control rate (500 Hz for Articore products), skips missed periods, and never
replays expired frames. Complete PV or MIT commands atomically overwrite a capacity-one
latest-value mailbox; if A, B, and C arrive before the next tick, only C is sent. The 500 Hz
control path reads the mailbox storage directly without copying a command queue or allocating a
per-tick snapshot, and keeps transmitting the latest valid target through `ControllerGroup`. Streaming
commands must be refreshed by the caller and are covered by the native watchdog; explicit
persistent setpoints and completed trajectory endpoints remain active until replaced. The worker
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
user targets and generated trajectory commands before transmission, but are not applied to measured
position, velocity, or torque. Feedback monitoring checks finite values, freshness, transport
health, motor status, and unexpected disable; mechanical feedback protection requires separately
defined thresholds, tolerance, and persistence rather than reusing URDF command limits.
Transient missing or stale feedback is counted but does not cancel an active trajectory until the
configured consecutive-failure threshold is reached. Motor fault status, unexpected disable, and
transport disconnect remain immediate hard faults, but a hard fault does not automatically torque
off unrelated healthy motors.

Runtime ABI 1.3 added time-parameterized joint trajectories. The runtime stores exactly one active
trajectory as start/goal/time/profile state and computes the current position and velocity at each
control tick; it does not allocate a point FIFO. `MIN_JERK` uses the normalized quintic profile and
accounts for its 1.875 peak-velocity factor when choosing duration; `LINEAR` is the only other
profile. Exactly one trajectory may be active. Another trajectory or a direct joint command is
rejected by the legacy start entry point until it completes; there is no trajectory waiting queue
and ordinary direct commands cannot preempt it. Runtime ABI 1.7 adds an explicit opt-in smooth
replacement entry point and explicit cancellation without changing that legacy behavior. Disable,
emergency stop, close, communication failure, and other safety faults may still cancel or fail it.
Terminal status remains queryable as `COMPLETED`, `PREEMPTED`, `FAILED`, or `CANCELED`, and the
result history is bounded to 64 entries. Enabling always seeds the mailbox from complete fresh
motor feedback, while disable, fault, recovery, and close clear both the old target and active
trajectory.
After a trajectory reaches `COMPLETED`, its exact endpoint remains an explicit internal trajectory
hold and is transmitted at the normal control rate. The user-command watchdog does not time out
this native hold.

Runtime ABI 1.5 separates command update lifetime from physical motion duration. The legacy direct
submission entry points remain `STREAMING`: callers must refresh them before `command_timeout_ms`.
The new `_ex` entry points accept either `ARTICORE_COMMAND_STREAMING` or
`ARTICORE_COMMAND_HOLD_UNTIL_REPLACED`. Product SDK one-shot position APIs use the latter, so a slow
move may take longer than the watchdog timeout while the native control thread keeps transmitting
its latest setpoint. Real-time servo loops use `STREAMING`, so a stalled caller still enters
`SAFE_HOLD`. Persistent MIT commands require zero target velocity and zero feedforward torque;
time-parameterized MIT motion should use the native trajectory API.

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

Runtime ABI 1.7 adds non-blocking trajectory management while retaining a single fixed-size task
slot. `articore_runtime_start_joint_trajectory_ex(..., SMOOTH_REPLACE)` atomically samples the
active minimum-jerk trajectory at one `steady_clock` instant and builds the replacement from that
position and velocity; no old trajectory frame can be sent after the replacement call returns.
The replaced ID becomes `PREEMPTED`. Invalid replacements leave the active task untouched, and
`LINEAR` replacement is rejected because it cannot preserve boundary velocity. The replacement
duration is checked against every configured position and velocity limit before installation.
`articore_runtime_cancel_trajectory(id)` atomically terminates the full dual-arm task as
`CANCELED` and installs a current-position, zero-velocity, zero-feedforward internal hold; fresh
feedback is preferred, with the last successfully sent target as a bounded fallback. The hold is
sent on the normal control loop and does not trigger the user-command watchdog. There is still no
FIFO, no background task accumulation, and no partial per-arm cancellation.

Runtime ABI 1.8 separates trajectory reference generation from measured completion. During
`PROFILE`, each control tick compares every joint's actual position with its current reference;
following error must remain above the configured limit for the configured persistence time before
the trajectory fails, so one noisy sample is not treated as a fault. When profile time expires,
the Runtime enters `SETTLING` and keeps sending the exact final reference with zero target velocity.
It reports `COMPLETED` only after every joint's measured position and velocity remain within their
dedicated trajectory tolerances for the complete stable-time window. Leaving tolerance resets that
window. Failure to converge before the settling timeout returns `FAILED` with channel, motor name,
CAN ID, measured errors, and thresholds. These trajectory execution tolerances are independent of
URDF command limits and can be configured before connect with
`articore_runtime_configure_trajectory_execution()`.

Runtime ABI 1.9 adds `ARTICORE_TRAJECTORY_SMOOTH_REPLACE_OR_HOLD` and the structured
`articore_runtime_start_joint_trajectory_report()` entry point. A feasible minimum-jerk
replacement preserves the old reference position, velocity, and acceleration. If replacement
cannot satisfy a position, velocity, or boundary constraint, the Runtime holds the control-path
lock, cancels the old trajectory, captures complete fresh enabled feedback for every arm joint,
synchronously transmits a full dual-arm current-position hold, and installs that hold as the
persistent mailbox value. The call returns
`ARTICORE_TRAJECTORY_START_REPLACEMENT_REJECTED_HELD`; this is a successful safety outcome, not a
global Runtime fault. MIT fallback holds use zero velocity and feedforward torque with product
safety Kp/Kd. DM POS_VEL has no signed target-velocity field, so PV uses a stationary position
reference with the configured low safe velocity limit. Missing/stale feedback, a hard-limit
feedback violation, or failure to transmit the fallback hold returns `FAULTED` and latches FAULT.

The additive `articore_runtime_configure_joint_safety_limits()` call keeps the original joint
configuration ABI stable while separating mechanical hard limits from normal-operation soft
limits. Feedback may be outside a soft limit while still inside the hard limits: stationary hold
and trajectories directed back into the safe region remain legal, but outward motion does not.
Within each soft-limit braking zone, outward trajectory speed is constrained both by the zone and
by `v <= sqrt(2 * braking_acceleration * distance_to_soft_limit)`. Only fresh feedback beyond a
configured hard limit is treated as a hard-limit safety fault; ordinary feedback values are not
compared with command velocity or torque limits.

Runtime ABI 1.10 adds `articore_runtime_set_gripper_commands()`. Each complete active-gripper
transaction contains only `opening`, normalized `speed`, and a stable LOW/NORMAL/HIGH
`force_level`; all fields become visible to the persistent worker under one command lock. Speed
uses the same product-independent scale as opening: 1000 means the maximum gripper speed calibrated
in the product motor descriptor. Both opening and closing advance from the previous native command
position through the same bounded ramp, so neither direction jumps directly to its endpoint.

Product bindings configure all three force levels before connect with
`articore_runtime_configure_gripper_force_profiles()`. A profile maps the public level to contact
and overload torque thresholds plus moving and holding MIT gains. The contact motion window,
stall displacement, minimum target error, contact/overload persistence, hold offset, retreat
distance, and retreat retry interval remain fixed in the product motor descriptor and cannot be
overridden by a per-motion command. Changing speed or force level during motion is atomic; a
force change resets only threshold-dependent contact evidence while motion continues. Existing
contact detection, low-gain holding, overload retreat, feedback supervision, and whole-Runtime
safety-state integration remain active. The legacy opening-only call is preserved and now uses the
same bidirectional ramp with maximum speed and the NORMAL force profile.

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
gripper state machine and MIT output run at the same `control_hz` as the arm (500 Hz for Yunyi).
Only `SAFE_HOLD` and protective `FAULT` holding use `safe_hold_hz` (normally 100 Hz). The legacy
`gripper_control_hz` config field remains in the ABI layout but no longer down-samples normal
gripper control.

Operational faults use protective fault hold rather than linked torque-off. One missing feedback
sample only increments the failure counters: arms continue their current 500 Hz output and a
gripper retransmits its last successful safe output. At the configured consecutive-failure
threshold, active trajectories stop and both arms enter protection. The runtime captures a fresh
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

The controller feedback callback introduced in runtime ABI 1.3 matches
`motor_controller_request_feedback_all_ex()`: it returns a stable motor error code and fills the
expected/received/missing counts plus missing motor IDs. Disable confirmation and safety diagnostics
consume those fields directly; diagnostic error text is retained for logs but is never parsed to
choose safety behavior.

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
