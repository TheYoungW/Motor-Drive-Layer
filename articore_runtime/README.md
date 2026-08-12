# Articore native safety runtime

This directory contains single- and dual-arm Articore product policy. It is a separate native
library in the Motor-Drive-Layer repository and depends on the generic motor C ABI without adding
Yunyi or Articore concepts to `libmotor_abi`.

The runtime owns one persistent worker thread. Its arm loop uses `steady_clock` absolute deadlines
at the configured control rate (500 Hz for Articore products), skips missed periods, and never
replays expired frames. Complete PV or MIT commands overwrite a capacity-one latest-value mailbox;
the control thread keeps transmitting the latest valid target through `ControllerGroup`. Only the
first successful full-group transmission of a newly submitted target refreshes the native
watchdog. The worker independently performs command timeout handling,
feedback and transport-health checks, safe-hold transmission, fault latching, linked disable, and
disable confirmation while Python is blocked or has stopped running.

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
SDK bindings create ABI 1.4 runtimes with `articore_runtime_create_ex()` and pass the generic
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
transport disconnect remain immediate hard faults.

Runtime ABI 1.3 added time-parameterized joint trajectories. The runtime stores exactly one active
trajectory as start/goal/time/profile state and computes the current position and velocity at each
control tick; it does not allocate a point FIFO. `MIN_JERK` uses the normalized quintic profile and
accounts for its 1.875 peak-velocity factor when choosing duration; `LINEAR` is the only other
profile. A direct joint command preempts the trajectory synchronously, a new trajectory replaces
the old one, and terminal status remains queryable as `COMPLETED`, `PREEMPTED`, `FAILED`, or
`CANCELED`. Enabling always seeds the mailbox from complete fresh motor feedback, while disable,
fault, recovery, and close clear both the old target and active trajectory.

When an arm enters safe hold, the runtime snapshots every arm motor's current position from the
non-blocking feedback cache. The snapshot is accepted only when every feedback entry is fresh,
finite, enabled, and fault-free; otherwise the runtime enters latched `FAULT` instead of replaying
an old motion target. PV safe hold uses the captured positions with a dedicated low velocity limit.
MIT safe hold uses the captured positions, zeros velocity and feedforward torque, and substitutes
product safety Kp/Kd. Grippers keep their independently generated low-stiffness safety targets.
The same persistent worker owns each configured product gripper's
`IDLE -> MOVING -> CONTACT -> HOLDING -> OVERLOAD_RETREAT` state machine. It maps public 0..1000
opening targets to motor position, ramps closing motion with the normal MIT gains, detects contact
from torque plus a position-motion window and target error, then switches to a low-gain hold with
zero feedforward torque. Sustained overload produces a rate-limited bounded retreat. No Python
gripper control loop is involved in the native dual-arm path.

In `FAULT`, arms are always linked-disabled while the product setting chooses whether grippers
keep the last safe target or are disabled. A failed gripper hold falls back to individual motor
disable attempts. An explicit `disable()` disables every held gripper without clearing the fault;
recovery confirms fresh disabled feedback before returning to `READY`. If no complete arm safety target exists, command failure enters
`FAULT` instead of sending an empty arm hold.

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
