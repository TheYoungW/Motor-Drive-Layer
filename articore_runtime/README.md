# Yunyi product Runtime

`libarticore_runtime.so` is the only public native library. Runtime ABI 11.3 is
an exact contract: the SDK must require `articore_runtime_abi_version() ==
0x000B0003` and bind only the declarations in `articore/runtime_abi.h`.

## Ownership

`articore_runtime_create_yunyi(mode, with_grippers, &runtime)` constructs the
complete product:

- `can-left` and `can-right` SocketCAN-FD+BRS transports;
- fourteen arm Motors and the optional paired grippers;
- Motor/Controller/ControllerGroup ownership and workers;
- product direction, limits, gains and feedback mapping;
- robot models, TCP offsets, Cartesian IK and trajectory execution;
- safety state, health, maintenance and shutdown.

The SDK does not construct or pass Motor handles, Controllers, transport
objects, joint tables or product bindings.

## Product API groups

- Lifecycle: `connect`, `disconnect`, `enable`, `disable`, `estop`, `recover`.
- Maintenance: `configure_mode`, `clear_faults`, `set_zero`.
- Joint control: `set_joint_pv`, `set_joint_mit`, `submit_mit_frame`.
- Ordinary PV: each command replaces the preceding complete joint endpoint.
  Runtime advances POS_VEL `P` online at 500 Hz; `speed_percent` selects its
  `0..2 rad/s` reference-speed scale while Motor `V` is an independent ceiling.
- PV acceleration: `set_max_acceleration` and `get_max_acceleration` use
  `rad/s^2` with `0.01` resolution and shape ordinary PV only. Complete
  trajectories own internal velocity, acceleration and jerk constraints;
  callers provide positions/path and time, not trajectory derivative limits.
- Native trajectories: joint trajectory plus Linear and Circular motion. PV
  follows the supplied/generated finite point list through internal real-time
  PV at speed 50; MIT joint trajectories remain direct quintic mode.
- Grippers: one paired `set_grippers` call using opening `0..1000`, strength
  `0..10`, and protected/direct mode.
- State: one coherent cached `get_state`, `get_pose`, joint limits and health.
- Product functions: gravity compensation, bimanual follow and TCP offset.

All fourteen-joint arrays use `left J1..J7, right J1..J7`. Public ABI values
never contain native Motor pointers.

## Motion semantics

`set_joint_pv(left, right, speed)` is ordinary latest-target-wins step control.
Runtime validates and atomically installs the complete 14-joint endpoint, then
advances the outgoing POS_VEL `P` reference toward it on the 500 Hz Runtime
control clock. A newer call replaces only the endpoint and preserves the current
reference position and velocity, so acceleration, braking and reversal stay
continuous. This is an online step generator, not a finite quintic/septic plan,
queue item or Motion ID. `speed=0..100` controls the P reference-speed scale;
Motor `V` remains a separate product drive ceiling. The pure
`articore_runtime_solve_ik(left_pose, right_pose, positions, 14)` query uses one
planned-reference snapshot (or fresh connected feedback before enable), the
active TCP and product limits to return fixed
left-J1..J7/right-J1..J7 joint order. It never enables Motors, sends commands or
changes the queue. Callers pass that result to `articore_runtime_set_joint_pv`.
The `articore_runtime_set_pose` compatibility symbol solves IK once and installs
the result through the currently selected ordinary PV or MIT mode. It has no
motion ID, status or cancellation API and is not a trajectory planner. Endpoint IK retains
the `1e-4` SE(3)
tolerance, reuses each arm's Pinocchio model and limits global fallback to an
8 ms soft steady-clock budget. Timeout or either-side failure leaves the active
target unchanged. Linear and Circular are asynchronous FIFO trajectory tasks.
Linear interpolates XYZ on the Cartesian line and orientation with true
shortest-path quaternion SLERP. The first path pose uses only the current
planned joints as its IK seed; each later pose uses only the preceding IK
result. Linear and Circular path IK never retry from fallback or random seeds,
and a null-space posture term keeps each solve near that preceding seed. The
preceding joint step also predicts a half-step-ahead preferred posture, biasing
the redundant solution toward continuous joint velocity instead of +/- chatter.
This is an optimization objective, not a reversal rejection rule: motion may
reverse whenever the Cartesian path requires it. Runtime rejects only a true
per-sample branch jump above 0.35 rad. Geometry is sampled at 2 mm / 0.1 rad or
better. Runtime applies one
global quintic time law and generates one internal real-time-PV Linear knot
every 10 ms. Runtime linearly resamples adjacent knots and sends the resulting
POS_VEL reference on its 500 Hz command clock, without applying the ordinary-PV
endpoint step generator. Circular builds
the directed circle through start/via/end, samples it at 2 mm / 0.1 rad or
better, and applies shortest-path SLERP through the via orientation. It uses
the same global quintic time law and 10 ms command period.
Contiguous Cartesian FIFO items hand off at
their planned shared endpoint without an extra 200 ms settling window when
physical tracking error is at most 0.04 rad; otherwise Runtime waits for safe
tracking recovery. Linear checks finite differences of its 10 ms joint
references against speed 50 and their own trajectory acceleration, automatically
stretching the duration to a complete 10 ms sample when necessary. Both paths
execute through internal real-time PV at speed 50.
Linear uses explicit `start_pose -> end_pose`; Circular uses explicit
`start_pose -> via_pose -> end_pose`. Runtime validates and plans the complete
task before installing it. Joint, Linear and Circular trajectories share one
Motion ID namespace and FIFO, and use `get_motion_status(id)`,
`cancel_motion(id)` and `cancel_all_motions()`. A joint trajectory returns its
Motion ID directly from `move_joint_trajectory()`. Cancelling a queued item removes
only that item; Runtime inserts and validates a native approach into its
immediate successor so execution cannot jump across the removed endpoint.
Cancelling the running item holds the last safe reference and cancels its
dependent queue tail, whose planned starts are no longer valid.
Completed, cancelled and faulted tasks remain queryable in bounded history.

The same Linear API also accepts 2 to 64 poses as one atomic path. Two poses
retain straight-Line behavior. With three or more poses, Runtime applies a
10 mm Cartesian fillet to each valid internal corner and automatically reduces
the radius on short adjacent segments so blends cannot overlap. One global
quintic time law covers the complete path, one Motion ID owns it, and
`segment_duration_s` retains the duration meaning for each original segment.

Linear and Circular accept `duration_s` instead of a speed percentage. Both
select `ceil(duration_s / 0.010)` execution segments. The reference list has
one more point and nominally spans the requested duration. Physical completion
may be later. Linear and Circular geometry enforce at least 2 mm / 0.1 rad
sampling. Ordinary PV command speed values remain `0..100` and select a
`0..2 rad/s` online P reference-speed scale. Persistent maximum acceleration
defaults to `6.00 rad/s^2`; its configured range is `0.01..8.00 rad/s^2`.
Runtime applies it only to ordinary PV (including PV `set_pose()`).
Joint/Linear/Circular own separate trajectory velocity, acceleration, jerk,
timing and synchronization constraints; changing ordinary PV acceleration does
not change any trajectory. Those trajectory derivative limits are not exposed;
callers provide positions/path and time. MIT commands and MIT joint trajectories remain
unchanged. The Damiao POS_VEL `V` drive ceiling remains independent at `3 rad/s`.

The public `set_joint_pv()` command is the ordinary latest-target-wins step PV
interface.
`RealtimePv` is an internal trajectory execution type: only a finite validated
Joint/Linear/Circular plan may install it, its planner-knot period must be
exactly 10 ms, and Runtime linearly resamples those knots on the 500 Hz command
clock. No raw or streaming PV symbol is exported through the C or C++ product
API.

Linear and Circular require PV product mode. Automatic approach is part of the
same internal trajectory-PV point sequence. `set_pose()` supports the current
ordinary PV or MIT mode, and MIT joint trajectories remain available.

## State and diagnostics

`articore_runtime_get_state()` returns one cache snapshot with positions,
velocities, torques, temperatures, actual enabled masks and optional gripper
state. It performs no CAN request. `articore_runtime_get_health()` is the sole
operation/safety diagnostic surface.

`ArticoreSafetyHealth::motor_feedback` reports every installed Motor in product
order, including its stable role, CAN ID, feedback age, update counter, state
availability, status code, and feedback issue bits. `feedback_issue_scope`
distinguishes one Motor, several Motors with a still-active bus, the left or
right receive path, and both receive paths. Side `last_error` strings aggregate
all affected Motor roles instead of retaining only the last failure.

Every public structure must set `struct_size` to its exact `sizeof(...)`.
ABI 11.3 does not branch on older layouts or capability bits.

## Shutdown

`disconnect()` is the hardware lifecycle operation: stop control, disable and
confirm the product, close transports and join workers. It is idempotent.
`free()` releases the opaque C handle after disconnect; it is not a second
business lifecycle operation.
