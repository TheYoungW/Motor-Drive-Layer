# Yunyi product Runtime

`libarticore_runtime.so` is the only public native library. Runtime ABI 13.0 is
an exact contract: the SDK must require `articore_runtime_abi_version() ==
0x000D0000` and bind only the declarations in `articore/runtime_abi.h`.

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
- Shared speed: `set/get_speed_percent` for ordinary PV, Linear and Circular.
- Joint control: `set_joint_pv`, optional ordinary-PV `set/get_max_speed` and
  `set/get_max_acceleration`, `set_joint_mit_direct`,
  `set_joint_mit_fast_follow`, `submit_mit_frame`.
- Ordinary PV: each command replaces the preceding complete joint endpoint.
  Runtime sends final P directly and shapes only the per-joint Motor V envelope
  while refreshing POS_VEL at 500 Hz. `speed_percent=1..100` time-scales the
  active limits. The default 100% J1..J7 limits are
  180/180/180/225/225/225/225 deg/s and
  450/450/900/900/900/900/900 deg/s^2. A positive `set_max_speed` or
  `set_max_acceleration` value becomes the common 14-joint 100% base; zero
  clears that override. Velocity scales by `s`, acceleration by `s^2`, and
  Motor `V` is derived per cycle from remaining distance and the V ramp state.
- Ordinary MIT direct: each complete 14-joint target replaces the previous
  endpoint and is sent without intermediate position-reference steps. Runtime
  owns J1..J7 `Kp=[15,15,12,12,8,7,6]` and
  `Kd=[0.8,0.8,0.7,0.7,0.5,0.5,0.4]`.
- MIT fast follow: a high-frequency teleoperation endpoint interface accepting
  joint angles only. Runtime owns J1..J7
  `Kp=[190,190,100,100,70,60,50]`,
  `Kd=[4.55,4.50,2.50,2.50,0.70,0.60,0.50]`, and the fixed internal
  100-percent (5 rad/s) position-reference step limit. The old MIT symbol with
  public `speed_percent` remains ABI-only compatibility and must not be exposed
  by a new SDK.
- Native trajectories: Linear and Circular motion only. Both follow the
  generated finite point list through internal real-time PV using the shared
  speed percentage.
- Grippers: one paired `set_grippers` call using opening `0..1000`, strength
  `0..10`, and protected/direct mode.
- State: one coherent cached `get_state`, `get_pose`, joint limits and health.
- Product functions: gravity compensation, bimanual follow and TCP offset.

All fourteen-joint arrays use `left J1..J7, right J1..J7`. Public ABI values
never contain native Motor pointers.

## Motion semantics

`set_joint_pv(left, right, speed)` is ordinary latest-target-wins endpoint control.
Runtime validates and atomically installs the complete 14-joint endpoint, then
sends that final POS_VEL `P` directly. Its 500 Hz control clock refreshes the
same endpoint and updates only Motor `V`. A newer call replaces the endpoint
and preserves the current V envelope. This is an ordinary endpoint controller,
not a finite quintic/septic plan, queue item or Motion ID. The shared
`set_speed_percent(1..100)` value time-scales the configured 100% user limits
when present, otherwise the fixed
per-joint velocity and acceleration defaults; Runtime
derives Motor `V` every cycle without generating intermediate P. The pure
`articore_runtime_solve_ik(left_pose, right_pose, positions, 14)` query uses one
planned-reference snapshot (or fresh connected feedback before enable), the
active TCP and product limits to return fixed
left-J1..J7/right-J1..J7 joint order. It never enables Motors, sends commands or
changes the queue. Callers pass that result to `articore_runtime_set_joint_pv`.
In MIT product mode, callers choose one of two position-only endpoint methods:
`articore_runtime_set_joint_mit_direct` sends the new target directly with the
ordinary low-gain profile, while
`articore_runtime_set_joint_mit_fast_follow` advances the Runtime-owned
reference at the fixed 100-percent limit with the fast-follow gain profile.
Both are persistent latest-target-wins commands refreshed on the 500 Hz Runtime
clock and neither creates a Motion ID. Only the fast-follow method generates
intermediate MIT position references.
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
better. Runtime applies one global quintic time law and automatically selects
the shortest complete 10 ms reference duration that satisfies the internal
joint-reference limits after applying the shared Runtime speed percentage. At
50%, velocity halves, acceleration becomes one quarter, and a comparable path
takes about twice as long. Runtime linearly resamples adjacent knots and sends the resulting
POS_VEL reference on its 500 Hz command clock, without applying the ordinary-PV
endpoint step generator. Circular builds
the directed circle through start/via/end, samples it at 2 mm / 0.1 rad or
better, and applies shortest-path SLERP through the via orientation. It uses
the same global quintic time law and 10 ms command period.
Contiguous Cartesian FIFO items hand off at
their planned shared endpoint without an extra 200 ms settling window when
physical tracking error is at most 0.04 rad; otherwise Runtime waits for safe
tracking recovery. Linear checks finite differences of its 10 ms joint
references against the shared percentage of their internal 1 rad/s velocity
and 6 rad/s^2 acceleration bases and automatically parameterizes time. Both
paths execute through internal
real-time PV with the percentage captured when each path is submitted.
Linear uses explicit `start_pose -> end_pose`; Circular uses explicit
`start_pose -> via_pose -> end_pose`. Runtime validates and plans the complete
task before installing it. Linear and Circular trajectories share one Motion
ID namespace and FIFO, and use `get_motion_status(id)`, `cancel_motion(id)` and
`cancel_all_motions()`. Cancelling a queued item removes only that item;
Runtime inserts and validates a native approach into its
immediate successor so execution cannot jump across the removed endpoint.
Cancelling the running item holds the last safe reference and cancels its
dependent queue tail, whose planned starts are no longer valid.
Completed, cancelled and faulted tasks remain queryable in bounded history.

The same Linear API also accepts 2 to 64 poses as one atomic path. Two poses
retain straight-Line behavior. With three or more poses, Runtime applies a
10 mm Cartesian fillet to each valid internal corner and automatically reduces
the radius on short adjacent segments so blends cannot overlap. One global
quintic time law covers the complete path, one Motion ID owns it, and
Runtime automatically computes the duration of the complete path.

Linear and Circular do not accept a duration. Callers provide only path
geometry, while the shared `set_speed_percent(1..100)` value selects the speed
for newly submitted paths. Each plan snapshots that value; changing it does
not retime a running or already queued path. Physical completion may be later
than the generated reference duration. Linear and Circular geometry enforce
at least 2 mm / 0.1 rad sampling. Ordinary PV command speed values update the
same shared percentage and time-scale either configured user base limits or
the fixed per-joint defaults. Linear/Circular keep separate internal trajectory
velocity, acceleration, timing and synchronization limits. Joint point-to-point
trajectory planning is not part of the public Runtime API.

The public `set_joint_pv()` command is ordinary latest-target-wins endpoint PV
interface.
`RealtimePv` is an internal trajectory execution type: only a finite validated
Linear/Circular plan may install it, its planner-knot period must be
exactly 10 ms, and Runtime linearly resamples those knots on the 500 Hz command
clock. Per-cycle P changes smaller than one Damiao 16-bit position-feedback
quantum (25/65535 rad, about 0.02186 degrees) accumulate against the last
effective command; Runtime repeats that P until the threshold is reached and
always forces the exact final endpoint. POS_VEL P
itself remains float32. The internal POS_VEL V field follows planned joint
speed while remaining bounded by the product drive ceiling. No raw or streaming
PV symbol is exported through the C or C++ product API.

Linear and Circular require PV product mode. Automatic approach is part of the
same internal trajectory-PV point sequence. `set_pose()` supports the current
ordinary PV or direct ordinary MIT mode.

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
ABI 13.0 does not branch on older layouts or capability bits. It removes the
Cartesian `duration_s` arguments and the former
`articore_runtime_move_linear_trajectory_with_point_count` compatibility
symbol, in addition to the already removed joint point-to-point trajectory
API.

## Shutdown

`disconnect()` is the hardware lifecycle operation: stop control, disable and
confirm the product, close transports and join workers. It is idempotent.
`free()` releases the opaque C handle after disconnect; it is not a second
business lifecycle operation.
