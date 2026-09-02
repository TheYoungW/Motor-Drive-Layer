# Yunyi product Runtime

`libarticore_runtime.so` is the only public native library. Runtime ABI 15.0 is
an exact contract: the SDK must require `articore_runtime_abi_version() ==
0x000F0000` and bind only the declarations in `articore/runtime_abi.h`.

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
- Shared speed: `set/get_speed_percent` for ordinary PV and finite Cartesian motion.
- Joint control: `set_joint_pv`, optional ordinary-PV `set/get_max_speed` and
  `set/get_max_acceleration`, standard `set_joint_mit`, and angle-only
  `set_joint_mit_fast`.
- Ordinary PV: each command replaces the preceding complete joint endpoint.
  Runtime sends final P on the first 500 Hz frame and shapes only Motor V from
  the speed ceiling, acceleration limit, and physical remaining distance.
  The physical-distance braking curve falls to zero at the endpoint, then a
  confirmed V=0 hold finishes arrival.
  `speed_percent=1..100` time-scales the active limits.
  The default 100% J1..J7 limits are
  180/180/180/225/225/225/225 deg/s and
  450/450/900/900/900/900/900 deg/s^2. A positive `set_max_speed` or
  `set_max_acceleration` value becomes the common 14-joint 100% base; zero
  clears that override. Velocity scales by `s`, acceleration by `s^2`, and
  Motor `V` is selected per cycle from the acceleration ramp and physical
  feedback-distance braking limit.
- Standard MIT: `set_joint_mit(q, dq, kp, kd, tau_ff)` accepts every field of
  one complete 14-joint frame. A newer frame atomically replaces the old one;
  Runtime performs no interpolation and retains the streaming watchdog.
- Fast MIT: `set_joint_mit_fast(q, speed_percent=100)` is a high-frequency
  teleoperation endpoint accepting joint angles and a `0..100`
  reference-speed percentage. Runtime owns `dq=0`, `tau_ff=0`, and J1..J7
  `Kp=[190,190,100,100,70,60,50]`,
  `Kd=[4.55,4.50,2.50,2.50,0.70,0.60,0.50]`, and the internal 100-percent
  (5 rad/s) position-reference step base. 50 percent selects 2.5 rad/s and
  0 percent keeps the current reference. Standard MIT has no speed parameter;
  neither MIT method creates a Motion ID.
- Native trajectories: `move_pose`, Linear and Circular. They follow generated
  finite point lists through internal trajectory PV using the shared speed
  percentage.
- Grippers: one paired `set_grippers` call using opening `0..1000`, strength
  `0..10`, and protected/direct mode.
- State: one coherent cached `get_state`, including `motion_arrived`, plus
  `get_pose`, joint limits and health.
- Product functions: `stop_motion`, gravity compensation, bimanual follow and
  TCP offset.

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
In MIT product mode, callers choose one of exactly two methods. The standard
`articore_runtime_set_joint_mit` call submits user-owned q, dq, kp, kd and
feedforward torque as one atomic watchdog-protected streaming frame. The
angle-only `articore_runtime_set_joint_mit_fast_with_speed` call advances the
Runtime-owned reference at the supplied `0..100` percentage of the 5 rad/s
base with the fast gain profile, zero velocity and zero feedforward torque. The
compatibility `articore_runtime_set_joint_mit_fast` symbol selects 100 percent.
Neither creates a Motion ID.
`articore_runtime_move_pose` plans a finite quintic Cartesian pose-to-pose
motion for one side. Like Linear and Circular, it is a nonblocking PV-mode
command and follows the same single-active-motion contract.
Linear interpolates XYZ on the Cartesian line and orientation with true
shortest-path quaternion SLERP. The first path pose uses only the current
planned joints as its IK seed; each later pose uses only the preceding IK
result. Linear and Circular path IK never retry from fallback or random seeds,
and a position-priority null-space posture term keeps each solve near exactly
that preceding seed; no extrapolated posture is introduced. One required joint
reversal remains valid, while repeated small
direction reversals are rejected as IK chatter before the trajectory can be
installed. Runtime also rejects a true per-sample branch jump above 0.35 rad.
Path IK keeps XYZ error within 0.5 mm and permits at most 0.035 rad orientation
residual. Geometry is sampled at 2 mm / 0.1 rad or better. Runtime applies a
global quintic time law and automatically selects adaptive 4..50 ms knots that
satisfy the internal joint-reference velocity, acceleration, 0.02 rad adjacent
step and linearization-error limits. At
50%, velocity halves, acceleration becomes one quarter, and a comparable path
takes about twice as long. Runtime linearly resamples adjacent knots and sends the resulting
POS_VEL reference on its 500 Hz command clock, without applying the ordinary-PV
endpoint step generator. Circular builds
the directed circle through start/via/end, samples it at 2 mm / 0.1 rad or
better, and applies shortest-path SLERP through the via orientation. It uses
the same global quintic time law and adaptive knot policy.
Linear checks finite differences of its variable-duration
joint references against the shared percentage of their internal 1 rad/s velocity
and 6 rad/s^2 acceleration bases and automatically parameterizes time. Both
paths execute through internal
trajectory PV with the percentage captured when each path is submitted.
`articore_runtime_move_linear(runtime, side, start_pose, end_pose)` is the
unified finite Linear entry point. When `start_pose == NULL`, the Cartesian
line begins at the current planned pose. With a non-null start, Runtime plans
`current planned pose -> start_pose -> end_pose` as one atomic task. The
approach uses the same deterministic multi-seed endpoint IK policy as ordinary
Cartesian PTP. Reachable start branches are tried from nearest to farthest from
the current planned joints, and a branch is accepted only when its complete
following Linear path also plans continuously. Circular applies the same joint
selection to explicit `start_pose -> via_pose -> end_pose`. Runtime validates
the approach and complete path before installing either segment, so a later
unreachable sample or branch jump produces no movement. At the approach
boundary Runtime holds until physical Cartesian feedback confirms start_pose;
`stop_motion()` cancels the same task in either stage. Final
`motion_arrived=true` is published only after the real endpoint is stable. The
SDK still makes one Runtime call and never composes `move_pose + wait + path`.
If the current planned pose already matches the explicit start tolerance, the
approach is omitted. The public calls return no Motion ID and Runtime accepts
only one active finite Cartesian motion. A new call while one is active returns
busy instead of entering a queue. Applications read
`ArticoreProductState.motion_arrived`, monitor health, implement their own wait
and timeout policy, and call `articore_runtime_stop_motion()` when needed.

Explicit-start planning errors distinguish an unreachable start (no endpoint
IK branch), a reachable start for which no branch can continuously finish the
path, and the underlying later path failure such as an unreachable sample or
an IK branch jump.

The same Linear API also accepts 2 to 64 poses as one atomic path. Two poses
retain straight-Line behavior. With three or more poses, every declared
internal pose is preserved and no Cartesian fillet is inserted. Each sharp
segment boundary has its own rest-to-rest quintic time law, so the TCP reaches
the corner without attempting an instantaneous non-zero velocity direction
change. Runtime automatically computes the complete path duration.

Linear and Circular do not accept a duration. Callers provide only path
geometry, while the shared `set_speed_percent(1..100)` value selects the speed
for newly submitted paths. Each plan snapshots that value; changing it does
not retime a running path. Physical completion may be later
than the generated reference duration. Linear and Circular geometry enforce
at least 2 mm / 0.1 rad sampling. Ordinary PV command speed values update the
same shared percentage and time-scale either configured user base limits or
the fixed per-joint defaults. Linear/Circular keep separate internal trajectory
velocity, acceleration, timing and synchronization limits. Joint point-to-point
trajectory planning is not part of the public Runtime API.

The public `set_joint_pv()` command is ordinary latest-target-wins endpoint PV
interface.
`TrajectoryPv` is an internal trajectory execution type: only a finite validated
Linear/Circular plan may install it. Cartesian planners select adaptive 4..50 ms
time-stamped knots from geometry, velocity, acceleration, joint-step and
linearization limits, and Runtime linearly resamples those knots on the 500 Hz
command clock. Per-cycle P changes smaller than one Damiao 16-bit position-feedback
quantum (25/65535 rad, about 0.02186 degrees) accumulate against the last
effective command; Runtime repeats that P until the threshold is reached and
always forces the exact final endpoint. POS_VEL P
itself remains float32. The internal POS_VEL V field follows planned joint
speed while remaining bounded by the product drive ceiling. No raw or streaming
PV symbol is exported through the C or C++ product API.

`move_pose`, Linear and Circular require PV product mode. Automatic approach is
part of the same internal trajectory-PV point sequence.

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
ABI 15.0 replaces the public MIT surface with standard full-frame
`set_joint_mit(q, dq, kp, kd, tau_ff)` and angle-only
`set_joint_mit_fast(q, speed_percent=100)`. It removes the mode-neutral
`set_pose()` shortcut;
callers use `solve_ik()` followed by an explicit joint command, or use
`move_pose()` for a finite Cartesian motion. It retains `motion_arrived` in
`ArticoreProductState` and the simplified
nonblocking `move_pose`, `move_linear`, `move_linear_path`, `move_circular` and
`stop_motion` entry points. New SDKs use these functions and do not expose the
legacy Motion-ID/FIFO management surface.

## Shutdown

`disconnect()` is the hardware lifecycle operation: stop control, disable and
confirm the product, close transports and join workers. It is idempotent.
`free()` releases the opaque C handle after disconnect; it is not a second
business lifecycle operation.
