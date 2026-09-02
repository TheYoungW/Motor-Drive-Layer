# Motor Drive Layer

Native C++ control stack for the Yunyi dual-arm product.

The `main` branch supports one production path: Linux SocketCAN-FD+BRS on
`can-left` and `can-right`. The earlier multi-transport/Damiao-SDK architecture
is retained separately on the `legacy-multi-can-damiao-sdk` branch.

## Architecture

```text
Articore SDK
    -> Yunyi Runtime C ABI
        -> 500 Hz product Runtime and safety state
            -> native C++ Motor core
                -> left/right Controller workers
                    -> SocketCAN-FD+BRS frames

can-left/right receive threads
    -> Motor feedback cache
        -> Runtime state/health snapshot
            -> Articore SDK
```

The public wheel contains only `libarticore_runtime.so`. Runtime calls the Motor
core directly in the same C++ process; there is no intermediate Motor C ABI and
no Python implementation in this repository.

## Current contract

- package version: `0.29.1`
- Runtime ABI: `15.0` / `0x000F0000`

Runtime health includes a product-order snapshot for every installed Motor,
with role, CAN ID, feedback age, status, issue bits, and a scope that separates
isolated Motor feedback loss from left/right/both-channel receive stalls.
- ABI matching: exact
- product: `yunyi_v1_0`
- transports: `can-left`, `can-right`
- arm order: left J1..J7, right J1..J7
- optional paired grippers selected at Runtime creation

The SDK creates the product with
`articore_runtime_create_yunyi(mode, with_grippers, &runtime)`. Runtime owns the
Motor mapping, controllers, limits, models, TCP offsets, workers and resource
lifetime.

Ordinary joint PV is public latest-target-wins endpoint control. One
`set_joint_pv()` call replaces the preceding complete 14-joint target; Runtime
does not generate a finite quintic/septic profile or a Motion-ID task. On each
500 Hz control cycle it sends the final P directly and shapes only Motor V.
V rises and falls at the configured acceleration, is capped by the commanded
maximum speed, and starts braking from physical feedback distance. This is
online V shaping, not a finite point list or trajectory task. P receives no
position feed-forward offset. The distance-based braking curve continues to
zero at the endpoint; Runtime then arms a quiet V=0 hold after stable feedback.
The shared `set_speed_percent(1..100)` value time-scales its motion limits;
per-command ordinary PV percentages update that same value.
Without a user override, the
100% J1..J7 velocity limits are `[180,180,180,225,225,225,225] deg/s` and
acceleration limits are `[450,450,900,900,900,900,900] deg/s^2`. Optional
`set_max_speed()` and `set_max_acceleration()` values become the common
14-joint 100% base; passing 0 clears that override and restores the per-joint
defaults. Scaling multiplies velocity by `s` and acceleration by `s²`. Runtime
selects each POS_VEL `V` from the acceleration ramp and feedback-distance
braking limit instead of sending one fixed maximum continuously.
MIT product mode exposes exactly two methods. Standard
`set_joint_mit(q, dq, kp, kd, tau_ff)` accepts every field of a complete
14-joint MIT frame from the user. Each new frame atomically replaces the old
one; Runtime does no interpolation and the streaming watchdog still applies.
`set_joint_mit_fast(q)` is the angle-only high-frequency teleoperation method:
users provide only the newest joint angles, while
Runtime applies fixed J1..J7 `Kp=[190,190,100,100,70,60,50]`,
`Kd=[4.55,4.50,2.50,2.50,0.70,0.60,0.50]`, and an internal 100-percent
(5 rad/s) position-reference step limit with `dq=0` and `tau_ff=0`. Neither
method accepts `speed_percent` or creates a finite trajectory/Motion ID.
Linear/Circular retain independent internal base velocity, acceleration,
timing and synchronization constraints, but use the same shared Runtime speed
percentage as ordinary PV. Users provide only the path geometry; Runtime
automatically selects a safe duration, and trajectory base limits remain
internal policy.
Pose callers may use the pure `solve_ik(left_pose, right_pose)` query to obtain
14 joint angles without moving, then explicitly submit them through ordinary
PV or standard MIT. ABI 15 removes the former mode-neutral `set_pose()`
shortcut. The public `move_pose(side, target_pose)` method instead plans a
finite quintic Cartesian pose-to-pose motion.
Linear constructs a true Cartesian line from the current/start FK pose: XYZ is
linearly interpolated and orientation follows shortest-path quaternion SLERP.
An implicit Linear start remains the current planned pose. For an explicit
Linear/Linear Path/Circular start, Runtime uses ordinary Cartesian PTP
multi-seed endpoint IK, orders the reachable start branches by joint distance,
and accepts only a branch that can continuously finish the complete later path.
After that selection, each later pose is solved only from the preceding joint
solution and path IK does not use fallback, random, or extrapolated posture
seeds. A position-priority
null-space term keeps each solution near exactly its preceding seed. XYZ error
is limited to 0.5 mm and orientation residual to 0.035 rad. True branch jumps
above 0.35 rad and repeated visible +/- joint-direction chatter are rejected;
sub-1 mrad sign changes are treated as numerical or local-extremum noise.
Geometry is sampled at 2 mm / 0.1 rad or better. A quintic time law generates
adaptive 4..50 ms trajectory-PV knots, additionally bounded by scaled reference
velocity, acceleration, 0.02 rad adjacent joint step and linearization error.
At 50%, a comparable path takes about twice as long as at 100%. Runtime linearly
resamples adjacent knots and sends the resulting reference at 500 Hz, without
applying the ordinary-PV endpoint step generator. Per-cycle P changes smaller
than one Damiao 16-bit position-feedback quantum (25/65535 rad, about 0.02186
degrees) accumulate against the last effective P; Runtime repeats that P until
the threshold is reached and always forces the exact endpoint.
POS_VEL P itself remains float32. The internal POS_VEL V limit follows the
current planned joint speed, bounded by the product ceiling, so slow
trajectories do not repeatedly chase discrete P targets at 3 rad/s.
Circular constructs the directed circle through start/via/end, samples the arc
at 2 mm / 0.1 rad or better, applies shortest-path SLERP through the via
orientation, and uses the same quintic adaptive trajectory-PV chain. Physical
arrival may be later due to PV limits and feedback stability. Linear and
Circular automatically time-parameterize against the shared percentage of the
internal 1 rad/s velocity and 6 rad/s^2 acceleration bases.
Multi-pose Linear preserves every declared internal corner and inserts no
Cartesian fillet. Each sharp boundary uses its own rest-to-rest quintic time law
so the TCP reaches the corner without an instantaneous non-zero velocity change.
The public `set_joint_pv()` command is the ordinary endpoint interface.
`TrajectoryPv` is internal-only and can be selected only by a validated finite
Linear/Circular trajectory; its adaptive time-stamped knots are resampled on the
500 Hz Runtime command clock. No raw or streaming PV entry point is exported.
Automatic approach stays inside that trajectory execution path.
The approach and complete path are preplanned before either can move. They run
as one cancelable motion task with a physical-feedback stability barrier at the
explicit start, and `motion_arrived` becomes true only after the final endpoint
is physically stable.
`move_pose`, Linear and Circular require PV product mode. Their public calls are
nonblocking and return no Motion ID. Runtime accepts only one active finite
Cartesian motion; `motion_arrived` in the product state reports physical
completion, and `stop_motion()` stops the active motion. Applications own wait,
fault and timeout policy. Joint point-to-point trajectory planning is not part
of the public Runtime API.

See [the Runtime reference](articore_runtime/README.md) for the current API.

## Build and test

```bash
cmake -S . -B builds/dev -DCMAKE_BUILD_TYPE=Release
cmake --build builds/dev -j
ctest --test-dir builds/dev --output-on-failure
```

Hardware diagnostics are built explicitly and are never registered in CTest.
They may enable or move the robot and must only be run with a guarded workspace.

## Packaging

```bash
python -m build packaging/pypi --wheel
```

Publishing is a separate release action. Building and testing local changes do
not require uploading to PyPI.
