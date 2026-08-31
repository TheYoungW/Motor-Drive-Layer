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

- package version: `0.24.0`
- Runtime ABI: `12.0` / `0x000C0000`

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
500 Hz control cycle it refreshes the final endpoint P and dynamically updates
only the Motor V envelope. A replacement sends the new final P directly while
preserving the current V ramp state; it does not generate intermediate P points.
`speed=1..100` time-scales its motion limits. Without a user override, the
100% J1..J7 velocity limits are `[180,180,180,225,225,225,225] deg/s` and
acceleration limits are `[450,450,900,900,900,900,900] deg/s^2`. Optional
`set_max_speed()` and `set_max_acceleration()` values become the common
14-joint 100% base; passing 0 clears that override and restores the per-joint
defaults. Scaling multiplies velocity by `s` and acceleration by `s²`. Runtime
derives each POS_VEL `V` from the effective limits, remaining distance and the
current V ramp state instead of sending a fixed maximum continuously.
MIT product mode exposes two position-only endpoint methods. Ordinary
`set_joint_mit()` sends each newest complete 14-joint endpoint directly, with
fixed J1..J7 `Kp=[15,15,12,12,8,7,6]` and
`Kd=[0.8,0.8,0.7,0.7,0.5,0.5,0.4]`; it does not generate intermediate
position references. `set_joint_mit_fast_follow()` is the high-frequency
teleoperation method: users provide only the newest joint angles, while
Runtime applies fixed J1..J7 `Kp=[190,190,100,100,70,60,50]`,
`Kd=[4.55,4.50,2.50,2.50,0.70,0.60,0.50]`, and an internal 100-percent
(5 rad/s) position-reference step limit. The former MIT function with a public
speed percentage remains an ABI-compatibility symbol only and new SDKs must not
expose it.
Linear/Circular own independent trajectory velocity, acceleration, jerk,
timing and synchronization constraints. Users provide path and time;
trajectory acceleration and jerk remain internal Runtime policy.
Pose callers use the pure `solve_ik(left_pose, right_pose)` query to obtain 14
joint angles and pass them to ordinary PV or MIT. The compatibility
`set_pose()` symbol solves IK once and atomically installs that endpoint through
the Runtime's current ordinary PV or MIT mode; it is not a trajectory planner.
Linear constructs a true Cartesian line from the current/start FK pose: XYZ is
linearly interpolated, orientation follows shortest-path quaternion SLERP, and
the first pose is solved only from the current planned joints while each later
pose is solved only from the preceding joint solution. Linear/Circular path IK
does not use fallback or random seeds. A null-space posture term keeps each
solution near its seed, while the preceding joint step predicts a half-step-
ahead preferred posture for the next solve. This biases redundant joints toward
continuous velocity instead of +/- chatter without rejecting a reversal that
the Cartesian path actually requires. Only an unsolved target or a true branch
jump above 0.35 rad fails. Geometry is
sampled at 2 mm / 0.1 rad or better, then one global quintic time law generates
fixed 10 ms internal real-time-PV knots. Thus `duration_s=3` normally produces
300 segments and 301 points. Runtime linearly resamples adjacent knots and
sends the resulting reference at 500 Hz, without applying the ordinary-PV
endpoint step generator. Per-cycle P changes smaller than one Damiao 16-bit
position-feedback quantum (25/65535 rad, about 0.02186 degrees) accumulate
against the last effective P; Runtime repeats that P until the threshold is
reached and always forces the exact endpoint without shortening duration.
POS_VEL P itself remains float32. The internal POS_VEL V limit follows the
current planned joint speed, bounded by the product ceiling, so slow
trajectories do not repeatedly chase discrete P targets at 3 rad/s.
Circular constructs the directed circle through start/via/end, samples the arc
at 2 mm / 0.1 rad or better, applies shortest-path SLERP through the via
orientation, and uses the same global quintic/10 ms real-time-PV chain.
Physical arrival may be later due to PV limits and feedback stability. Linear
and Circular check their 10 ms joint differences against speed 50 and their
trajectory acceleration limits, automatically stretching the reference duration to a complete
10 ms sample when needed.
The public `set_joint_pv()` command is the ordinary endpoint interface.
Real-time PV is internal-only and can be selected only by a validated finite
Linear/Circular trajectory; its 100 Hz plan knots are resampled on the
500 Hz Runtime command clock. No raw or streaming PV entry point is exported.
Automatic approach stays inside that trajectory execution path.
Linear and Circular require PV product mode; `set_pose()` supports either
ordinary PV or direct ordinary MIT. Joint point-to-point trajectory planning is
not part of the public Runtime API.

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
