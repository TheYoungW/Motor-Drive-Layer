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

- package version: `0.22.0`
- Runtime ABI: `11.3` / `0x000B0003`

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

Ordinary joint PV applies one common seventh-order rest-to-rest progress law
to all 14 joints. Reference updates and physical POS_VEL packets both run at
100 Hz; each point is sent once. Separate 500 Hz Runtime scheduling retains
safety, feedback and command-watchdog supervision. Each command maps
`speed=0..100` directly onto
`0..2 rad/s`; there is no second
persistent maximum-speed cap. Persistent maximum acceleration uses physical
units with `0.01` resolution, accepts `0.01..8.00 rad/s^2`, and defaults to
`6.00 rad/s^2`. Runtime selects one shared duration from velocity,
acceleration and its native jerk-shaping constraint, rounded up to a complete
10 ms sample. The current ABI has no separate jerk setting, so the native
policy uses `j_max = a_max / 0.10 s` (60 rad/s^3 at the default acceleration).
Joint PTP is the only point-to-point planner. Pose callers use the pure
`solve_ik(left_pose, right_pose)` query to obtain 14 joint angles and pass them
to `set_joint_pv()`. The legacy `set_pose()` symbol remains as a compatibility
shortcut for that same IK-to-joint-PTP chain. These changes do not affect MIT.
Linear constructs a true Cartesian line from the current/start FK pose: XYZ is
linearly interpolated, orientation follows shortest-path quaternion SLERP, and
each pose is solved by IK from the preceding joint solution. Geometry is
sampled at 2 mm / 0.1 rad or better, then one global quintic time law generates
fixed 10 ms ordinary-PV references. Thus `duration_s=3` normally produces 300
segments and 301 points. Each Linear reference is sent once through ordinary
PV at 100 Hz, with no second executor-side interpolation or step generator.
The separate 500 Hz Runtime scheduling continues safety and feedback work.
Circular constructs the directed circle through start/via/end, samples the arc
at 2 mm / 0.1 rad or better, applies shortest-path SLERP through the via
orientation, and uses the same global quintic/10 ms real-time-PV chain.
Physical arrival may be later due to PV limits and feedback stability. Linear
and Circular check their 10 ms joint differences against speed 50 and the
configured maximum acceleration, automatically stretching the reference duration to a complete
10 ms sample when needed.
The public `set_joint_pv()` command is the ordinary stepped/P2P interface.
Real-time PV is internal-only and can be selected only by a validated finite
Joint/Linear/Circular trajectory; no raw or streaming PV entry point is
exported. Automatic approach also uses ordinary PV. Linear, Circular and
`set_pose()` require PV product mode;
ordinary MIT and MIT joint trajectories are unchanged.

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
