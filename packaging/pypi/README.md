# motor-drive-layer native payload

This distribution contains the compiled Motor-Drive-Layer libraries only:

- `libmotor_abi.so`: generic transport, controller, motor and Damiao C ABI.
- `libarticore_runtime.so`: fixed Yunyi dual-arm Runtime, robot model and
  gravity-compensation C ABI.
- Redistributable DM Device runtime dependencies for the target platform.

It intentionally installs no Python module and exports no Python API. Python product SDKs own
their `ctypes` declarations, value types and user-facing interfaces. In particular,
Articore-SDK locates this distribution through `importlib.metadata` and calls the stable native
ABI without importing `motor_drive_layer`.

The supported product entry point is `articore_runtime_create_yunyi(mode,
with_grippers)`. Product clients do not pass a product identifier, Controller,
Motor handle, mapping table, gripper profile, gravity binding, or control rate.
Product trajectories are likewise planned and executed entirely by the C++
Runtime through the stable trajectory C ABI; the wheel contains no Python
interpolator or realtime playback loop.

Version 0.10.27 doubles the built-in `yunyi_gripper_v1` command strength at
every public force level (1 through 10), without changing the product C ABI.

Version 0.10.28 adds the ABI 2.25 product gripper v2 command with protected and
direct (no contact/stall or overload-retreat) modes and a 0-through-10 strength
scale. Runtime-wide safety remains active in both modes.

Version 0.10.29 guarantees that Yunyi grippers remain in MIT mode regardless
of whether the fourteen arm joints use PV or MIT control.

Version 0.10.30 multiplies both Kp and Kd by ten in direct gripper mode only;
protected mode and zero strength remain unchanged.

Version 0.10.31 adds native asynchronous Cartesian point-to-point motion for
the Yunyi product Runtime. `articore_runtime_move_pose()` performs C++ IK and
quintic safety validation before returning a motion id. New valid point targets
replace the current point motion atomically; rejected targets leave it running.
The same release also adds Cartesian-linear motion with straight XYZ
interpolation and shortest-path quaternion SLERP. Sequential native IK validates
the complete path before its precomputed joint polynomials enter the realtime
worker. PTP and linear targets may replace each other; the existing
multi-waypoint trajectory API remains strict and ordered.

Version 0.10.31 also includes three-pose circular motion. Start, via and end
XYZ values define the traversed arc, while their attitudes are joined with
piecewise shortest-path quaternion SLERP. Duplicate, collinear, start-mismatch,
unreachable, discontinuous and over-limit paths are rejected before install.
PTP, linear and circular Cartesian product motion are all PV-only; MIT product
Runtimes reject these calls without replacing an active motion.
Trajectory and Cartesian `COMPLETED` now means feedback-confirmed physical
arrival, not only expiration of the planned duration. The Runtime holds the
final setpoint while it requires consecutive fresh position/velocity samples;
an internal arrival timeout marks the motion failed and writes diagnostics to
health without switching the Motors into a fault mode.

Version 0.10.32 adds `articore_runtime_move_circular_v2()`. The caller supplies
only via and end poses; Runtime reads the start from its current planned
reference and installs the replacement in one native command transaction.
It never uses a lagging feedback pose or requires an SDK `get_pose()` round
trip. The legacy three-pose symbol remains available, and MIT Runtime instances
reject v2 before replacing an active motion.

Version 0.10.33 makes product Cartesian endpoint IK deterministic and more
robust. PTP endpoints use a fixed-seed 1000-retry global search. Linear motion
removes its redundant isolated endpoint solve, retains local seeded IK for
continuous samples, and applies the global fallback only at the endpoint;
circular endpoints use the same policy. Failed planning still leaves the
currently active motion untouched.

The required Pinocchio C++ template implementations are compiled into
`libarticore_runtime.so` with hidden visibility. The installed runtime has no dynamic dependency
on Pinocchio or Boost, so a ROS 2 `LD_LIBRARY_PATH` cannot substitute an incompatible robotics
library.

The payload is installed under:

```text
motor_drive_layer_native/
└── lib/
    ├── libmotor_abi.so
    ├── libarticore_runtime.so
    └── dm_device/
```

Use the headers and CMake package from the native SDK artifact for C/C++ development. Install
Articore-SDK for the supported Python product interface.
