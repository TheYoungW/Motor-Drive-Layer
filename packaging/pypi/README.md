# motor-drive-layer native payload

This distribution contains the compiled Motor-Drive-Layer libraries only:

- `libarticore_runtime.so`: fixed Yunyi dual-arm Runtime, direct C++ Motor core,
  robot model and gravity-compensation C ABI.

It intentionally installs no Python module and exports no Python API. Python product SDKs own
their `ctypes` declarations, value types and user-facing interfaces. In particular,
Articore-SDK locates this distribution through `importlib.metadata` and calls the stable native
ABI without importing `motor_drive_layer`.

Motor publishes one PyPI project and one version. Each release contains an
x86_64 wheel and an aarch64 wheel under the same `motor-drive-layer` project;
pip selects the matching platform artifact. Runtime, Motor and SocketCAN-FD
are never split into separate PyPI distributions.

The supported product entry point is `articore_runtime_create_yunyi(mode,
with_grippers, runtime_out)`. Product clients do not pass a product identifier,
Controller, Motor handle, mapping table, gripper profile, gravity binding, or
control rate.
Product trajectories are likewise planned and executed entirely by the C++
Runtime through the stable trajectory C ABI; the wheel contains no Python
interpolator or realtime playback loop.

Version 0.12.1 / Runtime ABI 3.0 fixes loaded J4 tracking in native PV
Cartesian motion. The Yunyi product profile uses `KP_APR=100` and `KI_APR=0`
for both 4340P J4 Motors. Runtime reads the actual register value, writes and
stores the product value only when it differs, then verifies the readback; it
does not repeatedly erase Motor flash. MIT gains, other Motors and their
existing PV loop values are unchanged. This release also fixes the native
worker at its internal 500 Hz cadence, lets physically arrived PV joints enter
low-speed settling independently, applies a consistent 5 rad/s product
velocity ceiling, and adds opt-in native planned/reference/feedback tracing
for hardware diagnosis without adding a public ABI.

Version 0.12.0 / Runtime ABI 3.0 removes the temporary dual factory ABI. The
only factory is `articore_runtime_create_yunyi(mode, with_grippers,
runtime_out)`: it returns an `ArticoreOperationError` and writes the opaque
handle through `runtime_out`. The two-argument pointer-returning factory and
the temporary `_v2` symbol are not exported. Articore-SDK and this Runtime are
released together and require ABI 3.0, so bindings do not branch between
factory signatures.

Version 0.11.0 / Runtime ABI 2.39 simplifies the native architecture to the
only supported Yunyi product. `libarticore_runtime.so` directly owns the
layered C++ Motor core and its two SocketCAN-FD+BRS channels. The separate
Motor C ABI, caller-assembled Runtime factories, classic SocketCAN, serial and
DM Device SDK transports, and vendor binary payloads are removed. Product API
calls remain on the stable Runtime C ABI used by Articore-SDK.

Version 0.10.27 doubles the built-in `yunyi_gripper_v1` command strength at
every public force level (1 through 10), without changing the product C ABI.

Version 0.10.28 adds the ABI 2.25 product gripper v2 command with protected and
direct (no contact/stall or overload-retreat) modes and a 0-through-10 strength
scale. Runtime-wide safety remains active in both modes.

Version 0.10.29 guarantees that Yunyi grippers remain in MIT mode regardless
of whether the fourteen arm joints use PV or MIT control.

Version 0.10.30 scales direct-mode Kp and Kd by ten. Runtime caps the resulting
MIT Kd at the protocol maximum of 5; protected mode and zero strength remain
unchanged.

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

Version 0.10.35 adds Runtime ABI 2.32 product temperature snapshots. Native
`get_state_v3()` returns cached MOS and rotor temperatures for every Yunyi arm
joint and installed gripper with explicit freshness validity, without sending
extra CAN requests. Existing state V1/V2 callers remain ABI compatible.

The same release advances the Runtime ABI to 2.33 for latched emergency-stop
position holding. `estop()` clears superseded motion, keeps enabled Motors
enabled, and continuously sends native PV/MIT holds; it no longer performs an
implicit whole-product torque-off. Existing disabled products remain disabled,
and only the full native `recover()` transaction clears the latch.

Runtime ABI 2.34 in this release also adds one immutable 14-joint angle and
velocity-limit snapshot. `articore_runtime_get_joint_angle_vel_limits()`
returns logical lower/upper angles in radians and product velocity limits in
radians/second, ordered left J1..J7 then right J1..J7; grippers are omitted.

Runtime ABI 2.35 adds a persistent product ordinary-motion speed setting.
`articore_runtime_set_speed()` accepts 0..100 and
`articore_runtime_get_speed()` returns the current value; a newly created
Runtime defaults to 70. All 14 arm joints map 100 to 5 rad/s. The setting can
change an active ordinary MIT/PV reference, and
`articore_runtime_set_joint_positions_v2()` uses it for later targets. Raw
frames, trajectories, and Cartesian motions retain their explicit limits.

Version 0.10.36 / Runtime ABI 2.36 corrects the public name of that setting to
maximum speed. New SDKs bind `articore_runtime_set_max_speed()` and
`articore_runtime_get_max_speed()` and expose `set_max_speed()` /
`get_max_speed()`. The 2.35 `set_speed/get_speed` symbols remain binary
compatibility aliases; all values and behavior are unchanged.

Version 0.10.37 / Runtime ABI 2.37 defines the existing product pose as the
single active Cartesian control point. With grippers, native FK/IK and all
Cartesian planners use the fixed `l-tool0` / `r-tool0` gripper center; without
grippers they continue to use link7. No extra public flange-pose API is added.

Version 0.10.38 / Runtime ABI 2.38 reduces ordinary PV control to one
product-owned stepped path. `set_max_speed(0..100)` is persistent and defaults
to 70; position commands carry no per-call speed. New SDKs do not expose raw
PV submission or the legacy explicit-speed ordinary command. Existing C ABI
symbols remain available only for binary compatibility. This setting is PV
only; MIT ordinary speed, raw targets, gains, and feedforward behavior are
unchanged.

Version 0.10.39 also fixes visible Yunyi PV Cartesian endpoint oscillation.
Native motion now performs staged low-speed settling, product FK endpoint
verification, a continuously refreshed zero-speed final hold, 200 ms stable
feedback confirmation, and post-completion supervision. No motor flash,
zero-offset, or firmware-gain values are rewritten.

The required Pinocchio C++ template implementations are compiled into
`libarticore_runtime.so` with hidden visibility. The installed runtime has no dynamic dependency
on Pinocchio or Boost, so a ROS 2 `LD_LIBRARY_PATH` cannot substitute an incompatible robotics
library.

The payload is installed under:

```text
motor_drive_layer_native/
└── lib/
    └── libarticore_runtime.so
```

Use the headers and CMake package from the native SDK artifact for C/C++ development. Install
Articore-SDK for the supported Python product interface.
