# motor-drive-layer

Binary-only Yunyi product Runtime for Linux.

Version 0.30.0 ships Runtime ABI 15.0 (`0x000F0000`). The wheel contains the
native `libarticore_runtime.so` and required data; it contains no Python module,
ctypes declarations or Motor ABI library.

The consuming SDK locates the shared library and must require the exact ABI
value. The public C contract is `articore/runtime_abi.h` and exposes one current
name/signature for each product operation. There are no capability bits or
version-suffixed entry points.

Runtime owns the two SocketCAN-FD+BRS channels, all installed Motors, product
configuration, native workers, safety state, IK and trajectory execution.
Python passes product arrays and reads coherent product state/health only.
Ordinary PV also exposes optional persistent maximum speed and acceleration
settings. They define the 100-percent joint-limit base. The shared
`set_speed_percent` setting scales ordinary-PV limits and the Linear/Circular
automatic time parameterization; trajectory base limits remain internal to
Runtime, and Cartesian callers do not provide a duration.
MIT exposes only standard full-frame `set_joint_mit(q, dq, kp, kd, tau_ff)`
and angle-only `set_joint_mit_fast(q, speed_percent=100)`. Fast MIT accepts
`0..100` as a percentage of its 5 rad/s reference-step base; standard MIT has
no speed parameter. ABI 15
does not export the former `set_pose()` shortcut; consumers use `solve_ik()`
plus an explicit joint command or the finite Cartesian `move_pose()` API.

Supported platforms:

- Linux x86_64
- Linux AArch64

The Runtime embeds its native robot-model implementation and does not resolve
Pinocchio or Boost from a ROS `LD_LIBRARY_PATH` at load time.
