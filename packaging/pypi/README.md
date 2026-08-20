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
