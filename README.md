# Motor-Drive-Layer

English | [简体中文](README.zh-CN.md)

Motor-Drive-Layer is the native C++ control foundation for Damiao motors and the Articore product
Runtime. Runtime behavior is implemented in C++ and exposed through stable C ABIs. This repository
does not ship a Python module or a Python implementation of control, safety, robot dynamics, or
gravity compensation.

## Architecture

```text
Articore-SDK / C++ SDK / ROS 2
          │ stable C ABI
          ▼
  libarticore_runtime.so
  watchdog, safety, robot model,
  gripper and gravity compensation
          │ stable motor ABI
          ▼
      libmotor_abi.so
  controller, motor and transport API
          │
          ▼
 C++ protocol and transport core
          │
          ▼
SocketCAN / SocketCAN-FD / serial / DM Device
```

The ownership boundary is deliberate:

- `cpp_damiao/` contains generic transport, protocol, Controller and Motor behavior.
- `articore_runtime/` contains product Runtime, robot-model and gravity-compensation behavior.
- Articore-SDK owns its Python `ctypes` declarations, value objects and user-facing API.
- The PyPI `motor-drive-layer` wheel is a binary payload only. It contains no `.py` or `.pyi`
  files and cannot be imported as `motor_drive_layer`.

The public native outputs are:

- `libmotor_abi.so`, declared by the motor C ABI headers.
- `libarticore_runtime.so`, declared by
  [`articore/runtime_abi.h`](articore_runtime/include/articore/runtime_abi.h).
- The C++17 RAII target `motorbridge::articore_runtime_cpp`.

## Native features

- Damiao MIT, position/velocity, velocity and force/position modes.
- Linux SocketCAN and SocketCAN-FD+BRS, Damiao serial bridge and DM Device transports.
- Background feedback reception, integrity checks and per-motor cached state.
- Structured feedback, transport and Runtime faults.
- Bounded non-blocking SocketCAN transmission so a full kernel queue cannot permanently block
  shutdown.
- Atomic Runtime enable/disable, watchdog, safe hold and deterministic fault handling.
- Product gripper profiles, joint limits and per-cycle MIT resultant-torque protection.
- Native seven-axis robot model: FK, IK, Jacobian, gravity, mass/Coriolis, RNEA and ABA.
- Native gravity-compensation hand-guiding mode.

## Pinocchio and ROS 2 isolation

Pinocchio is used as a C++ build dependency for the product robot model. Required template
implementations are compiled directly into `libarticore_runtime.so` with hidden visibility. The
installed Runtime has no dynamic dependency on `libpinocchio_default.so` or Boost.Serialization.

Consequently, a ROS 2 `LD_LIBRARY_PATH=/opt/ros/...` cannot replace the model implementation used
inside the Runtime. CI verifies both the ELF dependency table and model creation under a poisoned
Pinocchio library path.

## Build

Requirements are CMake 3.16+, a C++17 compiler and Pinocchio C++ development headers.

```bash
cmake -S . -B builds/cmake/default -DCMAKE_BUILD_TYPE=Release
cmake --build builds/cmake/default -j
ctest --test-dir builds/cmake/default --output-on-failure
```

Install the native SDK and CMake package with:

```bash
cmake --install builds/cmake/default --prefix /desired/prefix
```

All generated artifacts stay under the single ignored `builds/` tree:

- `builds/cmake/default/`: current native build.
- `builds/packages/`: package assembly intermediates.
- `builds/wheels/`: local and release wheels.
- `builds/archive/`: preserved historical build directories.

C++ consumers can use:

```cmake
find_package(MotorDriveLayer CONFIG REQUIRED)
target_link_libraries(robot_driver PRIVATE motorbridge::articore_runtime_cpp)
```

For RK3588, `scripts/build_aarch64_runtime.sh` cross-builds the native libraries, headers and
CMake package. Set `MOTOR_AARCH64_SYSROOT` when a board sysroot is required.

## PyPI binary payload

The PyPI wheel is built from `packaging/pypi/` only to distribute platform libraries. It installs:

```text
motor_drive_layer_native/
└── lib/
    ├── libmotor_abi.so
    ├── libarticore_runtime.so
    └── dm_device/
```

It intentionally provides no Python import surface. Articore-SDK locates these files with package
metadata and maintains its own ABI declarations. This keeps the low-level project C/C++-only while
preserving the existing high-level SDK interaction.

Build a local payload after compiling the native libraries:

```bash
python3 -m build --wheel --outdir builds/wheels/current packaging/pypi
```

The packaging command uses Python because PyPI's wheel tooling is Python-based; no Python runtime
code is installed by the resulting wheel.

## Transport behavior

A Controller uses no artificial TX delay for one motor. Adding a second motor enables a default
200 µs minimum interval between outgoing frames. Set `MOTOR_DRIVE_LAYER_TX_GAP_US` or use the
native configuration API to change it.

Linux SocketCAN and SocketCAN-FD sockets are non-blocking. When the kernel TX queue stays full, a
send fails after 20 ms by default, is recorded in transport health and propagates into Runtime
fault handling. Set `MOTOR_DRIVE_LAYER_SOCKETCAN_SEND_TIMEOUT_MS` to a value from 1 to 60000 ms to
change the bound.

`motor_controller_request_feedback_all_ex()` provides stable feedback failure codes and a
structured missing-motor report. Runtime integrations must use structured results rather than
parsing diagnostic strings.

## Runtime and gravity compensation

`libarticore_runtime.so` owns the fixed-rate worker, command mailbox, motor leases, watchdog,
enable/disable transactions, fault hold, gripper policy, torque limits and gravity-compensation
state machine. A language binding may submit configuration and commands, but it must not recreate
these algorithms.

Runtime ABI 2.8 provides the initial gravity-compensation mode. The SDK binds an installed
seven-axis side to `yunyi_v1_0`, enables MIT mode and starts gravity compensation. The native worker
ramps out stiffness/damping while ramping in posture-dependent gravity torque. Stopping performs
the inverse transition into a current-position MIT hold. Friction and Coriolis compensation are
not part of this initial mode.

## Safety

Motor control can cause unexpected motion and injury. Support the mechanism, keep an independent
emergency stop available, verify channel/IDs/model/mode before enable, and begin with conservative
limits. Register writes can permanently change motor configuration.

Motor-control defects may create physical safety risks. Do not open a public issue containing a
turnkey procedure for uncontrolled motion, bypassing limits, defeating a watchdog, or remotely
driving attached hardware. Use the repository's private vulnerability-reporting channel when it is
available. Include the affected release or commit, transport and adapter, motor model and firmware,
whether motors must be enabled, minimum safe reproduction steps, expected versus observed fail-safe
behavior, and any known mitigation. General bugs without a safety or security impact may use a
public issue. Maintainers will reproduce private reports in a safe environment where possible and
coordinate disclosure after a mitigation is available; no response-time guarantee is currently
offered.

## Tests

The repository's default tests do not enable physical motors:

```bash
cmake --build builds/cmake/default -j
ctest --test-dir builds/cmake/default --output-on-failure
```

CI additionally checks that the PyPI wheel contains no Python source, that both native ABI
libraries load, that the robot model works without a Pinocchio runtime dependency, and that the
DM Device vendor libraries resolve on each release architecture.

Hardware acceptance remains opt-in. Inspect the scripts under `scripts/` and provide explicit
motor mappings and acknowledgement flags before running them.

## Repository layout

```text
cpp_damiao/              Generic C++ protocol, transports and motor C ABI
articore_runtime/         Native product Runtime, robot model and C/C++ ABI
packaging/pypi/           Binary-only wheel assembly; no Python runtime module
third_party/dm_device/    Optional vendor headers and redistributable libraries
scripts/                  Build, diagnostic and hardware-acceptance helpers
tests/                    Native CMake package consumer tests
```

## License

Motor-Drive-Layer is MIT licensed. Bundled DM Device, libusb and libstdc++ runtime components retain
their respective notices under `packaging/pypi/LICENSES/`.
