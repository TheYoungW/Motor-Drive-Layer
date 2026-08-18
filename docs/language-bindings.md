# Native Runtime binding contract

`libmotor_abi` and `libarticore_runtime` are the only behavioral implementation. Python, C++,
ROS 2, and future bindings must delegate to the same C ABI; a binding may validate language-level
types and convert reports, but must not implement a second control loop, watchdog, safety state
machine, gripper controller, retry policy, or transport lifecycle.

## Product adapter boundary

A robot SDK supplies only product information:

- motor roles, channels, IDs, directions, and joint ordering;
- joint command limits and product hold gains;
- built-in native product profile IDs such as `yunyi_gripper_v1`;
- native product profile IDs for the built-in robot model and gravity controller.

It creates Controllers and Motors, performs pre-Runtime device/mode configuration, creates one
ControllerGroup, then creates `ArticoreRuntime`. From that point until Runtime close, normal motor
output and feedback scheduling go only through Runtime.

## Python ownership

This repository does not publish a Python module. Articore-SDK owns its private ctypes binding to
both native libraries. That SDK binding:

- retains and leases ControllerGroup, Controller, and Motor objects;
- rejects direct sends, device configuration, fresh-feedback traffic, or premature close while
  those objects belong to an active Runtime;
- converts native enable/disable/health structures to immutable Python values;
- preserves structured native failure reports on exceptions;
- calls native close before releasing dependent handles.

Applications use Articore-SDK's public API and must not call its private binding or `ctypes.CDLL`
directly. The PyPI `motor-drive-layer` distribution is only the native payload that Articore-SDK
locates through package metadata.

## C++ and ROS 2

Install the native SDK and use its exported CMake target:

```cmake
find_package(MotorDriveLayer CONFIG REQUIRED)
target_link_libraries(robot_driver PRIVATE motorbridge::articore_runtime_cpp)
```

`<articore/runtime.hpp>` supplies the move-only `articore::Runtime` RAII wrapper. ROS 2 packages
may use this target directly; another language can bind `articore/runtime_abi.h` without depending
on Python.

## Lifecycle order

The required order is:

1. Create Transport/Controller and register or discover Motors.
2. Configure modes and device parameters.
3. Create ControllerGroup.
4. Create Runtime and configure product bindings/limits.
5. Connect, enable, and command only through Runtime.
6. Close Runtime first.
7. Release ControllerGroup, Motors, and Controllers.

## RK3588/aarch64

`scripts/build_aarch64_runtime.sh` installs ARM64 `libmotor_abi.so`,
`libarticore_runtime.so`, public headers, the CMake package, the ARM64 DM Device v1.1 runtime, and
a private ARM64 libusb 1.0.27. This does not modify the board's system libusb. Set
`MOTOR_AARCH64_SYSROOT` for a board sysroot, or set `MOTOR_AARCH64_BUNDLE_DM_DEVICE=0` for a
runtime-only build that intentionally omits the vendor support dependency.
