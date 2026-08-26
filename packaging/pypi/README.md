# motor-drive-layer

Binary-only Yunyi product Runtime for Linux.

Version 0.15.0 ships Runtime ABI 6.0 (`0x00060000`). The wheel contains the
native `libarticore_runtime.so` and required data; it contains no Python module,
ctypes declarations or Motor ABI library.

The consuming SDK locates the shared library and must require the exact ABI
value. The public C contract is `articore/runtime_abi.h` and exposes one current
name/signature for each product operation. There are no capability bits or
version-suffixed entry points.

Runtime owns the two SocketCAN-FD+BRS channels, all installed Motors, product
configuration, native workers, safety state, IK and trajectory execution.
Python passes product arrays and reads coherent product state/health only.

Supported platforms:

- Linux x86_64
- Linux AArch64

The Runtime embeds its native robot-model implementation and does not resolve
Pinocchio or Boost from a ROS `LD_LIBRARY_PATH` at load time.
