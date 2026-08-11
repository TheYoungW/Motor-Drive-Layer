# DaMiao DM_Device runtime

`motor-drive-layer` loads DaMiao's vendor runtime dynamically; the binary is not required at
link time and is not bundled in wheels unless `MOTOR_DM_DEVICE_BUNDLE=1` is explicitly set while
building one.

Supported ABI families:

- v1.0.0: `damiao_handle_*` and `device_*` symbols. This is preferred on Linux because it works
  with older system libusb releases.
- v1.1.0: `dmcan_*` symbols. This is the normal Windows/macOS runtime and is also supported on
  Linux when its newer libusb/libstdc++ dependencies are available.

Install with `motor-drive-layer-install-dm-device --download`, or point
`MOTOR_DM_DEVICE_LIB` at an official runtime. The installer downloads from the package's release
assets/source tree and then the official
[dm-device-sdk](https://gitee.com/kit-miao/dm-device-sdk) repository as a fallback.

The runtime remains subject to DaMiao's distribution and licensing terms. Review those terms
before redistributing a wheel with `MOTOR_DM_DEVICE_BUNDLE=1`.
