# DaMiao DM_Device runtime

`motor-drive-layer` loads DaMiao's vendor runtime dynamically and redistributes the matching
runtime in each platform wheel with vendor permission; the binary is not required at native link
time.

Supported ABI families:

- v1.0.0: `damiao_handle_*` and `device_*` symbols. This is used by Linux x86_64 and is packaged
  with an isolated compatible libstdc++ runtime.
- v1.1.0: `dmcan_*` symbols. This is the normal Windows/macOS runtime and is also supported on
  Linux ARM64.

Normal users receive the runtime automatically with `pip install motor-drive-layer`.
`motor-drive-layer-install-dm-device --download` and `MOTOR_DM_DEVICE_LIB` remain recovery and
development overrides. `SHA256SUMS` records the exact redistributed vendor artifacts.

The runtime remains the property of DaMiao and is kept as a distinct runtime component in the
installed package.
