# DaMiao DM_Device runtime

`motor-drive-layer` loads DaMiao's vendor runtime dynamically and redistributes the matching
runtime in each platform wheel with vendor permission; the binary is not required at native link
time.

Supported ABI families:

- v1.0.0: `damiao_handle_*` and `device_*` symbols. This remains the Linux x86_64 default because
  it provides low-latency feedback with current `dual_app v1.0.0.3` firmware.
- v1.1.0: `dmcan_*` symbols. This is the normal Windows/macOS runtime and is also supported on
  Linux x86_64 and ARM64. Linux wheels pair it with a private libusb 1.0.27 built by
  `scripts/build_private_libusb.sh` because Ubuntu 22.04's libusb lacks
  `libusb_init_context`; x86_64 additionally carries a private compatible libstdc++. The x86_64
  wheel includes both ABIs, and `MOTOR_DM_DEVICE_ABI=v1.1` selects the fully private v1.1 stack.

Normal users receive the runtime automatically with `pip install motor-drive-layer`.
`motor-drive-layer-install-dm-device --download` and `MOTOR_DM_DEVICE_LIB` remain recovery and
development overrides. `SHA256SUMS` records the exact redistributed vendor artifacts.

The runtime remains the property of DaMiao and is kept as a distinct runtime component in the
installed package.
