# Private ARM64 libusb runtime

DaMiao's Linux ARM64 DM_Device v1.1 library imports `libusb_init_context`, but
Ubuntu 22.04 supplies libusb 1.0.25 without that symbol. The ARM64 wheel build
therefore compiles libusb 1.0.27 and lets `auditwheel` copy and rename the
resulting shared library as a wheel-private dependency.

Pinned source:

- release: `libusb 1.0.27`
- URL: <https://github.com/libusb/libusb/releases/download/v1.0.27/libusb-1.0.27.tar.bz2>
- SHA256: `ffaa41d741a8a3bee244ac8e54a72ea05bf2879663c098c82fc5757853441575`
- license: `LGPL-2.1-or-later`

The build disables the optional udev integration to avoid introducing another
non-system wheel dependency. USB enumeration and Linux netlink hotplug support
remain provided by libusb's Linux backend.
