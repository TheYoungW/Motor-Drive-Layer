# DM Serial 1M Baud Support Design

## Goal

Allow the C++ `dm-serial` transport to open POSIX serial devices at exactly
`1,000,000` baud while preserving all existing supported baud rates and error
handling.

## Scope

- Modify only the `/home/ubuntu/motorbridge` repository.
- Do not modify reBot, vr-pico, or any external YAML configuration.
- Add support only when the platform exposes the POSIX `B1000000` constant.
- Keep unsupported baud rates rejected with the existing
  `unsupported serial baud` error.

## Implementation

Add a conditional `1000000 -> B1000000` branch to `baud_to_constant()` in
`cpp_damiao/src/dm_serial_bus_posix.cpp`. The branch will be guarded with
`#ifdef B1000000`, matching the existing conditional handling for optional
POSIX baud constants.

## Testing

Extend `cpp_damiao/tests/dm_serial_codec_test.cpp` with a Linux/POSIX
pseudo-terminal regression test. The test will create a PTY pair and call
`DmSerialBus::open()` on the slave path at `1,000,000` baud. Before the
implementation, it must fail with `unsupported serial baud: 1000000`; after
the implementation, it must open and close successfully.

After the focused red/green cycle, run the complete C++ test suite and perform
a real-device constructor probe at `1,000,000` baud. The real-device probe only
opens and closes the serial bridge; it does not enable motors or send motion
commands.

## Safety and Compatibility

The change affects only baud-rate selection during serial initialization. It
does not alter CAN frame encoding, motor IDs, feedback matching, control modes,
or motor commands. Platforms without `B1000000` retain the current unsupported
baud behavior.
