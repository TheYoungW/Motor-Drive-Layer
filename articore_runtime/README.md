# Yunyi product Runtime

`libarticore_runtime.so` is the only public native library. Runtime ABI 7.0 is
an exact contract: the SDK must require `articore_runtime_abi_version() ==
0x00070000` and bind only the declarations in `articore/runtime_abi.h`.

## Ownership

`articore_runtime_create_yunyi(mode, with_grippers, &runtime)` constructs the
complete product:

- `can-left` and `can-right` SocketCAN-FD+BRS transports;
- fourteen arm Motors and the optional paired grippers;
- Motor/Controller/ControllerGroup ownership and workers;
- product direction, limits, gains and feedback mapping;
- robot models, TCP offsets, Cartesian IK and trajectory execution;
- safety state, health, maintenance and shutdown.

The SDK does not construct or pass Motor handles, Controllers, transport
objects, joint tables or product bindings.

## Product API groups

- Lifecycle: `connect`, `disconnect`, `enable`, `disable`, `estop`, `recover`.
- Maintenance: `configure_mode`, `clear_faults`, `set_zero`.
- Joint control: `set_joint_pv`, `set_joint_mit`, `submit_mit_frame`.
- Product speed: `set_max_speed`, `get_max_speed`. Ordinary PV commands also
  carry a per-command `speed_percent`; Runtime applies the lower of the command
  value and the persistent maximum.
- Native trajectories: joint quintic trajectory, Cartesian PTP, Linear and
  Circular motion.
- Grippers: one paired `set_grippers` call using opening `0..1000`, strength
  `0..10`, and protected/direct mode.
- State: one coherent cached `get_state`, `get_pose`, joint limits and health.
- Product functions: gravity compensation, bimanual follow and TCP offset.

All fourteen-joint arrays use `left J1..J7, right J1..J7`. Public ABI values
never contain native Motor pointers.

## Motion semantics

`articore_runtime_move_pose(left_pose, right_pose, speed)` is the only public
Cartesian PTP operation. Like `set_joint_pv(left, right, speed)`, it submits one
complete dual-arm product target; the only additional step is endpoint IK for
both poses. Both solutions use one planned-reference snapshot and the complete
14-joint PV target is installed atomically only after both succeed. PTP has no
motion ID, status or cancellation API. Endpoint IK retains the `1e-4` SE(3)
tolerance, reuses each arm's Pinocchio model and limits global fallback to an
8 ms soft steady-clock budget. Timeout or either-side failure leaves the active
target unchanged. Linear and Circular are asynchronous FIFO trajectory tasks.
Linear uses explicit `start_pose -> end_pose`; Circular uses explicit
`start_pose -> via_pose -> end_pose`. Runtime validates and plans the complete
task before installing it. Joint, Linear and Circular trajectories share one
Motion ID namespace and FIFO, and use `get_motion_status(id)`,
`cancel_motion(id)` and `cancel_all_motions()`. A joint trajectory returns its
Motion ID directly from `start_trajectory()`. Cancelling a queued item removes
only that item; Runtime inserts and validates a native approach into its
immediate successor so execution cannot jump across the removed endpoint.
Cancelling the running item holds the last safe reference and cancels its
dependent queue tail, whose planned starts are no longer valid.
Completed, cancelled and faulted tasks remain queryable in bounded history.

Linear and Circular accept `duration_s` instead of a speed percentage. The
duration is the complete planned task duration, including an automatic PTP
approach to the declared start pose when needed. Too-short durations are
rejected before the FIFO changes. Ordinary PV speed values remain `0..100`.
The persistent PV maximum defaults to 50, mapping to a 1 rad/s reference slew;
100 maps to 2 rad/s. The Damiao POS_VEL drive ceiling remains independent.

## State and diagnostics

`articore_runtime_get_state()` returns one cache snapshot with positions,
velocities, torques, temperatures, actual enabled masks and optional gripper
state. It performs no CAN request. `articore_runtime_get_health()` is the sole
operation/safety diagnostic surface.

Every public structure must set `struct_size` to its exact `sizeof(...)`.
ABI 7.0 does not branch on older layouts or capability bits.

## Shutdown

`disconnect()` is the hardware lifecycle operation: stop control, disable and
confirm the product, close transports and join workers. It is idempotent.
`free()` releases the opaque C handle after disconnect; it is not a second
business lifecycle operation.
