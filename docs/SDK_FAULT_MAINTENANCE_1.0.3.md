# SDK 故障维护会话对接说明

## Integration target

- RK3588 Runtime 包：`articore-runtime-service 1.0.3`。
- Python SDK：`arx-d-can 1.0.2`。
- DDS 协议保持 `1.0`；IDL、Topic、枚举值和序列化布局均未改变。
- Runtime 必须先升级到 1.0.3，清错事务才不会夹带模式寄存器配置。

本地验证产物：

```text
/home/ubuntu/motorbridge/dist/arm64/articore-runtime-service_1.0.3_arm64.deb
SHA256 805c5ff2550403da119a434a4a889e88b2d259540a7d59c49a0b64f351396517

/home/ubuntu/Articore-SDK/dist/arx_d_can-1.0.2-py3-none-any.whl
SHA256 c41f8f88ef3ae8428e74f47361a2a61caa58cb3570c8b66cf261032587b33c72
```

## SDK must do

1. `connect()` 发现 Runtime 后获取整机唯一 lease。
2. 获取 lease 后立即启动 20 Hz heartbeat。
3. 通过可靠 `QUERY_HEALTH` 请求读取当前 Runtime 状态。
4. 普通连接在 `READY` 时发送客户端要求的 `CONFIGURE_MODE`；在 `FAULT` 时
   跳过模式配置，保留 lease/heartbeat。
5. 清错工具使用 `connect(maintenance=True)`；即使 Runtime 为 `READY` 也不
   配置模式。
6. 用户显式调用 `clear_motor_faults()` 时发送 `CLEAR_FAULTS`。普通会话成功后
   再配置客户端要求的模式；维护会话成功后保持 READY、失能和未配置。
7. 任一步骤失败时原样保留 Runtime 的 `ControlReply.message` 和协议错误码。

## SDK must not do

- 不自动发送 `CLEAR_FAULTS`、`RECOVER`、`ENABLE` 或任何流式/有限运动命令。
- 不在 Python 重新判断哪些故障可清除；安全判定继续完全属于 Runtime。
- 不用重启板端服务绕过锁存 `FAULT`。
- 不允许普通 `CLEAR_FAULTS` 解除急停锁存。
- 不在 Runtime 的 `CLEAR_FAULTS` 事务中读写模式或通信看门狗寄存器。

## Runtime reply behavior

`CONFIGURE_MODE` 和 `CLEAR_FAULTS` 失败时，现有 bounded `message` 返回：

```text
<OPERATION> rejected: current_state=<STATE>, required_states=<...>, fault_reason=<...>, failed_motors=[...], cause=<...>
```

Runtime 从原生健康状态合并最近操作失败电机和锁存故障电机。急停拒绝包含
`emergency_stop_latched=true` 和 `fault_reason=emergency stop requested`。

## Acceptance tests

- FAULT 连接顺序必须是 `ACQUIRE_LEASE → heartbeat → QUERY_HEALTH`，且没有
  `CONFIGURE_MODE`。
- READY 连接顺序必须在 heartbeat 启动后才执行 `CONFIGURE_MODE`。
- 显式维护连接在 READY/FAULT 下都不得执行 `CONFIGURE_MODE`。
- 清错成功后，普通会话才允许执行模式配置；维护会话仍不得配置。
- 清错失败时不得执行模式配置、使能或运动。
- CAN/寄存器故障未恢复时，错误包含操作、当前状态、失败电机和底层原因。
- 急停锁存时普通清错返回 `WRONG_STATE`，并保留急停原因。
- 软件自动化测试不等同于真机验收；J7 超时、CAN 恢复和全程失能验证需要
  单独的硬件操作授权。
