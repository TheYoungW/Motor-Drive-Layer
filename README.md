# motorbridge 1.2

RK3588 上的云翼双臂底层服务。1.0 只交付一个 C++ 进程
`articore-runtime-service`；远端 SDK 通过 Cyclone DDS/IP 调用，不再加载
Python ctypes ABI、本地 wheel 或 `libarticore_runtime.so`。

## 架构

```text
remote SDK
   │ Cyclone DDS v1 / Ethernet or LAN
   ▼
dds/       protocol, QoS, WaitSet, lease, bounded mailbox
   ▼
runtime/   500 Hz clock, product model, safety, online control, trajectories
   ▼
motor/     Damiao protocol, Motor, Controller, SocketCAN-FD+BRS
   ▼
can-left + can-right
```

依赖只能向下：`dds → runtime → motor`。Runtime 仍然唯一拥有控制周期、
安全状态机、轨迹和模型；DDS 线程只验证消息和覆盖容量为一的 mailbox。
普通 PV/MIT 保持 latest-target-wins，有限 Cartesian 运动仍使用原有 Runtime
轨迹语义。

Runtime 内部调用链为 `YunyiRuntime → YunyiRuntimeCore → SafetyRuntime → motor`。
旧 `runtime_bridge_*`、opaque handle、独立 robot-model bridge 和 Motion-ID/FIFO
入口已删除；内部核心头文件不安装，跨进程边界只有 DDS v1 IDL。

## 网络协议

协议版本为 `1.2`，`robot_id` 是 DDS key。1.1 新增左右独立的启动拓扑查询，
仍兼容 1.0 的成对夹爪查询；1.2 在连续 `RobotState` 中新增左右夹爪开合度、
安装可用性和反馈新鲜度。数组顺序固定为 `[left, right]`。固定 Topic：

- `articore.robot.discovery`
- `articore.robot.control.request` / `articore.robot.control.reply`
- `articore.robot.stream.command`
- `articore.robot.state`
- `articore.robot.health`
- `articore.robot.motion.event`

`RobotState` 是 DDS `@final` 类型，因此 x86 SDK 必须使用 1.2 的
`articore_protocol.idl` 重新生成类型后才能匹配状态 Topic。deb 同时把该 IDL
安装到 `/usr/share/articore/idl/articore_protocol.idl`；客户端只应在对应的
`gripper_available[i]` 和 `gripper_feedback_valid[i]` 都为 true 时使用
`gripper_openings[i]`。

整机只有一个 250 ms 控制租约。客户端按 20 Hz heartbeat；有效流式命令也
续租。租约丢失会撤销流式目标、停止有限运动并 disable，重新获取租约不会
自动 enable。首版仅面向可信封闭网络，租约不是身份认证。

DDS I/O 线程直接处理 lease acquire、heartbeat 和 release；其他控制请求在
校验后进入容量为 32 的单消费者队列。专用 worker 串行执行模式切换、维护、
规划和查询，因此长控制操作不会阻塞 heartbeat，同时 Runtime 生命周期操作
仍保持严格顺序。

## 本机构建

必须使用 Cyclone DDS `11.0.1`，不能使用 Ubuntu 22.04 的旧包：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/cyclonedds-11.0.1
cmake --build build -j
ctest --test-dir build --output-on-failure
```

只验证 Motor/Runtime、不构建网络服务时可使用
`-DARTICORE_ENABLE_DDS=OFF`。

## RK3588 部署

目标系统是 Ubuntu 22.04 ARM64。配置入口为
`/etc/articore/runtime-service.conf` 和 `/etc/articore/can.conf`。
Debian 包安装服务、私有 `libddsc.so.11`、CAN 初始化单元、Runtime 单元和
udev 热插拔规则，不安装开发头文件。1.0.3 针对当前 RK3588 使用 `wlan0`
作为 DDS 默认网卡，并按两个 USB-CAN 的真实序列号稳定命名
`can-left`/`can-right`。

交叉编译时，`idlc` 必须是宿主机 11.0.1 可执行文件，而 Cyclone DDS CMake
包及 `libddsc` 必须来自 ARM64 sysroot：

```bash
MOTOR_AARCH64_SYSROOT=/path/to/rk3588-sysroot \
ARTICORE_HOST_IDLC=/opt/cyclonedds-host-11.0.1/bin/idlc \
ARTICORE_PINOCCHIO_HEADER_ROOT=/opt/pinocchio-3.8/include \
scripts/build_aarch64_runtime.sh
```

Pinocchio 3.8 的算法是模板实现，交叉编译时直接编译进主程序，因此目标板
不需要另装 `libpinocchio.so`。Cyclone DDS 11.0.1 则作为私有动态库随包安装。

Runtime 处于 `FAULT` 时仍可持有 DDS 控制租约并执行显式维护请求。模式配置和
普通清错的拒绝回复包含当前状态、要求状态、原始故障原因和失败电机；急停锁存
仍不能由普通 `CLEAR_FAULTS` 清除。

Runtime 每次进程启动时在左右 CAN 通道分别扫描电机：ID 1..7 是必需关节，
ID 8 是可选末端。左右夹爪可独立存在，扫描结果在该 Runtime 进程生命周期内
冻结；运行期间的通信丢帧不会被解释为拆除了末端。更换末端必须先失能，再重启
Runtime 触发重新扫描，重启后仍需用户显式 enable。

安装不会自动使能电机。真机运动、掉线和 watchdog 验收必须另行取得硬件
操作授权。

固定 RK3588 测试机的产物上传、校验、安装和服务启动步骤见
[`docs/RK3588_DEPLOYMENT.md`](docs/RK3588_DEPLOYMENT.md)。
