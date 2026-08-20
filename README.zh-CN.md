# Motor-Drive-Layer

[English](README.md) | 简体中文

Motor-Drive-Layer 是达妙电机与 Articore 产品 Runtime 的原生 C++ 控制底座。所有运行时行为
都在 C++ 中实现，并通过稳定的 C ABI 对外提供。本仓库不发布 Python 模块，也不使用 Python
实现控制、安全、机器人动力学或重力补偿。

## 架构边界

```text
Articore-SDK / C++ SDK / ROS 2
          │ 稳定 C ABI
          ▼
  libarticore_runtime.so
  看门狗、安全、机器人模型、
  夹爪与重力补偿
          │ 稳定电机 ABI
          ▼
      libmotor_abi.so
  Controller、Motor 与通信接口
          │
          ▼
     C++ 协议与通信核心
          │
          ▼
SocketCAN / SocketCAN-FD / 串口 / DM Device
```

职责划分如下：

- `cpp_damiao/` 只负责通用通信、协议、Controller 和 Motor。
- `articore_runtime/` 负责产品 Runtime、机器人模型和重力补偿。
- Articore-SDK 自己维护 Python `ctypes` 声明、值类型和用户接口。
- PyPI 的 `motor-drive-layer` wheel 只分发二进制文件，不包含 `.py` 或 `.pyi`，也不能
  `import motor_drive_layer`。

公开的原生产物包括：

- 通用电机 ABI：`libmotor_abi.so`。
- 产品 Runtime ABI：`libarticore_runtime.so`，声明位于
  [`articore/runtime_abi.h`](articore_runtime/include/articore/runtime_abi.h)。
- C++17 RAII 目标：`motorbridge::articore_runtime_cpp`。

## 原生能力

- 达妙 MIT、位置速度、速度和力位混合模式。
- Linux SocketCAN、SocketCAN-FD+BRS、达妙串口桥和 DM Device。
- 后台反馈接收、反馈一致性检查与电机状态缓存。
- 结构化反馈、通信和 Runtime 故障。
- 有限超时的非阻塞 SocketCAN 发送，内核队列堵塞不会永久卡死关闭流程。
- Runtime 原子使能/失能、看门狗、安全保持和确定性故障处理。
- 产品夹爪策略、关节限制和逐周期 MIT 合力矩保护。
- 七轴原生机器人模型：FK、IK、Jacobian、重力、质量/科氏项、RNEA 和 ABA。
- 原生重力补偿拖动模式。

## Pinocchio 与 ROS 2 隔离

Pinocchio 只作为机器人模型的 C++ 构建依赖。需要的模板实现直接编译进
`libarticore_runtime.so`，并使用隐藏符号。安装后的 Runtime 不动态依赖
`libpinocchio_default.so` 或 Boost.Serialization。

因此 ROS 2 的 `LD_LIBRARY_PATH=/opt/ros/...` 无法替换 Runtime 内部使用的机器人模型实现。
CI 会检查 ELF 依赖表，并在故意注入错误 Pinocchio 库的环境中创建机器人模型。

## 构建

需要 CMake 3.16+、C++17 编译器和 Pinocchio C++ 开发头文件。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

安装原生 SDK 与 CMake 包：

```bash
cmake --install build --prefix /desired/prefix
```

C++ 项目使用：

```cmake
find_package(MotorDriveLayer CONFIG REQUIRED)
target_link_libraries(robot_driver PRIVATE motorbridge::articore_runtime_cpp)
```

RK3588 可使用 `scripts/build_aarch64_runtime.sh` 交叉编译。需要板端 sysroot 时设置
`MOTOR_AARCH64_SYSROOT`。

## PyPI 二进制载荷

PyPI wheel 从 `packaging/pypi/` 构建，仅用于分发平台动态库：

```text
motor_drive_layer_native/
└── lib/
    ├── libmotor_abi.so
    ├── libarticore_runtime.so
    └── dm_device/
```

wheel 不提供 Python import 接口。Articore-SDK 通过发行包元数据定位动态库，并自行维护
ABI 声明。这样底层保持纯 C/C++，上层 SDK 的 `robot.connect()`、`enable()` 和重力补偿
交互方式不变。

编译原生库后可构建本地 wheel：

```bash
python3 -m build --wheel packaging/pypi
```

这里使用 Python 是因为 PyPI wheel 构建工具本身基于 Python；最终 wheel 不安装任何 Python
运行时代码。

## 通信行为

单电机 Controller 默认不增加发送延迟；添加第二个电机后，默认启用 200 µs 的最小帧间隔。
可通过 `MOTOR_DRIVE_LAYER_TX_GAP_US` 或原生配置接口修改。

Linux SocketCAN 与 SocketCAN-FD socket 使用非阻塞模式。内核发送队列持续满时，默认 20 ms
后返回错误，写入通信健康状态并传递给 Runtime 故障处理。可用
`MOTOR_DRIVE_LAYER_SOCKETCAN_SEND_TIMEOUT_MS` 设置 1 到 60000 ms 的超时。

`motor_controller_request_feedback_all_ex()` 返回稳定的反馈错误码与缺失电机报告。Runtime
集成必须使用结构化结果，不能解析错误字符串做安全决策。

## Runtime 与重力补偿

`libarticore_runtime.so` 独占固定频率 worker、最新命令 mailbox、电机 lease、看门狗、
使能/失能事务、故障保持、夹爪策略、力矩限制和重力补偿状态机。语言绑定只负责提交配置和
命令，不得重新实现这些算法。

Runtime ABI 2.8 提供第一版重力补偿模式。SDK 将已安装的七轴侧绑定到 `yunyi_v1_0`，以
MIT 模式使能后启动重力补偿。原生 worker 平滑移除刚度和阻尼，同时逐步加入随姿态变化的
重力力矩；停止时反向过渡到当前位置 MIT 保持。第一版暂不包含摩擦和科氏补偿。

Runtime ABI 2.9 将整机维护操作收归 C++ Runtime。`configure_mode`、`clear_faults` 和
`set_zero` 复用 Runtime 已持有的 Motor lease，在同一 worker/发送屏障内并行处理左右通道，
不会释放 ControllerGroup、关闭 Runtime 或重建资源。调零固定检查 READY、双 transport、
反馈新鲜、物理失能和静止速度，操作后逐电机验证失能、零位和零速；任何部分失败都以稳定
错误码写入统一 `SafetyHealthV2`，不会伪装为成功。产品工厂
`articore_runtime_create_yunyi(...)` 内部拥有 can-left/can-right、
SocketCAN-FD+BRS、14 个关节和两个夹爪。协议仍是逐电机写入，真正 all-or-none 需要固件增加
prepare/commit。

Runtime ABI 2.10 将 `yunyi_v1_0` 从“由语言绑定组装的通用容器”升级为完整产品
Runtime。产品工厂固定拥有双通道、16 个 Motor、方向/量程/限位、默认 MIT 参数、夹爪与
重力模型；新增固定 14 关节 MIT/PV/普通位置帧、双夹爪帧和一次性整机状态快照。SDK 只传
逻辑关节数组，不再传 Controller、ControllerGroup、Motor 或产品配置。普通
位置命令的速度传 0 时，由原生产品配置选择当前模式默认值。

Runtime ABI 2.11 为整机工厂增加固定拓扑参数 `with_grippers`。值为 true 时创建并验证
14 个关节和左右两个夹爪；值为 false 时只创建 14 个关节 Motor，不发送夹爪反馈请求，
也不把缺少夹爪写入 health。新的 `articore_runtime_set_grippers()` 只接收左右 0～1000
开合度和 1～10 力度等级；无夹爪产品调用它会安全返回成功。整机状态只公开夹爪是否可用、
开合度和力度等级，不再向普通 SDK 暴露夹爪 Motor 坐标、速度、力矩或句柄。

Runtime ABI 2.12 将通信质量与电机硬故障分离。少于
`feedback_failure_threshold` 的偶发反馈缺口只计数；持续延迟进入 `DEGRADED`，原生层将
速度参考和 MIT 力矩上限缩放到 25%；延迟达到三倍阈值后进入 `SAFE_STOP`，停止接受新轨迹
并持续发送当前位置保护保持，不主动失能，也不写入 `fault_reason`。只有确认的电机故障码、
意外失能、非有限反馈或 transport 断开才进入 `FAULT`。通信恢复不会自动重放旧目标；调用
`recover()` 后由 Runtime 执行完整整机恢复，最终回到已确认失能的 `READY`，不会继续旧轨迹。

Runtime ABI 2.13 将普通 MIT/PV 位置接口的统一速度参数改为 `0～100`。`0` 暂停位置
reference 推进，`100` 对应产品为当前模式配置的最大普通速度；百分比到 rad/s、逐周期
步长和关节绝对速度上限均在 C++ Runtime 内处理。Raw MIT/PV 帧继续使用物理量，不受影响。

Runtime ABI 2.14 增加 Runtime 所有的整机/单电机使能与失能接口。稳定角色名采用
`left/joint1`～`left/joint7`、`right/joint1`～`right/joint7`，夹爪为
`left/gripper`、`right/gripper`；空角色表示全部已安装电机。设置操作必须通过新反馈确认，
查询返回 `DISABLED`、`ENABLED`、`MIXED` 或 `UNKNOWN`。单电机切换仅允许在非运动状态，
并进入不可下发运动命令的 `PARTIALLY_ENABLED`；只有整机原子 `enable()` 建立当前位置保持后
才进入正常 `ENABLED`。

Runtime ABI 2.15 增加 `articore_runtime_get_pose()`。接口从原生反馈缓存读取指定左/右臂
完整七关节快照，并使用底层内置 Pinocchio 产品模型计算法兰位姿，固定输出
`[x, y, z, roll, pitch, yaw]`（米、弧度）以及参与计算的最旧反馈时间戳和序列号。
调用本身不发送 CAN 帧；当前产品没有 TCP 偏移配置，因此不提供虚构的 TCP 位姿，也不增加
驱动无法可靠提供的电压/相电流字段。

Runtime ABI 2.16 将 `articore_runtime_estop()` 改为无业务参数接口。调用后 Runtime 先停止
控制帧，再失能全部已安装关节和夹爪，并在 health 的 `fault_reason` 中记录固定原因
`emergency stop requested`。重复调用是安全的幂等操作；`disable()`、清除电机故障以及断开后
重连都不能解除急停锁存，只有确认整机失能后的 `recover()` 可以恢复到 READY。

Runtime ABI 2.17 重新定义 `recover()` 为完整整机恢复：先停止旧控制并确认失能，再并行清除
左右通道的可恢复故障、验证 transport 与全部新鲜反馈，然后只为低速回到已标定关节零位而
临时使能，回零验证通过后再次整机失能。任一步骤失败都会再次尝试整机失能，并把失败阶段、
稳定错误码、说明和失败电机写入 health。`clear_faults()` 仍然只清错、不运动；`set_zero()`
仍然表示把当前位置标定为新的零点，两者不会被 `recover()` 混用。

Runtime ABI 2.18 将控制调度频率完全收归底层内部实现。删除公开的控制频率 getter 和 capability；
配置结构中为保持二进制布局而保留的两个频率槽位改为忽略的 reserved 字段。产品 Runtime 根据
内部产品与 transport 策略自行调度，Articore-SDK 和用户既不设置也不读取该频率。

Runtime ABI 2.19 将产品 `disconnect()` 定义为唯一的终止型安全关闭：停止接收和发送控制命令，
失能并确认全部已安装关节与夹爪，停止并回收 worker，关闭左右 CAN，最后释放 Controller、
ControllerGroup、Motor、模型及产品资源。重复调用安全幂等；失能确认失败时仍会先终止 worker，
再通过 Runtime 错误返回原因。旧 `close/free` C 符号仅为 ABI 兼容保留，SDK 在 ctypes 内部自动
释放空句柄，不再向业务用户公开 `close()`。

Runtime ABI 2.20 将产品面彻底收口为唯一的 Yunyi V1.0 双臂。SDK 和新的 C++ 包装器只调用
`articore_runtime_create_yunyi(mode, with_grippers)`，不再传 `product_id`，也不再公开通用
Runtime 构造、Controller/Motor 组装、关节映射、夹爪 profile 或重力模型绑定。固定双 CAN、
14 关节、可选双夹爪、方向/量程/限位、模型和资源生命周期统一放在独立的 C++ Yunyi 产品模块。
旧 `create_product/create_ex*` C 符号只为已有二进制兼容保留，不再进入正常 SDK 路径。

Runtime ABI 2.21 增加产品级原子批量使能/失能接口。电机使用 `l-joint1..7`、
`r-joint1..7`、`l-gripper`、`r-gripper` 稳定名称；批量使能任一步失败会回滚并确认本次已
使能的电机，批量失能在异常状态下仍可执行并逐台确认。Runtime 正式管理
`PARTIALLY_ENABLED`：上层仍提交完整双臂帧，底层只向已使能电机发送，主动失能不会被当成
异常掉使能。每台电机的发送、反馈、确认和错误同时写入事务报告与统一 health。

Runtime ABI 2.22 新增 `articore_runtime_get_state_v2()`。它从 Motor 的低速反馈缓存一次读取
完整产品状态，不发送额外 CAN 请求；左右臂分别返回 `enabled_mask` 和
`enabled_valid_mask`，夹爪返回同语义的使能值与有效标志。状态码、反馈序列和反馈年龄在每台
Motor 的同一个缓存锁内读取；缺失、过期或未知状态不会被推断，而是由 SDK 映射为 `None`。
旧 `articore_runtime_get_state()` 和原结构保持不变，供已有二进制客户端继续使用。

Runtime ABI 2.23 增加产品级原生双臂五次轨迹。`start_trajectory()` 在返回前复制全部路点，
计算共享的中间速度/加速度和每段五次系数，并检查时间、有限值、产品关节限制以及多项式段内
位置、速度、加速度极值。执行不创建第二个线程：现有产品 worker 使用绝对单调时间在内部
500 Hz 调度下求值，并直接复用 Raw MIT/PV 整帧和 ControllerGroup 原子发送路径。MIT 每轴
显式携带目标速度、Kp、Kd 和前馈力矩；PV 每轴显式携带速度上限。部分使能时仍接收完整
14 轴轨迹，但只发送已主动使能的 Motor。取消为幂等操作，并把当帧转换为零目标速度、零
前馈的保持；disable、estop、disconnect、安全停机及 transport/send fault 都会终止轨迹。
轨迹状态通过统一原生 status/health 返回，底层不保存任何 Python 内存指针。

Runtime ABI 2.24 增加 `product_gripper_force_10_levels`。Yunyi 整机接口
`articore_runtime_set_grippers()` 的力度等级现在直接使用 1..10：1 最轻、10 最强、
5 为默认值。左右开合度仍为 0..1000，整机状态返回实际采用的同一十级值；SDK 应检查
新的产品级能力位，不能再把旧的通用十级夹爪能力误认为整机接口已经支持 1..10。

`motor-drive-layer` 0.10.27 将 `yunyi_gripper_v1` 的十级夹持标定整体增强一倍：运动和保持
刚度以及接触、过载力矩阈值均调整为原来的两倍。阻尼、运动速度、开合度换算、防堵转时序
和退让距离保持不变，因此用户接口、Runtime ABI 和 1..10 等级语义均不变。

Runtime ABI 2.25 增加 `articore_runtime_set_grippers_v2()` 和产品级
`product_gripper_direct_mode` 能力。默认 `PROTECTED` 模式保留接触/堵转检测、低刚度保持和
持续过载退让；新的 `DIRECT` 模式持续追踪目标开合度，不执行上述防堵转逻辑。v2 力度范围为
0..10：0 不施加主动夹持刚度，1..10 沿用现有十级标定。电机硬故障、反馈与 transport
安全、急停和整机失能始终有效。旧 `articore_runtime_set_grippers()` 保持 1..10 和默认保护
模式，现有 SDK 不受影响。

Runtime ABI 2.26 增加 `fixed_gripper_mit_mode` 保证。产品 `control_mode` 只作用于左右
14 个机械臂关节：PV 产品将关节配置为 PV，MIT 产品将关节配置为 MIT；左右夹爪在两种产品
模式下都固定配置并使用 MIT。这样夹爪不会因跟随整机 PV 模式而持续顶住目标并堵转，且模式
配置、使能当前位置保持、正常夹爪控制和恢复后的保持始终使用一致的 MIT 协议。

## 安全

电机控制可能造成意外运动和人身伤害。测试时必须支撑机构、准备独立急停、确认通道/ID/型号/
模式，并从保守限制开始。寄存器写入可能永久改变电机配置。

## 测试

默认测试不会使能真实电机：

```bash
cmake --build build -j
ctest --test-dir build --output-on-failure
```

CI 还会验证 wheel 不含 Python 源码、两个原生 ABI 库可加载、机器人模型不依赖 Pinocchio
动态库，以及各发布架构上的 DM Device 依赖可正确解析。

真机验收脚本位于 `scripts/`，必须先检查脚本，并显式提供电机映射和确认参数。

## 仓库结构

```text
cpp_damiao/              通用 C++ 协议、通信与电机 C ABI
articore_runtime/         原生产品 Runtime、机器人模型与 C/C++ ABI
packaging/pypi/           仅组装二进制 wheel；不含 Python 运行时模块
third_party/dm_device/    可选厂商头文件和可再分发动态库
scripts/                  构建、诊断与真机验收工具
tests/                    原生 CMake 包消费测试
```

## 许可证

Motor-Drive-Layer 使用 MIT 许可证。随 wheel 分发的 DM Device、libusb 和 libstdc++ 组件保留
其各自许可证说明，位于 `packaging/pypi/LICENSES/`。
