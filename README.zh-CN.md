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
          │ C++ 直接调用
          ▼
  分层 Motor C++ 核心
协议、Motor、Controller、双通道组
          │
          ▼
Linux SocketCAN-FD+BRS
```

职责划分如下：

- `cpp_damiao/` 负责内部 SocketCAN-FD、协议、Controller 和 Motor 分层。
- `articore_runtime/` 负责产品 Runtime、机器人模型和重力补偿。
- Articore-SDK 自己维护 Python `ctypes` 声明、值类型和用户接口。
- PyPI 的 `motor-drive-layer` wheel 只分发二进制文件，不包含 `.py` 或 `.pyi`，也不能
  `import motor_drive_layer`。

公开的原生产物包括：

- 产品 Runtime ABI：`libarticore_runtime.so`，声明位于
  [`articore/runtime_abi.h`](articore_runtime/include/articore/runtime_abi.h)。
- C++17 RAII 目标：`motorbridge::articore_runtime_cpp`。

## 原生能力

- 达妙 MIT 与位置速度产品控制。
- Yunyi 固定 `can-left`、`can-right` 两路 Linux SocketCAN-FD+BRS。
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
cmake -S . -B builds/cmake/default -DCMAKE_BUILD_TYPE=Release
cmake --build builds/cmake/default -j
ctest --test-dir builds/cmake/default --output-on-failure
```

安装原生 SDK 与 CMake 包：

```bash
cmake --install builds/cmake/default --prefix /desired/prefix
```

所有生成物统一放在 Git 忽略的 `builds/` 目录：

- `builds/cmake/default/`：当前原生构建。
- `builds/packages/`：打包中间产物。
- `builds/wheels/`：本地和发布 wheel。
- `builds/archive/`：保留的历史构建目录。

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
    └── libarticore_runtime.so
```

wheel 不提供 Python import 接口。Articore-SDK 通过发行包元数据定位动态库，并自行维护
ABI 声明。这样底层保持纯 C/C++，上层 SDK 的 `robot.connect()`、`enable()` 和重力补偿
交互方式不变。

编译原生库后可构建本地 wheel：

```bash
python3 -m build --wheel --outdir builds/wheels/current packaging/pypi
```

这里使用 Python 是因为 PyPI wheel 构建工具本身基于 Python；最终 wheel 不安装任何 Python
运行时代码。

## 通信行为

单电机 Controller 默认不增加发送延迟；添加第二个电机后，默认启用 120 µs 的最小帧间隔。
可通过 `MOTOR_DRIVE_LAYER_TX_GAP_US` 或原生配置接口修改。Yunyi 产品默认值已经完成
300 秒、16 电机、500 Hz 的 PV 保持真机测试：反馈稳定为 8,000 帧/秒，SocketCAN 队列
无积压且无 CAN 错误。

Linux SocketCAN-FD socket 使用非阻塞模式。内核发送队列持续满时，默认 20 ms
后返回错误，写入通信健康状态并传递给 Runtime 故障处理。可用
`MOTOR_DRIVE_LAYER_SOCKETCAN_SEND_TIMEOUT_MS` 设置 1 到 60000 ms 的超时。

内部 Controller 返回稳定的反馈错误码与缺失电机报告。Runtime 使用结构化结果，不解析错误
字符串做安全决策。

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

Runtime ABI 2.16 将 `articore_runtime_estop()` 改为无业务参数接口，并在 health 的
`fault_reason` 中记录固定原因 `emergency stop requested`。重复调用是安全的幂等操作；
`disable()`、清除电机故障以及断开后重连都不能解除急停锁存。ABI 2.33 已将早期的急停
整机失能行为替换为持续当前位置保持，只有 `recover()` 可以解除锁存。

Runtime ABI 2.17 重新定义 `recover()` 为可从任一活 Runtime 状态调用的完整整机恢复：必要时先
建立反馈连接，停止旧控制并确认失能，再并行清除左右通道的可恢复故障、验证 transport 与全部
新鲜反馈并配置产品控制模式，然后只为低速回到已标定关节零位而临时使能，回零验证通过后再次
整机失能。任一步骤失败都会再次尝试整机失能，并把失败阶段、稳定错误码、说明和失败电机写入
health。`clear_faults()` 仍然只清错、不运动；连接时发现可恢复电机故障会保持通信并进入
`FAULT`，清错成功后由底层重新配置产品模式和通信看门狗、再次确认整机失能，才进入
`READY`。`set_zero()` 仍然表示把当前位置标定为新的零点，两者不会被 `recover()` 混用。

Runtime ABI 2.18 将控制调度频率完全收归底层内部实现。删除公开的控制频率 getter 和 capability；
配置结构中为保持二进制布局而保留的两个频率槽位改为忽略的 reserved 字段。产品 Runtime 根据
内部产品与 transport 策略自行调度，Articore-SDK 和用户既不设置也不读取该频率。

Runtime ABI 2.19 将产品 `disconnect()` 定义为唯一的终止型安全关闭：停止接收和发送控制命令，
失能并确认全部已安装关节与夹爪，停止并回收 worker，关闭左右 CAN，最后释放 Controller、
ControllerGroup、Motor、模型及产品资源。重复调用安全幂等；失能确认失败时仍会先终止 worker，
再通过 Runtime 错误返回原因。旧 `close/free` C 符号仅为 ABI 兼容保留，SDK 在 ctypes 内部自动
释放空句柄，不再向业务用户公开 `close()`。

Runtime ABI 2.20 将产品面彻底收口为唯一的 Yunyi V1.0 双臂。SDK 和新的 C++ 包装器只调用
`articore_runtime_create_yunyi(mode, with_grippers, runtime_out)`，不再传 `product_id`，也不再公开通用
Runtime 构造、Controller/Motor 组装、关节映射、夹爪 profile 或重力模型绑定。固定双 CAN、
14 关节、可选双夹爪、方向/量程/限位、模型和资源生命周期统一放在独立的 C++ Yunyi 产品模块。
`create_product/create_ex*` 通用 C 构造入口已经删除；新主版本只保留固定产品入口，避免上层
重新组装 Motor、Controller 或产品配置。

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
轨迹状态通过统一原生 status/health 返回，底层不保存任何 Python 内存指针。规划时长结束后
Runtime 会继续发送最终保持目标，并使用新鲜的真实反馈检查位置误差与实际速度；只有连续
多个底层反馈样本稳定到位才返回 `COMPLETED`。此时 `progress=1` 但状态仍为 `RUNNING`
表示正在等待物理稳定，而不是已经到达。超过底层内部到位期限会将本次运动标记为 `FAULT`，
保持最终目标且不把电机切入故障模式，失败电机和具体误差统一写入 health。

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

Runtime ABI 2.27 增加 `direct_gripper_gain_x10`。仅在产品夹爪 `DIRECT` 模式下，Runtime
将所选力度等级的 Kp 和 Kd 放大 10 倍，同时把 MIT Kd 限制在协议最大值 5；默认 5 级为
Kp=80、Kd=5，10 级为 Kp=120、Kd=5。`PROTECTED` 模式的十级标定完全不变，力度 0 仍为
Kp=Kd=0。通信降级时仍应用 Runtime 的 25% 增益缩放。

Runtime ABI 2.28 增加 `product_cartesian_point_to_point` 与原生
`articore_runtime_move_pose()`。目标格式为 `[x, y, z, roll, pitch, yaw]`（米、弧度），
调用在底层完成 IK、产品限位及关节路径校验后立即返回，实际运动由 Runtime
内部控制线程异步执行。新的合法点目标会从当前规划运动状态原子覆盖旧点目标；新目标
校验失败时旧运动继续。显式多路点轨迹仍保持严格顺序且不会被此接口覆盖。该功能是关节
空间点到点，不保证笛卡尔直线。
全部产品笛卡尔运动接口仅支持 PV 产品模式；MIT 产品 Runtime 会在规划前拒绝调用，且不会
改变当前运动。

Runtime ABI 2.29 增加 `product_cartesian_linear`、统一的
`articore_runtime_move_cartesian()` 以及便捷接口 `articore_runtime_move_linear()`。
直线模式让 XYZ 始终位于起点到目标点的线段上，姿态采用最短路径四元数 SLERP。Runtime
按不大于 5 mm / 0.035 rad 的间距生成路径样本并连续求 IK；路径不可达、IK 分支跳变或任一
关节越界时会在安装前拒绝，新目标不会覆盖旧运动。实时控制线程只执行预先生成的关节
采样，并在每个底层周期发送普通 PV 帧，不在 500 Hz 周期内求 IK。PTP 与直线单目标运动
可以互相原子覆盖，显式多路点轨迹仍保持独立。

Runtime ABI 2.30 增加 `product_cartesian_circular` 和
`articore_runtime_move_circular()`。调用方传入三份完整的
`[x,y,z,roll,pitch,yaw]`：三点 XYZ 唯一确定从起点经过中间点到终点的圆弧，重复点和共线
点直接拒绝。声明起点必须在 5 mm / 0.035 rad 内匹配 Runtime 当前规划的法兰位姿，绝不会
被当作可瞬移的目标。姿态在起点→中间点、中间点→终点两段分别使用最短路径四元数
SLERP，并确保经过三份声明姿态。整条圆弧在原子覆盖旧的单目标笛卡尔运动之前，完成连续
IK、IK 分支连续性、产品限位和多项式段内极值校验。

Runtime ABI 2.31 增加 `product_cartesian_circular_auto_start` 和
`articore_runtime_move_circular_v2()`。调用方只传中间点与终点；Runtime 从当前规划关节
参考（不是滞后的电机反馈）计算圆弧起点，并在同一个底层命令事务内完成参考快照与新轨迹
安装，消除 SDK 先 `get_pose()` 再回传产生的竞态。旧三点接口继续保留 ABI 兼容。非 PV
模式会在覆盖现有运动之前拒绝调用。

Motor-Drive-Layer 0.10.33 在不改变 ABI 2.31 接口的前提下增强笛卡尔 IK。
点到点终点采用固定随机种子的 1000 次底层全局搜索；直线规划删除了生成连续路径之前那次
多余的孤立终点求解，中间采样继续使用当前关节解做局部连续 IK，最终采样再使用相同的
全局回退策略；圆弧终点同样处理。目标不可达、IK 分支不连续或越界时仍会在覆盖当前运动
之前返回失败。

Runtime ABI 2.32 新增 `ARTICORE_CAP_PRODUCT_TEMPERATURE_STATE` 和
`articore_runtime_get_state_v3()`。状态快照在 V2 的位置、速度、力矩和实际使能状态之外，
增加 14 个关节及已安装夹爪的 MOS 温度、转子温度和逐电机有效标志。温度单位为摄氏度，
来自现有 Motor 反馈缓存；接口不发送 CAN 请求，反馈缺失、过期或数值无效时返回 NaN 并
清除对应有效位。旧 V1/V2 状态接口和结构保持二进制兼容。

Runtime ABI 2.33 新增 `ARTICORE_CAP_LATCHED_ESTOP_POSITION_HOLD`，重新定义产品急停：
`estop()` 原子终止旧轨迹和用户目标，从新鲜反馈缓存捕获当前位置，并在 Runtime 安全周期内
持续发送 PV/MIT 位置保持帧。已使能电机保持使能，不再主动整机失能；原本已失能的产品也
不会被急停重新使能。急停保持继续检查反馈和发送结果，失败原因写入 health。急停锁存期间
拒绝新运动命令，重复调用幂等，仍只能通过 `recover()` 进入完整恢复流程。

Runtime ABI 2.34 新增 `ARTICORE_CAP_PRODUCT_JOINT_ANGLE_VEL_LIMITS` 和
`articore_runtime_get_joint_angle_vel_limits()`。接口一次返回固定 14 个机械臂关节的最小角度、
最大角度与产品速度上限，顺序为左 J1–J7、右 J1–J7，单位分别为 rad 和 rad/s。数据直接来自
Yunyi 内置产品配置，不发送 CAN 请求、不依赖连接状态，也不包含左右夹爪。

Runtime ABI 2.35 新增 `ARTICORE_CAP_PRODUCT_SPEED_SETTING`、
`articore_runtime_set_speed()` 和 `articore_runtime_get_speed()`。产品 Runtime 保存一个普通关节
运动速度设置，范围为 0–100，默认值为 70；14 个机械臂关节的 100 均对应 5 rad/s，因此默认
实际参考速度为 3.5 rad/s。修改设置会立即作用于正在执行的普通 MIT/PV 位置参考。
`articore_runtime_set_joint_positions_v2()` 使用当前设置，旧的带显式 `speed_percent` 参数接口
继续保留。Raw MIT/PV、原生轨迹和笛卡尔运动不受此全局设置影响。

Runtime ABI 2.36 将该设置的正式名称修正为“普通运动最大速度”，新增
`ARTICORE_CAP_PRODUCT_MAX_SPEED_SETTING`、`articore_runtime_set_max_speed()` 和
`articore_runtime_get_max_speed()`。范围、默认值、物理映射和动态生效行为不变。ABI 2.35 的
`set_speed/get_speed` 符号保留为兼容别名，新 SDK 应只公开 `set_max_speed/get_max_speed`。

Runtime ABI 2.37 新增 `ARTICORE_CAP_PRODUCT_TOOL_CENTER_POSE`，并将现有产品位姿统一定义为
实际笛卡尔控制点。有夹爪时，原生 FK、IK、点到点、直线和圆弧运动统一使用位于夹爪中心的
`l-tool0/r-tool0`；无夹爪时继续使用 `l-link7/r-link7`。公开接口仍然只有 `get_pose()`，
没有增加单独的法兰位姿方法。

Runtime ABI 2.38 新增 `ARTICORE_CAP_PV_MAX_SPEED_ONLY`。产品 SDK 的普通 PV 控制只保留
`set_max_speed(0..100)` 与不带速度参数的位置命令；默认最大速度仍为 70，Runtime 在原生周期
内按该上限逐步推进 reference。每次位置命令单独传速度以及 Raw PV 直发不再属于产品 SDK
接口。该最大速度设置只属于 PV；MIT 继续使用原有的逐命令速度、Raw 目标、Kp/Kd 和前馈
力矩逻辑，不受影响。旧 C ABI 符号只为已发布客户端保留二进制兼容，新绑定不得继续公开。

motor-drive-layer 0.10.39 修复 Yunyi 原生 PV 笛卡尔运动的终点保持抖动。Runtime 保持
0.02 rad / 0.05 rad/s 的公开到位窗口，但会先以低速继续收敛，并对笛卡尔终点执行
2.5 mm / 0.01 rad 的原生 FK 核验；核验通过后安装持续发送的零速最终位置帧，再用 200 ms
新鲜反馈确认稳定。`COMPLETED` 后仍持续监控反馈，持续失稳会重新进入等待稳定状态并写入
health。该修复不修改电机 Flash、零点或 PV 固件增益。

当前 PTP 在 C++ 中根据当前规划关节参考完成终点 IK，然后交给普通 PV 逐周期位置步进；
它不进入可查询的路径 motion FIFO。Linear 和 Circular 先生成连续笛卡尔样本并逐点求
IK，再由 Runtime 500 Hz worker 每 2 ms 采样并发送 PV 帧。只有直线和圆弧的新任务加入
FIFO，并在前一条经反馈确认到位后启动。

## 安全

电机控制可能造成意外运动和人身伤害。测试时必须支撑机构、准备独立急停、确认通道/ID/型号/
模式，并从保守限制开始。寄存器写入可能永久改变电机配置。

电机控制缺陷可能造成实体安全风险。不要在公开 Issue 中提供可直接导致失控运动、绕过限位、
关闭看门狗或远程驱动已连接硬件的完整操作步骤。GitHub 私密漏洞报告可用时应优先使用，并注明：
受影响的版本或提交、通信方式与适配器、电机型号与固件、复现是否必须使能电机、最小安全复现
步骤、预期与实际失效保护行为，以及已知缓解方案。没有安全影响的普通缺陷可以提交公开 Issue。
维护者会在条件允许时于安全环境中复现私密报告，并在修复可用后协调披露；目前不承诺固定响应
时限。

## 测试

默认测试不会使能真实电机：

```bash
cmake --build builds/cmake/default -j
ctest --test-dir builds/cmake/default --output-on-failure
```

CI 还会验证 wheel 不含 Python 源码、唯一的产品 Runtime 动态库可加载，以及机器人模型不依赖
Pinocchio 动态库。

Runtime ABI 3.1 延续 ABI 3.0 的唯一产品工厂：
`articore_runtime_create_yunyi(mode, with_grippers, runtime_out)`。函数返回稳定操作状态码，并
通过输出指针写入 Runtime 句柄。旧的两参数指针返回签名和临时 `_v2` 别名都已删除；
Articore-SDK 只绑定这一种工厂签名。ABI 3.1 另外统一了直线和圆弧运动的显式起点语义：
Linear 使用 start/end，Circular 使用 start/via/end。0.12.7 之前起点不匹配会拒绝；
0.12.7 起改为先完整预规划，再由同一底层任务自动 PTP 接近起点后执行路径。

Runtime ABI 3.2 明确区分 PTP 与路径运动。`move_pose()` 只提交点到点目标，不返回 motion
ID，也没有状态查询或取消接口；C++ 完成终点 IK 后，使用与普通 PV 相同的 500 Hz 逐周期
位置步进。只有直线和圆弧调用返回异步 motion ID，并支持状态、取消与 FIFO 排队。
Python 不实现 IK、插值、实时回放或队列调度。

Runtime ABI 3.3 新增 `articore_runtime_move_poses(left, right, speed)`，用于一次原子提交
双臂 PTP。C++ 从同一份规划参考求解左右终点 IK；只有两侧都成功后，才安装一份普通 PV
的 14 关节目标。这样两个单臂调用不会互相覆盖，也没有增加另一套插值器。

同一版本把显式起点的 Linear/Circular 改为底层复合 FIFO 任务。Runtime 在运动前一次性
规划并校验“普通 PV PTP 接近起点 + 完整直线/圆弧”；先到声明起点，再用真实反馈确认
位置误差不超过 5 mm、姿态误差不超过 0.035 rad，之后才放行路径。接近、反馈屏障和路径
共用一个 motion ID，取消时保持最后安全参考。笛卡尔路径时间戳也同时按速度和加速度约束
扩时，避免细分路径首尾五次曲线产生过大加速度。

| 参数 | 每周期 rad | 每周期 deg | 1 秒参考运动量 |
|---:|---:|---:|---:|
| 100 | 0.004 | 0.2292° | 2 rad / 114.59° |
| 75 | 0.003 | 0.1719° | 1.5 rad / 85.94° |
| 50 | 0.002 | 0.1146° | 1 rad / 57.30° |
| 16 | 0.00064 | 0.0367° | 0.32 rad / 18.33° |
| 0 | 0 | 0° | 0 |

真机验收脚本位于 `scripts/`，必须先检查脚本，并显式提供电机映射和确认参数。

## 仓库结构

```text
cpp_damiao/              内部分层协议、Motor 与 SocketCAN-FD 核心
articore_runtime/         原生产品 Runtime、机器人模型与 C/C++ ABI
packaging/pypi/           仅组装二进制 wheel；不含 Python 运行时模块
scripts/                  构建、诊断与真机验收工具
tests/                    原生 CMake 包消费测试
```
