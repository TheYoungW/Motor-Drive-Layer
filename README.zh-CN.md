# Motor Drive Layer

Yunyi 双臂产品的原生 C++ 控制底层。

`main` 只保留 Linux 下的双 SocketCAN-FD+BRS 产品路径：`can-left` 与
`can-right`。原来的多种 CAN、Damiao SDK 架构单独保存在
`legacy-multi-can-damiao-sdk` 分支。

## 架构

```text
Articore SDK
    -> Yunyi Runtime C ABI
        -> 500 Hz 整机 Runtime 与安全状态机
            -> 原生 C++ Motor 核心
                -> 左右 Controller 发送 worker
                    -> SocketCAN-FD+BRS 帧

can-left/right 接收线程
    -> Motor 反馈缓存
        -> Runtime 整机状态与 health 快照
            -> Articore SDK
```

公开 wheel 只包含 `libarticore_runtime.so`。Runtime 在同一 C++ 进程内直接
调用 Motor 核心，不再经过 Motor C ABI；本仓库也不包含 Python 控制实现。

## 当前接口契约

- 包版本：`0.28.0`
- Runtime ABI：`15.0` / `0x000F0000`

Runtime health 现在按产品顺序返回每个已安装电机的角色名、CAN ID、反馈年龄、
状态码和问题位，并明确区分单电机反馈异常、多电机异常、左/右通道停收和双通道停收。
- ABI 校验：必须完全相等
- 产品：`yunyi_v1_0`
- 关节顺序：左 J1～J7、右 J1～J7
- 夹爪：创建 Runtime 时选择是否安装左右成对夹爪

SDK 通过 `articore_runtime_create_yunyi(mode, with_grippers, &runtime)`
创建完整产品。Motor 映射、Controller、限位、模型、TCP 偏移、worker 和
资源生命周期都由 Runtime 管理。公开 ABI 不包含 Motor 指针、通用组装接口、
能力位或带版本后缀的重复函数。

普通关节 PV 是给用户开发使用的“最新终点覆盖”控制。一次
`set_joint_pv(left, right, speed)` 提交完整的 14 关节目标；下一次调用原子覆盖上一目标，
Runtime 不生成有限的七次/五次曲线，也不创建 Motion ID。Runtime 在每个 500 Hz
控制周期从第一帧开始直接发送最终 P，只整形电机 V。V 按配置的加速度上升和下降，
不超过用户给定的最大速度，并根据反馈的实际剩余距离提前刹车；这是在线 V 整形，
不是有限点列或轨迹任务。P 不叠加位置前馈。Runtime 根据实际剩余距离持续降低 V，
制动曲线在终点降到 0；最终误差由电机原有 KP/KI
位置环消除，反馈稳定后切换到安静的 V=0 保持。
共享的 `set_speed_percent(1～100)` 对普通 PV 的运动限制做时间缩放；
单次 PV 的百分比参数也会更新同一个共享值。用户未设置上限时，100% 对应
J1～J7 速度 `[180,180,180,225,225,225,225]°/s`、加速度
`[450,450,900,900,900,900,900]°/s²`。可选的 `set_max_speed()` 和
`set_max_acceleration()` 会把用户值作为全部 14 个关节在 100% 时的共同基础上限；
传入 0 清除对应设置并恢复逐关节默认值。比例 `s` 对速度按 `s` 缩放、对加速度按
`s²` 缩放。每周期 POS_VEL 的 `V` 根据有效的正常运动或末端阶段上限选择，不会一直
发送同一个固定最大值。
MIT 产品模式只向用户提供两个方法。标准
`set_joint_mit(q, dq, kp, kd, tau_ff)` 由用户填写完整的 14 关节 MIT 帧；新帧原子覆盖
旧帧，Runtime 不做插值，并继续受流式看门狗保护。快速
`set_joint_mit_fast(q)` 用于高频遥操，用户只传最新关节角；Runtime 内部固定使用
J1～J7 `Kp=[190,190,100,100,70,60,50]`、
`Kd=[4.55,4.50,2.50,2.50,0.70,0.60,0.50]`，并按 100%（5 rad/s）步进，
同时固定 `dq=0`、`tau_ff=0`。两个方法都不接收 `speed_percent`，也不创建有限轨迹或
Motion ID。
Linear/Circular 保留独立的内部基础速度、加速度、时间和同步约束，但与普通
PV 共用同一个 Runtime 速度百分比。用户只填写路径几何，不再填写时长；Runtime
自动计算安全执行时间，轨迹基础上限仍是内部策略，不作为接口暴露。
Pose 调用方可通过纯计算接口 `solve_ik(left_pose, right_pose)` 获得 14 个关节角而不
运动，再显式提交给普通 PV 或标准 MIT。ABI 15 删除了原先模式中立的 `set_pose()`
快捷接口；公开的 `move_pose(side, target_pose)` 则规划一条五次时间律 Pose-to-Pose
有限笛卡尔轨迹。

`solve_ik(left_pose, right_pose)` 只求解、不运动：两侧共用同一份规划参考快照；
若尚未使能，则使用已连接 Runtime 的新鲜完整反馈，
以当前 TCP、产品限位和最近关节分支求解固定顺序的 14 个关节角。它不会使能 Motor、
发送控制帧或启动运动。

Linear 从当前/指定起点的 FK Pose 出发，在笛卡尔空间对 XYZ 做直线插值、对姿态做
四元数最短路真 SLERP。第一个路径 Pose 只使用当前规划关节角作为 IK seed，后续每个
Pose 只使用上一个 IK 解；Linear/Circular 路径 IK 不使用备用、随机或外推 seed。
位置优先的零空间姿态约束使每个解保持靠近紧邻的上一个解，XYZ 误差限制为 0.5 mm，
姿态残差允许到 0.035 rad。单关节分支跳变超过 0.35 rad 或短距离内反复出现明显
`+/-` 换向时拒绝整条路径；小于 1 mrad 的符号变化视为数值噪声或自然极值。
几何路径按 2 mm / 0.1 rad 或更细离散，五次时间律根据距离、速度、加速度、相邻关节
步长和线性化误差生成 4～50 ms 自适应 `TrajectoryPv` 关键点；最大相邻关节步长为
0.02 rad。相同路径在 50% 下通常约为 100% 的两倍时间。Runtime 在相邻关键点之间按时间
线性重采样，并以 500 Hz 发送所得参考，不叠加普通 PV 的在线终点调速器。对于小于达妙
16-bit 位置反馈一个量化级（约 0.000381 rad / 0.02186°）的逐周期 P 变化，Runtime
保持上次有效 P 并累计差值，达到一个量化级后再更新；精确终点始终强制发送。
POS_VEL 的内部 V 同时根据当前规划关节速度动态调整，并受产品上限
约束，避免慢速轨迹仍以 3 rad/s 上限反复追赶离散 P。Circular 由
start/via/end 三点确定有向圆弧，
位置按 2 mm 或更细离散，姿态以最短路真 SLERP 经过 via 姿态；随后使用同一套全局五次
时间律和自适应 `TrajectoryPv` 参考。真实到位可能因
PV 限速、加速度限制和反馈稳定确认而更晚。Linear 和 Circular 都会根据共享百分比缩放后的内部 `1 rad/s` 速度基础
和 `6 rad/s²` 加速度基础自动完成时间参数化，
不会向 PV 驱动提交过大的阶跃。
`articore_runtime_move_linear(runtime, side, start_pose, end_pose)` 是统一的
Linear 有限运动入口：`start_pose == NULL` 时从当前规划位姿直接规划到 `end_pose`；
非空时由 Runtime 一次性规划“当前规划位姿 -> start_pose -> end_pose”，将自动接近段和
直线段作为一条有限轨迹、一次提交，不使用普通 PV 终点接口预定位。
同一个 `move_linear()` 也可一次提交 2～64 个 Pose：两个 Pose 保持
普通直线语义；三个及以上 Pose 精确保留每个内部角点，不再插入笛卡尔圆角。每个尖角
使用独立的静止到静止五次时间律，使 TCP 精确到点并避免非零速度瞬时改变方向。
整条折线的时间由 Runtime 自动计算。
公开的 `set_joint_pv()` 是用户使用的普通 PV 终点接口；`TrajectoryPv` 只允许由
Runtime 内部已经完成校验的有限 Linear/Circular 轨迹选择，不导出 Raw PV 或
流式 PV 普通接口。自动接近属于同一个内部轨迹执行链。`move_pose()`、Linear 和
Circular 要求 PV 产品模式。公开调用都是非阻塞发送接口，不返回 Motion ID；Runtime
同一时间只接受一条有限笛卡尔运动。整机状态中的 `motion_arrived` 表示真实反馈已稳定
到位，`stop_motion()` 停止当前运动，等待、故障与超时策略由应用负责。
公开 Runtime 不再提供关节点到点轨迹规划。

当前 API 见 [Runtime 说明](articore_runtime/README.md)。

## 构建与测试

```bash
cmake -S . -B builds/dev -DCMAKE_BUILD_TYPE=Release
cmake --build builds/dev -j
ctest --test-dir builds/dev --output-on-failure
```

真机诊断程序不会加入 CTest，需要单独构建。它们可能使能或移动机械臂，必须
确认现场安全后运行。本地开发、编译和测试不需要先发布 PyPI。
