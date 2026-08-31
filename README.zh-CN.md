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

- 包版本：`0.25.0`
- Runtime ABI：`13.0` / `0x000D0000`

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
控制周期重复刷新最终目标 P，并只动态更新电机 V 包络；不生成中间 P 点。目标覆盖时
立即发送新的最终 P，同时延续当前 V 的加减速状态。
共享的 `set_speed_percent(1～100)` 对普通 PV 的运动限制做时间缩放；旧的
单次 PV 与 `set_pose` 百分比参数会更新同一个共享值。用户未设置上限时，100% 对应
J1～J7 速度 `[180,180,180,225,225,225,225]°/s`、加速度
`[450,450,900,900,900,900,900]°/s²`。可选的 `set_max_speed()` 和
`set_max_acceleration()` 会把用户值作为全部 14 个关节在 100% 时的共同基础上限；
传入 0 清除对应设置并恢复逐关节默认值。比例 `s` 对速度按 `s` 缩放、对加速度按
`s²` 缩放。每周期 POS_VEL 的 `V` 根据有效上限、剩余距离和当前 V 包络状态动态得到，
不会一直发送最大值。
MIT 产品模式向用户提供两个只接收关节角的端点方法。普通
`set_joint_mit()` 直接发送每次最新的完整 14 关节终点，不生成中间位置参考；固定
J1～J7 参数为 `Kp=[15,15,12,12,8,7,6]`、
`Kd=[0.8,0.8,0.7,0.7,0.5,0.5,0.4]`。快速跟随
`set_joint_mit_fast_follow()` 用于高频遥操，用户只传最新关节角；Runtime 内部固定使用
J1～J7 `Kp=[190,190,100,100,70,60,50]`、
`Kd=[4.55,4.50,2.50,2.50,0.70,0.60,0.50]`，并按 100%（5 rad/s）步进，
速度参数不向用户开放。旧的带 MIT 速度百分比入口仅为 ABI 兼容保留，新 SDK 不应暴露。
Linear/Circular 保留独立的内部基础速度、加速度、时间和同步约束，但与普通
PV 共用同一个 Runtime 速度百分比。用户只填写路径几何，不再填写时长；Runtime
自动计算安全执行时间，轨迹基础上限仍是内部策略，不作为接口暴露。
Pose 调用方通过纯计算接口 `solve_ik(left_pose, right_pose)` 获得 14 个关节角，
再选择普通 PV 或 MIT 执行；`set_pose()` 只求解一次 IK，然后按 Runtime 当前选择的
普通 PV 或普通 MIT 直接端点模式原子安装关节终点，不是轨迹规划器。

`solve_ik(left_pose, right_pose)` 只求解、不运动：两侧共用同一份规划参考快照；
若尚未使能，则使用已连接 Runtime 的新鲜完整反馈，
以当前 TCP、产品限位和最近关节分支求解固定顺序的 14 个关节角。它不会使能 Motor、
发送控制帧或改变运动队列。兼容的 `set_pose()` 只有在两侧都于 `1e-4` 精度和
8 ms 软预算内成功后，才按当前普通 PV/MIT 模式原子安装求解出的关节目标；任一侧失败或超时都不改变
当前目标。它不属于 Linear/Circular 路径规划，也不创建关节轨迹。

Linear 和 Circular 共用一个 Motion ID、FIFO、状态查询和取消接口。
Linear 从当前/指定起点的 FK Pose 出发，在笛卡尔空间对 XYZ 做直线插值、对姿态做
四元数最短路真 SLERP。第一个路径 Pose 只使用当前规划关节角作为 IK seed，后续每个
Pose 只使用上一个 IK 解；Linear/Circular 路径 IK 不使用备用或随机 seed，当前局部分支
同时使用零空间姿态约束保持靠近上一个解，并根据上一段关节步长预测下一次向前半步的
偏好姿态，让冗余关节尽量保持速度方向连续、减少 `+/-` IK 抖动。这是优化目标，不是
关节换向拒绝规则；笛卡尔路径确实需要换向时仍允许通过。只有局部分支无法收敛或单个
采样关节跳变超过 0.35 rad 时才拒绝路径。几何路径按
2 mm / 0.1 rad 或更细离散，再用一条全局五次时间律生成固定 10 ms 间隔的内部实时
PV 关键点。Runtime 根据缩放后的关节参考速度/加速度限制，自动选取满足限制的最短
完整 10 ms 时长；相同路径在 50% 下通常约为 100% 的两倍时间。Runtime 在相邻关键点之间按时间
线性重采样，并以 500 Hz 发送所得参考，不叠加普通 PV 的在线终点调速器。对于小于达妙
16-bit 位置反馈一个量化级（约 0.000381 rad / 0.02186°）的逐周期 P 变化，Runtime
保持上次有效 P 并累计差值，达到一个量化级后再更新；精确终点始终强制发送。
POS_VEL 的内部 V 同时根据当前规划关节速度动态调整，并受产品上限
约束，避免慢速轨迹仍以 3 rad/s 上限反复追赶离散 P。Circular 由
start/via/end 三点确定有向圆弧，
位置按 2 mm 或更细离散，姿态以最短路真 SLERP 经过 via 姿态；随后使用同一套全局五次
时间律和 10 ms 实时 PV 参考。真实到位可能因
PV 限速、加速度限制和反馈稳定确认而更晚。连续的 Linear/Circular FIFO 在公共端点
且跟踪误差不超过 0.04 rad
时按计划时刻直接交接，不再为每段额外等待 200 ms 稳定窗口；超出门槛时仍等待
真实跟踪恢复。Linear 和 Circular 都会按 10 ms 相邻关节参考检查共享百分比缩放后的
内部 `1 rad/s` 速度基础和 `6 rad/s²` 加速度基础，并据此自动完成时间参数化，
不会向 PV 驱动提交过大的阶跃。
同一个 `move_linear_trajectory()` 也可一次提交 2～64 个 Pose：两个 Pose 保持
普通直线语义，三个及以上 Pose 对每个有效内部角点默认使用 10 mm 笛卡尔圆角，
短线段自动缩小圆角以避免相邻过渡重叠。整条折线只生成一个 Motion ID，并使用
一条全局五次时间律；整条路径的时间由 Runtime 自动计算。
公开的 `set_joint_pv()` 是用户使用的普通 PV 终点接口；实时 PV 只允许由
Runtime 内部已经完成校验的有限 Linear/Circular 轨迹选择，不导出 Raw PV 或
流式 PV 普通接口。自动接近属于同一个内部轨迹执行链。Linear、Circular 要求 PV
产品模式；`set_pose()` 支持当前普通 PV 或普通 MIT 直接端点模式，且不加入 Motion FIFO。
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
