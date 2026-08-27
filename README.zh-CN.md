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

- 包版本：`0.19.0`
- Runtime ABI：`10.0` / `0x000A0000`

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

普通 PV 与 `move_pose()` 共用同一套 500 Hz 参考生成器。每条命令的 `speed`
直接把 `0～100` 映射为 `0～2 rad/s`，不存在额外的持久化最大速度上限。
最大加速度使用真实物理单位和 `0.01` 设置精度，范围 `0.01～8.00 rad/s²`、
默认 `4.00 rad/s²`；它不影响 MIT，也不影响 Joint/Linear/Circular 原生轨迹。

笛卡尔 PTP 只有一个 `move_pose(left_pose, right_pose, speed)`：与
`set_joint_pv(left, right, speed)` 一样提交完整双臂目标，只额外在 C++ 中对两侧
Pose 求终点 IK。两侧共用同一份规划参考快照，全部在 `1e-4` 精度和 8 ms 软预算
内成功后才原子安装 14 关节 PV 目标；任一侧失败或超时都不改变当前目标。

关节轨迹、Linear 和 Circular 共用一个 Motion ID、FIFO、状态查询和取消
接口。Linear/Circular 传入计划运行时长 `duration_s`，不再使用速度百分比；
若 Runtime 需要先 PTP 到显式起点，该接近段也包含在总时长中。PTP 仍是普通
PV 点目标，不加入 Motion FIFO。

当前 API 见 [Runtime 说明](articore_runtime/README.md)。

## 构建与测试

```bash
cmake -S . -B builds/dev -DCMAKE_BUILD_TYPE=Release
cmake --build builds/dev -j
ctest --test-dir builds/dev --output-on-failure
```

真机诊断程序不会加入 CTest，需要单独构建。它们可能使能或移动机械臂，必须
确认现场安全后运行。本地开发、编译和测试不需要先发布 PyPI。
