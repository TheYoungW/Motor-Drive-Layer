# RK3588 产物上传与部署记录

本文记录 motorbridge ARM64 Debian 产物上传到固定 RK3588 测试机的流程。
后续版本沿用本流程，但上传前必须重新计算并核对当前产物的 SHA256。

## 固定目标

```text
SSH 用户：neardi
设备地址：192.168.1.185
远端目录：/home/neardi/runtime
设备架构：aarch64
```

仓库内当前 ARM64 产物约定为：

```text
dist/arm64/articore-runtime-service_<version>_arm64.deb
```

当前 1.1.0 产物：

```text
dist/arm64/articore-runtime-service_1.1.0_arm64.deb
SHA256: 7c05600c5512b071f792ba8481eaa0d674972324f40580ce84518ee7de9bb466
```

## 上传

从仓库根目录执行：

```bash
scp \
  dist/arm64/articore-runtime-service_1.1.0_arm64.deb \
  neardi@192.168.1.185:/home/neardi/runtime/
```

密码不得写入仓库、脚本或命令行参数。使用交互式密码认证，或者在测试机上
配置 SSH 公钥。自动化上传推荐使用 SSH key。

## 上传后校验

先计算本地产物摘要：

```bash
sha256sum dist/arm64/articore-runtime-service_1.1.0_arm64.deb
```

然后检查远端设备和产物：

```bash
ssh neardi@192.168.1.185 '
  uname -m
  dpkg-deb -f \
    /home/neardi/runtime/articore-runtime-service_1.1.0_arm64.deb \
    Package Version Architecture
  sha256sum \
    /home/neardi/runtime/articore-runtime-service_1.1.0_arm64.deb
'
```

验收条件：

- `uname -m` 必须是 `aarch64`。
- Debian `Architecture` 必须是 `arm64`。
- 本地与远端 SHA256 必须完全一致。
- 上传本身不授权安装、启动、enable、校零或运动电机。

## 安装与启动

取得安装授权后，在 RK3588 上执行：

```bash
cd /home/neardi/runtime
sudo apt-get -o Dpkg::Options::="--force-confnew" \
  install ./articore-runtime-service_1.1.0_arm64.deb
```

`--force-confnew` 仅用于从已手工修改配置的旧测试包升级：它选择 1.1.0
随包提供的新配置（`wlan0`、真实 USB-CAN 序列号和 0.875 数据相位采样点）。
如设备配置与本机不同，应先备份配置，并改用普通 `sudo apt install` 人工处理
conffile 差异。

1.1.0 的安装脚本会重新加载 udev、enable CAN/Runtime，并在 Runtime 尚未
运行时先初始化 CAN，再启动 Runtime。若安装前 Runtime 已在运行，为避免升级
过程打断机器人或清除锁存 FAULT，安装脚本不会自动重启它；应进入安全维护
状态并取得操作授权后执行：

```bash
sudo systemctl restart articore-can.service
sudo systemctl restart articore-runtime.service
```

安装脚本还会识别 1.0.0 在板上临时创建的两个、内容完全匹配的 `/usr/local`
systemd override，将它们移出 systemd 搜索路径并备份到
`/etc/articore/migration-backups/1.1.0/`。有任何额外定制的 override 都不会
自动移动，而会保留并打印提示，供操作员人工检查。

检查状态和日志：

```bash
ip -details link show can-left
ip -details link show can-right
systemctl status articore-can.service --no-pager
systemctl status articore-runtime.service --no-pager
journalctl -u articore-runtime.service -n 200 --no-pager
```

Runtime 启动后保持电机 disabled。电机 enable、维护操作和运动命令必须通过
DDS 控制租约另行明确执行。

SDK 1.0.2 的清错示例使用显式维护连接并维持 lease/heartbeat，无论 Runtime
处于 `READY` 还是 `FAULT` 都不发送 `CONFIGURE_MODE`。Runtime 1.1.0 的
`CLEAR_FAULTS` 只清错、验证反馈和确认失能，不再夹带模式/通信看门狗寄存器
配置；普通业务会话在清错成功后另行配置 PV/MIT。普通清错仍会拒绝急停锁存，
拒绝回复会携带当前状态、故障原因和失败电机。

## 1.1.0 板卡默认值、末端扫描与热插拔

- DDS 默认接口：`wlan0`。
- USB-CAN VID/PID：`1d50:606f`。
- 左侧序列号：`015213EF68D8345BBAA6D57818A4EC3A`。
- 右侧序列号：`AEEDE4FD23DEA4AFCA6B3EAA55ABC28A`。
- CAN-FD：1 Mbit/s arbitration、5 Mbit/s data、数据相位 sample point 0.875。
- Runtime 启动时分别扫描 `can-left` 和 `can-right`：ID 1..7 必须存在，ID 8
  作为可选夹爪独立识别；允许单侧夹爪。
- 更换末端必须先确认电机失能，再由操作员重启 Runtime。运行期间不会因单次
  丢帧自动改变拓扑，也不会在重启后自动使能。

udev 热插拔会重新配置 CAN。脚本会在操作 CAN 前记录 Runtime 是否运行；若
当时正在运行，即使 systemd 因 `Requires=` 联动停止了 Runtime，也绝不会自动
把它重新拉起，因为重启可能清除内存中的锁存 FAULT。此时应检查健康状态，再
由操作员决定是否在安全条件下重启。热插拔前 Runtime 已停止时，CAN 初始化
成功后热插拔服务才会启动 Runtime。

## 后续版本更新

发布新版本时需同时替换以下三处：

1. 本地 `.deb` 文件名。
2. 远端 `.deb` 文件名。
3. 本地和远端校验所使用的 SHA256。

不要仅凭文件名判断产物是否正确，也不要把 `amd64` 验证包上传到 RK3588。
