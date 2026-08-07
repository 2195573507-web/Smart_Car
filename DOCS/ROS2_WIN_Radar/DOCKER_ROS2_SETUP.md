# Windows Docker Desktop 的 ROS2_WIN 部署准备

## 目标架构

```text
Windows 游戏本
  -> Docker Desktop（WSL2 Linux containers）
  -> Ubuntu 22.04 container
  -> ROS2 Humble
  -> colcon workspace
  -> Radar Driver / 网关桥接
  -> SLAM Toolbox
  -> Navigation2
```

这是后续开发设计，未创建容器、安装 ROS2、编译 workspace 或接入雷达。独立审计推荐 Ubuntu 22.04 + ROS2 Humble；Iron 已结束支持，不建议作为新环境基线。见 [`ROS2_ARCHITECTURE_AUDIT.md`](ROS2_ARCHITECTURE_AUDIT.md)。

## 运行域与职责

建议第一阶段把 driver（或网关桥）、静态 TF、里程计桥、SLAM Toolbox、Nav2 放在同一个 Linux ROS2 域中。这样设备访问和 DDS 发现不必先跨 Windows、WSL2 和多个容器排障。

| 层 | 建议职责 | 验证状态 |
| --- | --- | --- |
| Windows | Docker Desktop、IDE、终端、可选 GUI 客户端 | 待验证 |
| Docker/WSL2 | Linux 容器运行时与设备转交边界 | 待验证 |
| Ubuntu + ROS2 Humble | ROS2/DDS、workspace 与算法依赖 | 待验证 |
| Radar 输入 | 直连 driver 或 ESP32-S3 网关桥接 | 路径待决策 |
| SLAM/导航 | `/scan`、TF、里程计到地图和速度命令 | 待验证 |

当前项目基线优先 `YDLIDAR -> ESP32-S3 -> Wi-Fi -> ROS2_WIN`；Docker 直连雷达是需新授权的开发机验证路径。不得让两端同时占用或解析同一雷达数据源。

## GPU 支持

2D 雷达 driver、SLAM Toolbox 与 Nav2 不以 GPU 为前置条件。若后续使用 NVIDIA GPU 进行 RViz、仿真或视觉计算，应分别验证 Windows/WSL2 GPU 驱动、容器 GPU 可见性和 OpenGL 图形路径。GUI 失败不能作为串口、`/scan` 或 SLAM 失败的证据。

## USB 与串口映射

Windows `COMx` 不能被直接假定为 Linux 容器的 `/dev/ttyUSB*` 或 `/dev/ttyACM*`。推荐先在 WSL2/Linux 层完成 USB/IP 或受支持的设备转交验证，再确认容器看见稳定的字符设备、权限和重插行为。

| 项目 | 设计要求 | 本次状态 |
| --- | --- | --- |
| USB 映射 | 设备先稳定转交到 WSL2/Linux | 未验证 |
| 串口映射 | 仅映射实际确认的一台字符设备，单进程独占 | 未验证 |
| 设备标识 | 优先稳定标识，不依赖固定 `ttyUSB0` | 未验证 |
| 资料参考 | 树莓派教程包含 Linux Docker 硬件交互说明 | 仅静态资料确认 |

Linux Docker 的 `--device=/dev/...` 例子只可作为资料参考，不能证明 Windows Docker Desktop 设备可用。相关本地资料：[`4、docker硬件交互和数据处理.md`](../../资料/EAI%20X3%26X3%20Pro激光雷达/树莓派5使用教程/4、docker硬件交互和数据处理/4、docker硬件交互和数据处理.md)。

## 网络模式与 DDS

ROS2 依赖 DDS 的 UDP、组播发现和多个端口；Docker Desktop NAT、WSL2 虚拟网络及 Windows 防火墙会造成跨域不稳定。建议分阶段验证：

1. 同一 Linux 容器内验证 driver/bridge、TF、SLAM 与 Nav2。
2. 再验证同一 Docker 网络内的容器间通信，并固定 ROS 发行版、RMW 和 `ROS_DOMAIN_ID`。
3. 最后才接入 Windows 原生 ROS2、局域网节点或车载网关，并基于真实网卡/IP 规划配置防火墙与显式发现策略。

不要把 `--network host` 当成 Docker Desktop 跨 Windows DDS 的通用答案；其语义与原生 Linux Docker 不同。`ROS_LOCALHOST_ONLY` 只适合明确的本地调试，跨容器/跨主机时必须由实际网络策略决定。

## 文件挂载与 workspace

建议将可编辑的 `src/` 挂载到容器内 workspace；将 `build/`、`install/`、`log/` 放在容器 named volume 或 Linux 文件系统，减少 Windows 文件共享的 UID、换行符和小文件 I/O 干扰。将 map、rosbag、YAML 参数和导出数据放入明确的数据挂载目录，避免把运行数据写进镜像层。

后续应以 Dockerfile/Compose 固定 Ubuntu/ROS/依赖版本和挂载规则，而不是把交互式容器作为唯一环境记录。本任务不创建这些配置文件。

## 建议的部署与验收顺序

1. 锁定“ESP32-S3 网关”或“Docker 直连”架构路线。
2. 建立空的 Humble workspace 与可复现镜像定义。
3. 验证 Linux 设备可见性、供电、权限、串口独占与重插。
4. 验证 SDK/driver 的 `/scan`、时间戳、`frame_id` 与异常恢复。
5. 验证静态 TF、里程计与 `map -> odom -> base_link -> laser_frame` 树。
6. 验证 SLAM Toolbox，再验证 Nav2 costmap 与 `/cmd_vel` 安全网关。
7. 最后进行封闭场地的建图、定位、导航、急停和断链降级测试。

第 1 至 7 步均待后续实施和验收。
