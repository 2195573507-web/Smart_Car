# ROS2_WIN Docker 架构独立审计

## 审计范围与结论

**范围。** 本文独立审计下列目标架构的可行性，不安装软件、不启动容器、不编译 ROS 2、不连接雷达，也不修改任何源码或工程配置：

```text
Windows 游戏本
  -> Docker Desktop（WSL 2 Linux containers）
  -> Ubuntu + ROS 2 Humble
  -> colcon workspace
  -> YDLIDAR X3/X3 Pro driver
  -> SLAM Toolbox / Navigation2
```

**结论。** 该方案可作为后续开发目标，但只有在「USB 转 UART 设备稳定进入 Linux 容器」和「DDS 图发现按预期工作」两项实际验证通过后，才可作为可运行平台。推荐基线为 **Ubuntu 22.04 + ROS 2 Humble**；Iron 已结束支持，不应作为新环境的首选。

现有项目基线存在必须先确认的边界：[`PROJECT_ARCHITECTURE.md`](../PROJECT_ARCHITECTURE.md) 将 YDLIDAR 采集与解析归属给 ESP32-S3，ROS2_WIN 通过 Wi-Fi 接收网关信息，并明确排除了 ROS2 直接雷达解析。因而，把雷达直接接入 Windows/Docker 并让容器运行 YDLIDAR driver 是一条新的所有权路线，不是现有基线已确认的实现。本文不替代该架构决策。

## 审计发现

| ID | 级别 | 发现 | 影响 / 处置 |
| --- | --- | --- | --- |
| A-01 | 高 | Docker Desktop 的 Linux 容器运行在 WSL 2/虚拟化 Linux 环境中，不等价于原生 Linux 主机。Windows 的 `COMx` 不能直接作为 Linux 容器的 `/dev/tty*` 使用。 | 必须先验证 USB/IP 或 WSL 2 到容器的设备可见性与读写权限；不能把 Raspberry Pi 的 `--device=/dev/ttyUSB0` 示例直接套用到 Windows。 |
| A-02 | 高 | 当前项目架构将雷达采集归 ESP32-S3；直接 Docker driver 与该归属冲突。 | 在实现前由任务所有者选择并记录「直接接入」或「ESP32-S3 网关」之一，避免两端同时竞争设备或重复解析。 |
| A-03 | 高 | ROS 2 DDS 依赖 UDP、组播发现及多端口通信。Docker Desktop NAT、WSL 2 虚拟网络和 Windows 防火墙会使跨 Windows/容器或跨主机发现不稳定。 | 初期把所有 ROS 2 计算节点放在同一个 Linux 容器网络域；若跨域，再使用固定 DDS 实现、统一 `ROS_DOMAIN_ID` 与显式单播 peer 配置验证。 |
| A-04 | 中 | `--network host` 的语义在 Linux Docker、Docker Desktop 和 Windows 主机之间不同，不能把它视为跨 Windows 的 DDS 解决方案。 | 不以 host 网络作为首个设计前提；先用单容器/同一 Docker 网络建立闭环。 |
| A-05 | 中 | 资料中的 Docker 设备映射和 udev 示例面向 Linux/树莓派宿主机。 | 可作为 Linux 容器设备模型参考；Windows 上的设备发现、转交和重插行为必须单独实测。 |
| A-06 | 中 | 2D SLAM 与 Nav2 不只需要 `/scan`；还需要时间戳、`laser` 到 `base_link` 的静态 TF、里程计及连续 `odom -> base_link` TF。 | 雷达可见不代表可建图/导航；将 TF、里程计、时间和底盘 `cmd_vel` 接口列为独立集成验证项。 |

## 推荐拓扑与待决策路径

### 路径 A：直接接入 Windows + Docker（仅在架构边界获批后）

```text
YDLIDAR X3/X3 Pro
  -> 官方 USB 转 UART 适配板 / 已验证的 USB-串口桥
  -> Windows USB 设备
  -> USB/IP 或受支持的 WSL 2 设备转交
  -> Linux 容器中的 /dev/ttyUSB* 或 /dev/ttyACM*
  -> ydlidar ROS 2 driver
  -> /scan (sensor_msgs/msg/LaserScan)
  -> SLAM Toolbox -> /map, map -> odom
  -> Nav2 -> /cmd_vel
  -> 已定义的 ESP32-S3/STM32 网关协议
```

驱动、SLAM Toolbox、Nav2 和 RViz2 应优先同处于一个 Linux ROS 2 运行域中。这样串口访问、DDS 发现和 Linux 依赖只在一个边界内处理。Windows 可承担 Docker Desktop、IDE、终端和可选的可视化客户端；不要在第一阶段把 Windows 原生 ROS 2 节点加入数据链。

这一路径的前置验收是：

1. 插拔后 Windows 能识别所用 USB-串口桥，且稳定得到预期设备身份。
2. 该设备能被转交给目标 WSL 2 / Docker Linux 环境；容器内出现稳定的字符设备名并具有读写权限。
3. 容器中只有一个进程打开该串口，使用资料与设备实物确认的波特率和扫描参数。
4. driver 发布有效 `/scan`，并且时间戳、`frame_id`、范围/角度数据合理。

若第 2 项不能稳定达成，不应以 `--privileged`、宽泛设备挂载或反复更换端口号掩盖问题；应改用路径 B 或独立 Linux 采集端。

### 路径 B：遵守当前 ESP32-S3 网关边界（当前基线一致）

```text
YDLIDAR
  -> UART
  -> ESP32-S3（采集 / 网关，具体协议尚未定义）
  -> Wi-Fi
  -> ROS2_WIN Linux 容器中的网关桥接节点
  -> /scan + TF / odom 约定
  -> SLAM Toolbox / Nav2
```

这一路径与项目现有文档一致，也避开了 Windows USB 到容器的直接映射难题。代价是必须在后续任务中明确定义网关协议、扫描数据完整性、时间同步、重连、背压和故障状态。当前禁止修改 ESP32-S3 功能，因此它只是架构建议，不表示网关已可用。

### 推荐决策

在没有新架构授权前，保留路径 B 作为项目一致的目标；把路径 A 作为“Windows 开发机直连雷达”的受控验证分支。若项目的真实用途是开发台先直连雷达并尽快学习 `/scan`、SLAM 与 Nav2，路径 A 的调试价值高；若目标是车载长期运行，推荐将采集留在车载 Linux/网关侧，避免依赖开发笔记本 USB、Docker Desktop 和 Windows 睡眠/重插状态。

## Docker Desktop 与设备接入

### WSL 2 / Docker Desktop 限制

- Docker Desktop 的 Linux container 是虚拟 Linux 环境的一部分；Windows 串口名、驱动权限和热插拔状态不会自动变为容器 Linux 设备。
- 对 USB 转 UART 雷达，目标设备通常应在 Linux 一侧最终表现为 `/dev/ttyUSB*` 或 `/dev/ttyACM*`。具体名称取决于实际桥接芯片与内核绑定，不能在文档中预设为固定端口。
- Windows + WSL 2 的 USB 转交通常需要专门的 USB/IP 工作流；Docker Desktop 对所转交设备的可见性取决于当前版本、后端和设备转交路径。先做最小设备可见性测试，再讨论 driver。
- Linux 上 udev 规则可提供稳定的 `/dev/ydlidar` 别名；Windows 不能直接提供同一规则。若设备最终由 WSL/Linux 管理，可在该 Linux 层建立基于厂商/产品/序列号的稳定标识，不能只依赖 `ttyUSB0`。
- 一个串口一次只能由一个消费者独占。LidarViewer、终端监视器和 ROS driver 不可并发打开同一设备。

### 串口方案

优先使用与 X3/X3 Pro 配套且已知电平、电源和 USB 转 UART 芯片的适配板。雷达本体 UART 与 PC USB 之间不能假定可直接互连；应依据设备资料和实物标签确认 TX/RX 电平、供电能力、线序、实际波特率及设备型号。容器启动配置应仅在 Linux 设备路径已验证后映射单一字符设备，并以非 root、加入相应设备组的方式运行。

资料内的树莓派教程展示了 Linux Docker 的 `--device=/dev/...` 与 udev 思路，见 [`4、docker硬件交互和数据处理.md`](../../资料/EAI%20X3%26X3%20Pro激光雷达/树莓派5使用教程/4、docker硬件交互和数据处理/4、docker硬件交互和数据处理.md)。它没有证明 Windows Docker Desktop 可获得相同设备路径。

## ROS 2 通信与 DDS 设计

### 阶段化通信策略

1. **阶段一，单一 Linux ROS 2 域：** driver、静态 TF、里程计桥、SLAM Toolbox、Nav2 都在一个容器或同一 Docker 网络内运行。固定 ROS 发行版、RMW 实现和 `ROS_DOMAIN_ID`，不依赖跨 Windows 的自动发现。
2. **阶段二，容器间拆分：** 仅在阶段一稳定后拆成 driver 与算法容器。两者使用同一 ROS domain、同一 DDS 实现和明确配置的网络接口；验证 `/scan`、TF 和 lifecycle service 均可发现。
3. **阶段三，跨 Windows/局域网：** Windows 原生 ROS 2、第二台计算机或车载网关加入时，配置 Windows 防火墙 UDP 规则、固定网卡、DNS/地址策略及 DDS 单播 peer/发现服务器。组播可用性必须实际测试，不能由 `ros2 topic list` 的单次结果推断稳定性。

建议全链先统一使用一个 RMW/DDS 实现，例如 Cyclone DDS 或 Fast DDS，避免不同默认配置造成难以复现的发现差异。跨虚拟网络时，使用显式 peer 或 discovery server 的策略通常比依赖组播更可控，但具体配置只能在已知网卡和 IP 规划后落地。

网络隔离还应考虑 `ROS_LOCALHOST_ONLY`：单容器调试可限制为本地；需要跨容器或跨主机时必须关闭该限制，并用最小开放的防火墙规则和独立 `ROS_DOMAIN_ID` 隔离本项目。

## SLAM 与导航数据流

```text
YDLIDAR scan packets
  -> driver decode / calibration
  -> sensor_msgs/msg/LaserScan on /scan
  -> static TF: base_link -> laser_frame
  -> odometry + TF: odom -> base_link
  -> SLAM Toolbox
  -> OccupancyGrid /map + TF: map -> odom
  -> Nav2 global/local costmaps consume /scan and TF
  -> Nav2 planner/controller publishes /cmd_vel
  -> explicit gateway protocol -> ESP32-S3 -> STM32H757 motor control
```

关键接口约束：

- `/scan` 必须带有正确 `header.stamp`、稳定 `frame_id`、单位一致的角度和距离字段；driver 输出不是对雷达串口原始帧的简单转发。
- `base_link -> laser_frame` 是固定的机械安装外参；安装位置或朝向不正确会直接破坏地图与避障。
- SLAM Toolbox 在建图/定位阶段都依赖连续 TF 与可信时间。仅持有激光数据时，车体移动建图精度通常不足。
- Nav2 的 `/cmd_vel` 是高层速度命令，不可直接等同于 STM32 电机电气控制。需要明确频率、单位、超时、急停、仲裁与故障降级；现有工程尚未定义该协议。
- 如采用 ROS 2 lifecycle，应在传感器、TF/里程计、SLAM、Nav2 的依赖已满足后依序 configure/activate，并对设备断连、TF 缺失和 map 不可用维持明确的 inactive/error 状态。YDLIDAR driver 是否实现 lifecycle 取决于具体版本，不能假定。

## GPU、GUI 与文件系统

### GPU

2D 激光 SLAM、Nav2 和串口 driver 不以 GPU 为前置条件。Windows NVIDIA GPU 若要供 Linux 容器用于 RViz、仿真或后续 AI，应使用 WSL 2 兼容的 NVIDIA 驱动与容器 GPU 支持，并在容器内实测设备可见性和 OpenGL 渲染。GPU 透传、RViz GUI 和 WSLg/Windows 图形链路应与雷达采集分开验证；不要因为 GUI 不可用而判断 driver 或 SLAM 失败。

### 文件挂载

- 将可编辑源码以 bind mount 挂入容器工作区，例如 `/workspaces/ros2_ws/src`；在 WSL/Linux 文件系统中的工作区通常比 Windows 文件共享目录有更好的大量小文件 I/O 表现。
- 将 `build/`、`install/`、`log/` 放入容器 named volume 或 Linux 侧目录，避免不同主机架构、UID 或换行符造成的污染。
- 将 map、rosbag、参数与导出数据放入明确的数据挂载目录；地图和 bag 是运行数据，不应写入镜像层。
- 初始镜像需固定 Ubuntu/ROS 发行版与依赖版本，后续用 Dockerfile/Compose 复现，不能把交互式容器当成唯一环境记录。

## 后续验证清单

以下项目均为未来硬件/集成验证，不是本文已获得的证据：

1. Windows 设备管理器确认雷达适配板及 USB 重新插拔后的身份。
2. WSL 2 与 Docker Desktop 版本、Linux container 模式、USB/IP 转交路径和容器内设备节点验证。
3. 串口独占、权限、供电、波特率、扫描启动/停止及断连恢复验证。
4. `LaserScan` 字段、频率、丢包、时钟和 `frame_id` 验证。
5. 静态 TF、里程计、`map -> odom -> base_link -> laser_frame` TF 树验证。
6. 单容器 DDS、容器间 DDS、Windows/局域网 DDS 逐层验证，并记录防火墙与网卡配置。
7. SLAM 建图、定位、Nav2 costmap、速度指令超时与硬件急停的封闭场地验证。

## 本次证据边界

本审计实际检查了项目 ROS2_WIN 责任边界、工作流规则，以及资料中面向 Linux/树莓派的 Docker 设备挂载说明。未安装或运行 Docker、ROS 2、YDLIDAR SDK/driver、SLAM Toolbox 或 Nav2；未解压、编译或修改任何雷达资料包；未接入 USB/UART 设备。因此本文是架构与部署准备建议，不构成 Windows、容器、串口、DDS、建图、导航或车辆控制的运行验收。
