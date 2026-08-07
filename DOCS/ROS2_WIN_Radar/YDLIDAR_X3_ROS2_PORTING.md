# YDLIDAR X3/X3 Pro ROS2 移植准备

## 资料分析

本地资料根目录为 [`资料/EAI X3&X3 Pro激光雷达`](../../资料/EAI%20X3%26X3%20Pro激光雷达)。静态清单确认下列关联材料：

| 类别 | 本地证据 | 本次结论 |
| --- | --- | --- |
| 型号资料 | `通用资料/YDLIDAR X3 数据手册 V1.0(211230).pdf`、`YDLIDAR X3PRO 数据手册 V1.0(230418).pdf` | X3 与 X3 Pro 资料均在库；规格未在本任务中逐页校验 |
| SDK | `ROS2教程/源码/YDLidar-SDK-master.tar.xz` | 存在 SDK 归档 |
| ROS2 driver | `ROS2教程/源码/ydlidar_ros2_driver-master.tar.xz` | 存在 ROS2 driver 归档及 X3 命名 launch/YAML 文件 |
| ROS2 学习与应用 | `ROS2教程/ROS2基础教程`、`ROS2教程/ROS2雷达应用` | 资料覆盖节点、topic、DDS、launch、TF2、建图和导航主题 |
| Docker 参考 | `树莓派5使用教程/4、docker硬件交互和数据处理`、`7、docker容器安装ROS使用配件` | 面向 Linux/树莓派的参考，不是 Windows Docker 可用性证明 |

## SDK 结构

静态源码分析确认 SDK 含 `core/base`、`core/common`、`core/serial`、`core/network`、`src`、`samples`、`python`、`csharp`、`doc` 和 `test`。其中 `CYdLidar` 负责编排配置、初始化、扫描开关和 SDK `LaserScan`，`YDlidarDriver` 负责三角测距设备驱动与协议帧读取/解析，`core/serial` 以 Unix/Windows 实现提供串口层。样例顺序为 `initialize()` -> `turnOn()` -> `doProcessSimple()` -> `turnOff()` -> `disconnecting()`。证据见 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md) 和 [SDK 源码目录](../../.analysis_extract/tar_xz/04/YDLidar-SDK-master)。

## 通信协议

SDK 资料的静态源码阅读确认扫描包为小端字节序，固定头 `0x55AA`（字节流 `AA 55`）；字段包括 `CT`、`LSN`、`FSA`、`LSA`、`CS` 与每点 `Si`。三角测距路径使用 `Distance(mm) = Si / 4`，SDK 还负责角度插值和二阶角度补偿。ROS 节点应复用 SDK，而不重新实现协议解析。证据见 [通信协议文档](../../.analysis_extract/tar_xz/04/YDLidar-SDK-master/doc/YDLidar-SDK-Communication-Protocol.md) 和 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。实际设备帧、固件差异、校验和串口质量仍需抓包验证。

## 串口参数

资料静态阅读确认 X3/X3 Pro 使用 3.3 V UART、115200 bps、8N1、无校验；设备数据由雷达 `Tx` 输出至外设。供电为 5 V（资料允许 4.8 至 5.2 V，要求 1 A 供电能力），接口还列出 `M_CTR` 电机控制输入。X3/X3 Pro 的接线、电平转换、供电能力、共地、`M_CTR` 实现、USB 桥接芯片和容器枚举均未经真机验证，不能把资料默认值当成实物验收。证据见 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。后续部署必须在接线前填写并复核：

| 项目 | 当前状态 | 未来验证来源 |
| --- | --- | --- |
| 雷达型号与固件 | 待验证 | 实物标签、设备信息 |
| 电源与电平/线序 | 待验证 | 数据手册、适配板资料、实测 |
| 串口格式与波特率 | 待验证 | 型号资料、driver 参数、串口测试 |
| Linux 设备节点 | 待验证 | WSL/容器内实际枚举 |
| 权限和独占 | 待验证 | Linux 设备组与运行测试 |

## ROS2 driver 部署方式

推荐将 SDK、driver、`/scan` 消费者和 TF/SLAM 放在同一 Linux ROS2 运行域中，先降低跨 Windows/容器 DDS 与设备访问变量。资料归档的 `launch/` 与 `params/` 可作为后续目标 package 的配置来源，但不能在未核对版本兼容性的情况下直接使用。

随附 ROS2 工作区的静态示例包含 `slam_gmapping` 与相关车体/导航包，而未发现 `slam_toolbox` package；因此本文中的 SLAM Toolbox 是面向本项目的推荐替换目标，不是资料包已经部署的组件。

当前项目基线将采集与解析归属 ESP32-S3，ROS2_WIN 通过 Wi-Fi 接收网关信息。因此有两条互斥的后续路径：

1. **基线路径：** `YDLIDAR -> ESP32-S3 -> Wi-Fi -> ROS2_WIN bridge -> /scan`。需另行定义扫描数据、时间、断连、重连、背压和故障协议。
2. **学习直连路径：** `YDLIDAR -> 已验证 USB/UART 转交 -> Docker Linux -> ROS2 driver -> /scan`。需获得架构授权，且一个串口只由一个消费者打开。

两条路径不能同时争用同一设备或重复解析同一数据。详细风险见 [`ROS2_ARCHITECTURE_AUDIT.md`](ROS2_ARCHITECTURE_AUDIT.md)。

## Docker 运行方案

推荐目标为 Windows 游戏本上的 Docker Desktop（WSL2 Linux containers）内运行 Ubuntu 22.04 + ROS2 Humble、colcon workspace、driver 和 SLAM。GPU 对 2D driver/SLAM 非前置条件；GUI、RViz 或仿真可在后续单独验证 GPU/图形链路。

Windows 的 `COMx` 并不等价于容器内的 `/dev/tty*`。需要先验证 USB/IP 或受支持 WSL2 转交，使设备稳定呈现为 Linux 字符设备；随后才讨论容器设备映射。树莓派资料中的 Linux `--device`/udev 范式不能直接作为 Windows 已通过的方案。

## 调试方法

建议按不可跳步的顺序保存证据：

1. 记录实物型号、适配板、供电、线序与 Windows 设备身份。
2. 验证 WSL2/Linux 获得稳定设备节点、权限和重插行为。
3. 确认同一时间仅一个程序打开串口，避免 Viewer、终端和 driver 冲突。
4. 验证 driver 参数与型号资料匹配，再检查 `/scan` 的消息、时间戳、帧名和异常值。
5. 验证 `base_link -> laser_frame` 静态 TF、里程计和 TF 链后再运行 SLAM。
6. 最后验证 Nav2 与到车载控制域的安全速度接口。

这些均为未来硬件/运行/集成验证，本次没有执行。
