# 雷达学习指南：YDLIDAR X3/X3 Pro 与 ROS2_WIN

## 阅读范围与证据等级

本指南是后续 ROS2_WIN 开发前的学习地图，不创建 ROS2 节点、不安装软件、不修改资料或源码。

| 标记 | 含义 |
| --- | --- |
| 已确认 | 由本项目文档、资料路径或归档文件清单直接确认的静态事实 |
| 建议 | 面向后续实现的架构或学习建议，尚未实施 |
| 待验证 | 需要阅读源码正文、接入设备、运行 ROS2 或现场测试后才能确认 |

本地资料入口：[`资料/EAI X3&X3 Pro激光雷达`](../../资料/EAI%20X3%26X3%20Pro激光雷达)。已确认其中包含 X3 与 X3 Pro 数据手册、ROS1/ROS2 教程、SDK 和 ROS driver 归档。资料中名为 `YDLIDAR X3 开发手册 V1.0(211223).pdf` 的 PDF 元数据标题为 S4，因此任何 X3 专属协议或电气参数必须以实物型号、数据手册正文和 SDK 版本的后续比对为准。

## 1. 雷达工作原理

二维激光雷达通常让测距光束随扫描机构转动，在每个角度采集一组距离观测，形成机器人周围平面的极坐标点集。驱动层通常需要处理设备连接、扫描启停、帧完整性、测距转换、角度排序、异常点和时间戳；上层才把一次扫描表示为 ROS 消息。

对随附资料的静态阅读已确认 X3/X3 Pro 走三角测距 SDK 路径，扫描角为 0 至 360 度，典型扫描频率为 8 Hz（资料范围 5 至 10 Hz），标称测距为 0.12 至 8 m，UART 为 3.3 V、115200 bps、8N1、无校验。供电、适配板、电平、线序、实际扫描频率与量程仍必须以实物和运行测试确认。证据见 [`X3 数据手册`](../../资料/EAI%20X3%26X3%20Pro激光雷达/通用资料/YDLIDAR%20X3%20数据手册%20V1.0%28211230%29.pdf)、[`X3 Pro 数据手册`](../../资料/EAI%20X3%26X3%20Pro激光雷达/通用资料/YDLIDAR%20X3PRO%20数据手册%20V1.0%28230418%29.pdf) 与 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。

## 2. 激光雷达数据格式

驱动向 ROS2 发布的目标接口是 `sensor_msgs/msg/LaserScan`，它是标准化扫描结果而非 UART 原始字节帧。学习时应区分以下两层：

| 层 | 责任 | 关键内容 |
| --- | --- | --- |
| 设备/SDK 层 | 串口或 USB 转串口收发与协议解析 | 原始帧、校验、扫描包、设备状态、时间关系 |
| ROS 消息层 | 发布给算法与可视化 | `header`、`angle_min`、`angle_max`、`angle_increment`、`time_increment`、`scan_time`、`range_min`、`range_max`、`ranges`、可选 `intensities` |

`ranges` 的下标与角度增量共同定义点的方向；`header.stamp` 和 `header.frame_id` 使该扫描能够参与 TF、建图和导航。字段单位、无效值表达、角度零点和旋转方向必须由实际 driver 输出验证，不能从文件名推断。

## 3. YDLIDAR X3/X3 Pro 学习路径

已确认本地资料含 X3/X3 Pro 数据手册、SDK、ROS1 driver、ROS2 driver 和 ROS2 雷达应用课程：

- [`ROS2教程/源码/YDLidar-SDK-master.tar.xz`](../../资料/EAI%20X3%26X3%20Pro激光雷达/ROS2教程/源码/YDLidar-SDK-master.tar.xz)
- [`ROS2教程/源码/ydlidar_ros2_driver-master.tar.xz`](../../资料/EAI%20X3%26X3%20Pro激光雷达/ROS2教程/源码/ydlidar_ros2_driver-master.tar.xz)
- [`ROS2教程/ROS2雷达应用`](../../资料/EAI%20X3%26X3%20Pro激光雷达/ROS2教程/ROS2雷达应用)

建议按“型号与接线条件 -> SDK 抽象 -> ROS2 driver 参数/launch -> `/scan` 与 TF -> SLAM -> Nav2”顺序学习。资料中出现 `x3_ydlidar_launch.py` 与 `ydlidar_x3.yaml`，已证明归档提供 X3 命名的 launch/参数入口。静态源码分析确认该 YAML 使用 `port: /dev/ydlidar`、`frame_id: laser`、`baudrate: 115200`、串口设备类型和三角测距类型；这些是资料包默认配置，不是当前硬件、WSL2 或 Docker 已可用的证明。详见 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。

## 4. ROS 中 LaserScan 数据流

```text
YDLIDAR X3/X3 Pro
  -> UART 或 USB 转 UART
  -> SDK / ROS driver
  -> /scan (sensor_msgs/msg/LaserScan)
  -> static TF: base_link -> laser_frame
  -> SLAM Toolbox
  -> /map 与 map -> odom
  -> Navigation2 消费 /scan、TF、里程计
  -> /cmd_vel
  -> 明确定义的车载网关协议
```

已确认项目的当前基线是 `YDLIDAR -> ESP32-S3 -> Wi-Fi -> ROS2_WIN`，不是容器直接占用雷达。Windows/Docker 直连 driver 是建议的学习验证分支，须先获得架构授权和 USB/UART 可见性验证。详见 [`ROS2_ARCHITECTURE_AUDIT.md`](ROS2_ARCHITECTURE_AUDIT.md)。

## 5. ROS2 生命周期

ROS2 lifecycle 的常见状态模型是 `unconfigured -> inactive -> active -> finalized`，并可通过 configure、activate、deactivate、cleanup、shutdown 等受控转换管理资源。其价值是将设备打开、topic 发布、算法激活与故障处理分开。

建议后续将“串口设备可用、静态 TF 可用、里程计可用、SLAM 可用”作为按序激活条件。静态源码分析发现 X3 launch 使用 `LifecycleNode`，但 driver 入口创建的是普通 `rclcpp::Node`，且没有 `on_configure`/`on_activate` 回调；因此不能把该资料包 driver 视为已实现受管 lifecycle。详见 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。

## 6. Driver -> Topic -> SLAM 链路

建议的最小验证顺序如下，任何一步失败都不应推断后续成功：

1. 验证设备能被唯一且稳定地访问，且串口参数与型号资料一致。
2. 验证 driver 能持续发布语义正确的 `/scan`。
3. 验证 `base_link -> laser_frame` 静态 TF 与机械安装一致。
4. 验证里程计和 `odom -> base_link` 连续可用。
5. 再验证 SLAM Toolbox 的建图、保存/加载与定位行为。
6. 最后验证 Nav2 costmap、`/cmd_vel` 约束、超时和底盘安全链。

第 1 至 6 步全部属于后续运行/硬件/集成验证；本次仅建立学习与准备文档。
