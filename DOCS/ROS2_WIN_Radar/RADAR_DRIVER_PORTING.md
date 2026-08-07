# 雷达 Driver 移植准备：ROS1 到 ROS2

## 范围

本文记录资料中的 ROS driver 和 SDK 归档所暴露的静态结构，并规划后续移植判断点。本轮仅在受控的 `.analysis_extract/` 临时目录无损展开副本并做静态阅读；未修改原始归档或源码，未编译、未运行 ROS。“移植步骤”是准备清单，不是已执行的代码移植。

## 1. 原始源码结构

已确认 ROS2 driver 归档 [`ydlidar_ros2_driver-master.tar.xz`](../../资料/EAI%20X3%26X3%20Pro激光雷达/ROS2教程/源码/ydlidar_ros2_driver-master.tar.xz) 的目录清单包含：

| 目录或文件 | 静态可确认用途 |
| --- | --- |
| `CMakeLists.txt`、`package.xml` | ROS2 package 构建与元数据入口 |
| `src/ydlidar_ros2_driver_client.cpp`、`src/ydlidar_ros2_driver_node.cpp` | 两个 C++ 实现入口文件 |
| `launch/` | 多个 Python launch 文件，含 `x3_ydlidar_launch.py` |
| `params/` | YAML 参数，含 `ydlidar_x3.yaml` |
| `config/` | RViz 相关配置 |
| `startup/initenv.sh`、`README.md`、`details.md` | 启动辅助与说明材料 |

ROS1 对应资料也存在：[`ROS1教程/源码/ydlidar_ros_driver-master.tar.xz`](../../资料/EAI%20X3%26X3%20Pro激光雷达/ROS1教程/源码/ydlidar_ros_driver-master.tar.xz)。本轮已核对其使用 catkin、`ros::NodeHandle`、`advertise`/`advertiseService` 与 `sensor_msgs/LaserScan`/`PointCloud`，而 ROS2 使用 ament、`rclcpp::Node`、`create_publisher`/`create_service` 与 `sensor_msgs/msg/LaserScan`。归档的目标 ROS 发行版兼容性仍需后续运行验证。

## 2. ROS1 与 ROS2 区别

| 主题 | ROS1 常见模式 | ROS2 对应关注点 |
| --- | --- | --- |
| 构建 | `ydlidar_ros_driver`，catkin，C++11 | `ydlidar_ros2_driver`，ament_cmake，C++14 |
| 运行时 | ROS master 参与发现 | DDS/RMW 参与发现与 QoS 行为 |
| 节点 API | `ros::NodeHandle`、`advertise`、`advertiseService` | `rclcpp::Node`、`create_publisher`、`create_service` |
| 发布 | `scan` LaserScan，另有 `point_cloud` PointCloud | `scan`（根命名空间下为 `/scan`）LaserScan |
| 服务 | `start_scan`、`stop_scan`，`std_srvs/Empty` | 同名服务，`std_srvs/srv/Empty` |
| X3 启动 | XML `launch/X3.launch` 与 `LIDAR_TYPE=x3` | Python `x3_ydlidar_launch.py` 加载 `params/ydlidar_x3.yaml` |
| 生命周期 | 通常由节点自行管理 | 可选 lifecycle；当前资料 launch/入口静态不一致，不能假定 driver 已实现 |

上表的 driver 特定差异来自 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)；对目标 ROS2 Humble 的 API/launch 兼容性仍需后续运行验证。

## 3. package.xml 分析

静态源码分析确认 ROS2 package 为 `ydlidar_ros2_driver`，版本为 1.0.1，构建工具为 `ament_cmake`，并声明 `rclcpp`、`sensor_msgs`、`visualization_msgs`、`geometry_msgs`、`std_srvs` 和 SDK 相关依赖。证据路径是 [package.xml](../../.analysis_extract/tar_xz/02/ydlidar_ros2_driver-master/package.xml)，摘要见 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。许可文本、全部导出标签及对 Humble 的兼容性仍应在后续目标环境中复核。

## 4. CMakeLists.txt 分析

`CMakeLists.txt` 已在 driver 与 SDK 两个归档中确认存在。静态源码分析确认 driver 通过 `find_package(ydlidar_sdk REQUIRED)` 获取 SDK，并安装 `ydlidar_ros2_driver_node` 与 `ydlidar_ros2_driver_client` 两个可执行文件；SDK 是 CMake C++11 项目，默认构建静态 `ydlidar_sdk` 库，并含样例、测试和多语言绑定的可选结构。证据路径是 [driver CMakeLists.txt](../../.analysis_extract/tar_xz/02/ydlidar_ros2_driver-master/CMakeLists.txt) 和 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。实际依赖是否能在 Humble 成功解析仍待构建验证，任何修改仍需单独授权。

## 5. node 节点结构

静态源码分析确认 ROS2 driver 包含 `ydlidar_ros2_driver_node` 和 `ydlidar_ros2_driver_client` 两个可执行文件。前者创建同名 `rclcpp::Node`，读取端口、波特率、雷达类型、单通道、反转、量程、频率等参数并写入 `CYdLidar`；后者订阅 `scan` 并输出角度/距离示例。driver 的已确认流程为：

```text
launch / YAML 参数
  -> ydlidar_ros2_driver_node 初始化
  -> CYdLidar::initialize() / turnOn()
  -> doProcessSimple() 获取 SDK LaserScan
  -> scan publisher 转换为 ROS LaserScan
  -> turnOff() / disconnecting()
```

静态代码显示 publisher 使用 KeepLast(10)，并提供 `start_scan`、`stop_scan` 的 `std_srvs/srv/Empty` 服务。实际消息频率、时间戳质量、异常恢复和设备行为仍需运行验证。证据见 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。

## 6. topic 发布流程

目标 ROS2 数据链为 `SDK/driver -> sensor_msgs/msg/LaserScan -> scan -> TF/SLAM/Nav2`。静态源码分析确认 publisher 使用相对 topic 名 `scan`；在根命名空间中它解析为 `/scan`。`header.stamp` 来自 SDK 扫描时间，`frame_id` 由参数决定，X3 YAML 默认 `laser`；`ranges`/`intensities` 由 SDK 点数组写入。消息是否连续、角度方向是否正确、时间戳是否可用于 SLAM，均待运行验证。

## 7. 参数文件说明

已确认 `params/ydlidar.yaml`、`params/ydlidar_x3.yaml`、`params/ydlidar_4ros.yaml` 存在，且 launch 目录存在与 X3、raw、view 等命名关联的 Python 文件。X3 YAML 的静态默认值包括 `/dev/ydlidar`、`laser`、115200、三角测距、串口、`sample_rate: 3`、单通道、DTR 电机支持和 `frequency: 10.0`。源码还发现 YAML/README 使用 `resolution_fixed` 而入口读取 `fixed_resolution`，所以应把这个参数名不一致作为移植前阻断项。建议后续建立“参数名、默认值、单位、适用型号、热修改能力、验证方法”表；尤其不要跨 X3、X3 Pro 或其它型号复制参数。详见 [`RADAR_SOURCE_ANALYSIS.md`](RADAR_SOURCE_ANALYSIS.md)。

## 8. 移植步骤

1. 由任务所有者选择“保持 ESP32-S3 网关”或“Windows Docker 直连”所有权路径。
2. 固定目标 ROS2 发行版；架构审计推荐 Ubuntu 22.04 + ROS2 Humble，Iron 不作为新环境首选。
3. 解压到受控的临时分析/开发位置后，逐文件核对 `package.xml`、CMake、node、launch、YAML 与 SDK API，不改原始资料。
4. 建立干净的 colcon workspace，先解决 package/SDK 依赖，再考虑编译。
5. 在无车体运动条件下验证设备独占、`/scan`、时间戳、`frame_id` 和 TF。
6. 逐层接入 SLAM、里程计与 Nav2；最后才定义到 ESP32-S3/STM32 的速度和安全接口。

第 3 至 6 步均未执行，也不属于本任务范围。
