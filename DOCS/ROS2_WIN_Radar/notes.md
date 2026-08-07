# ROS2_WIN 雷达资料笔记

## 静态可确认资料

- `资料/EAI X3&X3 Pro激光雷达/通用资料/` 含 X3、X3 Pro 数据手册，以及名为 `YDLIDAR X3 开发手册` 的 PDF；其 PDF 元数据标题却显示为 S4，不能据文件名之外的型号细节作结论。
- `ROS2教程/源码/` 含 `YDLidar-SDK-master.tar.xz` 与 `ydlidar_ros2_driver-master.tar.xz`。
- ROS2 driver 归档清单包含 `CMakeLists.txt`、`package.xml`、`src/`、`launch/`、`params/`；其中有 `x3_ydlidar_launch.py`、`ydlidar_x3.yaml` 和两个 C++ 源文件。
- SDK 归档清单包含通用、串口、协议、过滤器、Windows 与 Unix 串口实现，以及 `CYdLidar`、`ydlidar_driver` 等源文件。

## 架构限制

- 当前项目基线指定 ESP32-S3 负责 YDLIDAR X3/X3 Pro 数据采集和网关，ROS2_WIN 通过 Wi-Fi 接收网关信息。
- Windows Docker 直连 USB/UART 是后续受控验证路径，不能把 Linux 的 `/dev/ttyUSB*` 映射范式直接当成 Windows Docker Desktop 已验证能力。
