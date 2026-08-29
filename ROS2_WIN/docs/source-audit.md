# Official source audit

## SDK used

- Archive: `资料/EAI X3&X3 Pro激光雷达/ROS2教程/源码/YDLidar-SDK-master.tar.xz`
- Upstream tree: `YDLidar-SDK-master`, SDK CMake version `1.0.6`
- SHA-256: `50F869C3EB3CFE39C6CAE1022EFFE79D4EED9C3E1112D96788C91C27A77F7F54`
- License: `third_party/ydlidar_sdk/LICENSE.txt` (MIT notices in each copied
  source file)
- Reused files: the required `core/base` and `core/common` helpers,
  `core/network/ActiveSocket.*` and `SimpleSocket.*`, the Linux serial
  backend, and `src/ydlidar_driver.cpp/.h` as needed to build the official
  driver parser. Unreferenced math and passive-socket sources were not copied.
- Official functions used by the bridge: `parseResponseHeader`,
  `parseResponseScanData`, `calcuteCheckSum`, `parseNodeFromeBuffer`, and
  `waitPackage` through the project-only `parseMemoryChannel` entry point.

The vendor tree excludes SDK samples, Python/C# bindings, generated files,
documentation images, and the Windows serial backend. The upstream license is
kept unchanged.

## ROS 2 driver audited but not used as runtime

- Archive: `资料/EAI X3&X3 Pro激光雷达/ROS2教程/源码/ydlidar_ros2_driver-master.tar.xz`
- Package version: `1.0.1`
- SHA-256: `EEFE97D97397DA7048FBE18202AB6D193DF1C4D4395455693B7795547A777DB1`
- License: MIT (`LICENSE.txt` in the archive)
- Key reference: `src/ydlidar_ros2_driver_node.cpp`

That node calls `CYdLidar::initialize/turnOn`, reads a serial `port`, exposes
start/stop services, and has motor-DTR settings. It is therefore not copied
into the running package and is used only as a field-mapping reference.

## Manuals

The inspected data-manual references are:

- `资料/EAI X3&X3 Pro激光雷达/通用资料/YDLIDAR X3 数据手册 V1.0(211230).pdf`
- `资料/EAI X3&X3 Pro激光雷达/通用资料/YDLIDAR X3PRO 数据手册 V1.0(230418).pdf`
- `资料/EAI X3&X3 Pro激光雷达/通用资料/YDLIDAR X3 开发手册 V1.0(211223).pdf`

No S3 Wi-Fi gateway envelope or capture was found in these materials.

The development manual confirms the X3 scan field order (`PH`, `CT`, `LSN`,
`FSA`, `LSA`, `CS`, `Si`) and the little-endian `AA 55` wire header. The X3
and X3PRO data sheets specify a 115200, 8N1 UART. Exact q2 scaling and the
second-level angle correction remain delegated to the vendored SDK functions,
as required by this bridge.
