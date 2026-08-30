# Smart Car 构建错误报告

日期：2026-08-18

## App xcodebuild

命令：

```text
xcodebuild -scheme SmartCar_Control_MAC -destination 'platform=macOS' build
```

工作目录：`IOS_APP/SmartCar_Control_MAC`

结果：未进入 Swift 编译阶段，工具链报错：

```text
xcode-select: error: tool 'xcodebuild' requires Xcode, but active developer directory '/Library/Developer/CommandLineTools' is a command line tools instance
```

原因：当前 macOS active developer directory 是 Command Line Tools，不是完整 Xcode。

影响：本次没有取得 `xcodebuild` 证据；不代表 Swift 源码编译失败，也不代表 App bundle/UI 运行验证通过。

补充验证：使用仓库现有 SwiftPM 入口执行 `swift build`；该结果仅证明 SwiftPM 源代码构建，不替代 `xcodebuild` 或 staged App/UI 验证。

实际补充结果：`swift build` 通过（`ControlModeView.swift` 编译、链接和应用步骤完成）。
`swift test` 完成编译规划但返回 `error: no tests found; create a target in the 'Tests' directory`；仓库当前没有 Swift 测试 target。

## 其它构建

- STM32 CM7：warning 修正后 configure/build 通过，FLASH 118604 B / 1 MB，RAM 54768 B / 128 KB。
- ESP32-S3：`idf.py build` 通过，`smartcar_s3_gateway.bin` 0xB25C0 bytes。
- 未执行 flash、monitor、BLE/UART 捕获或车辆硬件测试。
