# Smart_Car Project Architecture

## System Chain

```text
STM32H757
    | UART / CAN
ESP32-S3
    | WiFi
ROS2_WIN
    | Network
IOS_APP
```

The chain identifies intended system ownership. It is an architecture baseline, not an implemented protocol, topology, or runtime acceptance result.

## Component Responsibilities

| Component | Intended responsibility | Explicitly outside this initialization |
| --- | --- | --- |
| STM32H757 | Motor control, encoder acquisition, and low-level real-time control | Motor-control logic, encoder algorithms, control loops, and board operation |
| ESP32-S3 | YDLIDAR X3/X3 Pro data acquisition, communication gateway, and WiFi | Lidar parsing, ESP32 feature changes, or gateway protocol implementation |
| ROS2_WIN | ROS2 runtime, SLAM, navigation, and map management | ROS2 packages, SLAM, navigation, and map generation |
| IOS_APP | Mobile control and status display | iOS application features or UI implementation |

## Initialization Scope

This repository stage establishes the directory layout, project documentation, Codex multi-agent governance, and an STM32H757 CubeMX baseline. It does not establish a complete vehicle behavior, an inter-node protocol, a hardware communications link, or physical motion.

## Interface Boundaries

- `STM32H757 -> ESP32-S3`: UART/CAN is the intended low-level to gateway boundary. Message formats, rates, ownership, and error behavior are not defined in this initialization.
- `ESP32-S3 -> ROS2_WIN`: WiFi is the intended gateway to ROS2 boundary. Network addressing, transport, and authentication are not defined here.
- `ROS2_WIN -> IOS_APP`: Network is the intended host-to-mobile boundary. API and application protocol work are deferred.

## Evidence Boundaries

- A Markdown architecture statement records an intended ownership boundary only.
- IOC parsing and a successful firmware build are configuration and build evidence only.
- Hardware wiring, electrical compatibility, bus transactions, wireless connectivity, and end-to-end behavior require separate hardware and integration validation.
