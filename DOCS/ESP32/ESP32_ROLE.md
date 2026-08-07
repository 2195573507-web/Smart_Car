# ESP32-S3 Role

## Intended Responsibility

ESP32-S3 is the Smart_Car communication gateway. Its intended responsibilities are YDLIDAR X3/X3 Pro data acquisition and WiFi-enabled communication between the low-level controller and the upper software stack.

## Boundary

- The intended lower interface is UART/CAN from STM32H757.
- The intended upper interface is WiFi toward ROS2_WIN.
- Lidar acquisition belongs to ESP32-S3; motor control, encoder handling, and low-level real-time control remain with STM32H757.

## Initialization Status

No ESP32 source, protocol, WiFi implementation, lidar parser, or application behavior is created or modified by this initialization task. Interface packet definitions and runtime acceptance remain future work.

## Validation Boundary

This document proves only assigned responsibility. It does not prove ESP32-S3 wiring, radar compatibility, serial traffic, WiFi connectivity, or ROS2 interoperability.
