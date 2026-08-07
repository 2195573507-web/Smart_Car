# ROS2_WIN Architecture

## Intended Responsibility

ROS2_WIN is the host-side autonomy and map-management domain. Its intended scope includes ROS2 integration, SLAM, navigation, and map management.

## Boundary

ROS2_WIN receives gateway-provided information through the planned ESP32-S3 WiFi boundary and exposes planned network-facing services to IOS_APP. It does not take ownership of direct motor electrical control, encoder acquisition, or lidar hardware parsing.

## Initialization Status

No ROS2 workspace, package, SLAM implementation, navigation configuration, map pipeline, or network API is added in this task.

## Validation Boundary

This document describes ownership only. It does not verify ROS2 installation, DDS connectivity, sensor messages, maps, localization, navigation, or end-to-end vehicle control.
