# YDLidar Viewer Mac Design

## Goal

Create a standalone, native macOS application at `资料/tools/YDLidarViewerMac` that replaces the usable capabilities of `LidarViewer_V0.3.5.exe` for supported YDLIDAR devices, beginning with X3 and X3 Pro. It must not modify any Smart_Car firmware, IOC, or ROS project.

## Scope

The app provides USB-UART and supported network connections, scan start and stop, live 2D Cartesian point-cloud display, device identity and health, configurable rendering, recording and export, operation logs, and capability-gated configuration panels.

The Windows executable is a multi-model program. Its strings show generic options including motor and laser switches, scan frequency, FOV, correction values, TCP/UDP data settings, DHCP, IP/netmask/gateway, data receiver settings, and recording. These controls are not assumed to be valid for X3 or X3 Pro. The app only enables a control after the detected model and the bundled official SDK/public protocol declare it supported.

Firmware updates are not implemented as a generic command. A future update flow can be added only for a model whose official SDK and update protocol explicitly support it. It will verify the model and image before sending any data.

## Architecture

The project is a SwiftPM macOS app, built with SwiftUI for the UI and a narrow C/C++ bridge around the vendor's published YDLidar SDK. The SDK is vendored as source with its license preserved and compiled locally for arm64 macOS. This keeps protocol decoding and supported control commands aligned with the vendor implementation rather than recreating undocumented packets.

`LidarSession` owns one transport lifecycle and serializes commands. `LidarBridge` owns SDK calls and yields device metadata, capability flags, scan frames, and structured errors. `ScanStore` retains only the most recent frame for rendering plus an optional bounded recorder stream. The rendering layer consumes immutable scan snapshots and never performs I/O.

## User Interface

The main window has a connection toolbar, a large radar canvas with scale rings and Cartesian axes, a concise status bar, and a trailing inspector.

The inspector has Device, Scan, Recording, Network, Calibration, and Logs tabs. Device shows model, firmware, hardware revision, serial number, and health. Scan exposes only model-approved range, frequency, and display parameters. Recording writes timestamped scan data to a user-selected directory and can export CSV for analysis. Network and Calibration are disabled with a reason when the device lacks their supported commands.

## Data and Safety

Connections default to USB-UART. X3 Pro uses the documented 115200 8N1 setting; model discovery decides the rest. Before a configuration write, the app reads the present configuration when the protocol supports it, shows the exact before and after values, requires explicit confirmation, and writes a local backup. Failed commands leave the session connected when possible and are recorded in the logs.

No cloud access, telemetry upload, Smart_Car firmware connection, or automatic firmware update is included.

## Verification

Verification has separate layers:

1. Static protocol and capability tests use known X3 frame fixtures and unsupported-command cases.
2. `swift build` proves compilation of the macOS app and local SDK bridge.
3. App launch inspection verifies the window, connection controls, radar empty state, inspector tabs, and disabled capability explanations.
4. A connected radar is required to prove live point-cloud reception, model detection, recording output, and any configuration write. Build and UI evidence alone do not prove device behavior.

## Acceptance Criteria

- Sources and run instructions are contained under `资料/tools/YDLidarViewerMac`.
- The app compiles on the current macOS arm64 Swift toolchain.
- A detected X3/X3 Pro can display received scan frames and expose documented identity and health information.
- Recording produces a readable CSV file.
- Unsupported configuration controls are unavailable rather than issuing speculative packets.
- No files outside the new tool and this design record are changed by the implementation.
