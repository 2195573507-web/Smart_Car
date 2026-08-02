# SmartCar Logger Mac

`SmartCar_Logger_MAC` is a standalone macOS SwiftUI application for viewing
STM32 USART1 logs through a CH340 USB-to-UART adapter. It is a receive-only
viewer: it does not expose controls and has no API that writes serial bytes.

## Scope

- Discovers available macOS serial endpoints automatically every two seconds,
  with CH340/CH341 and USB-Serial endpoints listed first.
- Uses `/dev/cu.*` endpoints when available to avoid duplicate `tty` entries.
- Opens the selected endpoint with fixed `115200 8N1` settings: 8 data bits,
  no parity, one stop bit, and no software or hardware flow control.
- Streams UTF-8 USART1 output into a selectable monospaced log view.
- Saves the current captured log as a UTF-8 `.txt` file using the macOS save
  panel.

The app does not alter or link with `STM32H757`, `ESPS3`, or `IOS_APP`.

## Build and Run

macOS 14 or later with the Xcode command-line tools is required.

```bash
cd /Users/zhiqin/Projects/Smart_Car/Tools/SmartCar_Logger_MAC
./script/build_and_run.sh
```

The script builds the SwiftPM executable, stages a local app bundle at
`dist/SmartCar_Logger_MAC.app`, and opens it. The Codex Run action uses the
same script. Useful modes are `--verify`, `--debug`, `--logs`, and
`--telemetry`.

## Use

1. Connect the CH340 adapter to the Mac and connect its UART receive line to
   the STM32 USART1 transmit line with a shared ground. Check the board's
   electrical levels before connecting.
2. Wait for the adapter to appear in the serial picker. The app does not
   automatically open any device.
3. Select the `/dev/cu.*` endpoint and click Connect. The fixed serial format
   shown in the window is `115200 8N1`.
4. View or select the live log text. Use the download icon or Command-Shift-S
   to save the current captured output.

The display refreshes at most ten times a second and retains the latest two
million characters so an unattended session cannot continually grow the SwiftUI
view. When this limit is reached, the earliest captured output is removed and a
marker is shown. `received bytes` remains the session total.

## Verification Boundary

`swift build` and `./script/build_and_run.sh --verify` establish host-side
compilation and app launch only. They do not establish CH340 driver access,
UART wiring, STM32 USART1 output, or sustained runtime behavior. Verify those
with the target adapter and board connected.
