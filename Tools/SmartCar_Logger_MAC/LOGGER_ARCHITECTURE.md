# SmartCar Logger Architecture

## Purpose

SmartCar Logger is a receive-only macOS viewer for STM32 USART1 output. This
architecture bounds host memory and SwiftUI update work while preserving the
existing serial configuration, raw device text, and all external protocol
boundaries. The tool does not implement BLE transport or send serial bytes.

## Data Flow

```text
SerialPortService Data callback
  -> UTF8StreamDecoder
  -> LogLineAssembler
  -> LogEntry(level, raw message, id)
  -> LogRingBuffer (500 lines by default)
  -> 100 ms visibleLogEntries snapshot
  -> SwiftUI LazyVStack
```

`SerialPortService` continues to open the selected endpoint as `115200 8N1`.
The callback still supplies raw `Data`; the logger does not alter serial data,
device packet framing, or any BLE protocol.

## Storage and UI Window

`LogRingBuffer` is a fixed-size circular array. Inserting while full overwrites
the oldest stored entry and increments `droppedLineCount`. It never calls
`removeFirst()` and it never maintains an ever-growing full-log string.

The default capacity is `500` complete lines. `visibleLogEntries` is a snapshot
of only this retained window and is the sole log collection observed by the UI.
SwiftUI renders individual entries in a `LazyVStack`; no whole-log string is
constructed or appended for display.

Serial data is decoded as UTF-8 before line assembly. An incomplete line is
kept separately and capped at 16,384 characters. A longer unterminated line is
split into bounded entries, preventing malformed or delimiter-free input from
creating unbounded host memory use.

## Levels

Each line has one presentation level: `DEBUG`, `INFO`, `WARN`, or `ERROR`.
Leading forms such as `[ERROR]`, `ERROR:`, `[WARN]`, `WARN:`, `[DEBUG]`, and
`DEBUG:` are recognized case-insensitively. Lines without a recognized prefix
remain unchanged and are presented as `INFO`. Logger-originated open events
are `INFO`, periodic read diagnostics are `DEBUG`, and reader errors are
`ERROR`.

Level classification changes only the local UI treatment. The stored message
is the original device line (or an explicitly prefixed local `[LOGGER]`
diagnostic), so it is not a serial or BLE protocol transformation.

## Statistics

`LoggerStatistics` exposes:

- `storedLineCount` and `capacity` for buffer usage;
- `utilization` as `storedLineCount / capacity`;
- `droppedLineCount` for overwritten oldest entries.

The status bar displays all three values. Clear resets the Ring Buffer and its
dropped count; `receivedBytes` continues to reset with the existing clear-log
behavior.

## Verification Boundary

Core checks cover circular overwrite order, discard counting, line assembly,
unterminated-line bounds, and level classification. `swift build` and the
project launch verifier prove host-side code only. They do not prove CH340
driver access, physical wiring, STM32 output, or long-running serial behavior.
