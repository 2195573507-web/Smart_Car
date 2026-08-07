# SmartCar Logger Architecture

## Data Flow

The macOS logger is a receive-only view of the STM32 UART stream. Incoming
bytes are owned by the reader path, retained in a bounded line buffer, and then
consumed independently by the display and copy paths:

```text
UART
 |
 |
Reader Thread
 |
 |
Ring Buffer (500)
 |
 +-- Display Filter
 |
 +-- Copy Export
```

The Ring Buffer retains the latest 500 complete log lines. When it is full,
the oldest line is overwritten. Display filtering changes visibility only; it
does not remove or rewrite retained lines. Copy Export reads the current
retained window and does not add historical storage.

## Log Level Filtering

The display filter supports these levels, in increasing verbosity:

```text
OFF -> ERROR -> WARN -> INFO -> DEBUG -> TRACE
```

The default level is `INFO`, so normal device log output is visible while
descriptor and termios diagnostics remain hidden. The remaining internal
diagnostic labels are hidden at the default level:

```text
FD_STATUS
TERMIOS
```

`TRACE` preserves the complete serial diagnostic stream, including the
descriptor and termios labels above. `OFF` suppresses all displayed lines.
Filtering is a display-stage operation after the reader thread and Ring Buffer,
so it does not stop UART reads, alter RX ownership, or change the fixed
500-line buffer.

## Ownership Boundaries

- UART and the reader thread receive and assemble incoming serial data.
- The Ring Buffer owns the bounded retained window.
- The Display Filter selects entries for the UI without changing the buffer.
- Copy Export snapshots the retained window for the existing Copy All Logs
  action without creating historical log storage.
