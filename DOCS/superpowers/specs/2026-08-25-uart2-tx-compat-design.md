# UART2 TX Compatibility Architecture

## Scope

This change tests the transport difference suspected between the current SRP
implementation and `a250dec`: restore blocking UART2 TX semantics while keeping
the current RX path and diagnostics intact. It does not change SRP wire bytes,
UART2/USART2 ownership, baud rate, RX DMA startup timing, parser behavior, or
hardware assumptions.

## Design

### STM32H757 CM7

- Keep `HAL_UARTEx_ReceiveToIdle_DMA()` on DMA1 Stream0 with the current D2
  SRAM buffer, cache handling, ISR entry points, error clearing, and RX state
  counters.
- Remove TX DMA from the active send path. `uart_link_send()` serializes valid
  SRP frames with a mutex and calls `HAL_UART_Transmit()` using a bounded timeout.
- Preserve TX success, timeout, and HAL-error counters. TX queue/preemption
  counters become inactive compatibility fields and must not affect RX status.
- Do not disable or repurpose DMA1 Stream0, USART2 IRQ, or RX recovery logic.

### ESP32-S3

- Keep the current RX task, UART event/error handling, storage ring, sync guard,
  and RX discontinuity reporting.
- Replace the software TX priority queues/TX task with the `a250dec` semantics:
  serialize `uart_write_bytes()` under a TX mutex and wait for
  `uart_wait_tx_done()` with a bounded timeout.
- Keep short-write, timeout, and HAL-error statistics so a failed TX is visible.
- Preserve SRP sync gating and frame validation at the service boundary.

## Expected impact

At 921600 baud with 8N1, a frame occupies `10 * length / 921600` seconds on
the wire: 16 bytes is about 0.174 ms, 42 bytes 0.456 ms, and the 512-byte SRP
maximum 5.556 ms. CM7 blocking TX consumes polling CPU time for that interval;
S3 blocks the caller task while the UART driver drains. Wire throughput is
unchanged. Emergency-frame latency can be delayed by an in-flight log frame,
so runtime acceptance must include the existing emergency response budget.

## Verification

1. Build CM7 Debug from an isolated build directory and inspect the map/ELF for
   absence of an active USART2 TX DMA start while DMA1 Stream0 remains present.
2. Build ESP-IDF with the repository's ESP-IDF 5.5.4 environment and confirm
   the TX task/queue symbols are no longer on the active path.
3. Run host SRP codec/service tests and static checks for the unchanged wire
   contract.
4. On a flashed image, capture CM7 RX-start status, RX callback/IRQ counters,
   S3 TX write/wait results, CM7 BOOT_INFO TX, and S3 RX counters. Source and
   build success alone do not prove the link is repaired.

## Rollback

The change is limited to the CM7 UART link implementation, S3 STM UART
implementation, and this design record. Reverting those implementation files
restores the current TX DMA queue architecture without changing RX ownership.
