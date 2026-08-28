# UART2 Raw Echo Test Design

## Scope

Add a forced physical-layer UART test firmware for the CM7 USART2 and S3 UART2 route.
It uses 115200-8-N-1 and the existing PA2/PA3 and GPIO17/18 pins.

## Isolation

`SMARTCAR_UART2_ECHO_TEST` is forced to `1` by both top-level build systems.
CM7 is a bare, blocking USART1/USART2 loop and compiles neither the production
SRP/DMA transport sources nor service sources. S3 links only its direct UART2
test component dependencies. Existing SRP/DMA/service initialization is absent
from the test targets.

## Validation

The test must build in dedicated build directories. Device acceptance requires
matching flashed images and captures from the STM USART1 and S3 USB consoles.
