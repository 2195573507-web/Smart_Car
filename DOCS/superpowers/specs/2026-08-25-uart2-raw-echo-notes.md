# UART2 Raw Echo Test Notes

## Confirmed Inputs

- CM7 USART2 HAL MSP configures PA2/PA3 as AF7 push-pull with pull-up.
- S3 UART2 is GPIO17 TX and GPIO18 RX.
- The normal route is occupied by CM7 `uart_link` DMA/SRP and S3 `stm_uart`/
  `smartcar_service`, so test mode must prevent their startup.

## Boundaries

No changes are made to the hardware pins, SRP wire format, radar UART1/GPIO44,
or safety ownership. A successful build remains source evidence only.
