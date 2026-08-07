# UART BSP

Board-support boundary for serial ports used by the controller and the S3
link. Blocking TX/RX, separate log output, and DMA-reserved APIs are present.
The current IOC only configures USART2; USART1/USART6 and DMA return an
explicit unsupported status until their CubeMX resources are assigned.
