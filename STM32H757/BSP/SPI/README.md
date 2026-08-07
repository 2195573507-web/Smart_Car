# SPI BSP

Board-support boundary for SPI peripherals used by local sensors. SPI1 is
configured for the IOC's PA5/PA6/PA7 pins and the BMI323 CS line is controlled
through the GPIO BSP. Transactions are blocking HAL-wrapped calls.
