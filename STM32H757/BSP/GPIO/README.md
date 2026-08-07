# GPIO BSP

Board-support boundary for GPIO pin operations used by higher layers.

`bsp_gpio_init`, `bsp_gpio_write`, `bsp_gpio_read`, and `bsp_gpio_toggle`
provide named CS, direction, and interrupt pins. HAL types and port mappings
remain private to `bsp_gpio.c`.
