# CM7

CM7 is enabled in `Smart_Car_H757.ioc` with a requested 480 MHz clock target.
CM7 firmware is configured and built only from this directory. The canonical
commands are:

```sh
cmake --preset Debug
cmake --build build/Debug --target Smart_Car_H757_CM7 -j2
```

The `Debug` preset keeps the scheduler and raw USART1 diagnostics disabled so
USART2 carries only standard SRP traffic. No startup frame, IRQ marker, or
isolated UART2 heartbeat is injected into the STM32-S3 link.

The only valid CM7 firmware artifact is:

`STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`
