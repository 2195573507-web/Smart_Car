# CM7

CM7 is enabled in `Smart_Car_H757.ioc` with a requested 480 MHz clock target.
CM7 firmware is configured and built only from this directory. The canonical
commands are:

```sh
cmake --preset Debug
cmake --build build/Debug --target Smart_Car_H757_CM7 -j2
```

The only valid CM7 firmware artifact is:

`STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`
