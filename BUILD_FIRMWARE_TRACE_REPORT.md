# BMI323 调试固件链路检查报告

检查方式：只读检查，没有修改 C/C++、CMake、构建目录，也没有烧录。

## 1. 源码状态

检查目录：

```text
/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Sensor/BMI323/
```

搜索结果：

| 字符串 | 结果 |
| --- | --- |
| `WHOAMI_FORCE` | 不存在 |
| `READ_REG_TRACE` | 不存在 |
| `[BMI323][WHOAMI_RAW]` | 存在于 `bmi323.c:93` |
| `[BMI323][SPI_RAW]` | 存在于 `bmi323.c:97` |

因此，用户描述的 `WHOAMI_FORCE`/`READ_REG_TRACE` 本轮检查时并不在当前源码中。

## 2. CMake 状态

CM7 target 位于：

```text
/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/CMakeLists.txt
```

实际源文件列表中包含：

| 行号 | 源文件 | target |
| ---: | --- | --- |
| 24 | `../Middleware/Sensor/BMI323/bmi323.c` | `Smart_Car_H757_CM7` |
| 25 | `../Middleware/Sensor/BMI323/bmi323_port.c` | `Smart_Car_H757_CM7` |

`build.ninja` 和 `compile_commands.json` 也分别生成了这两个 middleware 源文件的编译规则和输出对象，说明 CMake 配置链路已同步。

## 3. Build 状态

Debug 构建目录：

```text
/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/
```

| 文件 | 修改/生成时间 | 结论 |
| --- | --- | --- |
| `Middleware/Sensor/BMI323/bmi323.c` | 2026-08-09 09:36:15 | 当前源码 |
| `.../bmi323.c.obj` | 2026-08-09 09:36:48 | 在源码之后重新编译 |
| `Middleware/Sensor/BMI323/bmi323_port.c` | 2026-08-09 09:14:23 | 当前 port 源码 |
| `.../bmi323_port.c.obj` | 2026-08-09 09:29:21 | 在源码之后生成并被增量复用 |
| `Smart_Car_H757_CM7.elf` | 2026-08-09 09:36:48 | 与最新 `bmi323.c.obj` 同次链接 |

注意：CMake/Ninja 使用 GCC 对象后缀 `.obj`，不是用户示例中的 `.o`。

## 4. ELF 状态

检查文件：

```text
/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf
```

`strings` 结果：

| 字符串 | ELF 状态 |
| --- | --- |
| `WHOAMI_FORCE` | 不存在 |
| `READ_REG_TRACE` | 不存在 |
| `WHOAMI_RAW` | 存在 |
| `SPI_RAW` | 存在 |

这证明当前 ELF 链接的是现有 `WHOAMI_RAW`/`SPI_RAW` 实现，而不是用户描述的两个新标识。

## 5. 烧录文件状态

CM7 文档明确指定唯一固件产物：

```text
STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf
```

CM7 构建目录没有生成 `.hex` 或固件 `.bin`。搜索到的 `CMakeDetermineCompilerABI_*.bin` 是 CMake 编译器探测文件，不是可烧录固件。

仓库内没有发现 STM32 CM7 烧录脚本或烧录命令记录，因此无法从源码/构建目录确认设备当前实际烧录的是哪个版本。

## 6. 多工程副本检查

`/Users/zhiqin/Projects/Smart_Car/` 下名为 `Smart_Car_H757_CM7.elf` 的文件只有一个：

```text
/Users/zhiqin/Projects/Smart_Car/STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf
```

## 7. 根因判断

链路状态如下：

```text
目标调试字符串
    -> 源码：不存在
    -> CMake：BMI323 源文件已加入
    -> Build：BMI323 对象已重编译
    -> ELF：只包含现有 WHOAMI_RAW/SPI_RAW
    -> 烧录：仓库无记录，设备版本未知
```

确定根因：`WHOAMI_FORCE` 和 `READ_REG_TRACE` 没有进入当前源码，因此不可能通过当前 CMake、Build 或 ELF 出现在运行日志中。CMake 和本地 Debug ELF 对当前源码是同步的；运行时仍只有 `[BMI323][DEBUG]` 还可能表示设备没有烧录这个唯一 Debug ELF，但该点没有设备读回证据。

