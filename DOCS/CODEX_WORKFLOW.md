# Codex Workflow

## Required Start Sequence

Before a Smart_Car task, read:

1. [DEVELOPMENT_INDEX.md](DEVELOPMENT_INDEX.md)
2. [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md)
3. [MULTI_AGENT_RULES.md](MULTI_AGENT_RULES.md)
4. The documents for the affected subsystem

Define the single task objective, writable paths, protected paths, expected evidence, and excluded behavior before implementation starts.

## Three-Agent Flow

```text
Implementation Agent -> Review / Audit Agent -> Documentation Agent -> Final evidence summary
```

For this initialization, the roles are implemented independently: the Implementation Agent creates permitted project/configuration artifacts, the Review / Audit Agent performs a read-only independent audit, and the Documentation Agent maintains the governance set. No role may silently absorb another role's authority.

## Change Rules

- Keep changes within the explicitly authorized subsystem and paths.
- Do not add business algorithms, SLAM, navigation, app functions, motor-control logic, lidar parsing, or ESP32 functional changes during infrastructure work.
- Do not add a third-party library merely to make a baseline build pass.
- Do not claim a generated file, a parsed IOC, or a build as proof of electrical, device, network, or vehicle behavior.
- Record open configuration conflicts and pending validation instead of selecting an unverified workaround.

## Validation Levels

| Level | Examples | Does not prove |
| --- | --- | --- |
| Static | Markdown link checks, pin-table review, IOC text inspection | CubeMX generation, compilation, or hardware function |
| CubeMX | IOC opens, pin-conflict report is clean, generated project is parseable | Successful compiler/linker run or board behavior |
| Build | CM7/CM4 compilation and link of generated base project | Flashing, boot, peripheral transfer, or vehicle behavior |
| Hardware / integration | Board checks, bus capture, network exercise, end-to-end test | Must be explicitly run and recorded; never inferred from lower levels |

## Completion Record

The final task report must separately state the modified files, static/CubeMX/build evidence actually obtained, unresolved items, and all hardware or integration validation still pending.
