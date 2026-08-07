# Git Migration Report

Migration date: 2026-08-07

## Summary

- Original Git metadata locations: 12 total: one root repository, eight nested
  `.git/` directories, and three nested `.git` pointer files.
- Nested Git metadata migrated out of the workspace: 11 locations.
- Final repository metadata: one root repository at `Smart_Car/.git`.
- Unified initialization commit: `6de387a` (`chore: initialize unified
  Smart_Car repository`).

## Final Repository Structure

```text
Smart_Car/
|- .git/
|- .codex/
|- DOCS/                         (the local `docs/` spelling resolves here)
|- STM32H757/
|- ESPS3/                        (current ESP32-S3 source tree)
|- ESP32_S3/                     (requested structural placeholder)
|- IOS_APP/
|- ROS2_WIN/
|- Hardware/
|- Tools/
|- README.md
|- .gitignore
|- GIT_AUDIT_REPORT.md
`- GIT_MIGRATION_REPORT.md
```

`ROS2_WIN/`, `Hardware/`, and `ESP32_S3/` use Git placeholder files where no
source tree exists. The existing documentation authority is tracked as
`DOCS/`; on this case-insensitive macOS filesystem, `docs/` resolves to that
same directory, including `docs/development/git_workflow.md`. This preserves
the established documentation links without a repository-wide case-only move.

## Ignore Policy

The root `.gitignore` excludes:

- STM32/CMake outputs: `Debug/`, `Release/`, `build/`, CMake state, and
  `*.elf`, `*.hex`, `*.bin`, `*.map`, `*.o`, and `*.d`.
- ESP-IDF outputs and generated configuration: `build/`, `sdkconfig`, and
  `sdkconfig.*`, while retaining `sdkconfig.defaults`.
- Apple/SwiftPM outputs: `DerivedData/`, `.build/`, `xcuserdata/`,
  `*.xcuserstate`, `dist/`, and packaged `.app` directories.
- ROS2 outputs: `build/`, `install/`, and `log/`.
- General transient material: `.DS_Store`, `Thumbs.db`, temporary and log
  files, dependency caches, analysis output, and local tool session state.

Source files, headers, `.ioc`, linker scripts, CMake files,
`idf_component.yml`, and `sdkconfig.defaults` remain eligible for tracking.

## Validation Result

- `find . -type d -name .git -prune -print` returns only `./.git`.
- No file named `.git` and no `.gitmodules` file remains in the workspace.
- `git submodule status` is empty.
- `git worktree list --porcelain` reports one root worktree.
- Root `.gitignore` matches representative STM32, ESP32, Apple, ROS2, cache,
  and temporary-output paths.
- No compilation, flashing, device connection, source-code edit, or project
  configuration edit was performed for this migration.

## Risks And Recovery

- The 11 removed nested metadata locations were moved, not permanently
  deleted, to `/tmp/smartcar-git-metadata-backup-20260807/`. Their independent
  branches, remotes, and histories are no longer active in the workspace but
  can be inspected from that backup if required.
- `资料/` is intentionally ignored because it contains approximately 38.9 GB
  of external vendor archives and reference material. `.analysis_extract/` is
  also ignored as generated analysis material. Neither is present after a
  fresh clone unless restored separately.
- The single-repository checks validate Git structure only. They do not prove
  firmware, application, transport, hardware, or vehicle behavior.
