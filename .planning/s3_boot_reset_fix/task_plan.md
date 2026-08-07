# S3 Boot Reset Fix

## Objective

Prevent `radar_command_task()` from returning when USB Serial/JTAG stdin is
temporarily unavailable, while adding bounded diagnostics and preserving the
existing watchdog, brownout, and stack configuration.

## Phases

| Phase | Status | Evidence |
| --- | --- | --- |
| Confirm source-level reset chain | Complete | Analysis and current source agree on task return path |
| Implement bounded stdin recovery | Complete | `radar_test.c` retries after `fgets()` failure and never returns |
| Update reset analysis | Complete | Analysis records repaired path and runtime evidence boundary |
| Build and inspect | Complete | ESP-IDF 5.5.4 isolated build passed in `build_codex_s3_reset_fix` |

## Errors

| Error | Attempts | Resolution |
| --- | ---: | --- |
| Workspace is not a Git repository | 1 | Use explicit target-file checks; no commit claim |
