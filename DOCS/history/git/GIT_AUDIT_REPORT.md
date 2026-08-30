# Git Audit Report

Audit date: 2026-08-07. This report records repository metadata discovered
before the unified-repository migration. It does not alter source, project
configuration, hardware definitions, or runtime behavior.

## Repository List

| Path | Type | Current status | Migration required | Risk |
| --- | --- | --- | --- | --- |
| `.git/` | ROOT | Active `main` repository; one root worktree; no root submodules | No; preserve as the unified root | Existing root has tracked and untracked user work that must be staged deliberately |
| `.analysis_extract/tar_xz/07/src/rf2o_laser_odometry/.git/` | NESTED | Full external Git repository; working tree modified | Yes | Removing metadata preserves extracted files but makes its independent history inaccessible from this workspace |
| `.analysis_extract/tar_xz/07/src/robot_pose_publisher/.git/` | NESTED | Full external Git repository | Yes | Extracted reference remains ignored and is not a Smart_Car source authority |
| `.analysis_extract/zip/07/ros2_ws/.git/` | NESTED | Full external Git repository; working tree modified | Yes | Extracted reference remains ignored and is not a Smart_Car source authority |
| `S3-radartest/.git/` | NESTED | Full repository with existing modified/deleted paths | Yes | Its files remain; independent branch, remote, and history metadata are removed from this checkout |
| `S3-radartest/ESP-server/.git/` | NESTED | Full repository with local untracked notes | Yes | Its files become ordinary root-repository content |
| `S3-radartest/ESPS3-Radar-Debug/.git/` | NESTED | Repository metadata with no commits and local files | Yes | Its files become ordinary root-repository content |
| `S3-radartest/分支项目/ESP1/.git/` | NESTED | Full repository with local modifications and commits ahead of origin | Yes | Its files become ordinary root-repository content; remote/branch metadata are removed |
| `S3-radartest/分支项目/ESP1/ESP-server/.git/` | NESTED | Full repository with local untracked notes | Yes | Its files become ordinary root-repository content |
| `S3-radartest/ESPC51/managed_components/espressif__cjson/cJSON/.git` | GITFILE | Stale cJSON submodule pointer; referenced gitdir is absent | Yes | Removing the pointer is required for normal root tracking; no independent history is reachable |
| `S3-radartest/ESPC52/managed_components/espressif__cjson/cJSON/.git` | GITFILE | Stale cJSON submodule pointer; referenced gitdir is absent | Yes | Removing the pointer is required for normal root tracking; no independent history is reachable |
| `S3-radartest/分支项目/ESP1/ESPC52/managed_components/espressif__cjson/cJSON/.git` | GITFILE | Stale cJSON submodule pointer; referenced gitdir is absent | Yes | Removing the pointer is required for normal root tracking; no independent history is reachable |

## Submodules and Worktrees

- Root `.gitmodules` is absent.
- Root Git configuration has no `submodule.*` entries.
- `git submodule status` is empty.
- `git worktree list` reports only the root worktree.
- No nested `.gitmodules` file was found. The three cJSON Gitfiles are stale
  pointers, not active submodules in the root repository.

## Audit Result

Before migration there are 12 Git metadata locations: one root repository,
eight nested `.git/` directories, and three nested `.git` pointer files. The
migration removes the 11 nested locations only. No project directory, source
file, project file, or historical Markdown file is deleted.
