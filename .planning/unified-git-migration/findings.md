# Unified Git Migration Findings

## 2026-08-07 Baseline

- The root `Smart_Car/.git/` already exists, is non-bare, and has `main`
  checked out at `796deed`.
- The root has one worktree, no `.gitmodules`, no configured submodules, and
  no active root submodules.
- Eight nested `.git/` directories and three nested `.git` pointer files were
  found. The pointer targets are absent, so they are stale cJSON submodule
  metadata rather than usable repositories.
- The pre-existing root worktree is dirty. Its tracked source changes and
  untracked project trees must be preserved and will be captured by the
  requested repository-wide staging operation after generated output is ignored.
- `ESPS3/` is the current ESP32-S3 source path. `ESP32_S3/` does not exist, so
  the migration adds a tracked empty-directory marker without moving source.
- `Hardware/` does not exist, so the migration adds a tracked empty-directory
  marker only.
- A dry-run stage scan reached a 38.9 GB vendor archive under `资料/`.
  The directory is retained locally but ignored as external reference material
  so the unified source repository remains practical to clone and review.
