# Git Workflow

## Repository Strategy

Smart_Car uses one repository rooted at `Smart_Car/.git`. STM32H757, ESPS3,
IOS_APP, ROS2_WIN, Hardware, Tools, documentation, and future modules are
versioned together. Do not initialize a Git repository or add a submodule
inside any module directory.

`ESPS3/` is the current ESP32-S3 source path. `ESP32_S3/` is a structural
placeholder retained for the requested top-level layout; it is not a source
rename or an alternate firmware tree.

## Branch Strategy

| Branch | Purpose |
| --- | --- |
| `main` | Stable, integrated project state |
| `develop` | Integration work before promotion to `main` |
| `feature/*` | A bounded feature implementation |
| `fix/*` | A bounded defect repair |
| `docs/*` | Documentation-only work |

Create a branch before a substantial or risky change. Keep a branch focused on
one ownership boundary and merge only after the required validation evidence is
recorded.

## Commit Convention

Use:

```text
type(scope): message
```

Allowed `type` values are `feat`, `fix`, `refactor`, `docs`, `test`, and
`chore`.

Example:

```text
fix(stm32): repair uart timeout handling
```

## Staging Rules

1. Run `git status` before editing and before staging.
2. Stage only the files that belong to the current task, unless a documented
   repository-wide migration intentionally stages the whole tree.
3. Do not commit generated build products, dependency caches, transient logs,
   device captures, local IDE state, or external vendor archives/media. Verify
   with `git check-ignore` when in doubt.
4. One coherent feature or repair uses one commit. Do not combine unrelated
   work merely to make the worktree clean.
5. Do not run `git reset --hard`, delete a repository, or create a nested Git
   repository without explicit authorization.

## Verification

Before a commit, inspect `git diff --check`, `git status`, and the staged file
list. A clean Git check verifies repository metadata only; it does not replace
firmware builds, device tests, transport captures, or vehicle acceptance.
