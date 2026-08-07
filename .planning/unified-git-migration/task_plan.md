# Unified Git Migration Plan

## Scope

Convert the existing Smart_Car root repository into the only Git repository in
the workspace. Preserve all directories and files; remove only the audited
nested `.git` metadata. Do not modify source, project configuration, hardware,
or runtime behavior.

## Steps

| Step | Status | Evidence |
| --- | --- | --- |
| Audit root, submodule, worktree, and nested Git metadata | Complete | `GIT_AUDIT_REPORT.md` |
| Add ignore rules and Git workflow documentation | Complete | `.gitignore`, `docs/development/git_workflow.md` |
| Remove exact nested Git metadata locations | Complete | Only root `.git` remains; metadata archived outside the workspace |
| Stage intended unified content and verify ignores | Pending | `git status --ignored`, staged diff |
| Create requested root commit | Pending | Commit hash and `GIT_MIGRATION_REPORT.md` |
| Final one-repository verification | Pending | `find . -name .git`, submodule/worktree checks |

## Safety Rules

- Do not use `git reset`, `git checkout`, or regenerate any project.
- Remove only the 11 paths listed in `GIT_AUDIT_REPORT.md`.
- Keep `.analysis_extract/` as an ignored generated-analysis directory.
- Keep the existing `ESPS3/` source path; do not rename or relocate source.
