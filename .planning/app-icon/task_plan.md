# Task Plan: SmartCar App Icon Integration

## Goal

Use the approved clean reference-style SmartCar icon in the live app asset
surface without disturbing the user's in-progress app migration.

## Phases

- [x] Inspect live app targets and dirty-worktree boundaries
- [x] Create shared iOS/macOS icon asset catalog
- [x] Attach the catalog to the live iOS Xcode target
- [x] Verify dimensions, alpha, project generation, and host build
- [x] Record the macOS target boundary and deliver paths

## Status

Complete. The live iOS target consumes `AppIcon`; `MacAppIcon` compiles to an
`.icns` asset but the historical macOS target is absent from the worktree and
was not restored implicitly.
