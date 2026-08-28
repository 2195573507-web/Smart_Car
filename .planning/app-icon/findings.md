# Findings: SmartCar App Icon Integration

- `IOS-APP/SmartCarIOS.xcodeproj` is the live iOS Xcode target and already sets
  `ASSETCATALOG_COMPILER_APPICON_NAME = AppIcon`.
- `IOS-APP/project.yml` does not currently declare a resource catalog.
- `IOS_APP/SmartCar_Control_MAC` is absent from the filesystem and appears as
  user-deleted files in `git status`; restoring it would cross the protected
  dirty-worktree boundary.
- The cleaned reference asset is a 1024x1024 RGBA PNG with transparent outer
  corners and no visible watermark.
