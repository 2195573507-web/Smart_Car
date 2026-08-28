# Deliverable: SmartCar App Icon Integration

Completed deliverables:

- `IOS-APP/Resources/Assets.xcassets/AppIcon.appiconset`: active iPhone/iPad
  icon set selected by the Xcode target's existing `AppIcon` setting.
- `IOS-APP/Resources/Assets.xcassets/MacAppIcon.appiconset`: macOS set
  generated from the same artwork.
- `IOS-APP/project.yml`: declares the resource build phase so XcodeGen retains
  the catalog on future project generation.

Verification:

- `xcodebuild ... -sdk iphonesimulator ... build`: passed and compiled
  `AppIcon` into the application bundle.
- `xcrun actool ... --platform macosx --app-icon MacAppIcon`: passed and
  generated `MacAppIcon.icns`.
- `swift build --package-path IOS-APP`: passed.
