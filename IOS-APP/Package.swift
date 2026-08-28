// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "SmartCarIOS",
    defaultLocalization: "en",
    platforms: [
        .iOS(.v17),
        .macOS(.v14)
    ],
    products: [
        .executable(name: "SmartCarIOS", targets: ["SmartCarIOS"])
    ],
    dependencies: [
        .package(path: "../Shared/SmartCarAppCore")
    ],
    targets: [
        .executableTarget(
            name: "SmartCarIOS",
            dependencies: [
                .product(name: "SmartCarAppCore", package: "SmartCarAppCore")
            ],
            path: "Sources/SmartCarIOS",
            resources: [
                .copy("../../Resources")
            ]
        )
    ],
    swiftLanguageModes: [.v5]
)
