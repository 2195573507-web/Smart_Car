// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "SmartCarAppCore",
    platforms: [
        .iOS(.v17),
        .macOS(.v14)
    ],
    products: [
        .library(name: "SmartCarAppCore", targets: ["SmartCarAppCore"])
    ],
    targets: [
        .target(name: "SmartCarAppCore"),
        .testTarget(name: "SmartCarAppCoreTests", dependencies: ["SmartCarAppCore"])
    ]
)
