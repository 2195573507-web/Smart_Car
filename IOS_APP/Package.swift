// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "SmartCarApp",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "SmartCarApp", targets: ["SmartCarApp"])
    ],
    targets: [
        .target(
            name: "SmartCarApp",
            path: "SmartCarApp"
        )
    ]
)
