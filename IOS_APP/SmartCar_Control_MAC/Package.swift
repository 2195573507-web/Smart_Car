// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "SmartCar_Control_MAC",
    defaultLocalization: "en",
    platforms: [
        .macOS(.v14)
    ],
    products: [
        .executable(name: "SmartCar_Control_MAC", targets: ["SmartCar_Control_MAC"])
    ],
    targets: [
        .executableTarget(
            name: "SmartCar_Control_MAC",
            path: "Sources/SmartCar_Control_MAC",
            resources: [
                .process("Resources")
            ]
        )
    ]
)
