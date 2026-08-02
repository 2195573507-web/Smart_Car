// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "SmartCar_Logger_MAC",
    platforms: [.macOS(.v14)],
    products: [
        .executable(name: "SmartCar_Logger_MAC", targets: ["SmartCar_Logger_MAC"]),
    ],
    targets: [
        .executableTarget(name: "SmartCar_Logger_MAC"),
    ]
)
