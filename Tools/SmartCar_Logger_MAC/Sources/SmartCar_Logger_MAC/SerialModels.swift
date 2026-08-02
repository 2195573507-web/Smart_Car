import Foundation

struct SerialPortDescriptor: Identifiable, Hashable, Sendable {
    let path: String
    let name: String
    let isCH340: Bool

    var id: String { path }

    var label: String {
        isCH340 ? "\(name) · CH340/USB-Serial" : name
    }
}

enum LoggerConnectionState: Equatable, Sendable {
    case disconnected
    case connecting
    case connected
    case failed(String)

    var label: String {
        switch self {
        case .disconnected: "未连接"
        case .connecting: "连接中..."
        case .connected: "已连接"
        case .failed(let message): message
        }
    }
}
