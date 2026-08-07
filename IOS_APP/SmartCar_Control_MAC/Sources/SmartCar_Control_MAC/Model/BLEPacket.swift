enum ControlMode: String, CaseIterable, Identifiable {
    case control = "Control Mode"
    case developer = "Developer Mode"

    var id: String { rawValue }
}

enum BLEConnectionStatus: Equatable {
    case disconnected
    case scanning
    case connecting
    case connected
    case unavailable
    case failed(String)

    var displayText: String {
        switch self {
        case .disconnected: return "Disconnected"
        case .scanning: return "Scanning"
        case .connecting: return "Connecting"
        case .connected: return "Connected"
        case .unavailable: return "Bluetooth Unavailable"
        case .failed(let message): return "Error: \(message)"
        }
    }
}
