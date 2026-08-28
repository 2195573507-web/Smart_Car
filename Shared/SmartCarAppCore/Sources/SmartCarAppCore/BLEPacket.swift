public enum ControlMode: String, CaseIterable, Identifiable, Sendable {
    case control = "Control Mode"
    case developer = "Developer Mode"

    public var id: String { rawValue }
}

public enum BLEConnectionStatus: Equatable, Sendable {
    case disconnected
    case scanning
    case connecting
    case connected
    case unavailable
    case failed(String)

    public var displayText: String {
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
