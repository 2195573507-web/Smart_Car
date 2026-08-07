import Foundation

public enum BLEState: Equatable, Sendable {
    case unavailable
    case poweredOff
    case idle
    case scanning
    case connecting
    case connected
    case disconnected
    case failed(String)
}
