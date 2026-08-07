import Foundation

public enum EmergencyStopState: Equatable, Sendable {
    case available
    case requested
    case latched
}
