import Foundation

public enum VehicleLinkState: Equatable, Sendable {
    case idle
    case discovering
    case connecting
    case negotiating
    case synchronizing
    case ready
    case staleStatus
    case stopping
    case disconnected
    case fault(String)
    case emergencyStopLatched
}

public struct VehicleStatus: Equatable, Sendable {
    public var link: VehicleLinkState = .idle
    public var driveEnabled = false
    public var stm32Ready = false
    public var faultBits: UInt32 = 0
    public var lastAcceptedSequence: UInt32?
    public var lastUpdate: Date?

    public var hasFreshStatus: Bool {
        guard let lastUpdate else { return false }
        return Date().timeIntervalSince(lastUpdate) < 2.0
    }

    public var canAcceptManualInput: Bool {
        if case .ready = link { return driveEnabled && stm32Ready && hasFreshStatus }
        return false
    }
}
