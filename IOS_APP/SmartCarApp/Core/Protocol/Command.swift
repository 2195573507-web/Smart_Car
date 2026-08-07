import Foundation

public enum CommandKind: UInt8, Sendable {
    case stop = 0x02
    case emergencyStop = 0x03
    case manualPad = 0x10
    case manualJoystick = 0x11
    case autonomyIntent = 0x20
    case heartbeat = 0x30
}

public enum CommandAuthority: UInt8, Sendable {
    case manual = 0x01
    case autonomy = 0x02
    case emergency = 0xFF
}

public struct JoystickIntent: Equatable, Sendable {
    public let linear: Int8
    public let turn: Int8

    public init(linear: Int8, turn: Int8) {
        self.linear = linear
        self.turn = turn
    }
}

public enum ControlIntent: Equatable, Sendable {
    case stop
    case emergencyStop
    case manualPad(direction: Direction)
    case manualJoystick(JoystickIntent)
    case heartbeat

    public var kind: CommandKind {
        switch self {
        case .stop: return .stop
        case .emergencyStop: return .emergencyStop
        case .manualPad: return .manualPad
        case .manualJoystick: return .manualJoystick
        case .heartbeat: return .heartbeat
        }
    }
}

public enum Direction: UInt8, CaseIterable, Sendable {
    case forward = 0x01
    case reverse = 0x02
    case left = 0x03
    case right = 0x04
}
