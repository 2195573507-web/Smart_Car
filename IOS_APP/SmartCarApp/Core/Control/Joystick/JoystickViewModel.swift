import Foundation

public struct JoystickViewModel: Sendable {
    public private(set) var intent = JoystickIntent(linear: 0, turn: 0)
    public mutating func update(linear: Double, turn: Double) { intent = JoystickIntent(linear: Int8(max(-1, min(1, linear)) * 127), turn: Int8(max(-1, min(1, turn)) * 127)) }
    public mutating func reset() { intent = JoystickIntent(linear: 0, turn: 0) }
}
