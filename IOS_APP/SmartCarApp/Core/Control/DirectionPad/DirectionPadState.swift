import Foundation

public struct DirectionPadState: Equatable, Sendable {
    public private(set) var pressedDirection: Direction?

    public mutating func press(_ direction: Direction) {
        pressedDirection = direction
    }

    public mutating func release() {
        pressedDirection = nil
    }
}
