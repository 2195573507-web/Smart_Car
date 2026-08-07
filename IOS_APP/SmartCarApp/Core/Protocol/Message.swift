import Foundation

public enum Message {
    public static func packet(for intent: ControlIntent, sessionID: UInt32, sequence: UInt32, now: Date = Date()) -> Packet {
        let payload: [UInt8]
        let authority: CommandAuthority = intent.kind == .emergencyStop ? .emergency : .manual
        switch intent {
        case .manualPad(let direction): payload = [direction.rawValue]
        case .manualJoystick(let joystick): payload = [UInt8(bitPattern: joystick.linear), UInt8(bitPattern: joystick.turn)]
        default: payload = []
        }
        return Packet(type: intent.kind.rawValue, authority: authority.rawValue, sessionID: sessionID, sequence: sequence, timestampMilliseconds: UInt32(now.timeIntervalSince1970 * 1000), payload: payload)
    }
}
