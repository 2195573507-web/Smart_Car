import Foundation

enum SmartCarProtocol {
    static let head: UInt8 = 0xAA
    static let version: UInt8 = 0x01
    static let tail: UInt8 = 0x55
    static let maxPayload = 128

    enum FrameType: UInt8 {
        case control = 0x01
        case status = 0x02
        case ping = 0x05
        case ack = 0x06
        case imuStatus = 0x10
        case attitude = 0x11
        case imuCalibrationStatus = 0x12
        case imuCalibrationBias = 0x13
            case radarPWMControl = 0x14
            case radarStatus = 0x15
            case pwmSet = 0x16
            case pwmApplied = 0x17
            case radarCalibrationStatus = 0x18
            case pwmError = 0x19

        var displayName: String {
            switch self {
            case .control: return "CONTROL"
            case .status: return "STATUS"
            case .ping: return "PING"
            case .ack: return "ACK"
            case .imuStatus: return "IMU_STATUS"
            case .attitude: return "ATTITUDE"
            case .imuCalibrationStatus: return "IMU_CAL_STATUS"
            case .imuCalibrationBias: return "IMU_CAL_BIAS"
            case .radarPWMControl: return "RADAR_PWM_CONTROL"
            case .radarStatus: return "RADAR_STATUS"
            case .pwmSet: return "PWM_SET"
            case .pwmApplied: return "PWM_APPLIED"
            case .radarCalibrationStatus: return "RADAR_CAL_STATUS"
            case .pwmError: return "PWM_ERROR"
            }
        }
    }

    enum ControlCommand: UInt8 {
        case stop = 0x01
        case moveForward = 0x02
        case moveBack = 0x03
        case turnLeft = 0x04
        case turnRight = 0x05
        case speedControl = 0x06

        var displayName: String {
            switch self {
            case .stop: return "STOP"
            case .moveForward: return "MOVE_FORWARD"
            case .moveBack: return "MOVE_BACK"
            case .turnLeft: return "TURN_LEFT"
            case .turnRight: return "TURN_RIGHT"
            case .speedControl: return "SPEED_CONTROL"
            }
        }
    }

    struct Frame: Identifiable, Equatable {
        let id = UUID()
        let raw: Data
        let type: UInt8
        let payload: Data

        var typeName: String {
            FrameType(rawValue: type)?.displayName ?? String(format: "TYPE 0x%02X", type)
        }

        var decoded: DecodedMessage? {
            try? DecodedMessage(frame: self)
        }
    }

    struct Parser {
        private var buffer: [UInt8] = []

        mutating func reset() {
            buffer.removeAll(keepingCapacity: true)
        }

        mutating func feed(_ data: Data) -> [Frame] {
            buffer.append(contentsOf: data)
            var frames: [Frame] = []

            while true {
                guard let headIndex = buffer.firstIndex(of: head) else {
                    buffer.removeAll(keepingCapacity: true)
                    break
                }
                if headIndex > 0 {
                    buffer.removeFirst(headIndex)
                }
                guard buffer.count >= 5 else { break }

                let payloadLength = Int(buffer[3]) | (Int(buffer[4]) << 8)
                guard payloadLength <= maxPayload else {
                    buffer.removeFirst()
                    continue
                }
                let frameLength = payloadLength + 8
                guard buffer.count >= frameLength else { break }

                let candidate = Array(buffer.prefix(frameLength))
                guard candidate.last == tail,
                      candidate[1] == version else {
                    buffer.removeFirst()
                    continue
                }

                let crcOffset = 5 + payloadLength
                let receivedCRC = UInt16(candidate[crcOffset]) |
                    (UInt16(candidate[crcOffset + 1]) << 8)
                guard crc16Modbus(Array(candidate[1..<crcOffset])) == receivedCRC else {
                    buffer.removeFirst()
                    continue
                }

                frames.append(Frame(raw: Data(candidate),
                                    type: candidate[2],
                                    payload: Data(candidate[5..<crcOffset])))
                buffer.removeFirst(frameLength)
            }
            return frames
        }
    }

    static func encode(type: FrameType, payload: Data = Data()) -> Data {
        encode(type: type.rawValue, payload: payload)
    }

    static func encode(type: UInt8, payload: Data = Data()) -> Data {
        precondition(payload.count <= maxPayload, "SmartCar Protocol v1 payload exceeds 128 bytes")
        let bytes = [version, type,
                     UInt8(payload.count & 0xFF),
                     UInt8((payload.count >> 8) & 0xFF)] + Array(payload)
        let crc = crc16Modbus(bytes)
        return Data([head] + bytes + [UInt8(crc & 0xFF), UInt8(crc >> 8), tail])
    }

    static func control(_ command: ControlCommand, data: Data = Data()) -> Data {
        encode(type: .control, payload: Data([command.rawValue]) + data)
    }

    static func ping() -> Data { encode(type: .ping) }

    static func crc16Modbus(_ bytes: [UInt8]) -> UInt16 {
        var crc: UInt16 = 0xFFFF
        for byte in bytes {
            crc ^= UInt16(byte)
            for _ in 0..<8 {
                crc = (crc & 1) == 1 ? (crc >> 1) ^ 0xA001 : crc >> 1
            }
        }
        return crc
    }
}
