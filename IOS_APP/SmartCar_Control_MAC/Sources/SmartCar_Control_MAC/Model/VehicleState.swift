import Foundation

struct Vector3: Equatable {
    var x: Float = 0
    var y: Float = 0
    var z: Float = 0
}

struct Quaternion: Equatable {
    var q0: Float = 1
    var q1: Float = 0
    var q2: Float = 0
    var q3: Float = 0
}

struct AttitudeData: Equatable {
    var roll: Float = 0
    var pitch: Float = 0
    var yaw: Float = 0
    var valid = false
    var source: AttitudeSource = .none
}

enum AttitudeSource: UInt8, Equatable {
    case none = 0x00
    case lsm303 = 0x01
    case bmi323Fusion = 0x02

    var displayName: String {
        switch self {
        case .none: return "NONE"
        case .lsm303: return "LSM303"
        case .bmi323Fusion: return "BMI323_FUSION"
        }
    }
}

struct IMUData: Equatable {
    var online = false
    var accel = Vector3()
    var gyro = Vector3()
    var mag = Vector3()
}

enum IMUSensorID: UInt8 {
    case bmi323 = 0x01
    case lsm303 = 0x02
}

struct SmartCarStatus: Equatable {
    let battery: UInt8
    let motorState: UInt8
    let errorCode: UInt16
}

struct RadarStatus: Equatable {
    let online: Bool
    let speedPercent: UInt8
}

struct RadarCalibrationStatus: Equatable {
    let currentPWM: UInt8
    let active: Bool
}

struct IMUStatus: Equatable {
    let sensorID: IMUSensorID
    let data: IMUData
    let calibrationState: IMUCalibrationSummaryState
    let calibrationSample: UInt16
    let calibrationTotal: UInt16
}

enum IMUCalibrationSummaryState: UInt8, Equatable {
    case idle = 0
    case waitStable = 1
    case collecting = 2
    case complete = 3
    case error = 4
}

enum IMUCalibrationState: UInt8, Equatable {
    case idle = 0
    case setPWM = 1
    case waitStable = 2
    case sample = 3
    case complete = 4
    case error = 5
}

enum IMUCalibrationSampleMode: UInt8, Equatable {
    case `static` = 0
    case vibration = 1
}

struct IMUCalibrationStatus: Equatable {
    let state: IMUCalibrationState
    let sampleMode: IMUCalibrationSampleMode
    let totalProgress: UInt8
    var currentPWM: UInt8
    let sampleProgress: UInt16
    let errorCode: UInt8
    var lastUpdatedAt: Date?

    func isWaitingForSTM(at date: Date = Date()) -> Bool {
        guard let lastUpdatedAt else { return true }
        return date.timeIntervalSince(lastUpdatedAt) > 3.0
    }
}

struct IMUCalibrationBias: Equatable {
    let x: Float
    let y: Float
    let z: Float
}

enum DecodedMessage: Equatable {
    case control(command: SmartCarProtocol.ControlCommand, speed: UInt8?)
    case status(SmartCarStatus)
    case ping
    case ack(Data)
    case imuStatus(IMUStatus)
    case attitude(AttitudeData)
    case imuCalibrationStatus(IMUCalibrationStatus)
    case imuCalibrationBias(IMUCalibrationBias)
    case radarStatus(RadarStatus)
    case radarCalibrationStatus(RadarCalibrationStatus)

    init(frame: SmartCarProtocol.Frame) throws {
        let bytes = Array(frame.payload)
        guard let type = SmartCarProtocol.FrameType(rawValue: frame.type) else { throw DecodeError.unsupportedType }
        switch type {
        case .control:
            guard let commandRaw = bytes.first, let command = SmartCarProtocol.ControlCommand(rawValue: commandRaw) else { throw DecodeError.invalidPayload("CONTROL command missing") }
            if command == .speedControl {
                guard bytes.count == 2 else { throw DecodeError.invalidPayload("SPEED_CONTROL requires speed") }
                self = .control(command: command, speed: bytes[1])
            } else {
                guard bytes.count == 1 else { throw DecodeError.invalidPayload("CONTROL payload length") }
                self = .control(command: command, speed: nil)
            }
        case .status:
            guard bytes.count == 4 else { throw DecodeError.invalidPayload("STATUS requires 4 bytes") }
            self = .status(SmartCarStatus(battery: bytes[0], motorState: bytes[1], errorCode: uint16(bytes, at: 2)))
        case .ping:
            guard bytes.isEmpty else { throw DecodeError.invalidPayload("PING payload must be empty") }
            self = .ping
        case .ack:
            self = .ack(frame.payload)
        case .imuStatus:
            #if SMARTCAR_ENABLE_IMU_STATUS_LOG
            let typeString = String(format: "%02X", frame.type)
            print("RX type=0x\(typeString) length=\(frame.payload.count) raw=\(hex(frame.raw))")
            #endif
            guard (bytes.count == 38 || bytes.count == 43),
                  let sensor = IMUSensorID(rawValue: bytes[0]) else { throw DecodeError.invalidPayload("IMU_STATUS requires 38 or 43 bytes") }
            let calibrationState = bytes.count == 43
                ? (IMUCalibrationSummaryState(rawValue: bytes[38]) ?? .idle)
                : .idle
            let calibrationSample = bytes.count == 43 ? uint16(bytes, at: 39) : 0
            let calibrationTotal = bytes.count == 43 ? uint16(bytes, at: 41) : 0
            let status = IMUStatus(sensorID: sensor, data: IMUData(
                online: bytes[1] == 0x01,
                accel: Vector3(x: float32(bytes, at: 2), y: float32(bytes, at: 6), z: float32(bytes, at: 10)),
                gyro: Vector3(x: float32(bytes, at: 14), y: float32(bytes, at: 18), z: float32(bytes, at: 22)),
                mag: Vector3(x: float32(bytes, at: 26), y: float32(bytes, at: 30), z: float32(bytes, at: 34))
            ), calibrationState: calibrationState, calibrationSample: calibrationSample,
            calibrationTotal: calibrationTotal)
            self = .imuStatus(status)

            #if SMARTCAR_ENABLE_IMU_STATUS_LOG
            let sensorName = sensor == .lsm303 ? "LSM303" : "BMI323"
            let lsmOnline = sensor == .lsm303 && status.data.online
            print("IMU_STATUS sensor=\(sensorName) lsm_online=\(lsmOnline ? 1 : 0) ax=\(status.data.accel.x) ay=\(status.data.accel.y) az=\(status.data.accel.z) mx=\(status.data.mag.x) my=\(status.data.mag.y) mz=\(status.data.mag.z)")
            #endif
        case .attitude:
            guard bytes.count == 14,
                  bytes[12] <= 1,
                  let source = AttitudeSource(rawValue: bytes[13]) else {
                throw DecodeError.invalidPayload("ATTITUDE requires 14 bytes")
            }
            self = .attitude(AttitudeData(
                roll: float32(bytes, at: 0), pitch: float32(bytes, at: 4), yaw: float32(bytes, at: 8),
                valid: bytes[12] == 1,
                source: source
            ))
        case .imuCalibrationStatus:
            guard bytes.count == 7,
                  let state = IMUCalibrationState(rawValue: bytes[0]),
                  let sampleMode = IMUCalibrationSampleMode(rawValue: bytes[1]),
                  bytes[5] <= 100 else {
                throw DecodeError.invalidPayload("IMU_CAL_STATUS payload length or progress")
            }
            let currentPWM = bytes[2]
            let sampleProgress = uint16(bytes, at: 3)
            self = .imuCalibrationStatus(IMUCalibrationStatus(
                state: state,
                sampleMode: sampleMode,
                totalProgress: bytes[5],
                currentPWM: currentPWM,
                sampleProgress: sampleProgress,
                errorCode: bytes[6],
                lastUpdatedAt: nil
            ))
        case .imuCalibrationBias:
            guard bytes.count == 12 else {
                throw DecodeError.invalidPayload("IMU_CAL_BIAS requires three float values")
            }
            self = .imuCalibrationBias(IMUCalibrationBias(
                x: float32(bytes, at: 0),
                y: float32(bytes, at: 4),
                z: float32(bytes, at: 8)
            ))
        case .radarPWMControl:
            throw DecodeError.invalidPayload("RADAR_PWM_CONTROL is app to S3 only")
        case .pwmSet:
            throw DecodeError.invalidPayload("PWM_SET is STM32 to S3 only")
        case .pwmApplied:
            throw DecodeError.invalidPayload("PWM_APPLIED is S3 to STM32 only")
        case .radarStatus:
            guard bytes.count == 2,
                  bytes[0] <= 1,
                  bytes[1] <= 100 else {
                throw DecodeError.invalidPayload("RADAR_STATUS requires online and speed percent")
            }
            self = .radarStatus(RadarStatus(online: bytes[0] == 1, speedPercent: bytes[1]))
        case .radarCalibrationStatus:
            guard bytes.count == 2, bytes[0] <= 100, bytes[1] <= 1 else {
                throw DecodeError.invalidPayload("RADAR_CAL_STATUS requires PWM and active")
            }
            self = .radarCalibrationStatus(
                RadarCalibrationStatus(currentPWM: bytes[0], active: bytes[1] == 1))
        case .pwmError:
            throw DecodeError.invalidPayload("PWM_ERROR is S3 to STM32 only")
        }
    }

    var summary: String {
        switch self {
        case .control(let command, let speed): return speed.map { "CONTROL \(command.displayName) speed=\($0)" } ?? "CONTROL \(command.displayName)"
        case .status(let status): return "STATUS battery=\(status.battery)% motor=\(status.motorState) error=0x\(String(format: "%04X", status.errorCode))"
        case .ping: return "PING"
        case .ack(let data): return "ACK \(hex(data))"
        case .imuStatus(let imu): return "IMU_STATUS \(imu.sensorID == .bmi323 ? "BMI323" : "LSM303") \(imu.data.online ? "ONLINE" : "OFFLINE") cal=\(imu.calibrationState.rawValue) \(imu.calibrationSample)/\(imu.calibrationTotal)"
        case .attitude(let attitude): return "ATTITUDE roll=\(attitude.roll.displayValue) pitch=\(attitude.pitch.displayValue) yaw=\(attitude.yaw.displayValue) valid=\(attitude.valid ? 1 : 0) source=\(attitude.source.displayName)"
        case .imuCalibrationStatus(let status): return "IMU_CAL_STATUS state=\(status.state.rawValue) mode=\(status.sampleMode.rawValue) progress=\(status.totalProgress)% pwm=\(status.currentPWM) sample=\(status.sampleProgress) error=\(status.errorCode)"
        case .imuCalibrationBias(let bias): return "IMU_CAL_BIAS x=\(bias.x.displayValue) y=\(bias.y.displayValue) z=\(bias.z.displayValue)"
        case .radarStatus(let status): return "RADAR_STATUS \(status.online ? "ONLINE" : "OFFLINE") speed=\(status.speedPercent)%"
        case .radarCalibrationStatus(let status): return "RADAR_CAL_STATUS pwm=\(status.currentPWM)% active=\(status.active ? 1 : 0)"
        }
    }
}

struct DecodedMessageRecord: Identifiable, Equatable {
    let id = UUID()
    let message: DecodedMessage
    let receivedAt: Date

    var displayText: String {
        "\(receivedAt.formatted(date: .omitted, time: .standard))  \(message.summary)"
    }
}

enum DecodeError: Error { case unsupportedType; case invalidPayload(String) }

struct VehicleState: Equatable {
    var connection: BLEConnectionStatus = .disconnected
    var battery: UInt8 = 0
    var motorState: UInt8 = 0
    var errorCode: UInt16 = 0
    var bmi323 = IMUData()
    var lsm303 = IMUData()
    var attitude = AttitudeData()
    var radar = RadarStatus(online: false, speedPercent: 0)
    var lastAttitudeAt: Date?
    var lastStatus: Date?
    var lastRadarStatus: Date?

    func attitudeStatus(now: Date = Date()) -> AttitudeDisplayStatus {
        guard let lastAttitudeAt,
              now.timeIntervalSince(lastAttitudeAt) < 3.0 else {
            return .timeout
        }
        return attitude.valid ? .valid : .invalid
    }

    var smartCarS3Status: String {
        guard connection == .connected else { return "OFFLINE" }
        guard let lastStatus else { return "WAITING" }
        return Date().timeIntervalSince(lastStatus) < 3 ? "ONLINE" : "STALE"
    }

    mutating func apply(_ message: DecodedMessage, at date: Date) {
        switch message {
        case .status(let status): battery = status.battery; motorState = status.motorState; errorCode = status.errorCode; lastStatus = date
        case .imuStatus(let imu): if imu.sensorID == .bmi323 { bmi323 = imu.data } else { lsm303 = imu.data }
        case .attitude(let attitude): self.attitude = attitude; lastAttitudeAt = date
        case .radarStatus(let status): radar = status; lastRadarStatus = date
        default: break
        }
    }
}

enum AttitudeDisplayStatus: Equatable {
    case valid
    case invalid
    case timeout
}

extension Float {
    var displayValue: String { String(format: "%.3f", self) }
    var displayDegreeValue: String { String(format: "%.2f°", self) }
    var displayControlDegreeValue: String { String(format: "%.2f°", self) }
}

private func uint16(_ bytes: [UInt8], at index: Int) -> UInt16 { UInt16(bytes[index]) | (UInt16(bytes[index + 1]) << 8) }
private func float32(_ bytes: [UInt8], at index: Int) -> Float {
    let bits = UInt32(bytes[index]) | (UInt32(bytes[index + 1]) << 8) | (UInt32(bytes[index + 2]) << 16) | (UInt32(bytes[index + 3]) << 24)
    return Float(bitPattern: bits)
}
private func hex(_ data: Data) -> String { data.map { String(format: "%02X", $0) }.joined(separator: " ") }
