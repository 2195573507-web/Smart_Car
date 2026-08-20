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
    var rollRad: Float = 0
    var pitchRad: Float = 0
    var yawRad: Float = 0
    var rollDeg: Float = 0
    var pitchDeg: Float = 0
    var yawDeg: Float = 0
    var timestampMs: UInt32 = 0
    var valid = false
    var source: AttitudeSource = .none

    func value(for axis: AttitudeAxis, unit: AngleUnit) -> Float {
        switch (axis, unit) {
        case (.roll, .radian): return rollRad
        case (.pitch, .radian): return pitchRad
        case (.yaw, .radian): return yawRad
        case (.roll, .degree): return rollDeg
        case (.pitch, .degree): return pitchDeg
        case (.yaw, .degree): return yawDeg
        }
    }
}

struct DualAttitudePose: Equatable {
    var rollRad: Float
    var pitchRad: Float
    var yawRad: Float
    var quaternion: Quaternion
    var valid: Bool

    var quaternionIsFiniteUnit: Bool {
        let values = [rollRad, pitchRad, yawRad, quaternion.q0, quaternion.q1,
                      quaternion.q2, quaternion.q3]
        let norm = sqrtf(quaternion.q0 * quaternion.q0 +
                         quaternion.q1 * quaternion.q1 +
                         quaternion.q2 * quaternion.q2 +
                         quaternion.q3 * quaternion.q3)
        return values.allSatisfy(\.isFinite) &&
            abs(norm - 1.0) <= 0.2
    }

    var isFiniteUnit: Bool { valid && quaternionIsFiniteUnit }

    var attitudeData: AttitudeData {
        let radiansToDegrees = Float(180.0 / Double.pi)
        return AttitudeData(
            rollRad: rollRad,
            pitchRad: pitchRad,
            yawRad: yawRad,
            rollDeg: rollRad * radiansToDegrees,
            pitchDeg: pitchRad * radiansToDegrees,
            yawDeg: yawRad * radiansToDegrees,
            valid: valid,
            source: .bmi323Fusion
        )
    }
}

struct DualAttitude: Equatable {
    let timestampMs: UInt32
    let sequence: UInt32
    let flags: UInt8
    let primary: DualAttitudePose
    let redundant: DualAttitudePose
    let deltaRad: Vector3

    var primaryValid: Bool { (flags & 0x01) != 0 && primary.quaternionIsFiniteUnit }
    var redundantValid: Bool { (flags & 0x02) != 0 && redundant.quaternionIsFiniteUnit }
    var stale: Bool { (flags & 0x40) != 0 }
}

enum AttitudeAxis {
    case roll
    case pitch
    case yaw
}

enum AngleUnit: String, CaseIterable, Identifiable {
    case degree
    case radian

    var id: Self { self }

    var titleKey: String {
        switch self {
        case .degree: return "angle_unit.degree"
        case .radian: return "angle_unit.radian"
        }
    }

    var symbol: String {
        switch self {
        case .degree: return "°"
        case .radian: return "rad"
        }
    }

    func format(_ value: Float, precision: Int) -> String {
        String(format: "%.*f%@", precision, value, symbol)
    }
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

struct LSM303Data: Equatable {
    var online = false
    var accelValid = false
    var magValid = false
    var accel = Vector3()
    var mag = Vector3()
    var timestamp: UInt32 = 0
}

struct BMI323Data: Equatable {
    var online = false
    var accelValid = false
    var gyroValid = false
    var accel = Vector3()
    var gyro = Vector3()
    var timestamp: UInt32 = 0
}

struct IMUDataModel: Equatable {
    var lsm303 = LSM303Data()
    var bmi323 = BMI323Data()
}

enum IMUSensorID: UInt8 {
    case bmi323 = 0x01
    case lsm303 = 0x02
}

/// Explicit source values used by the new dual-IMU payloads. The legacy
/// IMU_STATUS values above remain unchanged for compatibility.
enum IMUSource: UInt8 {
    case lsm303 = 0x01
    case bmi323 = 0x02
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

enum StaticCalibrationPhase: Equatable {
    case waiting
    case sampling
    case completed
    case error
}

/// App-owned view data for the static calibration result.
struct StaticCalibrationResult: Equatable {
    var phase: StaticCalibrationPhase = .waiting
    var sampleCount: UInt32 = 0
    var sampleTotal: UInt32 = 0
    var accelOffsetX: Float?
    var accelOffsetY: Float?
    var accelOffsetZ: Float?
    var errorCode: UInt8?
    var lsmAccelBias: Vector3?
    var bmiAccelBias: Vector3?
    var bmiGyroBias: Vector3?
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
}

enum IMUCalibrationStage: UInt8, Equatable {
    case waitRadarReady = 0
    case staticStableWait = 1
    case staticSample = 2
    case complete = 3
    case error = 4

    var displayName: String {
        switch self {
        case .waitRadarReady: return "WAIT_RADAR_READY"
        case .staticStableWait: return "STATIC_STABLE_WAIT"
        case .staticSample: return "STATIC_SAMPLE"
        case .complete: return "COMPLETE"
        case .error: return "ERROR"
        }
    }
}

enum CalibrationEventID: UInt8, Equatable {
    case staticCalibrationComplete = 0x01

    var displayName: String {
        switch self {
        case .staticCalibrationComplete: return "STATIC_CAL_DONE"
        }
    }
}

struct IMUCalibrationStatus: Equatable {
    let state: IMUCalibrationState
    let sampleMode: IMUCalibrationSampleMode
    let totalProgress: UInt8
    var currentPWM: UInt8
    let sampleProgress: UInt16
    /// Canonical sample counters, when sent by the extended status payload.
    let sampleCount: UInt32
    let totalSample: UInt32
    let stageCode: UInt8
    let errorCode: UInt8
    var lastUpdatedAt: Date?

    var stage: IMUCalibrationStage {
        IMUCalibrationStage(rawValue: stageCode) ?? .error
    }

    func isWaitingForSTM(at date: Date = Date()) -> Bool {
        guard let lastUpdatedAt else { return true }
        return date.timeIntervalSince(lastUpdatedAt) > 3.0
    }

    init(state: IMUCalibrationState,
         sampleMode: IMUCalibrationSampleMode,
         totalProgress: UInt8,
         currentPWM: UInt8,
         sampleProgress: UInt16,
         sampleCount: UInt32? = nil,
         totalSample: UInt32? = nil,
         stageCode: UInt8? = nil,
         errorCode: UInt8,
         lastUpdatedAt: Date?) {
        self.state = state
        self.sampleMode = sampleMode
        self.totalProgress = totalProgress
        self.currentPWM = currentPWM
        self.sampleProgress = sampleProgress
        self.sampleCount = sampleCount ?? UInt32(sampleProgress)
        // The legacy 7-byte frame carried percentage progress rather than a
        // total sample counter. Keep that value in totalProgress and expose a
        // zero total so consumers do not mistake percent for a sample count.
        self.totalSample = totalSample ?? 0
        if let stageCode {
            self.stageCode = stageCode
        } else {
            switch state {
            case .idle, .setPWM: self.stageCode = 0
            case .waitStable: self.stageCode = 1
            case .sample: self.stageCode = 2
            case .complete: self.stageCode = 3
            case .error: self.stageCode = 4
            }
        }
        self.errorCode = errorCode
        self.lastUpdatedAt = lastUpdatedAt
    }
}

struct IMUCalibrationBias: Equatable {
    let x: Float
    let y: Float
    let z: Float
}

struct IMUCalibrationResult: Equatable {
    let sensorID: IMUSource
    let accelBias: Vector3?
    let gyroBias: Vector3?
}

enum IMUTelemetry: Equatable {
    case lsm303(LSM303Data)
    case bmi323(BMI323Data)
}

enum DualIMUPhase: UInt8, Equatable {
    case idle = 0
    case initialize = 1
    case selfTest = 2
    case staticCalibration = 3
    case reserved4 = 4
    case reserved5 = 5
    case ready = 6
    case failed = 7

    var displayName: String {
        switch self {
        case .idle: return "IDLE"
        case .initialize: return "INIT"
        case .selfTest: return "SELF_TEST"
        case .staticCalibration: return "STATIC_CALIBRATION"
        case .reserved4, .reserved5: return "RESERVED"
        case .ready: return "READY"
        case .failed: return "FAILED"
        }
    }
}

struct DualIMULifecycleStatus: Equatable {
    let phase: DualIMUPhase
    let lsmProgress: UInt8
    let bmiProgress: UInt8
    let overallProgress: UInt8
    let errorCode: UInt8
    let flags: UInt8
    let phaseStartTimeMs: UInt32
    let phaseEndTimeMs: UInt32

    var lsmPhaseComplete: Bool { (flags & 0x01) != 0 }
    var bmiPhaseComplete: Bool { (flags & 0x02) != 0 }
    var phaseActive: Bool { (flags & 0x04) != 0 }
}

enum DecodedMessage: Equatable {
    case control(command: SmartCarProtocol.ControlCommand, speed: UInt8?)
    case status(SmartCarStatus)
    case ping
    case ack(Data)
    case imuStatus(IMUStatus)
    case attitude(AttitudeData)
    case dualAttitude(DualAttitude)
    case imuCalibrationStatus(IMUCalibrationStatus)
    case imuCalibrationBias(IMUCalibrationBias)
    case imuCalibrationResult(IMUCalibrationResult)
    case calibrationEvent(CalibrationEventID)
    case imuTelemetry(IMUTelemetry)
    case dualIMUStatus(DualIMULifecycleStatus)
    case radarStatus(RadarStatus)

    private static func mapExtendedStage(_ raw: UInt8) -> (IMUCalibrationState, IMUCalibrationSampleMode)? {
        switch raw {
        // App stage values relayed by S3 (SC_CAL_STAGE_*).
        case 0: return (.idle, .static) // WAIT_RADAR_READY
        case 1: return (.waitStable, .static) // STATIC_STABLE_WAIT
        case 2: return (.sample, .static) // STATIC_SAMPLE
        case 3: return (.complete, .static)
        case 4: return (.error, .static)
        default: return nil
        }
    }

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
            switch bytes.count {
            case 80:
                guard bytes[0] == 2,
                      bytes[2] == 0, bytes[3] == 0 else {
                    throw DecodeError.invalidPayload("DUAL_ATTITUDE schema/reserved")
                }
                let primary = DualAttitudePose(
                    rollRad: float32(bytes, at: 12),
                    pitchRad: float32(bytes, at: 16),
                    yawRad: float32(bytes, at: 20),
                    quaternion: Quaternion(
                        q0: float32(bytes, at: 24),
                        q1: float32(bytes, at: 28),
                        q2: float32(bytes, at: 32),
                        q3: float32(bytes, at: 36)),
                    valid: (bytes[1] & 0x01) != 0)
                let redundant = DualAttitudePose(
                    rollRad: float32(bytes, at: 40),
                    pitchRad: float32(bytes, at: 44),
                    yawRad: float32(bytes, at: 48),
                    quaternion: Quaternion(
                        q0: float32(bytes, at: 52),
                        q1: float32(bytes, at: 56),
                        q2: float32(bytes, at: 60),
                        q3: float32(bytes, at: 64)),
                    valid: (bytes[1] & 0x02) != 0)
                let dual = DualAttitude(
                    timestampMs: uint32(bytes, at: 4),
                    sequence: uint32(bytes, at: 8),
                    flags: bytes[1],
                    primary: primary,
                    redundant: redundant,
                    deltaRad: Vector3(x: float32(bytes, at: 68),
                                      y: float32(bytes, at: 72),
                                      z: float32(bytes, at: 76)))
                guard primary.quaternionIsFiniteUnit,
                      redundant.quaternionIsFiniteUnit,
                      dual.deltaRad.x.isFinite, dual.deltaRad.y.isFinite,
                      dual.deltaRad.z.isFinite else {
                    throw DecodeError.invalidPayload("DUAL_ATTITUDE non-finite quaternion/vector")
                }
                self = .dualAttitude(dual)
            case 30:
                // Active SCBP ATTITUDE layout: timestamp_ms, source, valid.
                guard bytes[29] <= 1,
                      let source = AttitudeSource(rawValue: bytes[28]) else {
                    throw DecodeError.invalidPayload("ATTITUDE requires timestamp/source/valid fields")
                }
                self = .attitude(AttitudeData(
                    rollRad: float32(bytes, at: 0),
                    pitchRad: float32(bytes, at: 4),
                    yawRad: float32(bytes, at: 8),
                    rollDeg: float32(bytes, at: 12),
                    pitchDeg: float32(bytes, at: 16),
                    yawDeg: float32(bytes, at: 20),
                    timestampMs: uint32(bytes, at: 24),
                    valid: bytes[29] == 1,
                    source: source
                ))
            case 26:
                guard bytes[24] <= 1,
                      let source = AttitudeSource(rawValue: bytes[25]) else {
                    throw DecodeError.invalidPayload("ATTITUDE requires valid/source fields")
                }
                self = .attitude(AttitudeData(
                    rollRad: float32(bytes, at: 0),
                    pitchRad: float32(bytes, at: 4),
                    yawRad: float32(bytes, at: 8),
                    rollDeg: float32(bytes, at: 12),
                    pitchDeg: float32(bytes, at: 16),
                    yawDeg: float32(bytes, at: 20),
                    timestampMs: 0,
                    valid: bytes[24] == 1,
                    source: source
                ))
            case 14:
                // Legacy App ATTITUDE: roll/pitch/yaw radians, valid, source.
                guard bytes[12] <= 1,
                      let source = AttitudeSource(rawValue: bytes[13]) else {
                    throw DecodeError.invalidPayload("ATTITUDE legacy valid/source fields")
                }
                let rollRad = float32(bytes, at: 0)
                let pitchRad = float32(bytes, at: 4)
                let yawRad = float32(bytes, at: 8)
                let radiansToDegrees = Float(180.0 / Double.pi)
                self = .attitude(AttitudeData(
                    rollRad: rollRad,
                    pitchRad: pitchRad,
                    yawRad: yawRad,
                    rollDeg: rollRad * radiansToDegrees,
                    pitchDeg: pitchRad * radiansToDegrees,
                    yawDeg: yawRad * radiansToDegrees,
                    timestampMs: 0,
                    valid: bytes[12] == 1,
                    source: source
                ))
            default:
                throw DecodeError.invalidPayload("ATTITUDE requires 80, 30, 26 or 14 bytes")
            }
        case .imuCalibrationStatus:
            switch bytes.count {
            case 7:
                // Protocol v1 legacy layout:
                // state, sample mode, PWM, sample uint16, progress %, error.
                guard let state = IMUCalibrationState(rawValue: bytes[0]),
                      let sampleMode = IMUCalibrationSampleMode(rawValue: bytes[1]),
                      bytes[2] <= 100,
                      bytes[5] <= 100 else {
                    throw DecodeError.invalidPayload("IMU_CAL_STATUS legacy payload")
                }
                let sampleProgress = uint16(bytes, at: 3)
                self = .imuCalibrationStatus(IMUCalibrationStatus(
                    state: state,
                    sampleMode: sampleMode,
                    totalProgress: bytes[5],
                    currentPWM: bytes[2],
                    sampleProgress: sampleProgress,
                    errorCode: bytes[6],
                    lastUpdatedAt: nil
                ))
            case 8:
                // Extended layout:
                // stage, PWM, sample_count uint16, total_sample uint16,
                // progress %, error.
                guard let (state, sampleMode) = Self.mapExtendedStage(bytes[0]),
                      bytes[1] <= 100,
                      bytes[6] <= 100 else {
                    throw DecodeError.invalidPayload("IMU_CAL_STATUS extended payload")
                }
                let sampleCount = uint16(bytes, at: 2)
                let totalSample = uint16(bytes, at: 4)
                let progress = totalSample == 0
                    ? 0
                    : UInt8(min(100, (UInt32(sampleCount) * 100) / UInt32(totalSample)))
                self = .imuCalibrationStatus(IMUCalibrationStatus(
                    state: state,
                    sampleMode: sampleMode,
                    totalProgress: progress,
                    currentPWM: bytes[1],
                    sampleProgress: sampleCount,
                    sampleCount: UInt32(sampleCount),
                    totalSample: UInt32(totalSample),
                    stageCode: bytes[0],
                    errorCode: bytes[7],
                    lastUpdatedAt: nil
                ))
            case 9:
                // Extended layout with explicit sample mode and progress:
                // stage, mode, PWM, sample_count uint16, total_sample uint16,
                // progress %, error.
                guard let (state, mappedMode) = Self.mapExtendedStage(bytes[0]),
                      let sampleMode = IMUCalibrationSampleMode(rawValue: bytes[1]),
                      sampleMode == mappedMode || bytes[0] <= 5,
                      bytes[2] <= 100,
                      bytes[7] <= 100 else {
                    throw DecodeError.invalidPayload("IMU_CAL_STATUS extended mode payload")
                }
                let sampleCount = uint16(bytes, at: 3)
                let totalSample = uint16(bytes, at: 5)
                self = .imuCalibrationStatus(IMUCalibrationStatus(
                    state: state,
                    sampleMode: sampleMode,
                    totalProgress: bytes[7],
                    currentPWM: bytes[2],
                    sampleProgress: sampleCount,
                    sampleCount: UInt32(sampleCount),
                    totalSample: UInt32(totalSample),
                    stageCode: bytes[0],
                    errorCode: bytes[8],
                    lastUpdatedAt: nil
                ))
            case 11:
                // Current S3-relayed layout:
                // stage, PWM, sample_count uint32, total_sample uint32, error.
                guard let (state, sampleMode) = Self.mapExtendedStage(bytes[0]),
                      bytes[1] <= 100 else {
                    throw DecodeError.invalidPayload("IMU_CAL_STATUS current payload")
                }
                let sampleCount = uint32(bytes, at: 2)
                let totalSample = uint32(bytes, at: 6)
                let progress = totalSample == 0
                    ? 0
                    : UInt8(min(100, (sampleCount * 100) / totalSample))
                self = .imuCalibrationStatus(IMUCalibrationStatus(
                    state: state,
                    sampleMode: sampleMode,
                    totalProgress: progress,
                    currentPWM: bytes[1],
                    sampleProgress: UInt16(min(UInt32(UInt16.max), sampleCount)),
                    sampleCount: sampleCount,
                    totalSample: totalSample,
                    stageCode: bytes[0],
                    errorCode: bytes[10],
                    lastUpdatedAt: nil
                ))
            default:
                throw DecodeError.invalidPayload("IMU_CAL_STATUS payload length")
            }
        case .imuCalibrationBias:
            guard bytes.count == 12 else {
                throw DecodeError.invalidPayload("IMU_CAL_BIAS requires three float values")
            }
            self = .imuCalibrationBias(IMUCalibrationBias(
                x: float32(bytes, at: 0),
                y: float32(bytes, at: 4),
                z: float32(bytes, at: 8)
            ))
        case .imuCalibrationResult:
            guard (bytes.count == 14 || bytes.count == 26),
                  let sensor = IMUSource(rawValue: bytes[0]) else {
                throw DecodeError.invalidPayload("IMU_CAL_RESULT source/length")
            }
            let flags = bytes[1]
            if sensor == .lsm303 {
                guard bytes.count == 14, (flags & 0x01) != 0 else {
                    throw DecodeError.invalidPayload("LSM303 calibration result")
                }
                self = .imuCalibrationResult(IMUCalibrationResult(
                    sensorID: sensor,
                    accelBias: Vector3(x: float32(bytes, at: 2), y: float32(bytes, at: 6), z: float32(bytes, at: 10)),
                    gyroBias: nil))
            } else {
                guard bytes.count == 26, (flags & 0x03) == 0x03 else {
                    throw DecodeError.invalidPayload("BMI323 calibration result")
                }
                self = .imuCalibrationResult(IMUCalibrationResult(
                    sensorID: sensor,
                    accelBias: Vector3(x: float32(bytes, at: 2), y: float32(bytes, at: 6), z: float32(bytes, at: 10)),
                    gyroBias: Vector3(x: float32(bytes, at: 14), y: float32(bytes, at: 18), z: float32(bytes, at: 22))))
            }
        case .imuTelemetry:
            guard bytes.count == 30,
                  let sensor = IMUSource(rawValue: bytes[0]) else {
                throw DecodeError.invalidPayload("IMU_TELEMETRY source/length")
            }
            let flags = bytes[1]
            let timestamp = uint32(bytes, at: 2)
            if sensor == .lsm303 {
                let accelValid = (flags & 0x01) != 0
                let magValid = (flags & 0x02) != 0
                let online = (flags & 0x04) != 0 || (accelValid && magValid)
                self = .imuTelemetry(.lsm303(LSM303Data(
                    online: online,
                    accelValid: accelValid,
                    magValid: magValid,
                    accel: Vector3(x: float32(bytes, at: 6), y: float32(bytes, at: 10), z: float32(bytes, at: 14)),
                    mag: Vector3(x: float32(bytes, at: 18), y: float32(bytes, at: 22), z: float32(bytes, at: 26)),
                    timestamp: timestamp)))
            } else {
                let accelValid = (flags & 0x01) != 0
                let gyroValid = (flags & 0x02) != 0
                let online = (flags & 0x04) != 0 || (accelValid && gyroValid)
                self = .imuTelemetry(.bmi323(BMI323Data(
                    online: online,
                    accelValid: accelValid,
                    gyroValid: gyroValid,
                    accel: Vector3(x: float32(bytes, at: 6), y: float32(bytes, at: 10), z: float32(bytes, at: 14)),
                    gyro: Vector3(x: float32(bytes, at: 18), y: float32(bytes, at: 22), z: float32(bytes, at: 26)),
                    timestamp: timestamp)))
            }
        case .dualIMUStatus:
            guard bytes.count == 16,
                  let phase = DualIMUPhase(rawValue: bytes[0]),
                  bytes[1] <= 100,
                  bytes[2] <= 100,
                  bytes[3] <= 100 else {
                throw DecodeError.invalidPayload("DUAL_IMU_STATUS payload")
            }
            self = .dualIMUStatus(DualIMULifecycleStatus(
                phase: phase,
                lsmProgress: bytes[1],
                bmiProgress: bytes[2],
                overallProgress: bytes[3],
                errorCode: bytes[4],
                flags: bytes[5],
                phaseStartTimeMs: uint32(bytes, at: 8),
                phaseEndTimeMs: uint32(bytes, at: 12)
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
        case .attitude(let attitude): return "ATTITUDE roll_rad=\(attitude.rollRad.displayValue) pitch_rad=\(attitude.pitchRad.displayValue) yaw_rad=\(attitude.yawRad.displayValue) roll_deg=\(attitude.rollDeg.displayValue) pitch_deg=\(attitude.pitchDeg.displayValue) yaw_deg=\(attitude.yawDeg.displayValue) valid=\(attitude.valid ? 1 : 0) source=\(attitude.source.displayName)"
        case .dualAttitude(let dual): return "DUAL_ATTITUDE seq=\(dual.sequence) primary=\(dual.primary.rollRad.displayValue),\(dual.primary.pitchRad.displayValue),\(dual.primary.yawRad.displayValue) redundant=\(dual.redundant.rollRad.displayValue),\(dual.redundant.pitchRad.displayValue),\(dual.redundant.yawRad.displayValue) delta=\(dual.deltaRad.x.displayValue),\(dual.deltaRad.y.displayValue),\(dual.deltaRad.z.displayValue)"
        case .imuCalibrationStatus(let status): return "IMU_CAL_STATUS stage=\(status.stageCode) mode=\(status.sampleMode.rawValue) progress=\(status.totalProgress)% pwm=\(status.currentPWM) sample=\(status.sampleCount)/\(status.totalSample) error=\(status.errorCode)"
        case .imuCalibrationBias(let bias): return "IMU_CAL_BIAS x=\(bias.x.displayValue) y=\(bias.y.displayValue) z=\(bias.z.displayValue)"
        case .imuCalibrationResult(let result): return "IMU_CAL_RESULT sensor=\(result.sensorID)"
        case .calibrationEvent(let event): return "CAL_EVENT id=\(event.rawValue) \(event.displayName)"
        case .imuTelemetry(let telemetry):
            switch telemetry {
            case .lsm303(let value): return "IMU_TELEMETRY LSM303 online=\(value.online ? 1 : 0)"
            case .bmi323(let value): return "IMU_TELEMETRY BMI323 online=\(value.online ? 1 : 0)"
            }
        case .dualIMUStatus(let status): return "DUAL_IMU_STATUS phase=\(status.phase.displayName) lsm=\(status.lsmProgress)% bmi=\(status.bmiProgress)% overall=\(status.overallProgress)% error=\(status.errorCode)"
        case .radarStatus(let status): return "RADAR_STATUS \(status.online ? "ONLINE" : "OFFLINE") speed=\(status.speedPercent)%"
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
    var dualAttitude: DualAttitude?
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
        case .dualAttitude(let dual): self.dualAttitude = dual; self.attitude = dual.primary.attitudeData; lastAttitudeAt = date
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
private func uint32(_ bytes: [UInt8], at index: Int) -> UInt32 {
    UInt32(bytes[index]) |
        (UInt32(bytes[index + 1]) << 8) |
        (UInt32(bytes[index + 2]) << 16) |
        (UInt32(bytes[index + 3]) << 24)
}
private func float32(_ bytes: [UInt8], at index: Int) -> Float {
    let bits = UInt32(bytes[index]) | (UInt32(bytes[index + 1]) << 8) | (UInt32(bytes[index + 2]) << 16) | (UInt32(bytes[index + 3]) << 24)
    return Float(bitPattern: bits)
}
private func hex(_ data: Data) -> String { data.map { String(format: "%02X", $0) }.joined(separator: " ") }
