import Foundation

public struct Vector3: Equatable {
    public var x: Float = 0
    public var y: Float = 0
    public var z: Float = 0

    public init(x: Float = 0, y: Float = 0, z: Float = 0) {
        self.x = x
        self.y = y
        self.z = z
    }
}

public struct Quaternion: Equatable {
    public var q0: Float = 1
    public var q1: Float = 0
    public var q2: Float = 0
    public var q3: Float = 0

    public init(q0: Float = 1, q1: Float = 0, q2: Float = 0, q3: Float = 0) {
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
    }
}

public struct AttitudeData: Equatable {
    public var rollRad: Float = 0
    public var pitchRad: Float = 0
    public var yawRad: Float = 0
    public var rollDeg: Float = 0
    public var pitchDeg: Float = 0
    public var yawDeg: Float = 0
    public var timestampMs: UInt32 = 0
    public var valid = false
    public var source: AttitudeSource = .none

    public init(rollRad: Float = 0, pitchRad: Float = 0, yawRad: Float = 0,
                rollDeg: Float = 0, pitchDeg: Float = 0, yawDeg: Float = 0,
                timestampMs: UInt32 = 0, valid: Bool = false,
                source: AttitudeSource = .none) {
        self.rollRad = rollRad
        self.pitchRad = pitchRad
        self.yawRad = yawRad
        self.rollDeg = rollDeg
        self.pitchDeg = pitchDeg
        self.yawDeg = yawDeg
        self.timestampMs = timestampMs
        self.valid = valid
        self.source = source
    }

    public func value(for axis: AttitudeAxis, unit: AngleUnit) -> Float {
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

public struct DualAttitudePose: Equatable {
    public var rollRad: Float
    public var pitchRad: Float
    public var yawRad: Float
    public var quaternion: Quaternion
    public var valid: Bool

    public init(rollRad: Float, pitchRad: Float, yawRad: Float,
                quaternion: Quaternion, valid: Bool) {
        self.rollRad = rollRad
        self.pitchRad = pitchRad
        self.yawRad = yawRad
        self.quaternion = quaternion
        self.valid = valid
    }

    public var quaternionIsFiniteUnit: Bool {
        let values = [rollRad, pitchRad, yawRad, quaternion.q0, quaternion.q1,
                      quaternion.q2, quaternion.q3]
        let norm = sqrtf(quaternion.q0 * quaternion.q0 +
                         quaternion.q1 * quaternion.q1 +
                         quaternion.q2 * quaternion.q2 +
                         quaternion.q3 * quaternion.q3)
        return values.allSatisfy(\.isFinite) &&
            abs(norm - 1.0) <= 0.2
    }

    public var isFiniteUnit: Bool { valid && quaternionIsFiniteUnit }

    public var attitudeData: AttitudeData {
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

public struct DualAttitude: Equatable {
    public let timestampMs: UInt32
    public let sequence: UInt32
    public let flags: UInt8
    public let primary: DualAttitudePose
    public let redundant: DualAttitudePose
    public let deltaRad: Vector3

    public init(timestampMs: UInt32, sequence: UInt32, flags: UInt8,
                primary: DualAttitudePose, redundant: DualAttitudePose,
                deltaRad: Vector3) {
        self.timestampMs = timestampMs
        self.sequence = sequence
        self.flags = flags
        self.primary = primary
        self.redundant = redundant
        self.deltaRad = deltaRad
    }

    public var primaryValid: Bool { (flags & 0x01) != 0 && primary.quaternionIsFiniteUnit }
    public var redundantValid: Bool { (flags & 0x02) != 0 && redundant.quaternionIsFiniteUnit }
    public var stale: Bool { (flags & 0x40) != 0 }
}

public enum AttitudeAxis {
    case roll
    case pitch
    case yaw
}

public enum AngleUnit: String, CaseIterable, Identifiable {
    case degree
    case radian

    public var id: Self { self }

    public var titleKey: String {
        switch self {
        case .degree: return "angle_unit.degree"
        case .radian: return "angle_unit.radian"
        }
    }

    public var symbol: String {
        switch self {
        case .degree: return "°"
        case .radian: return "rad"
        }
    }

    public func format(_ value: Float, precision: Int) -> String {
        String(format: "%.*f%@", precision, value, symbol)
    }
}

public enum AttitudeSource: UInt8, Equatable {
    case none = 0x00
    case lsm303 = 0x01
    case bmi323Fusion = 0x02

    public var displayName: String {
        switch self {
        case .none: return "NONE"
        case .lsm303: return "LSM303"
        case .bmi323Fusion: return "BMI323_FUSION"
        }
    }
}

public struct IMUData: Equatable {
    public var online = false
    public var accel = Vector3()
    public var gyro = Vector3()
    public var mag = Vector3()

    public init(online: Bool = false, accel: Vector3 = Vector3(),
                gyro: Vector3 = Vector3(), mag: Vector3 = Vector3()) {
        self.online = online
        self.accel = accel
        self.gyro = gyro
        self.mag = mag
    }
}

public struct LSM303Data: Equatable {
    public var online = false
    public var accelValid = false
    public var magValid = false
    public var accel = Vector3()
    public var mag = Vector3()
    public var timestamp: UInt32 = 0

    public init(online: Bool = false, accelValid: Bool = false,
                magValid: Bool = false, accel: Vector3 = Vector3(),
                mag: Vector3 = Vector3(), timestamp: UInt32 = 0) {
        self.online = online
        self.accelValid = accelValid
        self.magValid = magValid
        self.accel = accel
        self.mag = mag
        self.timestamp = timestamp
    }
}

public struct BMI323Data: Equatable {
    public var online = false
    public var accelValid = false
    public var gyroValid = false
    public var accel = Vector3()
    public var gyro = Vector3()
    public var timestamp: UInt32 = 0

    public init(online: Bool = false, accelValid: Bool = false,
                gyroValid: Bool = false, accel: Vector3 = Vector3(),
                gyro: Vector3 = Vector3(), timestamp: UInt32 = 0) {
        self.online = online
        self.accelValid = accelValid
        self.gyroValid = gyroValid
        self.accel = accel
        self.gyro = gyro
        self.timestamp = timestamp
    }
}

public struct IMUDataModel: Equatable {
    public var lsm303 = LSM303Data()
    public var bmi323 = BMI323Data()

    public init(lsm303: LSM303Data = LSM303Data(), bmi323: BMI323Data = BMI323Data()) {
        self.lsm303 = lsm303
        self.bmi323 = bmi323
    }
}

public enum IMUSensorID: UInt8 {
    case bmi323 = 0x01
    case lsm303 = 0x02
}

/// Explicit source values used by the new dual-IMU payloads. The legacy
/// IMU_STATUS values above remain unchanged for compatibility.
public enum IMUSource: UInt8 {
    case lsm303 = 0x01
    case bmi323 = 0x02
}

public struct SmartCarStatus: Equatable {
    public let battery: UInt8
    public let motorState: UInt8
    public let errorCode: UInt16

    public init(battery: UInt8, motorState: UInt8, errorCode: UInt16) {
        self.battery = battery
        self.motorState = motorState
        self.errorCode = errorCode
    }
}

public struct RadarStatus: Equatable {
    public let online: Bool
    public let speedPercent: UInt8

    public init(online: Bool, speedPercent: UInt8) {
        self.online = online
        self.speedPercent = speedPercent
    }
}

public struct WheelSpeedStatus: Equatable {
    public let speeds: [Float]

    public init(speeds: [Float]) { self.speeds = speeds }
}

public enum ChassisControlMode: UInt8, Equatable, Hashable, Sendable {
    case chassisDiff = 0
    case wheelIndependent = 1
}

public struct WheelControlStatus: Equatable {
    public let mode: ChassisControlMode
    public let timestampMs: UInt32
    public let masterScale: Float
    public let rawTargets: [Float]
    public let actualSpeeds: [Float]

    public init(mode: ChassisControlMode, timestampMs: UInt32, masterScale: Float,
                rawTargets: [Float], actualSpeeds: [Float]) {
        self.mode = mode
        self.timestampMs = timestampMs
        self.masterScale = masterScale
        self.rawTargets = rawTargets
        self.actualSpeeds = actualSpeeds
    }
}

public struct PowerStatus: Equatable {
    public let voltage: Float

    public init(voltage: Float) { self.voltage = voltage }
}

public struct ChassisStateTelemetry: Equatable {
    public let safetyFused: Bool
    public let headingLocked: Bool
    public let odometryValid: Bool
    public let attitudeReady: Bool
    public let timestampMs: UInt32
    public let xMm: Float
    public let yMm: Float
    public let yawDeg: Float
    public let totalDistanceM: Float

    public init(safetyFused: Bool, headingLocked: Bool, odometryValid: Bool,
                attitudeReady: Bool, timestampMs: UInt32, xMm: Float, yMm: Float,
                yawDeg: Float, totalDistanceM: Float) {
        self.safetyFused = safetyFused
        self.headingLocked = headingLocked
        self.odometryValid = odometryValid
        self.attitudeReady = attitudeReady
        self.timestampMs = timestampMs
        self.xMm = xMm
        self.yMm = yMm
        self.yawDeg = yawDeg
        self.totalDistanceM = totalDistanceM
    }
}

public enum StaticCalibrationPhase: Equatable {
    case waiting
    case sampling
    case completed
    case error
}

/// App-owned view data for the static calibration result.
public struct StaticCalibrationResult: Equatable {
    public var phase: StaticCalibrationPhase = .waiting
    public var sampleCount: UInt32 = 0
    public var sampleTotal: UInt32 = 0
    public var accelOffsetX: Float?
    public var accelOffsetY: Float?
    public var accelOffsetZ: Float?
    public var errorCode: UInt8?
    public var lsmAccelBias: Vector3?
    public var bmiAccelBias: Vector3?
    public var bmiGyroBias: Vector3?

    public init(phase: StaticCalibrationPhase = .waiting, sampleCount: UInt32 = 0,
                sampleTotal: UInt32 = 0, accelOffsetX: Float? = nil,
                accelOffsetY: Float? = nil, accelOffsetZ: Float? = nil,
                errorCode: UInt8? = nil, lsmAccelBias: Vector3? = nil,
                bmiAccelBias: Vector3? = nil, bmiGyroBias: Vector3? = nil) {
        self.phase = phase
        self.sampleCount = sampleCount
        self.sampleTotal = sampleTotal
        self.accelOffsetX = accelOffsetX
        self.accelOffsetY = accelOffsetY
        self.accelOffsetZ = accelOffsetZ
        self.errorCode = errorCode
        self.lsmAccelBias = lsmAccelBias
        self.bmiAccelBias = bmiAccelBias
        self.bmiGyroBias = bmiGyroBias
    }
}

public struct IMUStatus: Equatable {
    public let sensorID: IMUSensorID
    public let data: IMUData
    public let calibrationState: IMUCalibrationSummaryState
    public let calibrationSample: UInt16
    public let calibrationTotal: UInt16

    public init(sensorID: IMUSensorID, data: IMUData,
                calibrationState: IMUCalibrationSummaryState,
                calibrationSample: UInt16, calibrationTotal: UInt16) {
        self.sensorID = sensorID
        self.data = data
        self.calibrationState = calibrationState
        self.calibrationSample = calibrationSample
        self.calibrationTotal = calibrationTotal
    }
}

public enum IMUCalibrationSummaryState: UInt8, Equatable {
    case idle = 0
    case waitStable = 1
    case collecting = 2
    case complete = 3
    case error = 4
}

public enum IMUCalibrationState: UInt8, Equatable {
    case idle = 0
    case setPWM = 1
    case waitStable = 2
    case sample = 3
    case complete = 4
    case error = 5
}

public enum IMUCalibrationSampleMode: UInt8, Equatable {
    case `static` = 0
}

public enum IMUCalibrationStage: UInt8, Equatable {
    case waitRadarReady = 0
    case staticStableWait = 1
    case staticSample = 2
    case complete = 3
    case error = 4

    public var displayName: String {
        switch self {
        case .waitRadarReady: return "WAIT_RADAR_READY"
        case .staticStableWait: return "STATIC_STABLE_WAIT"
        case .staticSample: return "STATIC_SAMPLE"
        case .complete: return "COMPLETE"
        case .error: return "ERROR"
        }
    }
}

public enum CalibrationEventID: UInt8, Equatable {
    case staticCalibrationComplete = 0x01

    public var displayName: String {
        switch self {
        case .staticCalibrationComplete: return "STATIC_CAL_DONE"
        }
    }
}

public struct IMUCalibrationStatus: Equatable {
    public let state: IMUCalibrationState
    public let sampleMode: IMUCalibrationSampleMode
    public let totalProgress: UInt8
    public var currentPWM: UInt8
    public let sampleProgress: UInt16
    /// Canonical sample counters, when sent by the extended status payload.
    public let sampleCount: UInt32
    public let totalSample: UInt32
    public let stageCode: UInt8
    public let errorCode: UInt8
    public var lastUpdatedAt: Date?

    public var stage: IMUCalibrationStage {
        IMUCalibrationStage(rawValue: stageCode) ?? .error
    }

    public func isWaitingForSTM(at date: Date = Date()) -> Bool {
        guard let lastUpdatedAt else { return true }
        return date.timeIntervalSince(lastUpdatedAt) > 3.0
    }

    public init(state: IMUCalibrationState,
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

public struct IMUCalibrationBias: Equatable {
    public let x: Float
    public let y: Float
    public let z: Float

    public init(x: Float, y: Float, z: Float) {
        self.x = x
        self.y = y
        self.z = z
    }
}

public struct IMUCalibrationResult: Equatable {
    public let sensorID: IMUSource
    public let accelBias: Vector3?
    public let gyroBias: Vector3?

    public init(sensorID: IMUSource, accelBias: Vector3?, gyroBias: Vector3?) {
        self.sensorID = sensorID
        self.accelBias = accelBias
        self.gyroBias = gyroBias
    }
}

public enum IMUTelemetry: Equatable {
    case lsm303(LSM303Data)
    case bmi323(BMI323Data)
}

public enum DualIMUPhase: UInt8, Equatable {
    case idle = 0
    case initialize = 1
    case selfTest = 2
    case staticCalibration = 3
    case reserved4 = 4
    case reserved5 = 5
    case ready = 6
    case failed = 7

    public var displayName: String {
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

public struct DualIMULifecycleStatus: Equatable {
    public let phase: DualIMUPhase
    public let lsmProgress: UInt8
    public let bmiProgress: UInt8
    public let overallProgress: UInt8
    public let errorCode: UInt8
    public let flags: UInt8
    public let phaseStartTimeMs: UInt32
    public let phaseEndTimeMs: UInt32

    public init(phase: DualIMUPhase, lsmProgress: UInt8, bmiProgress: UInt8,
                overallProgress: UInt8, errorCode: UInt8, flags: UInt8,
                phaseStartTimeMs: UInt32, phaseEndTimeMs: UInt32) {
        self.phase = phase
        self.lsmProgress = lsmProgress
        self.bmiProgress = bmiProgress
        self.overallProgress = overallProgress
        self.errorCode = errorCode
        self.flags = flags
        self.phaseStartTimeMs = phaseStartTimeMs
        self.phaseEndTimeMs = phaseEndTimeMs
    }

    public var lsmPhaseComplete: Bool { (flags & 0x01) != 0 }
    public var bmiPhaseComplete: Bool { (flags & 0x02) != 0 }
    public var phaseActive: Bool { (flags & 0x04) != 0 }
}

public enum DecodedMessage: Equatable, @unchecked Sendable {
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
    case wheelSpeedStatus(WheelSpeedStatus)
    case wheelControlStatus(WheelControlStatus)
    case powerStatus(PowerStatus)
    case chassisState(ChassisStateTelemetry)

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

    public init(frame: SmartCarProtocol.Frame) throws {
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
        case .wheelSpeedCommand:
            throw DecodeError.invalidPayload("WHEEL_SPEED_CMD is app to S3 only")
        case .wheelSpeedSingleCommand:
            throw DecodeError.invalidPayload("WHEEL_SPEED_SINGLE_CMD is app to S3 only")
        case .masterSpeedCommand:
            throw DecodeError.invalidPayload("MASTER_SPEED_CMD is app to S3 only")
        case .chassisSpeedCommand:
            throw DecodeError.invalidPayload("CHASSIS_SPEED_CMD is app to S3 only")
        case .chassisHeadingCommand:
            throw DecodeError.invalidPayload("CHASSIS_HEADING_CMD is app to S3 only")
        case .pidParams:
            throw DecodeError.invalidPayload("PID_PARAMS_CMD is app to S3 only")
        case .wheelSpeedStatus:
            guard bytes.count == 16 else {
                throw DecodeError.invalidPayload("WHEEL_SPEED_STATUS requires four float values")
            }
            let speeds = (0..<4).map { float32(bytes, at: $0 * 4) }
            guard speeds.allSatisfy({ $0.isFinite }) else {
                throw DecodeError.invalidPayload("WHEEL_SPEED_STATUS contains non-finite value")
            }
            self = .wheelSpeedStatus(WheelSpeedStatus(speeds: speeds))
        case .wheelControlStatus:
            guard bytes.count == 44,
                  bytes[0] == 1,
                  let mode = ChassisControlMode(rawValue: bytes[1]),
                  bytes[2] == 0,
                  bytes[3] == 0 else {
                throw DecodeError.invalidPayload("WHEEL_CONTROL_STATUS schema/length")
            }
            let masterScale = float32(bytes, at: 8)
            let rawTargets = (0..<4).map { float32(bytes, at: 12 + $0 * 4) }
            let actualSpeeds = (0..<4).map { float32(bytes, at: 28 + $0 * 4) }
            guard masterScale.isFinite, masterScale >= 0, masterScale <= 4,
                  rawTargets.allSatisfy(\.isFinite),
                  actualSpeeds.allSatisfy(\.isFinite) else {
                throw DecodeError.invalidPayload("WHEEL_CONTROL_STATUS contains invalid value")
            }
            self = .wheelControlStatus(WheelControlStatus(
                mode: mode,
                timestampMs: uint32(bytes, at: 4),
                masterScale: masterScale,
                rawTargets: rawTargets,
                actualSpeeds: actualSpeeds))
        case .radarPWMControl:
            throw DecodeError.invalidPayload("RADAR_PWM_CONTROL is app to S3 only")
        case .radarStatus:
            guard bytes.count == 2,
                  bytes[0] <= 1,
                  bytes[1] <= 100 else {
                throw DecodeError.invalidPayload("RADAR_STATUS requires online and speed percent")
            }
            self = .radarStatus(RadarStatus(online: bytes[0] == 1, speedPercent: bytes[1]))
        case .powerStatus:
            guard bytes.count == 4 else {
                throw DecodeError.invalidPayload("POWER_STATUS requires one float value")
            }
            let voltage = float32(bytes, at: 0)
            guard voltage.isFinite else {
                throw DecodeError.invalidPayload("POWER_STATUS contains non-finite value")
            }
            self = .powerStatus(PowerStatus(voltage: voltage))
        case .chassisState:
            guard bytes.count == 24,
                  bytes[0] == 1,
                  (bytes[1] & 0xF0) == 0,
                  bytes[2] == 0,
                  bytes[3] == 0 else {
                throw DecodeError.invalidPayload("CHASSIS_STATE schema/length")
            }
            let xMm = float32(bytes, at: 8)
            let yMm = float32(bytes, at: 12)
            let yawDeg = float32(bytes, at: 16)
            let totalDistanceM = float32(bytes, at: 20)
            guard xMm.isFinite, yMm.isFinite, yawDeg.isFinite,
                  totalDistanceM.isFinite else {
                throw DecodeError.invalidPayload("CHASSIS_STATE contains non-finite value")
            }
            let flags = bytes[1]
            self = .chassisState(ChassisStateTelemetry(
                safetyFused: (flags & 0x01) != 0,
                headingLocked: (flags & 0x02) != 0,
                odometryValid: (flags & 0x04) != 0,
                attitudeReady: (flags & 0x08) != 0,
                timestampMs: uint32(bytes, at: 4),
                xMm: xMm,
                yMm: yMm,
                yawDeg: yawDeg,
                totalDistanceM: totalDistanceM))
        }
    }

    public var summary: String {
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
        case .wheelSpeedStatus(let status): return "WHEEL_SPEED_STATUS " + status.speeds.map { String(format: "%.1f", $0) }.joined(separator: ",")
        case .wheelControlStatus(let status): return String(format: "WHEEL_CONTROL_STATUS mode=%d scale=%.2f raw=[%.1f,%.1f,%.1f,%.1f] actual=[%.1f,%.1f,%.1f,%.1f]", status.mode.rawValue, status.masterScale, status.rawTargets[0], status.rawTargets[1], status.rawTargets[2], status.rawTargets[3], status.actualSpeeds[0], status.actualSpeeds[1], status.actualSpeeds[2], status.actualSpeeds[3])
        case .powerStatus(let status): return String(format: "POWER_STATUS %.2fV", status.voltage)
        case .chassisState(let state): return String(format: "CHASSIS_STATE x=%.1fmm y=%.1fmm yaw=%.1fdeg dist=%.3fm fuse=%d lock=%d valid=%d", state.xMm, state.yMm, state.yawDeg, state.totalDistanceM, state.safetyFused ? 1 : 0, state.headingLocked ? 1 : 0, state.odometryValid ? 1 : 0)
        }
    }
}

public struct DecodedMessageRecord: Identifiable, Equatable, @unchecked Sendable {
    public let id = UUID()
    public let message: DecodedMessage
    public let receivedAt: Date

    public var displayText: String {
        "\(receivedAt.formatted(date: .omitted, time: .standard))  \(message.summary)"
    }

    public init(message: DecodedMessage, receivedAt: Date) {
        self.message = message
        self.receivedAt = receivedAt
    }
}

public enum DecodeError: Error { case unsupportedType; case invalidPayload(String) }

public struct VehicleState: Equatable {
    public var connection: BLEConnectionStatus = .disconnected
    public var battery: UInt8 = 0
    public var motorState: UInt8 = 0
    public var errorCode: UInt16 = 0
    public var bmi323 = IMUData()
    public var lsm303 = IMUData()
    public var attitude = AttitudeData()
    public var dualAttitude: DualAttitude?
    public var radar = RadarStatus(online: false, speedPercent: 0)
    public var lastAttitudeAt: Date?
    public var lastStatus: Date?
    public var lastRadarStatus: Date?

    public func attitudeStatus(now: Date = Date()) -> AttitudeDisplayStatus {
        guard let lastAttitudeAt,
              now.timeIntervalSince(lastAttitudeAt) < 3.0 else {
            return .timeout
        }
        return attitude.valid ? .valid : .invalid
    }

    public var smartCarS3Status: String {
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

public enum AttitudeDisplayStatus: Equatable {
    case valid
    case invalid
    case timeout
}

extension AttitudeDisplayStatus {
    public var label: String {
        switch self {
        case .valid: return "VALID"
        case .invalid: return "INVALID"
        case .timeout: return "TIMEOUT"
        }
    }
}

extension Float {
    public var displayValue: String { String(format: "%.3f", self) }
    public var displayDegreeValue: String { String(format: "%.2f°", self) }
    public var displayControlDegreeValue: String { String(format: "%.2f°", self) }
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
