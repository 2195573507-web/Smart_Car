import Combine
import Foundation

struct AttitudeStateSnapshot: Equatable {
    var data = AttitudeData()
    var displayStatus: AttitudeDisplayStatus = .timeout
    var lastUpdatedAt: Date?
}

struct DualAttitudeStateSnapshot: Equatable {
    var data: DualAttitude?
    var displayStatus: AttitudeDisplayStatus = .timeout
    var lastUpdatedAt: Date?
}

struct IMUStateSnapshot: Equatable {
    var bmi323 = IMUData()
    var lsm303 = IMUData()
    var model = IMUDataModel()
}

enum RadarAvailability: Equatable {
    case offline
    case waiting
    case online
}

struct RadarStateSnapshot: Equatable {
    var connection: BLEConnectionStatus = .disconnected
    var online = false
    var speedPercent: UInt8 = 0
    var lastUpdatedAt: Date?

    var availability: RadarAvailability {
        guard connection == .connected else { return .offline }
        guard lastUpdatedAt != nil else { return .waiting }
        return online ? .online : .offline
    }
}

struct WheelSpeedStateSnapshot: Equatable {
    var actual = Array(repeating: Float(0), count: 4)
    var history = Array(repeating: [Float](), count: 4)
    var voltage: Float?
    var lastUpdatedAt: Date?
}

struct CalibrationStateSnapshot: Equatable {
    var status = IMUCalibrationStatus(state: .idle, sampleMode: .static,
                                      totalProgress: 0, currentPWM: 0,
                                      sampleProgress: 0, errorCode: 0,
                                      lastUpdatedAt: nil)
    var bias: IMUCalibrationBias?
    var timestamp: Date?
    var availability: CalibrationAvailability = .waiting
}

enum CalibrationAvailability: Equatable {
    case waiting
    case current
}

struct DualIMUStateSnapshot: Equatable {
    var status: DualIMULifecycleStatus?
    var lastUpdatedAt: Date?
}

struct VehicleStatusSnapshot: Equatable {
    var connection: BLEConnectionStatus = .disconnected
    var battery: UInt8 = 0
    var motorState: UInt8 = 0
    var errorCode: UInt16 = 0
    var lastStatusAt: Date?
    var smartCarS3Status = "OFFLINE"
}

@MainActor
final class DualAttitudeLogState: ObservableObject {
    @Published private(set) var lines: [String] = []

    private let capacity: Int

    init(capacity: Int) {
        self.capacity = capacity
    }

    func append(_ line: String) {
        lines.append(line)
        if lines.count > capacity {
            lines.removeFirst(lines.count - capacity)
        }
    }

    func clear() {
        lines.removeAll(keepingCapacity: true)
    }
}

@MainActor
final class AttitudeState: ObservableObject {
    @Published private(set) var snapshot = AttitudeStateSnapshot()

    private var pendingData: AttitudeData?
    private var pendingDate: Date?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.flush() }
        }
    }

    deinit { timer?.invalidate() }

    func ingest(_ data: AttitudeData, at date: Date) {
        pendingData = data
        pendingDate = date
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        guard connection == .connected else {
            pendingData = nil
            pendingDate = nil
            snapshot = AttitudeStateSnapshot()
            return
        }
    }

    private func flush() {
        var next = snapshot
        if let pendingData {
            next.data = pendingData
            next.lastUpdatedAt = pendingDate
            self.pendingData = nil
            pendingDate = nil
        }
        let now = Date()
        if let lastUpdatedAt = next.lastUpdatedAt,
           now.timeIntervalSince(lastUpdatedAt) < 3.0 {
            next.displayStatus = next.data.valid ? .valid : .invalid
        } else {
            next.displayStatus = .timeout
        }
        if next != snapshot {
            snapshot = next
        }
    }
}

@MainActor
final class DualAttitudeState: ObservableObject {
    private static let logInterval: TimeInterval = 1.0
    private static let logCapacity = 60

    @Published private(set) var snapshot = DualAttitudeStateSnapshot(data: nil)
    let log = DualAttitudeLogState(capacity: 60)

    private var pendingData: DualAttitude?
    private var pendingDate: Date?
    private var lastTimestampMs: UInt32?
    private var lastLogAt: Date?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) {
            [weak self] _ in Task { @MainActor in self?.flush() }
        }
    }

    deinit { timer?.invalidate() }

    func ingest(_ data: DualAttitude, at date: Date) {
        if let lastTimestampMs {
            let forwardDelta = data.timestampMs &- lastTimestampMs
            if forwardDelta > 0x8000_0000 {
                return
            }
        }
        lastTimestampMs = data.timestampMs
        pendingData = data
        pendingDate = date
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        guard connection == .connected else {
            pendingData = nil
            pendingDate = nil
            lastTimestampMs = nil
            lastLogAt = nil
            log.clear()
            snapshot = DualAttitudeStateSnapshot(data: nil)
            return
        }
    }

    private func flush() {
        var next = snapshot
        if let pendingData {
            next.data = pendingData
            next.lastUpdatedAt = pendingDate
            if let pendingDate,
               lastLogAt == nil || pendingDate.timeIntervalSince(lastLogAt!) >= Self.logInterval {
                log.append(Self.formatLogLine(pendingData))
                lastLogAt = pendingDate
            }
            self.pendingData = nil
            pendingDate = nil
        }
        if let lastUpdatedAt = next.lastUpdatedAt,
           Date().timeIntervalSince(lastUpdatedAt) < 3.0,
           let data = next.data {
            next.displayStatus = data.primaryValid || data.redundantValid
                ? .valid : .invalid
        } else {
            next.displayStatus = .timeout
        }
        if next != snapshot {
            snapshot = next
        }
    }

    private static func formatLogLine(_ data: DualAttitude) -> String {
        let radiansToDegrees = 180.0 / Double.pi
        return String(
            format: "[ATT_DUAL] PRI[R:%.2f° P:%.2f° Y:%.2f°] RED[R:%.2f° P:%.2f° Y:%.2f°] DIFF[ΔR:%.2f° ΔP:%.2f° ΔY:%.2f°] FLAGS:0x%02X",
            locale: Locale(identifier: "en_US_POSIX"),
            Double(data.primary.rollRad) * radiansToDegrees,
            Double(data.primary.pitchRad) * radiansToDegrees,
            Double(data.primary.yawRad) * radiansToDegrees,
            Double(data.redundant.rollRad) * radiansToDegrees,
            Double(data.redundant.pitchRad) * radiansToDegrees,
            Double(data.redundant.yawRad) * radiansToDegrees,
            Double(data.deltaRad.x) * radiansToDegrees,
            Double(data.deltaRad.y) * radiansToDegrees,
            Double(data.deltaRad.z) * radiansToDegrees,
            Int(data.flags)
        )
    }
}

@MainActor
final class IMUState: ObservableObject {
    @Published private(set) var snapshot = IMUStateSnapshot()

    private var pendingBMI323: IMUData?
    private var pendingLSM303: IMUData?
    private var pendingBMITelemetry: BMI323Data?
    private var pendingLSMTelemetry: LSM303Data?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.flush() }
        }
    }

    deinit { timer?.invalidate() }

    func ingest(sensorID: IMUSensorID, data: IMUData) {
        switch sensorID {
        case .bmi323: pendingBMI323 = data
        case .lsm303: pendingLSM303 = data
        }
    }

    func ingest(_ telemetry: IMUTelemetry) {
        switch telemetry {
        case .lsm303(let data): pendingLSMTelemetry = data
        case .bmi323(let data): pendingBMITelemetry = data
        }
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        guard connection == .connected else {
            pendingBMI323 = nil
            pendingLSM303 = nil
            pendingBMITelemetry = nil
            pendingLSMTelemetry = nil
            snapshot = IMUStateSnapshot()
            return
        }
    }

    private func flush() {
        var next = snapshot
        if let pendingBMI323 {
            next.bmi323 = pendingBMI323
            self.pendingBMI323 = nil
        }
        if let pendingLSM303 {
            next.lsm303 = pendingLSM303
            self.pendingLSM303 = nil
        }
        if let pendingLSMTelemetry {
            next.model.lsm303 = pendingLSMTelemetry
            self.pendingLSMTelemetry = nil
        }
        if let pendingBMITelemetry {
            next.model.bmi323 = pendingBMITelemetry
            self.pendingBMITelemetry = nil
        }
        if next != snapshot {
            snapshot = next
        }
    }
}

@MainActor
final class RadarState: ObservableObject {
    @Published private(set) var snapshot = RadarStateSnapshot()

    func setConnection(_ connection: BLEConnectionStatus) {
        guard snapshot.connection != connection else { return }
        var next = snapshot
        next.connection = connection
        if connection != .connected {
            next.online = false
            next.speedPercent = 0
            next.lastUpdatedAt = nil
        }
        snapshot = next
    }

    func ingest(_ status: RadarStatus, at date: Date) {
        var next = snapshot
        next.online = status.online
        next.speedPercent = status.speedPercent
        next.lastUpdatedAt = date
        if next != snapshot {
            snapshot = next
        }
    }

}

@MainActor
final class WheelSpeedState: ObservableObject {
    @Published private(set) var snapshot = WheelSpeedStateSnapshot()

    func ingest(_ status: WheelSpeedStatus, at date: Date) {
        guard status.speeds.count == 4 else { return }
        var next = snapshot
        next.actual = status.speeds
        for index in 0..<4 {
            next.history[index].append(status.speeds[index])
            if next.history[index].count > 48 { next.history[index].removeFirst() }
        }
        next.lastUpdatedAt = date
        if next != snapshot { snapshot = next }
    }

    func ingest(_ status: PowerStatus, at date: Date) {
        var next = snapshot
        next.voltage = status.voltage
        next.lastUpdatedAt = date
        if next != snapshot { snapshot = next }
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        if connection != .connected { snapshot = WheelSpeedStateSnapshot() }
    }
}

struct StaticCalibrationStateSnapshot: Equatable {
    var result = StaticCalibrationResult()
    var lastUpdatedAt: Date?
}

@MainActor
final class StaticCalibrationState: ObservableObject {
    @Published private(set) var snapshot = StaticCalibrationStateSnapshot()

    private var loggedSignature: String?

    func ingest(_ message: DecodedMessage, at date: Date) {
        switch message {
        case .imuCalibrationStatus(let status):
            ingest(status, at: date)
        case .imuCalibrationBias(let bias):
            ingest(bias, at: date)
        case .imuCalibrationResult(let result):
            var next = snapshot
            switch result.sensorID {
            case .lsm303: next.result.lsmAccelBias = result.accelBias
            case .bmi323:
                next.result.bmiAccelBias = result.accelBias
                next.result.bmiGyroBias = result.gyroBias
            }
            next.lastUpdatedAt = date
            if next != snapshot { snapshot = next }
        default:
            break
        }
    }

    func ingest(_ status: IMUCalibrationStatus, at date: Date) {
        var next = snapshot
        var result = next.result
        let staticStatus = status.sampleMode == .static
        let staticComplete = staticStatus &&
            status.stageCode == IMUCalibrationStage.complete.rawValue &&
            status.currentPWM == 0 &&
            status.totalSample > 0 &&
            status.sampleCount >= status.totalSample

        result.sampleCount = status.sampleCount
        result.sampleTotal = status.totalSample
        result.errorCode = status.state == .error ? status.errorCode : nil

        if status.state == .error && result.phase != .completed {
            result.phase = .error
        } else if result.phase != .completed {
            if staticComplete {
                result.phase = .completed
            } else if staticStatus &&
                        (status.sampleCount > 0 || status.stageCode != IMUCalibrationStage.waitRadarReady.rawValue) {
                result.phase = .sampling
            } else {
                result.phase = .waiting
            }
        }

        next.result = result
        next.lastUpdatedAt = date
        if next != snapshot {
            snapshot = next
        }
        logCompletedResultIfNeeded()
    }

    func ingest(_ bias: IMUCalibrationBias, at date: Date) {
        print("[IMU_CAL_BIAS_RX] x=\(scalar(bias.x)) y=\(scalar(bias.y)) z=\(scalar(bias.z))")
        var next = snapshot
        next.result.accelOffsetX = bias.x
        next.result.accelOffsetY = bias.y
        next.result.accelOffsetZ = bias.z
        next.lastUpdatedAt = date
        if next != snapshot {
            snapshot = next
        }
        logCompletedResultIfNeeded()
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        guard connection == .connected else {
            snapshot = StaticCalibrationStateSnapshot()
            loggedSignature = nil
            return
        }
    }

    private func logCompletedResultIfNeeded() {
        let result = snapshot.result
        guard result.phase == .completed else { return }
        let signature = "\(result.sampleCount)/\(result.sampleTotal)|\(scalar(result.accelOffsetX))|\(scalar(result.accelOffsetY))|\(scalar(result.accelOffsetZ))"
        guard loggedSignature != signature else { return }
        loggedSignature = signature
        print("[APP_CAL] STATIC RESULT RX samples=\(result.sampleCount)/\(result.sampleTotal) offset=(\(scalar(result.accelOffsetX)),\(scalar(result.accelOffsetY)),\(scalar(result.accelOffsetZ)))")
    }

    private func scalar(_ value: Float?) -> String {
        value.map { String(format: "%.6f", $0) } ?? "--"
    }
}

@MainActor
final class CalibrationState: ObservableObject {
    @Published private(set) var snapshot = CalibrationStateSnapshot()
    private var pendingSnapshot: CalibrationStateSnapshot?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.flush() }
        }
    }

    deinit { timer?.invalidate() }

    func ingest(_ message: DecodedMessage, at date: Date) {
        var next = pendingSnapshot ?? snapshot
        switch message {
        case .imuCalibrationStatus(let status):
            var updated = status
            updated.lastUpdatedAt = date
            next.status = updated
            next.timestamp = date
            next.availability = .current
        case .dualIMUStatus(let lifecycle) where lifecycle.phase == .ready:
            markComplete(in: &next, at: date)
        case .imuCalibrationBias(let bias):
            next.bias = bias
        default:
            return
        }
        if next != snapshot {
            pendingSnapshot = next
        }
    }

    private func markComplete(in snapshot: inout CalibrationStateSnapshot, at date: Date) {
        let current = snapshot.status
        snapshot.status = IMUCalibrationStatus(
            state: .complete,
            sampleMode: current.sampleMode,
            totalProgress: 100,
            currentPWM: current.currentPWM,
            sampleProgress: current.sampleProgress,
            sampleCount: current.sampleCount,
            totalSample: current.totalSample,
            stageCode: IMUCalibrationStage.complete.rawValue,
            errorCode: 0,
            lastUpdatedAt: date
        )
        snapshot.timestamp = date
        snapshot.availability = .current
    }

    private func flush() {
        var next = pendingSnapshot ?? snapshot
        pendingSnapshot = nil
        // Completion is an edge-triggered terminal result. Retain it until a
        // newer status frame or disconnect supersedes it instead of reverting
        // the UI to its initial waiting state when the stream becomes quiet.
        if let timestamp = next.timestamp,
           next.status.state != .complete,
           Date().timeIntervalSince(timestamp) > 3.0 {
            next.availability = .waiting
        }
        if next != snapshot {
            snapshot = next
        }
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        guard connection != .connected else { return }
        pendingSnapshot = nil
        snapshot = CalibrationStateSnapshot()
    }
}

@MainActor
final class DualIMUState: ObservableObject {
    @Published private(set) var snapshot = DualIMUStateSnapshot()

    func ingest(_ status: DualIMULifecycleStatus, at date: Date) {
        let next = DualIMUStateSnapshot(status: status, lastUpdatedAt: date)
        if next != snapshot {
            snapshot = next
        }
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        guard connection != .connected else { return }
        snapshot = DualIMUStateSnapshot()
    }
}

@MainActor
final class VehicleStatusState: ObservableObject {
    @Published private(set) var snapshot = VehicleStatusSnapshot()

    private var pendingStatus: SmartCarStatus?
    private var pendingDate: Date?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.flush() }
        }
    }

    deinit { timer?.invalidate() }

    func setConnection(_ status: BLEConnectionStatus) {
        guard snapshot.connection != status else { return }
        var next = snapshot
        next.connection = status
        next.smartCarS3Status = Self.smartCarStatus(
            connection: status,
            lastStatusAt: next.lastStatusAt,
            now: Date()
        )
        snapshot = next
    }

    func ingest(_ status: SmartCarStatus, at date: Date) {
        pendingStatus = status
        pendingDate = date
    }

    private func flush() {
        var next = snapshot
        if let pendingStatus {
            next.battery = pendingStatus.battery
            next.motorState = pendingStatus.motorState
            next.errorCode = pendingStatus.errorCode
            next.lastStatusAt = pendingDate
            self.pendingStatus = nil
            pendingDate = nil
        }
        next.smartCarS3Status = Self.smartCarStatus(
            connection: next.connection,
            lastStatusAt: next.lastStatusAt,
            now: Date()
        )
        if next != snapshot {
            snapshot = next
        }
    }

    private static func smartCarStatus(
        connection: BLEConnectionStatus,
        lastStatusAt: Date?,
        now: Date
    ) -> String {
        guard connection == .connected else { return "OFFLINE" }
        guard let lastStatusAt else { return "WAITING" }
        return now.timeIntervalSince(lastStatusAt) < 3.0 ? "ONLINE" : "STALE"
    }
}

@MainActor
final class TelemetryStore {
    let attitude = AttitudeState()
    let dualAttitude = DualAttitudeState()
    let imu = IMUState()
    let radar = RadarState()
    let wheelSpeed = WheelSpeedState()
    let staticCalibration = StaticCalibrationState()
    let calibration = CalibrationState()
    let dualIMU = DualIMUState()
    let status = VehicleStatusState()

    func ingest(_ message: DecodedMessage, at date: Date) {
        staticCalibration.ingest(message, at: date)
        calibration.ingest(message, at: date)
        switch message {
        case .status(let status):
            self.status.ingest(status, at: date)
        case .imuStatus(let imu):
            self.imu.ingest(sensorID: imu.sensorID, data: imu.data)
        case .imuTelemetry(let telemetry):
            self.imu.ingest(telemetry)
        case .dualIMUStatus(let lifecycle):
            dualIMU.ingest(lifecycle, at: date)
        case .attitude(let attitude):
            self.attitude.ingest(attitude, at: date)
        case .dualAttitude(let dual):
            self.dualAttitude.ingest(dual, at: date)
            self.attitude.ingest(dual.primary.attitudeData, at: date)
        case .radarStatus(let status):
            radar.ingest(status, at: date)
        case .wheelSpeedStatus(let status):
            wheelSpeed.ingest(status, at: date)
        case .powerStatus(let status):
            wheelSpeed.ingest(status, at: date)
        default:
            break
        }
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        attitude.setConnection(connection)
        dualAttitude.setConnection(connection)
        imu.setConnection(connection)
        status.setConnection(connection)
        radar.setConnection(connection)
        wheelSpeed.setConnection(connection)
        staticCalibration.setConnection(connection)
        calibration.setConnection(connection)
        dualIMU.setConnection(connection)
    }
}
