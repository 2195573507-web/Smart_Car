import Combine
import Foundation

struct AttitudeStateSnapshot: Equatable {
    var data = AttitudeData()
    var displayStatus: AttitudeDisplayStatus = .timeout
    var lastUpdatedAt: Date?
}

struct IMUStateSnapshot: Equatable {
    var bmi323 = IMUData()
    var lsm303 = IMUData()
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
    var calibrationPWM: UInt8 = 0
    var calibrationActive = false
    var lastCalibrationUpdatedAt: Date?

    var availability: RadarAvailability {
        guard connection == .connected else { return .offline }
        guard lastUpdatedAt != nil else { return .waiting }
        return online ? .online : .offline
    }
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

struct VehicleStatusSnapshot: Equatable {
    var connection: BLEConnectionStatus = .disconnected
    var battery: UInt8 = 0
    var motorState: UInt8 = 0
    var errorCode: UInt16 = 0
    var lastStatusAt: Date?
    var smartCarS3Status = "OFFLINE"
}

@MainActor
final class AttitudeState: ObservableObject {
    @Published private(set) var snapshot = AttitudeStateSnapshot()

    private var pendingData: AttitudeData?
    private var pendingDate: Date?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true) { [weak self] _ in
            self?.flush()
        }
    }

    deinit { timer?.invalidate() }

    func ingest(_ data: AttitudeData, at date: Date) {
        pendingData = data
        pendingDate = date
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
final class IMUState: ObservableObject {
    @Published private(set) var snapshot = IMUStateSnapshot()

    private var pendingBMI323: IMUData?
    private var pendingLSM303: IMUData?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
            self?.flush()
        }
    }

    deinit { timer?.invalidate() }

    func ingest(sensorID: IMUSensorID, data: IMUData) {
        switch sensorID {
        case .bmi323: pendingBMI323 = data
        case .lsm303: pendingLSM303 = data
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
            next.calibrationPWM = 0
            next.calibrationActive = false
            next.lastCalibrationUpdatedAt = nil
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

    func ingest(_ status: RadarCalibrationStatus, at date: Date) {
        var next = snapshot
        next.calibrationPWM = status.currentPWM
        next.calibrationActive = status.active
        next.lastCalibrationUpdatedAt = date
        if next != snapshot {
            snapshot = next
        }
    }
}

@MainActor
final class CalibrationState: ObservableObject {
    @Published private(set) var snapshot = CalibrationStateSnapshot()
    private var pendingSnapshot: CalibrationStateSnapshot?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
            self?.flush()
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
        case .imuCalibrationBias(let bias):
            next.bias = bias
        default:
            return
        }
        if next != snapshot {
            pendingSnapshot = next
        }
    }

    private func flush() {
        var next = pendingSnapshot ?? snapshot
        pendingSnapshot = nil
        if let timestamp = next.timestamp,
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
final class VehicleStatusState: ObservableObject {
    @Published private(set) var snapshot = VehicleStatusSnapshot()

    private var pendingStatus: SmartCarStatus?
    private var pendingDate: Date?
    private var timer: Timer?

    init() {
        timer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            self?.flush()
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
    let imu = IMUState()
    let radar = RadarState()
    let calibration = CalibrationState()
    let status = VehicleStatusState()

    func ingest(_ message: DecodedMessage, at date: Date) {
        calibration.ingest(message, at: date)
        switch message {
        case .status(let status):
            self.status.ingest(status, at: date)
        case .imuStatus(let imu):
            self.imu.ingest(sensorID: imu.sensorID, data: imu.data)
        case .attitude(let attitude):
            self.attitude.ingest(attitude, at: date)
        case .radarStatus(let status):
            radar.ingest(status, at: date)
        case .radarCalibrationStatus(let status):
            radar.ingest(status, at: date)
        default:
            break
        }
    }

    func setConnection(_ connection: BLEConnectionStatus) {
        status.setConnection(connection)
        radar.setConnection(connection)
        calibration.setConnection(connection)
    }
}
