import Foundation

public struct AppTelemetrySnapshot: @unchecked Sendable {
    public let critical: [DecodedMessageRecord]
    public let telemetry: [DecodedMessageRecord]
    public let other: [DecodedMessageRecord]

    public init(critical: [DecodedMessageRecord],
                telemetry: [DecodedMessageRecord],
                other: [DecodedMessageRecord]) {
        self.critical = critical
        self.telemetry = telemetry
        self.other = other
    }

    public var records: [DecodedMessageRecord] {
        critical + telemetry + other
    }

    public var isEmpty: Bool { critical.isEmpty && telemetry.isEmpty && other.isEmpty }
}

/// Coalesces decoded messages away from MainActor. The UI receives an
/// immutable batch at its scheduled publication rate rather than reducing each
/// BLE callback independently.
public actor AppTelemetryReducer {
    public let otherCapacity: Int

    private var critical: [DecodedMessageRecord] = []
    private var latestTelemetry: [UInt8: DecodedMessageRecord] = [:]
    private var other: AppBLEBoundedRingBuffer<DecodedMessageRecord>

    public init(otherCapacity: Int = 128) {
        self.otherCapacity = max(1, otherCapacity)
        self.other = AppBLEBoundedRingBuffer(capacity: max(1, otherCapacity))
    }

    public func reset() {
        critical.removeAll(keepingCapacity: true)
        latestTelemetry.removeAll(keepingCapacity: true)
        other.removeAll()
    }

    public func submit(_ record: DecodedMessageRecord) {
        switch Self.priority(for: record.message) {
        case .critical:
            critical.append(record)
        case .telemetry:
            latestTelemetry[Self.telemetryKey(for: record.message)] = record
        case .other:
            _ = other.append(record)
        }
    }

    public func drain() -> AppTelemetrySnapshot {
        let telemetry = latestTelemetry.values.sorted { lhs, rhs in
            if lhs.receivedAt == rhs.receivedAt { return lhs.id.uuidString < rhs.id.uuidString }
            return lhs.receivedAt < rhs.receivedAt
        }
        let snapshot = AppTelemetrySnapshot(
            critical: critical,
            telemetry: telemetry,
            other: other.elements
        )
        critical.removeAll(keepingCapacity: true)
        latestTelemetry.removeAll(keepingCapacity: true)
        other.removeAll()
        return snapshot
    }

    private enum Priority { case critical, telemetry, other }

    private static func priority(for message: DecodedMessage) -> Priority {
        switch message {
        case .ack, .control, .status:
            return .critical
        case .attitude, .dualAttitude, .imuStatus,
             .imuCalibrationStatus, .imuCalibrationBias, .imuCalibrationResult,
             .imuTelemetry, .dualIMUStatus, .radarStatus,
             .wheelSpeedStatus, .wheelControlStatus, .powerStatus,
             .chassisState:
            return .telemetry
        case .ping, .calibrationEvent:
            return .other
        }
    }

    private static func telemetryKey(for message: DecodedMessage) -> UInt8 {
        switch message {
        case .status: return SmartCarProtocol.FrameType.status.rawValue
        case .imuStatus: return SmartCarProtocol.FrameType.imuStatus.rawValue
        case .attitude: return SmartCarProtocol.FrameType.attitude.rawValue
        case .dualAttitude: return 0x2E
        case .imuCalibrationStatus: return SmartCarProtocol.FrameType.imuCalibrationStatus.rawValue
        case .imuCalibrationBias: return SmartCarProtocol.FrameType.imuCalibrationBias.rawValue
        case .imuCalibrationResult: return SmartCarProtocol.FrameType.imuCalibrationResult.rawValue
        case .imuTelemetry: return SmartCarProtocol.FrameType.imuTelemetry.rawValue
        case .dualIMUStatus: return SmartCarProtocol.FrameType.dualIMUStatus.rawValue
        case .radarStatus: return SmartCarProtocol.FrameType.radarStatus.rawValue
        case .wheelSpeedStatus: return SmartCarProtocol.FrameType.wheelSpeedStatus.rawValue
        case .wheelControlStatus: return SmartCarProtocol.FrameType.wheelControlStatus.rawValue
        case .powerStatus: return SmartCarProtocol.FrameType.powerStatus.rawValue
        case .chassisState: return SmartCarProtocol.FrameType.chassisState.rawValue
        default: return 0xFF
        }
    }
}
