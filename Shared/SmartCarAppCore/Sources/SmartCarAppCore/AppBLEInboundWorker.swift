import Foundation

public enum AppBLEInboundPriority: Sendable {
    case critical
    case telemetry
    case other
}

public struct AppBLEInboundFrame: Sendable {
    public let frame: AppBLEFrame
    public let receivedAt: Date
    public let priority: AppBLEInboundPriority

    public init(frame: AppBLEFrame, receivedAt: Date, priority: AppBLEInboundPriority) {
        self.frame = frame
        self.receivedAt = receivedAt
        self.priority = priority
    }
}

public struct AppBLEInboundBatch: Sendable {
    public let critical: [AppBLEInboundFrame]
    public let telemetry: [AppBLEInboundFrame]
    public let other: [AppBLEInboundFrame]
    public let droppedTelemetry: Int
    public let droppedOther: Int

    public init(critical: [AppBLEInboundFrame], telemetry: [AppBLEInboundFrame],
                other: [AppBLEInboundFrame], droppedTelemetry: Int,
                droppedOther: Int) {
        self.critical = critical
        self.telemetry = telemetry
        self.other = other
        self.droppedTelemetry = droppedTelemetry
        self.droppedOther = droppedOther
    }

    public var isEmpty: Bool {
        critical.isEmpty && telemetry.isEmpty && other.isEmpty
    }
}

/// Serializes raw BLE notifications away from the UI actor. Critical traffic
/// uses backpressure when its reserved queue is full; telemetry is coalesced
/// by type and diagnostic traffic uses a bounded oldest-drop ring.
public actor AppBLEInboundWorker {
    public let criticalCapacity: Int
    public let otherCapacity: Int

    private var parser = AppBLEFrameParser()
    private var criticalQueue: [AppBLEInboundFrame] = []
    private var waitingCritical: [(AppBLEInboundFrame, CheckedContinuation<Void, Never>)] = []
    private var waitingCriticalHead = 0
    private var latestTelemetry: [UInt8: AppBLEInboundFrame] = [:]
    private var otherQueue: AppBLEBoundedRingBuffer<AppBLEInboundFrame>
    private var droppedTelemetryTotal = 0
    private var droppedOtherTotal = 0

    public init(criticalCapacity: Int = 64, otherCapacity: Int = 128) {
        self.criticalCapacity = max(1, criticalCapacity)
        self.otherCapacity = max(1, otherCapacity)
        self.otherQueue = AppBLEBoundedRingBuffer(capacity: max(1, otherCapacity))
        self.criticalQueue.reserveCapacity(max(1, criticalCapacity))
    }

    public func reset() {
        parser.reset()
        criticalQueue.removeAll(keepingCapacity: true)
        latestTelemetry.removeAll(keepingCapacity: true)
        otherQueue.removeAll()
        droppedTelemetryTotal = 0
        droppedOtherTotal = 0
        resumeWaitingCritical()
    }

    public func submit(_ data: Data, receivedAt: Date = Date()) async {
        guard !data.isEmpty else { return }
        let frames = parser.feed(data)
        for frame in frames {
            let priority = Self.priority(for: frame)
            let item = AppBLEInboundFrame(frame: frame, receivedAt: receivedAt, priority: priority)
            switch priority {
            case .critical:
                await enqueueCritical(item)
            case .telemetry:
                if latestTelemetry.updateValue(item, forKey: frame.type) != nil {
                    droppedTelemetryTotal += 1
                }
            case .other:
                if otherQueue.append(item) {
                    droppedOtherTotal += 1
                }
            }
        }
    }

    public func drain() -> AppBLEInboundBatch {
        let critical = criticalQueue
        criticalQueue.removeAll(keepingCapacity: true)

        let telemetry = latestTelemetry.values.sorted {
            if $0.receivedAt == $1.receivedAt {
                return $0.frame.type < $1.frame.type
            }
            return $0.receivedAt < $1.receivedAt
        }
        latestTelemetry.removeAll(keepingCapacity: true)
        let other = otherQueue.elements
        otherQueue.removeAll()

        resumeWaitingCritical()
        let batch = AppBLEInboundBatch(
            critical: critical,
            telemetry: telemetry,
            other: other,
            droppedTelemetry: droppedTelemetryTotal,
            droppedOther: droppedOtherTotal
        )
        droppedTelemetryTotal = 0
        droppedOtherTotal = 0
        return batch
    }

    public func pendingCount() -> Int {
        criticalQueue.count + latestTelemetry.count + otherQueue.count
    }

    private func enqueueCritical(_ item: AppBLEInboundFrame) async {
        if criticalQueue.count < criticalCapacity {
            criticalQueue.append(item)
            return
        }
        await withCheckedContinuation { continuation in
            waitingCritical.append((item, continuation))
        }
    }

    private func resumeWaitingCritical() {
        while criticalQueue.count < criticalCapacity,
              waitingCriticalHead < waitingCritical.count {
            let pending = waitingCritical[waitingCriticalHead]
            waitingCriticalHead += 1
            let (item, continuation) = pending
            criticalQueue.append(item)
            continuation.resume()
        }
        if waitingCriticalHead == waitingCritical.count {
            waitingCritical.removeAll(keepingCapacity: true)
            waitingCriticalHead = 0
        } else if waitingCriticalHead >= waitingCritical.count / 2 {
            waitingCritical.removeSubrange(0..<waitingCriticalHead)
            waitingCriticalHead = 0
        }
    }

    private static func priority(for frame: AppBLEFrame) -> AppBLEInboundPriority {
        if frame.version == AppBLEVersion.v2.rawValue {
            switch AppBLEV2Type(rawValue: frame.type) {
            case .helloAck, .heartbeatAck, .commandAck:
                return .critical
            default:
                return .other
            }
        }
        switch SmartCarProtocol.FrameType(rawValue: frame.type) {
        case .ack, .control, .status:
            return .critical
        case .attitude, .imuStatus, .imuCalibrationStatus,
             .imuCalibrationBias, .imuCalibrationResult, .imuTelemetry,
             .dualIMUStatus, .radarStatus, .wheelSpeedStatus,
             .wheelControlStatus, .powerStatus, .chassisState:
            return .telemetry
        default:
            return .other
        }
    }
}
