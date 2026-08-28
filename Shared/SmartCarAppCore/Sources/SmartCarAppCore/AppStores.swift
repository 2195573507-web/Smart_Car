import Combine
import Foundation

public struct ConnectionSnapshot: Equatable, Sendable {
    public var status: BLEConnectionStatus
    public var deviceName: String
    public var rssi: Int?
    public var errorDescription: String?

    public init(status: BLEConnectionStatus = .disconnected,
                deviceName: String = "SmartCar_S3", rssi: Int? = nil,
                errorDescription: String? = nil) {
        self.status = status
        self.deviceName = deviceName
        self.rssi = rssi
        self.errorDescription = errorDescription
    }
}

@MainActor
public final class ConnectionStore: ObservableObject {
    @Published public private(set) var snapshot: ConnectionSnapshot

    public init(snapshot: ConnectionSnapshot = ConnectionSnapshot()) {
        self.snapshot = snapshot
    }

    public func setStatus(_ status: BLEConnectionStatus) {
        snapshot.status = status
        if status != .connected {
            snapshot.rssi = nil
        }
    }

    public func setDevice(name: String, rssi: Int?) {
        snapshot.deviceName = name
        snapshot.rssi = rssi
    }

    public func setError(_ description: String?) {
        snapshot.errorDescription = description
    }
}

public struct MotionIntent: Equatable, Sendable {
    public var mode: ChassisControlMode
    public var wheelTargets: [Float]
    public var chassisBaseSpeed: Float
    public var yawRate: Float
    public var masterScale: Float
    public var targetYawDeg: Float
    public var headingLockEnabled: Bool
    public var isStop: Bool

    public init(mode: ChassisControlMode = .chassisDiff,
                wheelTargets: [Float] = Array(repeating: 0, count: 4),
                chassisBaseSpeed: Float = 0, yawRate: Float = 0,
                masterScale: Float = 1, targetYawDeg: Float = 0,
                headingLockEnabled: Bool = false, isStop: Bool = false) {
        self.mode = mode
        self.wheelTargets = wheelTargets
        self.chassisBaseSpeed = chassisBaseSpeed
        self.yawRate = yawRate
        self.masterScale = masterScale
        self.targetYawDeg = targetYawDeg
        self.headingLockEnabled = headingLockEnabled
        self.isStop = isStop
    }
}

@MainActor
public final class DriveStore: ObservableObject {
    @Published public private(set) var intent = MotionIntent()

    public init() {}

    public func update(_ intent: MotionIntent) {
        self.intent = intent
    }

    public func stop() {
        intent = MotionIntent(mode: .wheelIndependent, isStop: true)
    }
}

public struct DiagnosticsSnapshot: Equatable, Sendable {
    public var transmittedFrames = 0
    public var receivedFrames = 0
    public var decodeFailures = 0
    public var droppedMessages = 0
    public var lastPacketType = "--"
    public var lastPacketAt: Date?

    public init() {}
}

@MainActor
public final class DiagnosticsStore: ObservableObject {
    @Published public private(set) var snapshot = DiagnosticsSnapshot()

    public init() {}

    public func recordTransmit() {
        snapshot.transmittedFrames += 1
    }

    public func recordReceive(type: String, at date: Date) {
        snapshot.receivedFrames += 1
        snapshot.lastPacketType = type
        snapshot.lastPacketAt = date
    }

    public func update(decodeFailures: Int, droppedMessages: Int) {
        snapshot.decodeFailures = decodeFailures
        snapshot.droppedMessages = droppedMessages
    }

    public func sync(transmittedFrames: Int, receivedFrames: Int,
                     decodeFailures: Int, droppedMessages: Int) {
        snapshot.transmittedFrames = transmittedFrames
        snapshot.receivedFrames = receivedFrames
        snapshot.decodeFailures = decodeFailures
        snapshot.droppedMessages = droppedMessages
    }
}
