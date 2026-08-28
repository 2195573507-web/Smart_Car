import Combine
import Foundation
import SmartCarAppCore
#if canImport(UIKit)
import UIKit
#elseif canImport(AppKit)
import AppKit
#endif

enum DeveloperPage: Equatable {
    case overview
    case loggerSTM
    case loggerS3
}

@MainActor
final class SmartCarViewModel: ObservableObject {
    private struct PendingWheelControl {
        let rawTargets: [Float]
        let masterScale: Float
        let expiresAt: Date
    }

    private static let angleUnitDefaultsKey = "smartcar.angleUnit"
    private static let wheelControlConfirmationWindow: TimeInterval = 1.0
    private static let chassisSpeedMin: Float = -800.0
    private static let chassisSpeedMax: Float = 800.0
    private static let targetYawMin: Float = -180.0
    private static let targetYawMax: Float = 180.0
    private static let motionCommandInterval: TimeInterval = 0.05

    let bleManager: BLEManager
    let telemetryStore: TelemetryStore
    let calibrationViewModel: CalibrationViewModel
    let connectionStore: ConnectionStore
    let driveStore: DriveStore
    let diagnosticsStore: DiagnosticsStore
    let pidTuning = PIDTuningState()

    @Published var mode: AppMode = .control
    @Published var developerPage: DeveloperPage = .overview
    @Published var angleUnit: AngleUnit {
        didSet {
            UserDefaults.standard.set(angleUnit.rawValue, forKey: Self.angleUnitDefaultsKey)
        }
    }
    @Published var speed: Double = 50
    @Published var radarSpeed: Double = 0
    @Published var wheelTargets: [Float] = Array(repeating: 0, count: 4)
    @Published var chassisBaseSpeed: Float = 0.0
    @Published var masterSpeedScale: Float = 1.0
    @Published var targetYawDeg: Float = 0.0
    @Published var headingLockEnabled = false
    @Published private(set) var controlMode: ChassisControlMode = .chassisDiff
    @Published private(set) var status: BLEConnectionStatus
    @Published private(set) var discoveredDeviceName: String
    @Published private(set) var discoveredRSSI: Int?
    @Published private(set) var lastError: BLEUserFacingError?
    @Published private(set) var vehicleStatus: VehicleStatusSnapshot
    @Published private(set) var transmittedFrameCount = 0
    @Published private(set) var receivedFrameCount = 0
    @Published private(set) var lastPacketType = "--"
    @Published private(set) var logSnapshot: [DecodedMessageRecord] = []

    private var pendingReceivedFrameCount = 0
    private var pendingLastPacketType = "--"
    private var debugTimer: Timer?
    private var statusCancellable: AnyCancellable?
    private var wheelCommandTimer: Timer?
    private var chassisCommandTimer: Timer?
    private var pendingControlMode: (mode: ChassisControlMode, expiresAt: Date)?
    private var pendingWheelControl: PendingWheelControl?
    private var lifecycleObservers: [NSObjectProtocol] = []

    var decodedMessages: [DecodedMessageRecord] { bleManager.decodedMessages }
    var stmLogStore: DeviceLogStore { bleManager.stmLogStore }
    var s3LogStore: DeviceLogStore { bleManager.s3LogStore }
    var decodeFailureCount: Int { bleManager.decodeFailureCount }
    var droppedMessageCount: Int { bleManager.droppedMessageCount }
    var lastPacketReceivedAt: Date? { bleManager.lastPacketReceivedAt }

    init(bleManager: BLEManager? = nil) {
        let manager = bleManager ?? BLEManager()
        self.bleManager = manager
        self.telemetryStore = manager.telemetryStore
        self.calibrationViewModel = CalibrationViewModel(telemetryStore: manager.telemetryStore)
        self.connectionStore = ConnectionStore(snapshot: ConnectionSnapshot(
            status: manager.status,
            deviceName: manager.discoveredDeviceName,
            rssi: manager.discoveredRSSI,
            errorDescription: manager.lastError?.localizedDescription(locale: .current)
        ))
        self.driveStore = DriveStore()
        self.diagnosticsStore = DiagnosticsStore()
        self.status = manager.status
        self.discoveredDeviceName = manager.discoveredDeviceName
        self.discoveredRSSI = manager.discoveredRSSI
        self.lastError = manager.lastError
        self.vehicleStatus = manager.telemetryStore.status.snapshot
        self.angleUnit = AngleUnit(
            rawValue: UserDefaults.standard.string(forKey: Self.angleUnitDefaultsKey) ?? "degree"
        ) ?? .degree

        statusCancellable = manager.telemetryStore.status.$snapshot
            .removeDuplicates()
            .sink { [weak self] snapshot in
                self?.vehicleStatus = snapshot
            }

        manager.onStatusChange = { [weak self] status in
            guard let self else { return }
            self.status = status
            self.connectionStore.setStatus(status)
            if status != .connected {
                self.cancelScheduledWheelCommand()
                self.cancelChassisCommand()
                self.wheelTargets = Array(repeating: 0, count: 4)
                self.chassisBaseSpeed = 0.0
                self.masterSpeedScale = 1.0
                self.targetYawDeg = 0.0
                self.headingLockEnabled = false
                self.controlMode = .chassisDiff
                self.pendingControlMode = nil
                self.pendingWheelControl = nil
                self.pidTuning.applyStatus = .unavailable
            }
        }
        manager.onDeviceNameChange = { [weak self] name in
            self?.discoveredDeviceName = name
            guard let self else { return }
            self.connectionStore.setDevice(name: name, rssi: self.discoveredRSSI)
        }
        manager.onRSSIChange = { [weak self] rssi in
            self?.discoveredRSSI = rssi
            guard let self else { return }
            self.connectionStore.setDevice(name: self.discoveredDeviceName, rssi: rssi)
        }
        manager.onErrorChange = { [weak self] error in
            self?.lastError = error
            self?.connectionStore.setError(error?.localizedDescription(locale: .current))
        }
        manager.onDecodedMessage = { [weak self] record in
            guard let self else { return }
            self.pendingReceivedFrameCount += 1
            self.pendingLastPacketType = Self.packetType(for: record.message)
            self.diagnosticsStore.recordReceive(
                type: self.pendingLastPacketType, at: record.receivedAt)
            if case .ack(let payload) = record.message,
               payload.count >= 2,
               payload[0] == SmartCarProtocol.FrameType.pidParams.rawValue {
                self.pidTuning.applyStatus = payload[1] == 0
                    ? .applied : .rejected
            }
            if case .wheelControlStatus(let status) = record.message {
                self.acceptWheelControlTelemetry(status)
            }
        }

        debugTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.flushDebugMetrics() }
        }
#if canImport(UIKit)
        let center = NotificationCenter.default
        lifecycleObservers = [
            center.addObserver(forName: UIApplication.didEnterBackgroundNotification, object: nil, queue: .main) { [weak self] _ in
                Task { @MainActor [weak self] in self?.sendZeroWheelSpeeds() }
            },
            center.addObserver(forName: UIApplication.willTerminateNotification, object: nil, queue: .main) { [weak self] _ in
                Task { @MainActor [weak self] in self?.sendZeroWheelSpeeds() }
            }
        ]
#elseif canImport(AppKit)
        let center = NotificationCenter.default
        lifecycleObservers = [
            center.addObserver(forName: NSApplication.didResignActiveNotification, object: nil, queue: .main) { [weak self] _ in
                Task { @MainActor [weak self] in self?.sendZeroWheelSpeeds() }
            },
            center.addObserver(forName: NSApplication.didHideNotification, object: nil, queue: .main) { [weak self] _ in
                Task { @MainActor [weak self] in self?.sendZeroWheelSpeeds() }
            },
            center.addObserver(forName: NSApplication.willTerminateNotification, object: nil, queue: .main) { [weak self] _ in
                Task { @MainActor [weak self] in self?.sendZeroWheelSpeeds() }
            }
        ]
#endif

        // Start discovery on launch; BLEManager defers the actual scan until
        // CoreBluetooth reports poweredOn when Bluetooth is still initializing.
        Task { @MainActor [weak self] in
            self?.scan()
        }
    }

    deinit {
        debugTimer?.invalidate()
        wheelCommandTimer?.invalidate()
        chassisCommandTimer?.invalidate()
        for observer in lifecycleObservers { NotificationCenter.default.removeObserver(observer) }
    }

    func scan() { bleManager.startScanning() }
    func connect() { bleManager.connectToDevice() }
    func disconnect() {
        cancelScheduledWheelCommand()
        cancelChassisCommand()
        wheelTargets = Array(repeating: 0, count: 4)
        targetYawDeg = 0.0
        headingLockEnabled = false
        pendingWheelControl = nil
        pendingControlMode = nil
        bleManager.disconnect()
    }

    func setWheelTarget(index: Int, value: Double) {
        guard controlMode == .wheelIndependent,
              wheelTargets.indices.contains(index) else { return }
        wheelTargets[index] = Float(max(-800, min(800, value)))
        publishDriveIntent()
        markWheelControlPending()
        if wheelTargets.allSatisfy({ $0 == 0.0 }) {
            sendZeroWheelSpeeds()
        } else {
            scheduleWheelCommand()
        }
    }

    /// Applies a button-sized single-wheel adjustment using the active
    /// App-BLE 0x2A single-wheel mapping. The full-wheel publisher then keeps
    /// the latest tuple flowing while the independent controls remain active.
    func nudgeWheelTarget(index: Int, delta: Double) {
        guard controlMode == .wheelIndependent,
              wheelTargets.indices.contains(index) else { return }
        wheelTargets[index] = Float(max(-800, min(800, Double(wheelTargets[index]) + delta)))
        markWheelControlPending()
        guard status == .connected else { return }

        transmittedFrameCount += 1
        bleManager.sendSingleWheelSpeed(wheelID: UInt8(index), speed: wheelTargets[index])
        if wheelTargets.allSatisfy({ $0 == 0.0 }) {
            sendZeroWheelSpeeds()
        } else {
            scheduleWheelCommand()
        }
    }

    func setAllWheelTargets(_ value: Double) {
        guard controlMode == .wheelIndependent else { return }
        let clamped = Float(max(-800, min(800, value)))
        wheelTargets = Array(repeating: clamped, count: 4)
        publishDriveIntent()
        markWheelControlPending()
        if clamped == 0.0 {
            sendZeroWheelSpeeds()
        } else {
            scheduleWheelCommand()
        }
    }

    func setChassisBaseSpeed(_ value: Double) {
        guard controlMode == .chassisDiff else { return }
        chassisBaseSpeed = Float(max(Double(Self.chassisSpeedMin),
                                     min(Double(Self.chassisSpeedMax), value)))
        if abs(chassisBaseSpeed) <= 1.0 {
            sendZeroWheelSpeeds()
        } else {
            publishDriveIntent()
            scheduleChassisCommand()
        }
    }

    func setTargetYaw(_ value: Double) {
        guard value.isFinite else { return }
        targetYawDeg = Float(max(Double(Self.targetYawMin),
                                 min(Double(Self.targetYawMax), value)))
        if headingLockEnabled && abs(chassisBaseSpeed) > 1.0 {
            sendCurrentChassisSpeed()
        }
    }

    func alignTargetYawToCurrent() {
        let snapshot = telemetryStore.attitude.snapshot
        guard snapshot.displayStatus == .valid,
              snapshot.data.valid,
              snapshot.data.yawDeg.isFinite else { return }
        setTargetYaw(Double(snapshot.data.yawDeg))
    }

    func setHeadingLockEnabled(_ enabled: Bool) {
        guard controlMode == .chassisDiff else {
            headingLockEnabled = false
            return
        }
        headingLockEnabled = enabled
        if abs(chassisBaseSpeed) > 1.0 {
            sendCurrentChassisSpeed()
        }
    }

    /// Explicitly switches the STM control mode. Chassis mode is entered by
    /// App-BLE 0x2D (the current chassis target speed); independent mode is entered
    /// by App-BLE 0x15 with all four targets at zero.
    func setControlMode(_ mode: ChassisControlMode) {
        guard status == .connected else { return }
        cancelScheduledWheelCommand()
        cancelChassisCommand()
        pendingWheelControl = nil
        bleManager.cancelQueuedMotionCommands()
        controlMode = mode
        pendingControlMode = (mode, Date().addingTimeInterval(Self.wheelControlConfirmationWindow))
        wheelTargets = Array(repeating: 0, count: 4)
        masterSpeedScale = 1.0

        switch mode {
        case .chassisDiff:
            chassisBaseSpeed = 0.0
            sendCurrentChassisSpeed()
        case .wheelIndependent:
            targetYawDeg = 0.0
            headingLockEnabled = false
            markWheelControlPending()
            transmittedFrameCount += 1
            bleManager.sendWheelSpeeds(wheelTargets)
            transmittedFrameCount += 1
            bleManager.sendMasterSpeedScale(masterSpeedScale)
        }
    }

    func driveStraight() {
        setControlMode(.chassisDiff)
    }

    func setMasterSpeedScale(_ value: Double) {
        masterSpeedScale = Float(max(0, min(4, value)))
        publishDriveIntent()
        markWheelControlPending()
        guard status == .connected else { return }
        transmittedFrameCount += 1
        bleManager.sendMasterSpeedScale(masterSpeedScale)
    }

    func emergencyWheelBrake() {
        sendZeroWheelSpeeds()
    }

    func applyPIDParameters() {
        guard status == .connected else {
            pidTuning.applyStatus = .unavailable
            return
        }
        guard let values = pidTuning.values else {
            pidTuning.applyStatus = .invalid(
                pidTuning.validationMessage ?? "Invalid PID values")
            return
        }
        pidTuning.normalizeValidFields()
        guard bleManager.sendPIDParameters(values) else {
            pidTuning.applyStatus = .unavailable
            return
        }
        transmittedFrameCount += 1
        pidTuning.applyStatus = .sending
    }

    func sendZeroWheelSpeeds() {
        cancelScheduledWheelCommand()
        cancelChassisCommand()
        bleManager.cancelQueuedMotionCommands()
        wheelTargets = Array(repeating: 0, count: 4)
        chassisBaseSpeed = 0.0
        masterSpeedScale = 1.0
        targetYawDeg = 0.0
        headingLockEnabled = false
        controlMode = .wheelIndependent
        if status == .connected {
            transmittedFrameCount += 1
            _ = bleManager.sendChassisHeading(vMmS: 0.0, targetYawDeg: 0.0)
        }
        publishDriveIntent(isStop: true)
        pendingControlMode = (.wheelIndependent,
                              Date().addingTimeInterval(Self.wheelControlConfirmationWindow))
        markWheelControlPending()
        guard status == .connected else { return }
        transmittedFrameCount += 1
    }

    func ping() {
        guard status == .connected else { return }
        transmittedFrameCount += 1
        bleManager.sendPing()
    }

    func send(_ command: SmartCarProtocol.ControlCommand) {
        guard status == .connected else { return }
        transmittedFrameCount += 1
        bleManager.sendControl(command)
    }

    func stop() { sendZeroWheelSpeeds() }
    func emergencyStop() { sendZeroWheelSpeeds() }

    func updateSpeed() {
        guard status == .connected else { return }
        transmittedFrameCount += 1
        bleManager.sendSpeed(UInt8(speed.rounded()))
    }

    func updateRadarSpeed() {
        guard status == .connected else { return }
        let percent = UInt8(max(0, min(100, Int(radarSpeed.rounded()))))
        radarSpeed = Double(percent)
        transmittedFrameCount += 1
        bleManager.sendRadarSpeed(percent)
    }

    private func cancelScheduledWheelCommand() {
        wheelCommandTimer?.invalidate()
        wheelCommandTimer = nil
    }

    private func cancelChassisCommand() {
        chassisCommandTimer?.invalidate()
        chassisCommandTimer = nil
    }

    private func scheduleChassisCommand() {
        guard status == .connected, controlMode == .chassisDiff else { return }
        guard chassisCommandTimer == nil else { return }
        if chassisCommandTimer == nil {
            sendCurrentChassisSpeed()
        }
        let timer = Timer(timeInterval: Self.motionCommandInterval,
                          repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self, self.status == .connected,
                      self.controlMode == .chassisDiff,
                      abs(self.chassisBaseSpeed) > 1.0 else {
                    self?.cancelChassisCommand()
                    return
                }
                self.sendCurrentChassisSpeed()
            }
        }
        chassisCommandTimer = timer
        RunLoop.main.add(timer, forMode: .common)
    }

    private func sendCurrentChassisSpeed() {
        guard status == .connected else { return }
        publishDriveIntent()
    }

    private func markWheelControlPending() {
        guard status == .connected else {
            pendingWheelControl = nil
            return
        }
        pendingWheelControl = PendingWheelControl(
            rawTargets: wheelTargets,
            masterScale: masterSpeedScale,
            expiresAt: Date().addingTimeInterval(Self.wheelControlConfirmationWindow)
        )
    }

    private func acceptWheelControlTelemetry(_ status: WheelControlStatus) {
        // The STM32 is authoritative after a mode command reaches the chassis.
        if let pendingMode = pendingControlMode {
            if pendingMode.mode == status.mode {
                pendingControlMode = nil
            } else if Date() < pendingMode.expiresAt {
                return
            } else {
                pendingControlMode = nil
            }
        }
        controlMode = status.mode
        if let pending = pendingWheelControl {
            if status.rawTargets.count == pending.rawTargets.count,
               status.rawTargets.enumerated().allSatisfy({ index, target in
                   abs(target - pending.rawTargets[index]) <= 0.5
               }),
               abs(status.masterScale - pending.masterScale) <= 0.005 {
                pendingWheelControl = nil
            } else if Date() < pending.expiresAt {
                return
            } else {
                pendingWheelControl = nil
            }
        }
        wheelTargets = status.rawTargets
        masterSpeedScale = status.masterScale
        publishDriveIntent()
    }

    private func scheduleWheelCommand() {
        guard status == .connected, controlMode == .wheelIndependent,
              wheelTargets.contains(where: { abs($0) > 1.0 }) else { return }
        guard wheelCommandTimer == nil else { return }
        if wheelCommandTimer == nil {
            sendCurrentWheelSpeeds()
        }
        let timer = Timer(timeInterval: Self.motionCommandInterval, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self, self.status == .connected,
                      self.controlMode == .wheelIndependent,
                      self.wheelTargets.contains(where: { abs($0) > 1.0 }) else {
                    self?.cancelScheduledWheelCommand()
                    return
                }
                self.sendCurrentWheelSpeeds()
            }
        }
        wheelCommandTimer = timer
        RunLoop.main.add(timer, forMode: .common)
    }

    private func sendCurrentWheelSpeeds() {
        guard status == .connected else {
            cancelScheduledWheelCommand()
            return
        }
        publishDriveIntent()
    }

    func refreshLogs() {
        logSnapshot = bleManager.decodedMessages
    }

    private func flushDebugMetrics() {
        if pendingReceivedFrameCount != 0 {
            receivedFrameCount += pendingReceivedFrameCount
            pendingReceivedFrameCount = 0
        }
        if pendingLastPacketType != lastPacketType {
            lastPacketType = pendingLastPacketType
        }
        diagnosticsStore.sync(
            transmittedFrames: transmittedFrameCount,
            receivedFrames: receivedFrameCount,
            decodeFailures: bleManager.decodeFailureCount,
            droppedMessages: bleManager.droppedMessageCount
        )
    }

    private func publishDriveIntent(isStop: Bool = false) {
        let intent = MotionIntent(
            mode: controlMode,
            wheelTargets: wheelTargets,
            chassisBaseSpeed: chassisBaseSpeed,
            masterScale: masterSpeedScale,
            targetYawDeg: targetYawDeg,
            headingLockEnabled: headingLockEnabled,
            isStop: isStop
        )
        driveStore.update(intent)
        bleManager.updateMotionIntent(intent)
    }

    private static func packetType(for message: DecodedMessage) -> String {
        switch message {
        case .control: return "CONTROL"
        case .status: return "STATUS"
        case .ping: return "PING"
        case .ack: return "ACK"
        case .imuStatus: return "IMU_STATUS"
        case .attitude: return "ATTITUDE"
        case .dualAttitude: return "DUAL_ATTITUDE"
        case .imuCalibrationStatus: return "IMU_CAL_STATUS"
        case .imuCalibrationBias: return "IMU_CAL_BIAS"
        case .imuCalibrationResult: return "IMU_CAL_RESULT"
        case .calibrationEvent: return "CAL_EVENT"
        case .imuTelemetry: return "IMU_TELEMETRY"
        case .dualIMUStatus: return "DUAL_IMU_STATUS"
        case .radarStatus: return "RADAR_STATUS"
        case .wheelSpeedStatus: return "WHEEL_SPEED_STATUS"
        case .wheelControlStatus: return "WHEEL_CONTROL_STATUS"
        case .powerStatus: return "POWER_STATUS"
        case .chassisState: return "CHASSIS_STATE"
        }
    }
}
