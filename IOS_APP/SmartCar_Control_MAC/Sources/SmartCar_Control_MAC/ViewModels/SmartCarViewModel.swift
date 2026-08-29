import Combine
import Foundation
import AppKit

enum DeveloperPage: Equatable {
    case overview
    case loggerSTM
    case loggerS3
}

@MainActor
final class SmartCarViewModel: ObservableObject {
    private static let angleUnitDefaultsKey = "smartcar.angleUnit"
    private static let joystickMaximumSpeed: Float = 800.0

    let bleManager: BLEManager
    let telemetryStore: TelemetryStore
    let calibrationViewModel: CalibrationViewModel
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
    @Published private(set) var status: BLEConnectionStatus
    @Published private(set) var discoveredDeviceName: String
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
    private var wheelHeartbeatTimer: Timer?
    private var lifecycleObservers: [NSObjectProtocol] = []
    private var activeJoystickCommand: SmartCarProtocol.ControlCommand?

    var decodedMessages: [DecodedMessageRecord] { bleManager.decodedMessages }
    var stmLogStore: DeviceLogStore { bleManager.stmLogStore }
    var s3LogStore: DeviceLogStore { bleManager.s3LogStore }

    init(bleManager: BLEManager? = nil) {
        let manager = bleManager ?? BLEManager()
        self.bleManager = manager
        self.telemetryStore = manager.telemetryStore
        self.calibrationViewModel = CalibrationViewModel(telemetryStore: manager.telemetryStore)
        self.status = manager.status
        self.discoveredDeviceName = manager.discoveredDeviceName
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
            if status != .connected {
                self.stopWheelHeartbeat()
                self.wheelTargets = Array(repeating: 0, count: 4)
                self.activeJoystickCommand = nil
                self.pidTuning.applyStatus = .unavailable
            }
        }
        manager.onDeviceNameChange = { [weak self] name in
            self?.discoveredDeviceName = name
        }
        manager.onErrorChange = { [weak self] error in
            self?.lastError = error
        }
        manager.onDecodedMessage = { [weak self] record in
            guard let self else { return }
            self.pendingReceivedFrameCount += 1
            self.pendingLastPacketType = Self.packetType(for: record.message)
            if case .ack(let payload) = record.message,
               payload.count >= 2,
               payload[0] == SmartCarProtocol.FrameType.pidParams.rawValue {
                self.pidTuning.applyStatus = payload[1] == 0
                    ? .applied : .rejected
            }
        }

        debugTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.flushDebugMetrics() }
        }
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
    }

    deinit {
        debugTimer?.invalidate()
        wheelCommandTimer?.invalidate()
        wheelHeartbeatTimer?.invalidate()
        for observer in lifecycleObservers { NotificationCenter.default.removeObserver(observer) }
    }

    func scan() { bleManager.startScanning() }
    func connect() { bleManager.connectToDevice() }
    func disconnect() {
        stopWheelHeartbeat()
        wheelTargets = Array(repeating: 0, count: 4)
        activeJoystickCommand = nil
        bleManager.disconnect()
    }

    func setWheelTarget(index: Int, value: Double) {
        guard wheelTargets.indices.contains(index) else { return }
        activeJoystickCommand = nil
        wheelTargets[index] = Float(max(-800, min(800, value)))
        if wheelTargets.allSatisfy({ $0 == 0.0 }) {
            sendZeroWheelSpeeds()
        } else {
            scheduleWheelCommand()
        }
    }

    func setAllWheelTargets(_ value: Double) {
        activeJoystickCommand = nil
        let clamped = Float(max(-800, min(800, value)))
        wheelTargets = Array(repeating: clamped, count: 4)
        if clamped == 0.0 {
            sendZeroWheelSpeeds()
        } else {
            scheduleWheelCommand()
        }
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
        stopWheelHeartbeat()
        activeJoystickCommand = nil
        wheelTargets = Array(repeating: 0, count: 4)
        guard status == .connected else { return }
        transmittedFrameCount += 1
        bleManager.sendWheelSpeeds(wheelTargets)
    }

    func setJoystickCommand(_ command: SmartCarProtocol.ControlCommand?) {
        guard status == .connected else { return }
        guard let command else {
            sendZeroWheelSpeeds()
            return
        }

        let speed = joystickSpeed
        let targets: [Float]
        switch command {
        case .moveForward:
            targets = [speed, speed, speed, speed]
        case .moveBack:
            targets = [-speed, -speed, -speed, -speed]
        case .turnLeft:
            targets = [speed, speed, -speed, -speed]
        case .turnRight:
            targets = [-speed, -speed, speed, speed]
        case .stop, .speedControl:
            sendZeroWheelSpeeds()
            return
        }

        activeJoystickCommand = command
        wheelTargets = targets
        if targets.allSatisfy({ $0 == 0.0 }) {
            sendZeroWheelSpeeds()
        } else {
            scheduleWheelCommand()
        }
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

    func stop() { send(.stop) }
    func emergencyStop() { send(.stop) }

    func updateSpeed() {
        guard let activeJoystickCommand else { return }
        setJoystickCommand(activeJoystickCommand)
    }

    func updateRadarSpeed() {
        guard status == .connected else { return }
        let percent = UInt8(max(0, min(100, Int(radarSpeed.rounded()))))
        radarSpeed = Double(percent)
        transmittedFrameCount += 1
        bleManager.sendRadarSpeed(percent)
    }

    private func scheduleWheelCommand() {
        wheelCommandTimer?.invalidate()
        let timer = Timer(timeInterval: 0.05, repeats: false) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self, self.status == .connected,
                      self.wheelTargets.contains(where: { $0 != 0.0 }) else {
                    self?.stopWheelHeartbeat()
                    return
                }
                self.sendCurrentWheelSpeeds()
                self.startWheelHeartbeat()
            }
        }
        wheelCommandTimer = timer
        RunLoop.main.add(timer, forMode: .common)
    }

    private func sendCurrentWheelSpeeds() {
        guard status == .connected else {
            stopWheelHeartbeat()
            return
        }
        transmittedFrameCount += 1
        bleManager.sendWheelSpeeds(wheelTargets)
    }

    private func startWheelHeartbeat() {
        wheelHeartbeatTimer?.invalidate()
        let timer = Timer(timeInterval: 0.1, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self, self.status == .connected else {
                    self?.stopWheelHeartbeat()
                    return
                }
                guard self.wheelTargets.contains(where: { $0 != 0.0 }) else {
                    self.stopWheelHeartbeat()
                    return
                }
                self.sendCurrentWheelSpeeds()
            }
        }
        wheelHeartbeatTimer = timer
        RunLoop.main.add(timer, forMode: .common)
    }

    private func stopWheelHeartbeat() {
        wheelCommandTimer?.invalidate()
        wheelCommandTimer = nil
        wheelHeartbeatTimer?.invalidate()
        wheelHeartbeatTimer = nil
    }

    private var joystickSpeed: Float {
        let percent = min(max(speed, 0.0), 100.0)
        return Float(percent / 100.0) * Self.joystickMaximumSpeed
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
        case .powerStatus: return "POWER_STATUS"
        }
    }
}
