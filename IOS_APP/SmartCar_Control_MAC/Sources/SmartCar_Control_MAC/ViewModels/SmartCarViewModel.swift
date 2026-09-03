import Combine
import Foundation
import AppKit

enum DeveloperPage: Equatable {
    case overview
    case loggerSTM
    case loggerS3
}

struct JoystickIntent: Equatable {
    let horizontal: Float
    let vertical: Float

    static let neutral = JoystickIntent(horizontal: 0.0, vertical: 0.0)

    var isNeutral: Bool {
        horizontal == 0.0 && vertical == 0.0
    }
}

@MainActor
final class SmartCarViewModel: ObservableObject {
    private static let angleUnitDefaultsKey = "smartcar.angleUnit"
    private static let joystickMaximumSpeed: Float = 800.0
    private static let chassisTrackWidthMM: Float = 193.0
    private static let joystickSteeringCurve = 1.5

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
    private var activeJoystickIntent: JoystickIntent?

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
                self.activeJoystickIntent = nil
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
        activeJoystickIntent = nil
        bleManager.disconnect()
    }

    func setWheelTarget(index: Int, value: Double) {
        guard wheelTargets.indices.contains(index) else { return }
        activeJoystickCommand = nil
        activeJoystickIntent = nil
        wheelTargets[index] = Float(max(-Double(Self.joystickMaximumSpeed),
                                         min(Double(Self.joystickMaximumSpeed), value)))
        if wheelTargets.allSatisfy({ $0 == 0.0 }) {
            sendZeroWheelSpeeds()
        } else {
            scheduleWheelCommand()
        }
    }

    func setAllWheelTargets(_ value: Double) {
        activeJoystickCommand = nil
        activeJoystickIntent = nil
        let clamped = Float(max(-Double(Self.joystickMaximumSpeed),
                                min(Double(Self.joystickMaximumSpeed), value)))
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
        activeJoystickIntent = nil
        wheelTargets = Array(repeating: 0, count: 4)
        guard status == .connected else { return }
        sendCurrentWheelSpeeds()
    }

    func setJoystickInput(_ input: JoystickIntent) {
        guard status == .connected else { return }

        let sanitized = JoystickIntent(
            horizontal: Self.clampJoystickAxis(input.horizontal),
            vertical: Self.clampJoystickAxis(input.vertical)
        )
        guard !sanitized.isNeutral else {
            sendZeroWheelSpeeds()
            return
        }

        activeJoystickCommand = nil
        activeJoystickIntent = sanitized
        applyJoystickIntent()
    }

    func setJoystickCommand(_ command: SmartCarProtocol.ControlCommand?) {
        guard status == .connected else { return }
        guard let command else {
            sendZeroWheelSpeeds()
            return
        }

        activeJoystickIntent = nil
        activeJoystickCommand = command
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

        applyWheelTargets(targets)
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

    // All UI stop actions use the wheel-stop primitive supported by the S3
    // bridge. The legacy CONTROL/0x01 frame is not handled by the current
    // command bridge and therefore cannot stop the vehicle.
    func stop() { sendZeroWheelSpeeds() }
    func emergencyStop() { sendZeroWheelSpeeds() }

    func updateSpeed() {
        if activeJoystickIntent != nil {
            applyJoystickIntent()
        } else if let activeJoystickCommand {
            setJoystickCommand(activeJoystickCommand)
        }
    }

    func updateRadarSpeed() {
        guard status == .connected else { return }
        let percent = UInt8(max(0, min(100, Int(radarSpeed.rounded()))))
        radarSpeed = Double(percent)
        transmittedFrameCount += 1
        bleManager.sendRadarSpeed(percent)
    }

    private func scheduleWheelCommand() {
        wheelHeartbeatTimer?.invalidate()
        wheelHeartbeatTimer = nil
        guard wheelCommandTimer == nil else { return }

        let timer = Timer(timeInterval: 0.05, repeats: false) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self else { return }
                self.wheelCommandTimer = nil
                guard self.status == .connected,
                      self.wheelTargets.contains(where: { $0 != 0.0 }) else {
                    self.stopWheelHeartbeat()
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
        let percent = speed.isFinite ? min(max(speed, 0.0), 100.0) : 0.0
        return Float(percent / 100.0) * Self.joystickMaximumSpeed
    }

    private func applyJoystickIntent() {
        guard let activeJoystickIntent else { return }
        applyWheelTargets(Self.wheelTargets(for: activeJoystickIntent,
                                            speed: joystickSpeed))
    }

    private func applyWheelTargets(_ targets: [Float]) {
        wheelTargets = targets
        if targets.allSatisfy({ $0 == 0.0 }) {
            stopWheelHeartbeat()
            sendCurrentWheelSpeeds()
        } else {
            scheduleWheelCommand()
        }
    }

    private static func clampJoystickAxis(_ value: Float) -> Float {
        guard value.isFinite else { return 0.0 }
        return min(max(value, -1.0), 1.0)
    }

    private static func wheelTargets(for input: JoystickIntent,
                                     speed: Float) -> [Float] {
        let horizontal = clampJoystickAxis(input.horizontal)
        let vertical = clampJoystickAxis(input.vertical)
        let safeSpeed = speed.isFinite
            ? min(max(speed, 0.0), joystickMaximumSpeed)
            : 0.0
        let halfTrack = 0.5 * chassisTrackWidthMM
        let linearSpeed = -vertical * safeSpeed
        let turnFraction = Float(
            pow(Double(abs(horizontal)), joystickSteeringCurve)
        )
        let angularSpeed: Float
        if horizontal < 0.0 {
            angularSpeed = turnFraction * safeSpeed / halfTrack
        } else if horizontal > 0.0 {
            angularSpeed = -turnFraction * safeSpeed / halfTrack
        } else {
            angularSpeed = 0.0
        }

        /* Keep horizontal polarity unchanged while reversing; only the linear
         * component changes sign with the vertical joystick input. */
        var rightSpeed = linearSpeed + angularSpeed * halfTrack
        var leftSpeed = linearSpeed - angularSpeed * halfTrack
        let peak = max(abs(rightSpeed), abs(leftSpeed))
        if peak > safeSpeed, peak > 0.0 {
            let scale = safeSpeed / peak
            rightSpeed *= scale
            leftSpeed *= scale
        }
        return [rightSpeed, rightSpeed, leftSpeed, leftSpeed]
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
