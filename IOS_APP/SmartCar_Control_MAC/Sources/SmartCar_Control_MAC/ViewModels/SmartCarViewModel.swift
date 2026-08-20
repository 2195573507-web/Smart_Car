import Combine
import Foundation

enum DeveloperPage: Equatable {
    case overview
    case loggerSTM
    case loggerS3
}

@MainActor
final class SmartCarViewModel: ObservableObject {
    private static let angleUnitDefaultsKey = "smartcar.angleUnit"

    let bleManager: BLEManager
    let telemetryStore: TelemetryStore
    let calibrationViewModel: CalibrationViewModel

    @Published var mode: AppMode = .control
    @Published var developerPage: DeveloperPage = .overview
    @Published var angleUnit: AngleUnit {
        didSet {
            UserDefaults.standard.set(angleUnit.rawValue, forKey: Self.angleUnitDefaultsKey)
        }
    }
    @Published var speed: Double = 50
    @Published var radarSpeed: Double = 0
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
            self?.status = status
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
        }

        debugTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.flushDebugMetrics() }
        }
    }

    deinit { debugTimer?.invalidate() }

    func scan() { bleManager.startScanning() }
    func connect() { bleManager.connectToDevice() }
    func disconnect() { bleManager.disconnect() }

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
        }
    }
}
