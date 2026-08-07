import Foundation

@MainActor
public final class RemoteViewModel: ObservableObject {
    @Published public private(set) var status = VehicleStatus()
    @Published public private(set) var bleState: BLEState = .idle
    @Published public private(set) var devices: [BLEDevice] = []
    @Published public var joystick = JoystickViewModel()
    @Published public private(set) var imuCalibration = IMUCalibrationStatus()
    private let transport: VehicleTransport
    private let sessionID = UInt32.random(in: 1...UInt32.max)
    private var sequence: UInt32 = 0
    private var packetTask: Task<Void, Never>?

    public init(transport: VehicleTransport = BLEManager()) {
        self.transport = transport
        packetTask = Task { [weak self, transport] in
            for await packet in transport.incomingPackets {
                guard !Task.isCancelled else { return }
                self?.receive(packet)
            }
        }
    }
    public func scan() { transport.startScanning(); bleState = transport.state; devices = transport.devices; status.link = .discovering }
    public func connect(_ device: BLEDevice) { transport.connect(to: device); bleState = transport.state; status.link = .connecting }
    public func stop() { send(.stop); status.link = .stopping }
    public func emergencyStop() { send(.emergencyStop); status.link = .emergencyStopLatched }
    public func sendJoystick() { send(.manualJoystick(joystick.intent)) }
    public func sendPad(_ direction: Direction) { send(.manualPad(direction: direction)) }
    private func send(_ intent: ControlIntent) {
        guard intent.kind == .stop || intent.kind == .emergencyStop || status.canAcceptManualInput else { return }
        sequence &+= 1
        let packet = Message.packet(for: intent, sessionID: sessionID, sequence: sequence)
        Task { @MainActor [transport] in try? await transport.send(packet) }
    }

    private func receive(_ packet: Packet) {
        guard packet.type == 0x12,
              packet.payload.count == 7 else { return }
        guard let state = IMUCalibrationState(rawValue: packet.payload[0]) else { return }
        guard let sampleMode = IMUCalibrationSampleMode(rawValue: packet.payload[1]) else { return }
        let currentPWM = packet.payload[2]
        let sampleProgress = UInt16(packet.payload[3]) | (UInt16(packet.payload[4]) << 8)
        let totalProgress = packet.payload[5]
        let errorCode = packet.payload[6]
        imuCalibration = IMUCalibrationStatus(
            state: state,
            sampleMode: sampleMode,
            totalProgress: totalProgress,
            currentPWM: min(currentPWM, 100),
            sampleProgress: sampleProgress,
            errorCode: errorCode,
            lastUpdatedAt: Date()
        )
    }
}
