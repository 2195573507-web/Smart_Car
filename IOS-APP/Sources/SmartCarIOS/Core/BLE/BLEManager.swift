@preconcurrency import CoreBluetooth
import Foundation
import SmartCarAppCore

private enum BLEReceiveEvent {
    case message(DecodedMessageRecord)
    case session(AppBLEFrame)
    case decodeFailure(String)
    case drops(Int)
}

private struct BLEQueuedWrite {
    let data: Data
    let frameID: UInt64
    let motionType: UInt8?
    let writeType: CBCharacteristicWriteType
}

/// Selects the BLE ATT write procedure without changing the App-BLE frame.
/// Only replaceable, nonzero chassis-motion frames are eligible for the
/// no-response procedure. Stops and every reliable command stay acknowledged.
enum BLEWriteTransportPolicy {
    static func writeType(
        for frame: Data,
        motionType: UInt8?,
        supportsWriteWithoutResponse: Bool
    ) -> CBCharacteristicWriteType {
        guard supportsWriteWithoutResponse,
              isNonzeroLatestMotion(frame, motionType: motionType) else {
            return .withResponse
        }
        return .withoutResponse
    }

    private static func isNonzeroLatestMotion(_ frame: Data, motionType: UInt8?) -> Bool {
        guard let motionType,
              frame.count > 2,
              frame[2] == motionType,
              let payload = payload(in: frame) else {
            return false
        }
        switch motionType {
        case SmartCarProtocol.FrameType.wheelSpeedCommand.rawValue:
            return hasNonzeroFloat32(payload, valueCount: 4)
        case SmartCarProtocol.FrameType.chassisSpeedCommand.rawValue:
            guard payload.count == 16 else { return false }
            return hasNonzeroFloat32(payload, valueCount: 2)
        case SmartCarProtocol.FrameType.chassisHeadingCommand.rawValue:
            guard payload.count == 12 else { return false }
            return hasNonzeroFloat32(payload, valueCount: 2) ||
                payload[8...].contains { $0 != 0 }
        default:
            return false
        }
    }

    private static func hasNonzeroFloat32(_ payload: Data, valueCount: Int) -> Bool {
        let byteCount = valueCount * MemoryLayout<UInt32>.size
        guard payload.count >= byteCount else {
            return false
        }
        for offset in stride(from: 0, to: byteCount, by: MemoryLayout<UInt32>.size) {
            let bits = UInt32(payload[offset]) |
                (UInt32(payload[offset + 1]) << 8) |
                (UInt32(payload[offset + 2]) << 16) |
                (UInt32(payload[offset + 3]) << 24)
            if Float(bitPattern: bits) != 0.0 {
                return true
            }
        }
        return false
    }

    private static func payload(in frame: Data) -> Data? {
        guard frame.count >= 8,
              frame[0] == SmartCarProtocol.head,
              frame[1] == SmartCarProtocol.version,
              frame[frame.count - 1] == SmartCarProtocol.tail else {
            return nil
        }
        let length = Int(frame[3]) | (Int(frame[4]) << 8)
        guard frame.count == length + 8 else { return nil }
        return Data(frame[5..<(5 + length)])
    }
}

/// Invalidates delayed timeout work after the corresponding write completes or
/// a newer write takes its place. Kept independent from CoreBluetooth so the
/// stale-timeout rule remains unit-testable.
struct BLEWriteResponseWatchdog {
    private(set) var generation: UInt64 = 0

    mutating func arm() -> UInt64 {
        generation &+= 1
        return generation
    }

    mutating func invalidate() {
        generation &+= 1
    }

    func isCurrent(_ token: UInt64) -> Bool {
        token == generation
    }
}

private final class BLEReceivePipeline: @unchecked Sendable {
    let queue = DispatchQueue(
        label: "SmartCar_Control_MAC.ble.receive",
        qos: .userInitiated
    )
    private let ingressQueue = DispatchQueue(
        label: "SmartCar_Control_MAC.ble.receive.ingress",
        qos: .userInitiated
    )
    private let worker = AppBLEInboundWorker()
    // A notification callback must remain non-blocking. Limit the number of
    // in-flight actor submissions instead of allowing suspended Tasks to grow
    // without bound when the reserved critical mailbox applies backpressure.
    private let submitSlots = DispatchSemaphore(value: 256)

    func reset() {
        Task { await self.worker.reset() }
    }

    func submit(_ data: Data) {
        ingressQueue.async {
            self.submitSlots.wait()
            Task {
                defer { self.submitSlots.signal() }
                await self.worker.submit(data)
            }
        }
    }

    func drain(handler: @escaping ([BLEReceiveEvent]) -> Void) {
        Task {
            let batch = await self.worker.drain()
            guard !batch.isEmpty else { return }
            handler(Self.decode(batch))
        }
    }

    private static func decode(_ batch: AppBLEInboundBatch) -> [BLEReceiveEvent] {
        let items = batch.critical + batch.telemetry + batch.other
        var events: [BLEReceiveEvent] = []
        events.reserveCapacity(items.count)
        let dropped = batch.droppedTelemetry + batch.droppedOther
        if dropped > 0 {
            events.append(.drops(dropped))
        }
        for item in items {
            let frame = item.frame
            if frame.version == AppBLEVersion.v2.rawValue {
                events.append(.session(frame))
                continue
            }
            do {
                events.append(.message(
                    DecodedMessageRecord(
                        message: try DecodedMessage(frame: SmartCarProtocol.Frame(
                            raw: frame.raw,
                            version: frame.version,
                            type: frame.type,
                            payload: frame.payload
                        )),
                        receivedAt: item.receivedAt
                    )
                ))
            } catch {
                events.append(.decodeFailure(error.localizedDescription))
            }
        }
        return events
    }
}

@MainActor
final class BLEManager: NSObject {
    nonisolated static let targetDeviceName = "SmartCar_S3"
    nonisolated static let serviceUUID = CBUUID(string: "0000FFE0-0000-1000-8000-00805F9B34FB")
    nonisolated static let rxCharacteristicUUID = CBUUID(string: "0000FFE1-0000-1000-8000-00805F9B34FB")
    nonisolated static let txCharacteristicUUID = CBUUID(string: "0000FFE2-0000-1000-8000-00805F9B34FB")

    private(set) var status: BLEConnectionStatus = .disconnected
    private(set) var discoveredDeviceName = targetDeviceName
    private(set) var discoveredRSSI: Int?
    private(set) var lastError: BLEUserFacingError?
    private(set) var lastPacketReceivedAt: Date?
    private(set) var decodeFailureCount = 0
    private(set) var droppedMessageCount = 0

    let telemetryStore: TelemetryStore
    var onStatusChange: ((BLEConnectionStatus) -> Void)?
    var onDecodedMessage: ((DecodedMessageRecord) -> Void)?
    var onDeviceNameChange: ((String) -> Void)?
    var onRSSIChange: ((Int?) -> Void)?
    var onErrorChange: ((BLEUserFacingError?) -> Void)?

    private let central: CBCentralManager
    private nonisolated let receivePipeline: BLEReceivePipeline
    private let telemetryReducer = AppTelemetryReducer()
    private var peripheral: CBPeripheral?
    private var writeCharacteristic: CBCharacteristic?
    private var notifyCharacteristic: CBCharacteristic?
    private var scanWhenPoweredOn = false
    private var shouldReconnect = true
    private var reconnectAttempt = 0
    private var reconnectWorkItem: DispatchWorkItem?
    private var writeQueue: [BLEQueuedWrite] = []
    private var outboundScheduler = AppBLEOutboundScheduler(reliableCapacity: 12)
    private var pendingMotionIntent: MotionIntent?
    private var pendingStopIntent: MotionIntent?
    private var writeInFlight = false
    private var writeResponseTimeoutWorkItem: DispatchWorkItem?
    private var writeResponseWatchdog = BLEWriteResponseWatchdog()
    private var nextFrameID: UInt64 = 0
    private var session = AppBLESession()
    private var sessionTimer: Timer?
    private var receiveFlushTimer: Timer?
    private var sessionFallbackWorkItem: DispatchWorkItem?
    private var lastSessionActivityAt: Date?
    private var disconnectAfterStop = false
    private var disconnectStopFrameID: UInt64?
    private var disconnectStopTimeoutWorkItem: DispatchWorkItem?

    private(set) var sessionMode: AppBLESessionMode = .idle
    private(set) var lastCommandAck: AppBLEV2CommandAck?
    private(set) var sessionTimeoutCount = 0

    override convenience init() {
        self.init(telemetryStore: TelemetryStore())
    }

    init(telemetryStore: TelemetryStore) {
        let receivePipeline = BLEReceivePipeline()
        self.telemetryStore = telemetryStore
        self.receivePipeline = receivePipeline
        central = CBCentralManager(delegate: nil, queue: receivePipeline.queue)
        super.init()
        central.delegate = self
        startReceiveFlushTimer()
    }

    deinit {
        receiveFlushTimer?.invalidate()
        disconnectStopTimeoutWorkItem?.cancel()
        writeResponseTimeoutWorkItem?.cancel()
    }

    func startScanning() {
        guard central.state == .poweredOn else {
            scanWhenPoweredOn = true
            setError(nil)
            return
        }

        discoveredRSSI = nil
        onRSSIChange?(nil)
        receivePipeline.reset()
        Task { await telemetryReducer.reset() }
        outboundScheduler.removeAll()
        pendingMotionIntent = nil
        pendingStopIntent = nil
        writeQueue.removeAll(keepingCapacity: true)
        writeInFlight = false
        cancelWriteResponseTimeout()
        writeCharacteristic = nil
        notifyCharacteristic = nil
        clearDeferredDisconnect()
        resetSession()
        setError(nil)
        updateStatus(.scanning)
        // Scan by name so discovery also works when the peripheral omits FFE0 from its advertisement.
        central.scanForPeripherals(withServices: nil, options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
    }

    func stopScanning() {
        scanWhenPoweredOn = false
        central.stopScan()
        if status == .scanning {
            updateStatus(.disconnected)
        }
    }

    func connectToDevice() {
        guard let peripheral else {
            setError(.deviceNotFound)
            return
        }

        stopScanning()
        shouldReconnect = true
        reconnectWorkItem?.cancel()
        reconnectAttempt = 0
        updateStatus(.connecting)
        writeCharacteristic = nil
        notifyCharacteristic = nil
        self.peripheral = peripheral
        peripheral.delegate = self
        central.connect(peripheral, options: nil)
    }

    func disconnect() {
        shouldReconnect = false
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        guard let peripheral else {
            clearOutboundState()
            resetSession()
            return
        }

        // A link that has not completed GATT setup cannot acknowledge a safe
        // stop write. In that case discard local state and disconnect directly.
        guard isGATTReady else {
            clearOutboundState()
            resetSession()
            central.cancelPeripheralConnection(peripheral)
            return
        }

        disconnectAfterStop = true
        disconnectStopFrameID = nil
        cancelQueuedMotionCommands()
        sendWheelSpeeds([0, 0, 0, 0])
        scheduleDisconnectAfterStopTimeout()
    }

    func sendPing() {
        sendFrame(SmartCarProtocol.ping())
    }

    func sendControl(_ command: SmartCarProtocol.ControlCommand) {
        sendFrame(SmartCarProtocol.control(command))
    }

    func sendSpeed(_ speed: UInt8) {
        sendFrame(SmartCarProtocol.control(.speedControl, data: Data([speed])))
    }

    func sendRadarSpeed(_ speed: UInt8) {
        let clampedSpeed = min(speed, 100)
        sendFrame(SmartCarProtocol.encode(type: .radarPWMControl, payload: Data([clampedSpeed])))
    }

    func sendWheelSpeeds(_ speeds: [Float]) {
        guard speeds.count == 4, speeds.allSatisfy(\.isFinite) else { return }
        _ = sendFrame(SmartCarProtocol.encode(type: .wheelSpeedCommand,
                                              payload: Self.floatPayload(speeds)))
    }

    /// Stores only the latest motion state. The wire frame is built in
    /// `stageNextScheduledFrame`, immediately before the write queue consumes
    /// it, so timer ticks never accumulate stale encoded packets.
    func updateMotionIntent(_ intent: MotionIntent) {
        guard isGATTReady, !disconnectAfterStop else {
            return
        }
        if intent.isStop {
            cancelQueuedMotionCommands()
            pendingStopIntent = intent
            pendingMotionIntent = nil
        } else if pendingStopIntent == nil {
            pendingMotionIntent = intent
        }
        pumpWriteQueue()
    }

    /// Sends App-BLE 0x2D, which the S3 maps to SCBP-CAN 0x114.
    /// The payload is exactly two little-endian Float32 values plus eight
    /// zero reserved bytes.
    func sendChassisSpeed(baseSpeed: Float, yawRate: Float) {
        guard baseSpeed.isFinite, yawRate.isFinite else { return }
        var payload = Self.floatPayload([baseSpeed, yawRate])
        payload.append(Data(repeating: 0, count: 8))
        guard payload.count == 16 else { return }
        _ = sendFrame(SmartCarProtocol.encode(type: .chassisSpeedCommand,
                                              payload: payload))
    }

    @discardableResult
    func sendChassisHeading(vMmS: Float, targetYawDeg: Float) -> Bool {
        guard let payload = SmartCarProtocol.chassisHeadingPayload(
            vMmS: vMmS, targetYawDeg: targetYawDeg) else {
            return false
        }
        return sendFrame(SmartCarProtocol.encode(
            type: .chassisHeadingCommand, payload: payload))
    }

    func sendSingleWheelSpeed(wheelID: UInt8, speed: Float) {
        guard wheelID < 4, speed.isFinite else { return }
        var payload = Data([wheelID])
        payload.append(Self.floatPayload([speed]))
        _ = sendFrame(SmartCarProtocol.encode(type: .wheelSpeedSingleCommand,
                                              payload: payload))
    }

    func sendMasterSpeedScale(_ scale: Float) {
        guard scale.isFinite else { return }
        _ = sendFrame(SmartCarProtocol.encode(type: .masterSpeedCommand,
                                              payload: Self.floatPayload([scale])))
    }

    @discardableResult
    func sendPIDParameters(_ values: PIDParameterValues) -> Bool {
        sendFrame(SmartCarProtocol.encode(type: .pidParams,
                                          payload: Self.floatPayload([
                                              values.kp, values.ki,
                                              values.kd, values.maxAccel
                                          ])))
    }

    @discardableResult
    private func sendFrame(_ data: Data) -> Bool {
        guard isGATTReady else {
            setError(.notConnected)
            return false
        }
        let motionType = Self.motionType(in: data)
        let isStop = Self.isStopFrame(data, motionType: motionType)
        guard !disconnectAfterStop || isStop else {
            return false
        }
        if isStop {
            cancelQueuedMotionCommands()
        }
        guard enqueueFrame(data, motionType: motionType, isStop: isStop) else {
            setError(.system("BLE outbound queue full"))
            return false
        }
        pumpWriteQueue()
        return true
    }

    /// Reliable commands remain ordered behind one ATT write response. Latest
    /// nonzero motion writes use CoreBluetooth's no-response backpressure and
    /// are removed before submission because they have no completion callback.
    private func pumpWriteQueue() {
        guard !writeInFlight, isGATTReady,
              let peripheral,
              let writeCharacteristic else { return }
        while !writeInFlight {
            if writeQueue.isEmpty {
                stageNextScheduledFrame()
            }
            guard let next = writeQueue.first else { return }
            switch next.writeType {
            case .withResponse:
                writeInFlight = true
                armWriteResponseTimeout(for: next)
                peripheral.writeValue(next.data, for: writeCharacteristic, type: .withResponse)
            case .withoutResponse:
                guard peripheral.canSendWriteWithoutResponse else { return }
                writeQueue.removeFirst()
                peripheral.writeValue(next.data, for: writeCharacteristic, type: .withoutResponse)
            @unknown default:
                setError(.system("Unsupported BLE write type"))
                return
            }
        }
    }

    private func finishWrite(error: Error?) {
        cancelWriteResponseTimeout()
        guard !writeQueue.isEmpty else {
            writeInFlight = false
            return
        }
        let completedFrameID = writeQueue[0].frameID
        writeQueue.removeSubrange(0...0)
        writeInFlight = false
        let completedWholeFrame = !writeQueue.contains { $0.frameID == completedFrameID }
        if let error {
            writeQueue.removeAll { $0.frameID == completedFrameID }
            setError(.system(error.localizedDescription))
        }
        if disconnectAfterStop,
           disconnectStopFrameID == completedFrameID,
           error != nil || completedWholeFrame {
            completeDeferredDisconnect()
            return
        }
        pumpWriteQueue()
    }

    /// Drop queued motion frames that have not reached the peripheral yet.
    /// The current in-flight frame is retained; the replacement mode command
    /// is then sent after it, preserving wire order.
    func cancelQueuedMotionCommands() {
        outboundScheduler.cancelMotion()
        pendingMotionIntent = nil
        pendingStopIntent = nil
        let inFlightFrameID = writeInFlight ? writeQueue.first?.frameID : nil
        let prefixCount: Int
        if let inFlightFrameID {
            prefixCount = writeQueue.prefix { $0.frameID == inFlightFrameID }.count
        } else {
            prefixCount = 0
        }
        let prefix = Array(writeQueue.prefix(prefixCount))
        let pending = writeQueue.dropFirst(prefixCount).filter { $0.motionType == nil }
        writeQueue = prefix + pending
    }

    @discardableResult
    private func enqueueFrame(_ frame: Data, motionType: UInt8?, isStop: Bool = false) -> Bool {
        guard !frame.isEmpty else { return false }
        if isStop {
            outboundScheduler.replaceMotion(frame, legacyType: motionType, isStop: true)
            return true
        }
        if let motionType {
            outboundScheduler.replaceMotion(frame, legacyType: motionType, isStop: isStop)
            return true
        }
        return outboundScheduler.enqueueReliable(frame)
    }

    private func stageNextScheduledFrame() {
        guard writeQueue.isEmpty else { return }
        if outboundScheduler.hasPendingStop, let scheduled = outboundScheduler.dequeueNext() {
            stageFrame(scheduled.data, motionType: scheduled.legacyType)
            return
        }
        if let stopIntent = pendingStopIntent {
            guard let staged = encodeMotionIntent(stopIntent) else { return }
            pendingStopIntent = nil
            stageFrame(staged.data, motionType: staged.motionType)
            return
        }
        if let scheduled = outboundScheduler.dequeueNext() {
            stageFrame(scheduled.data, motionType: scheduled.legacyType)
            return
        }
        guard let intent = pendingMotionIntent,
              let staged = encodeMotionIntent(intent) else { return }
        pendingMotionIntent = nil
        stageFrame(staged.data, motionType: staged.motionType)
    }

    private func stageFrame(_ data: Data, motionType: UInt8?) {
        guard !data.isEmpty else { return }
        nextFrameID &+= 1
        let frameID = nextFrameID
        let supportsWriteWithoutResponse =
            writeCharacteristic?.properties.contains(.writeWithoutResponse) == true
        let writeType = BLEWriteTransportPolicy.writeType(
            for: data,
            motionType: motionType,
            supportsWriteWithoutResponse: supportsWriteWithoutResponse
        )
        if disconnectAfterStop, Self.isStopFrame(data, motionType: motionType) {
            disconnectStopFrameID = frameID
        }
        let maximumWriteLength: Int
        if let peripheral, writeCharacteristic != nil {
            let negotiated = peripheral.maximumWriteValueLength(for: writeType)
            maximumWriteLength = max(1, negotiated)
        } else {
            // CoreBluetooth may report the characteristic before ATT MTU
            // negotiation completes. 20 bytes is the mandatory ATT payload.
            maximumWriteLength = 20
        }
        var offset = 0
        while offset < data.count {
            let end = min(offset + maximumWriteLength, data.count)
            writeQueue.append(BLEQueuedWrite(
                data: Data(data[offset..<end]),
                frameID: frameID,
                motionType: motionType,
                writeType: writeType
            ))
            offset = end
        }
    }

    private func encodeMotionIntent(_ intent: MotionIntent) ->
        (data: Data, motionType: UInt8)? {
        let type: SmartCarProtocol.FrameType
        let payload: Data
        if intent.isStop {
            type = .wheelSpeedCommand
            payload = Self.floatPayload([0, 0, 0, 0])
        } else {
            switch intent.mode {
            case .wheelIndependent:
                guard intent.wheelTargets.count == 4,
                      intent.wheelTargets.allSatisfy(\.isFinite) else { return nil }
                type = .wheelSpeedCommand
                payload = Self.floatPayload(intent.wheelTargets)
            case .chassisDiff:
                guard intent.chassisBaseSpeed.isFinite, intent.yawRate.isFinite else { return nil }
                if intent.headingLockEnabled {
                    guard let headingPayload = SmartCarProtocol.chassisHeadingPayload(
                        vMmS: intent.chassisBaseSpeed,
                        targetYawDeg: intent.targetYawDeg) else { return nil }
                    type = .chassisHeadingCommand
                    payload = headingPayload
                } else {
                    type = .chassisSpeedCommand
                    var chassisPayload = Self.floatPayload([intent.chassisBaseSpeed, intent.yawRate])
                    chassisPayload.append(Data(repeating: 0, count: 8))
                    payload = chassisPayload
                }
            }
        }
        return (SmartCarProtocol.encode(type: type, payload: payload), type.rawValue)
    }

    private static func motionType(in frame: Data) -> UInt8? {
        guard frame.count > 2 else { return nil }
        switch frame[2] {
        case SmartCarProtocol.FrameType.wheelSpeedCommand.rawValue,
             SmartCarProtocol.FrameType.wheelSpeedSingleCommand.rawValue,
             SmartCarProtocol.FrameType.masterSpeedCommand.rawValue,
             SmartCarProtocol.FrameType.chassisSpeedCommand.rawValue,
             SmartCarProtocol.FrameType.chassisHeadingCommand.rawValue:
            return frame[2]
        default:
            return nil
        }
    }

    private static func isStopFrame(_ frame: Data, motionType: UInt8?) -> Bool {
        if frame.count > 5,
           frame[2] == SmartCarProtocol.FrameType.control.rawValue,
           let payload = payload(in: frame), payload.first == SmartCarProtocol.ControlCommand.stop.rawValue {
            return true
        }
        guard let motionType, let payload = payload(in: frame) else { return false }
        switch motionType {
        case SmartCarProtocol.FrameType.wheelSpeedCommand.rawValue,
             SmartCarProtocol.FrameType.chassisSpeedCommand.rawValue,
             SmartCarProtocol.FrameType.masterSpeedCommand.rawValue,
             SmartCarProtocol.FrameType.chassisHeadingCommand.rawValue:
            return payload.allSatisfy { $0 == 0 }
        default:
            return false
        }
    }

    private static func payload(in frame: Data) -> Data? {
        guard frame.count >= 8,
              frame[0] == SmartCarProtocol.head,
              frame[1] == SmartCarProtocol.version,
              frame[frame.count - 1] == SmartCarProtocol.tail else { return nil }
        let length = Int(frame[3]) | (Int(frame[4]) << 8)
        guard frame.count == length + 8 else { return nil }
        return Data(frame[5..<(5 + length)])
    }

    private static func floatPayload(_ values: [Float]) -> Data {
        var payload = Data(capacity: values.count * MemoryLayout<UInt32>.size)
        for value in values {
            var bits = value.bitPattern.littleEndian
            withUnsafeBytes(of: &bits) { payload.append(contentsOf: $0) }
        }
        return payload
    }

    private func handleReceivedEvents(_ events: [BLEReceiveEvent]) {
        for event in events {
            switch event {
            case .message(let record):
                lastPacketReceivedAt = record.receivedAt
                Task { await telemetryReducer.submit(record) }
                onDecodedMessage?(record)
            case .session(let frame):
                handleSessionFrame(frame)
            case .decodeFailure(let reason):
                decodeFailureCount += 1
                setError(.protocolDecodeFailed(reason))
            case .drops(let count):
                droppedMessageCount += count
            }
        }
    }

    private func startReceiveFlushTimer() {
        receiveFlushTimer?.invalidate()
        let timer = Timer(timeInterval: 0.05, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self else { return }
                self.receivePipeline.drain { [weak self] events in
                    Task { @MainActor [weak self] in
                        self?.handleReceivedEvents(events)
                    }
                }
                self.flushTelemetryReducer()
            }
        }
        receiveFlushTimer = timer
        RunLoop.main.add(timer, forMode: .common)
    }

    private func flushTelemetryReducer() {
        Task { [weak self] in
            guard let self else { return }
            let snapshot = await telemetryReducer.drain()
            guard !snapshot.isEmpty else { return }
            await MainActor.run {
                for record in snapshot.records {
                    self.telemetryStore.ingest(record.message, at: record.receivedAt)
                }
            }
        }
    }

    private func handleSessionFrame(_ frame: AppBLEFrame) {
        let now = Date()
        let event = session.handle(frame, now: now)
        sessionMode = session.mode
        switch event {
        case .v2Ready:
            lastSessionActivityAt = now
            sessionFallbackWorkItem?.cancel()
            sessionFallbackWorkItem = nil
            startSessionTimer()
            pumpWriteQueue()
        case .heartbeatAcknowledged, .commandAcknowledged:
            lastSessionActivityAt = now
            if case .commandAcknowledged(let ack) = event {
                lastCommandAck = ack
            }
        case .ignored:
            break
        }
    }

    private func startProtocolNegotiation() {
        sessionFallbackWorkItem?.cancel()
        let now = Date()
        let hello = session.begin(now: now)
        sessionMode = session.mode
        lastSessionActivityAt = now
        enqueueFrame(hello, motionType: nil)
        pumpWriteQueue()

        let work = DispatchWorkItem { [weak self] in
            Task { @MainActor [weak self] in
                guard let self else { return }
                guard self.session.fallbackIfNeeded(now: Date()) else { return }
                self.sessionMode = self.session.mode
                self.startSessionTimer()
                self.pumpWriteQueue()
            }
        }
        sessionFallbackWorkItem = work
        DispatchQueue.main.asyncAfter(deadline: .now() + AppBLESession.defaultNegotiationTimeout,
                                      execute: work)
    }

    private func startSessionTimer() {
        sessionTimer?.invalidate()
        let timer = Timer(timeInterval: 0.25, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.tickSession() }
        }
        sessionTimer = timer
        RunLoop.main.add(timer, forMode: .common)
    }

    private func tickSession() {
        guard status == .connected else { return }
        let now = Date()
        if case .v2Ready = session.mode,
           let lastSessionActivityAt,
           now.timeIntervalSince(lastSessionActivityAt) > session.sessionTTL {
            sessionTimeoutCount += 1
            session.markExpired()
            sessionMode = session.mode
            self.lastSessionActivityAt = nil
            cancelQueuedMotionCommands()
            let headingZero = SmartCarProtocol.encode(
                type: .chassisHeadingCommand,
                payload: SmartCarProtocol.chassisHeadingPayload(
                    vMmS: 0.0, targetYawDeg: 0.0) ?? Data(repeating: 0, count: 12))
            _ = enqueueFrame(headingZero,
                             motionType: SmartCarProtocol.FrameType.chassisHeadingCommand.rawValue,
                             isStop: true)
            let zero = SmartCarProtocol.encode(type: .wheelSpeedCommand,
                                               payload: Self.floatPayload([0, 0, 0, 0]))
            _ = enqueueFrame(zero,
                             motionType: SmartCarProtocol.FrameType.wheelSpeedCommand.rawValue,
                             isStop: true)
            pumpWriteQueue()
            setError(.system("BLE session expired; motion commands paused"))
            return
        }
        if let heartbeat = session.heartbeatIfDue(now: now) {
            enqueueFrame(heartbeat, motionType: nil)
            pumpWriteQueue()
        }
    }

    private func resetSession() {
        sessionTimer?.invalidate()
        sessionTimer = nil
        sessionFallbackWorkItem?.cancel()
        sessionFallbackWorkItem = nil
        session.reset()
        sessionMode = session.mode
        lastSessionActivityAt = nil
        lastCommandAck = nil
    }

    private var hasRequiredCharacteristics: Bool {
        writeCharacteristic != nil && notifyCharacteristic != nil
    }

    private var isGATTReady: Bool {
        status == .connected && hasRequiredCharacteristics &&
            notifyCharacteristic?.isNotifying == true
    }

    private func completeGATTSetupIfReady() {
        guard status == .connecting,
              hasRequiredCharacteristics,
              notifyCharacteristic?.isNotifying == true,
              let peripheral,
              let writeCharacteristic else {
            return
        }
        guard writeCharacteristic.properties.contains(.write) else {
            setError(.system("FFE1 does not support Write With Response"))
            central.cancelPeripheralConnection(peripheral)
            return
        }

        // Keep iPhone command framing identical to the working macOS app.
        // V1 carries all currently exposed 0x15/0x2D/0x2E commands directly.
        sessionMode = .v1Fallback
        updateStatus(.connected)
        pumpWriteQueue()
    }

    private func clearOutboundState() {
        cancelWriteResponseTimeout()
        writeQueue.removeAll(keepingCapacity: true)
        outboundScheduler.removeAll()
        pendingMotionIntent = nil
        pendingStopIntent = nil
        writeInFlight = false
    }

    /// A missing ATT Write Response otherwise leaves a reliable command in flight
    /// forever. No-response motion writes use CoreBluetooth backpressure and do
    /// not arm this watchdog.
    private func armWriteResponseTimeout(for queuedWrite: BLEQueuedWrite) {
        guard queuedWrite.writeType == .withResponse else { return }
        writeResponseTimeoutWorkItem?.cancel()
        writeResponseTimeoutWorkItem = nil
        let token = writeResponseWatchdog.arm()
        let frameID = queuedWrite.frameID
        let work = DispatchWorkItem { [weak self] in
            Task { @MainActor [weak self] in
                self?.handleWriteResponseTimeout(frameID: frameID, token: token)
            }
        }
        writeResponseTimeoutWorkItem = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0, execute: work)
    }

    private func cancelWriteResponseTimeout() {
        writeResponseTimeoutWorkItem?.cancel()
        writeResponseTimeoutWorkItem = nil
        writeResponseWatchdog.invalidate()
    }

    private func handleWriteResponseTimeout(frameID: UInt64, token: UInt64) {
        guard writeInFlight,
              writeResponseWatchdog.isCurrent(token),
              writeQueue.first?.writeType == .withResponse,
              writeQueue.first?.frameID == frameID else {
            return
        }

        setError(.system("FFE1 write response timed out; resetting connection"))
        clearDeferredDisconnect()
        clearOutboundState()
        resetSession()
        writeCharacteristic = nil
        notifyCharacteristic = nil

        guard let peripheral else {
            updateStatus(.disconnected)
            return
        }
        updateStatus(.connecting)
        central.cancelPeripheralConnection(peripheral)
    }

    private func clearDeferredDisconnect() {
        disconnectStopTimeoutWorkItem?.cancel()
        disconnectStopTimeoutWorkItem = nil
        disconnectAfterStop = false
        disconnectStopFrameID = nil
    }

    private func scheduleDisconnectAfterStopTimeout() {
        disconnectStopTimeoutWorkItem?.cancel()
        let work = DispatchWorkItem { [weak self] in
            Task { @MainActor [weak self] in
                guard let self, self.disconnectAfterStop else { return }
                self.setError(.system("Zero-speed write was not confirmed before disconnect"))
                self.completeDeferredDisconnect()
            }
        }
        disconnectStopTimeoutWorkItem = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0, execute: work)
    }

    private func completeDeferredDisconnect() {
        guard disconnectAfterStop else { return }
        clearDeferredDisconnect()
        clearOutboundState()
        guard let peripheral else {
            resetSession()
            updateStatus(.disconnected)
            return
        }
        central.cancelPeripheralConnection(peripheral)
    }

    private func updateStatus(_ newStatus: BLEConnectionStatus) {
        status = newStatus
        telemetryStore.setConnection(newStatus)
        onStatusChange?(newStatus)
    }

    private func setError(_ error: BLEUserFacingError?) {
        lastError = error
        onErrorChange?(error)
    }
}

extension BLEManager: CBCentralManagerDelegate {
    nonisolated func centralManagerDidUpdateState(_ central: CBCentralManager) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            switch central.state {
            case .poweredOn:
                self.updateStatus(.disconnected)
                if self.scanWhenPoweredOn {
                    self.startScanning()
                } else if self.shouldReconnect, let peripheral = self.peripheral {
                    self.updateStatus(.connecting)
                    central.connect(peripheral, options: nil)
                }
            case .poweredOff, .unauthorized, .unsupported, .resetting, .unknown:
                self.updateStatus(.unavailable)
            @unknown default:
                self.updateStatus(.unavailable)
            }
        }
    }

    nonisolated func centralManager(
        _ central: CBCentralManager,
        didDiscover peripheral: CBPeripheral,
        advertisementData: [String: Any],
        rssi RSSI: NSNumber
    ) {
        let advertisedName = peripheral.name ?? advertisementData[CBAdvertisementDataLocalNameKey] as? String
        guard advertisedName == Self.targetDeviceName else { return }

        Task { @MainActor [weak self] in
            guard let self else { return }
            self.peripheral = peripheral
            self.discoveredDeviceName = advertisedName ?? Self.targetDeviceName
            self.discoveredRSSI = RSSI.intValue
            self.onDeviceNameChange?(self.discoveredDeviceName)
            self.onRSSIChange?(self.discoveredRSSI)
            self.setError(nil)
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.peripheral = peripheral
            self.reconnectAttempt = 0
            self.reconnectWorkItem?.cancel()
            peripheral.delegate = self
            print("BLE CONNECT SUCCESS")
            peripheral.discoverServices([Self.serviceUUID])
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.writeCharacteristic = nil
            self.notifyCharacteristic = nil
            let message = error?.localizedDescription ?? "Connection failed"
            self.setError(.connectionFailed(message))
            self.updateStatus(.failed(message))
            self.scheduleReconnect()
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.writeCharacteristic = nil
            self.notifyCharacteristic = nil
            self.clearOutboundState()
            self.clearDeferredDisconnect()
            self.receivePipeline.reset()
            self.resetSession()
            self.updateStatus(.disconnected)
            if let error {
                self.setError(.system(error.localizedDescription))
            }
            self.scheduleReconnect()
        }
    }
}

private extension BLEManager {
    func scheduleReconnect() {
        guard shouldReconnect, let peripheral else { return }
        reconnectWorkItem?.cancel()
        reconnectAttempt = min(reconnectAttempt + 1, 5)
        let delay = min(pow(2.0, Double(reconnectAttempt)), 20.0)
        let work = DispatchWorkItem { [weak self, weak peripheral] in
            guard let self, let peripheral, self.shouldReconnect else { return }
            self.updateStatus(.connecting)
            self.central.connect(peripheral, options: nil)
        }
        reconnectWorkItem = work
        DispatchQueue.main.asyncAfter(deadline: .now() + delay, execute: work)
    }
}

extension BLEManager: CBPeripheralDelegate {
    nonisolated func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        let errorDescription = error?.localizedDescription
        Task { @MainActor [weak self] in
            guard let self else { return }
            guard errorDescription == nil else {
                self.setError(.system(errorDescription ?? "Unknown error"))
                return
            }
            guard let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID }) else {
                self.setError(.serviceNotFound)
                return
            }
            peripheral.discoverCharacteristics(
                [Self.rxCharacteristicUUID, Self.txCharacteristicUUID],
                for: service
            )
        }
    }

    nonisolated func peripheral(
        _ peripheral: CBPeripheral,
        didDiscoverCharacteristicsFor service: CBService,
        error: Error?
    ) {
        let errorDescription = error?.localizedDescription
        Task { @MainActor [weak self] in
            guard let self else { return }
            guard errorDescription == nil else {
                self.setError(.system(errorDescription ?? "Unknown error"))
                return
            }
            for characteristic in service.characteristics ?? [] {
                if characteristic.uuid == Self.rxCharacteristicUUID {
                    self.writeCharacteristic = characteristic
                } else if characteristic.uuid == Self.txCharacteristicUUID {
                    self.notifyCharacteristic = characteristic
                    peripheral.setNotifyValue(true, for: characteristic)
                }
            }
            guard self.hasRequiredCharacteristics else {
                self.setError(.system("Missing required SmartCar BLE characteristic"))
                self.central.cancelPeripheralConnection(peripheral)
                return
            }
            guard self.notifyCharacteristic?.properties.contains(.notify) == true else {
                self.setError(.system("FFE2 does not support notifications"))
                self.central.cancelPeripheralConnection(peripheral)
                return
            }
            self.completeGATTSetupIfReady()
        }
    }

    nonisolated func peripheral(
        _ peripheral: CBPeripheral,
        didUpdateNotificationStateFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        let errorDescription = error?.localizedDescription
        Task { @MainActor [weak self] in
            guard let self else { return }
            guard errorDescription == nil else {
                self.setError(.system(errorDescription ?? "Unable to enable BLE notifications"))
                if characteristic.uuid == Self.txCharacteristicUUID {
                    self.central.cancelPeripheralConnection(peripheral)
                }
                return
            }
            if characteristic.uuid == Self.txCharacteristicUUID {
                self.completeGATTSetupIfReady()
            }
        }
    }

    nonisolated func peripheral(
        _ peripheral: CBPeripheral,
        didUpdateValueFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        if let error {
            let errorDescription = error.localizedDescription
            Task { @MainActor [weak self] in
                self?.setError(.system(errorDescription))
            }
            return
        }
        guard let data = characteristic.value else { return }
        if characteristic.uuid == Self.txCharacteristicUUID {
            receivePipeline.submit(data)
        }
    }

    nonisolated func peripheral(
        _ peripheral: CBPeripheral,
        didWriteValueFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        guard characteristic.uuid == Self.rxCharacteristicUUID else { return }
        Task { @MainActor [weak self] in
            self?.finishWrite(error: error)
        }
    }

    nonisolated func peripheralIsReady(toSendWriteWithoutResponse peripheral: CBPeripheral) {
        Task { @MainActor [weak self] in
            guard let self, self.peripheral === peripheral else { return }
            self.pumpWriteQueue()
        }
    }
}
