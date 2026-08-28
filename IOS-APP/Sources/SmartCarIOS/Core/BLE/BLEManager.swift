@preconcurrency import CoreBluetooth
import Foundation
import SmartCarAppCore

private enum BLEReceiveEvent {
    case message(DecodedMessageRecord)
    case session(AppBLEFrame)
    case decodeFailure(String)
    case drops(Int)
}

private enum BLELogReceiveEvent {
    case record(SmartCarLogRecord)
}

private struct BLEQueuedWrite {
    let data: Data
    let frameID: UInt64
    let motionType: UInt8?
    let writeType: CBCharacteristicWriteType
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

private final class BLELogReceivePipeline: @unchecked Sendable {
    let queue = DispatchQueue(
        label: "SmartCar_Control_MAC.ble.log.receive",
        qos: .utility
    )
    private var parser = SmartCarLogParser()

    func reset() {
        queue.sync {
            parser.reset()
        }
    }

    func submit(_ data: Data, handler: @escaping ([BLELogReceiveEvent]) -> Void) {
        queue.async {
            let records = self.parser.feed(data, receivedAt: Date())
            handler(records.map(BLELogReceiveEvent.record))
        }
    }
}

@MainActor
final class BLEManager: NSObject {
    nonisolated static let targetDeviceName = "SmartCar_S3"
    nonisolated static let serviceUUID = CBUUID(string: "0000FFE0-0000-1000-8000-00805F9B34FB")
    nonisolated static let rxCharacteristicUUID = CBUUID(string: "0000FFE1-0000-1000-8000-00805F9B34FB")
    nonisolated static let txCharacteristicUUID = CBUUID(string: "0000FFE2-0000-1000-8000-00805F9B34FB")
    nonisolated static let loggerCharacteristicUUID = CBUUID(string: "0000FFE3-0000-1000-8000-00805F9B34FB")

    private(set) var status: BLEConnectionStatus = .disconnected
    private(set) var decodedMessages: [DecodedMessageRecord] = []
    private(set) var discoveredDeviceName = targetDeviceName
    private(set) var discoveredRSSI: Int?
    private(set) var lastError: BLEUserFacingError?
    private(set) var lastPacketReceivedAt: Date?
    private(set) var decodeFailureCount = 0
    private(set) var droppedMessageCount = 0

    let telemetryStore: TelemetryStore
    let stmLogStore: DeviceLogStore
    let s3LogStore: DeviceLogStore
    var onStatusChange: ((BLEConnectionStatus) -> Void)?
    var onDecodedMessage: ((DecodedMessageRecord) -> Void)?
    var onDeviceNameChange: ((String) -> Void)?
    var onRSSIChange: ((Int?) -> Void)?
    var onErrorChange: ((BLEUserFacingError?) -> Void)?

    private let central: CBCentralManager
    private nonisolated let receivePipeline: BLEReceivePipeline
    private nonisolated let logReceivePipeline: BLELogReceivePipeline
    private let telemetryReducer = AppTelemetryReducer()
    private var peripheral: CBPeripheral?
    private var writeCharacteristic: CBCharacteristic?
    private var notifyCharacteristic: CBCharacteristic?
    private var loggerCharacteristic: CBCharacteristic?
    private var scanWhenPoweredOn = false
    private var shouldReconnect = true
    private var reconnectAttempt = 0
    private var reconnectWorkItem: DispatchWorkItem?
    private var writeQueue: [BLEQueuedWrite] = []
    private var outboundScheduler = AppBLEOutboundScheduler(reliableCapacity: 12)
    private var pendingMotionIntent: MotionIntent?
    private var pendingStopIntent: MotionIntent?
    private var decodedMessageBuffer = AppBLEBoundedRingBuffer<DecodedMessageRecord>(capacity: 200)
    private var writeInFlight = false
    private var nextFrameID: UInt64 = 0
    private var session = AppBLESession()
    private var sessionTimer: Timer?
    private var receiveFlushTimer: Timer?
    private var sessionFallbackWorkItem: DispatchWorkItem?
    private var lastSessionActivityAt: Date?

    private(set) var sessionMode: AppBLESessionMode = .idle
    private(set) var lastCommandAck: AppBLEV2CommandAck?
    private(set) var sessionTimeoutCount = 0

    override convenience init() {
        self.init(telemetryStore: TelemetryStore())
    }

    init(telemetryStore: TelemetryStore) {
        let receivePipeline = BLEReceivePipeline()
        let logReceivePipeline = BLELogReceivePipeline()
        self.telemetryStore = telemetryStore
        self.stmLogStore = DeviceLogStore(source: .stm32)
        self.s3LogStore = DeviceLogStore(source: .s3)
        self.receivePipeline = receivePipeline
        self.logReceivePipeline = logReceivePipeline
        central = CBCentralManager(delegate: nil, queue: receivePipeline.queue)
        super.init()
        central.delegate = self
        startReceiveFlushTimer()
    }

    deinit {
        receiveFlushTimer?.invalidate()
    }

    func startScanning() {
        guard central.state == .poweredOn else {
            scanWhenPoweredOn = true
            setError(nil)
            return
        }

        decodedMessageBuffer.removeAll()
        decodedMessages.removeAll(keepingCapacity: true)
        discoveredRSSI = nil
        onRSSIChange?(nil)
        receivePipeline.reset()
        Task { await telemetryReducer.reset() }
        logReceivePipeline.reset()
        outboundScheduler.removeAll()
        pendingMotionIntent = nil
        pendingStopIntent = nil
        writeQueue.removeAll(keepingCapacity: true)
        writeInFlight = false
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
        self.peripheral = peripheral
        peripheral.delegate = self
        central.connect(peripheral, options: nil)
    }

    func disconnect() {
        shouldReconnect = false
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        sendChassisHeading(vMmS: 0.0, targetYawDeg: 0.0)
        sendWheelSpeeds([0, 0, 0, 0])
        resetSession()
        guard let peripheral else { return }
        central.cancelPeripheralConnection(peripheral)
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
        guard peripheral != nil,
              status == .connected || status == .connecting else {
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
        // didConnect is reported before service/characteristic discovery. Keep
        // frames queued during that short window so the first motion command
        // is not silently discarded when the UI becomes active early.
        guard peripheral != nil,
              status == .connected || status == .connecting else {
            setError(.notConnected)
            return false
        }
        let motionType = Self.motionType(in: data)
        let isStop = Self.isStopFrame(data, motionType: motionType)
        guard session.isCommandReady || isStop else {
            // Motion producers run at 20 Hz and will retry after negotiation;
            // refusing stale pre-session commands is safer than queueing them.
            return false
        }
        let wireData: Data
        if session.usesV2 {
            guard let type = data.count > 2 ? data[2] : nil,
                  let payload = Self.payload(in: data),
                  let wrapped = session.command(type: type, payload: payload,
                                                validForMs: 1_000) else {
                return false
            }
            wireData = wrapped
        } else {
            wireData = data
        }
        if isStop {
            cancelQueuedMotionCommands()
        }
        guard enqueueFrame(wireData, motionType: motionType, isStop: isStop) else {
            setError(.system("BLE outbound queue full"))
            return false
        }
        pumpWriteQueue()
        return true
    }

    /// CoreBluetooth does not guarantee ordering when several .withResponse
    /// writes are issued back-to-back. Keep one write in flight and advance
    /// only from didWriteValueFor so repeated mode/wheel commands are all
    /// delivered in order.
    private func pumpWriteQueue() {
        guard !writeInFlight,
              let peripheral,
              let writeCharacteristic else { return }
        while !writeInFlight {
            if writeQueue.isEmpty {
                stageNextScheduledFrame()
            }
            guard let next = writeQueue.first else { return }
            if next.writeType == .withoutResponse {
                guard writeCharacteristic.properties.contains(.writeWithoutResponse) else {
                    writeQueue.removeFirst()
                    setError(.system("FFE1 does not support Write Without Response"))
                    continue
                }
                writeQueue.removeFirst()
                peripheral.writeValue(next.data, for: writeCharacteristic,
                                      type: .withoutResponse)
                continue
            }
            writeInFlight = true
            peripheral.writeValue(next.data, for: writeCharacteristic, type: .withResponse)
        }
    }

    private func finishWrite(error: Error?) {
        guard !writeQueue.isEmpty else {
            writeInFlight = false
            return
        }
        let completedFrameID = writeQueue[0].frameID
        writeQueue.removeSubrange(0...0)
        writeInFlight = false
        if let error {
            writeQueue.removeAll { $0.frameID == completedFrameID }
            setError(.system(error.localizedDescription))
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
        let writeType: CBCharacteristicWriteType =
            motionType == SmartCarProtocol.FrameType.chassisHeadingCommand.rawValue
                ? .withoutResponse : .withResponse
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
        guard let wire = session.command(type: type.rawValue, payload: payload,
                                          validForMs: 1_000) else { return nil }
        return (wire, type.rawValue)
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
                if decodedMessageBuffer.append(record) {
                    droppedMessageCount += 1
                }
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
        if !events.isEmpty {
            decodedMessages = decodedMessageBuffer.elements
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

    private func handleReceivedLogEvents(_ events: [BLELogReceiveEvent]) {
        for event in events {
            switch event {
            case .record(let record):
                switch record.source {
                case .stm32:
                    stmLogStore.append(record)
                case .s3:
                    s3LogStore.append(record)
                }
            }
        }
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
            self.updateStatus(.connected)
            print("BLE CONNECT SUCCESS")
            peripheral.discoverServices([Self.serviceUUID])
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        Task { @MainActor [weak self] in
            guard let self else { return }
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
            self.loggerCharacteristic = nil
            self.writeQueue.removeAll(keepingCapacity: true)
            self.outboundScheduler.removeAll()
            self.writeInFlight = false
            self.receivePipeline.reset()
            self.logReceivePipeline.reset()
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
                [Self.rxCharacteristicUUID, Self.txCharacteristicUUID, Self.loggerCharacteristicUUID],
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
                    self.pumpWriteQueue()
                } else if characteristic.uuid == Self.txCharacteristicUUID {
                    self.notifyCharacteristic = characteristic
                    peripheral.setNotifyValue(true, for: characteristic)
                } else if characteristic.uuid == Self.loggerCharacteristicUUID {
                    self.loggerCharacteristic = characteristic
                    peripheral.setNotifyValue(true, for: characteristic)
                }
            }
            if self.writeCharacteristic != nil, self.notifyCharacteristic != nil {
                self.startProtocolNegotiation()
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
        } else if characteristic.uuid == Self.loggerCharacteristicUUID {
            logReceivePipeline.submit(data) { [weak self] events in
                Task { @MainActor [weak self] in
                    self?.handleReceivedLogEvents(events)
                }
            }
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
}
