import CoreBluetooth
import Foundation

private enum BLEReceiveEvent {
    case message(DecodedMessageRecord)
    case decodeFailure(String)
}

private enum BLELogReceiveEvent {
    case record(sessionGeneration: UInt64, record: SmartCarLogRecord)
}

private enum BLEOutboundWriteKind: Equatable {
    case normal
    case motion
    case stop
}

private struct BLEOutboundWrite {
    let data: Data
    let kind: BLEOutboundWriteKind
}

private final class BLEReceivePipeline: @unchecked Sendable {
    let queue = DispatchQueue(
        label: "SmartCar_Control_MAC.ble.receive",
        qos: .userInitiated
    )
    private var parser = SmartCarProtocol.Parser()

    func reset() {
        queue.sync {
            parser.reset()
        }
    }

    func submit(_ data: Data, handler: @escaping ([BLEReceiveEvent]) -> Void) {
        queue.async {
            let receivedAt = Date()
            let frames = self.parser.feed(data)
            var events: [BLEReceiveEvent] = []
            events.reserveCapacity(frames.count)
            for frame in frames {
                do {
                    events.append(.message(
                        DecodedMessageRecord(
                            message: try DecodedMessage(frame: frame),
                            receivedAt: receivedAt
                        )
                    ))
                } catch {
                    events.append(.decodeFailure(error.localizedDescription))
                }
            }
            handler(events)
        }
    }
}

private final class BLELogReceivePipeline: @unchecked Sendable {
    let queue = DispatchQueue(
        label: "SmartCar_Control_MAC.ble.log.receive",
        qos: .utility
    )
    private var parser = SmartCarLogParser()
    private var sessionGeneration: UInt64 = 0

    func beginSession() -> UInt64 {
        queue.sync {
            sessionGeneration &+= 1
            parser.reset()
            return sessionGeneration
        }
    }

    func invalidateSession() {
        queue.sync {
            sessionGeneration &+= 1
            parser.reset()
        }
    }

    func submit(_ data: Data, handler: @escaping ([BLELogReceiveEvent]) -> Void) {
        let generation = queue.sync { sessionGeneration }
        queue.async {
            let records = self.parser.feed(data, receivedAt: Date())
            handler(records.map {
                BLELogReceiveEvent.record(sessionGeneration: generation, record: $0)
            })
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
    private(set) var lastError: BLEUserFacingError?

    let telemetryStore: TelemetryStore
    let stmLogStore: DeviceLogStore
    let s3LogStore: DeviceLogStore
    var onStatusChange: ((BLEConnectionStatus) -> Void)?
    var onDecodedMessage: ((DecodedMessageRecord) -> Void)?
    var onDeviceNameChange: ((String) -> Void)?
    var onErrorChange: ((BLEUserFacingError?) -> Void)?

    private let central: CBCentralManager
    private nonisolated let receivePipeline: BLEReceivePipeline
    private nonisolated let logReceivePipeline: BLELogReceivePipeline
    private let sessionLogWriter = SessionLogWriter()
    private var peripheral: CBPeripheral?
    private var writeCharacteristic: CBCharacteristic?
    private var notifyCharacteristic: CBCharacteristic?
    private var loggerCharacteristic: CBCharacteristic?
    private var scanWhenPoweredOn = false
    private var activeLogSessionGeneration: UInt64?
    private var txNotifyConfirmed = false
    private var outboundWriteQueue: [BLEOutboundWrite] = []
    private var inFlightWrite: BLEOutboundWrite?
    private var pendingMotionAfterStop: BLEOutboundWrite?
    private var stopBarrierPending = false
    private var disconnectAfterStop = false
    private var submittedWriteCount: UInt64 = 0
    private var completedWriteCount: UInt64 = 0
    private var failedWriteCount: UInt64 = 0

    private static let maxOutboundWriteQueueDepth = 32

    private func logDiagnostic(_ message: String) {
        print("[BLE_DIAG] \(message)")
    }

    private func shouldLogWrite(_ count: UInt64, kind: BLEOutboundWriteKind) -> Bool {
        kind != .motion || count <= 3 || count.isMultiple(of: 20)
    }

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
    }

    deinit {
        sessionLogWriter.closeAndSynchronizeAndWait()
    }

    func startScanning() {
        guard central.state == .poweredOn else {
            scanWhenPoweredOn = true
            setError(nil)
            return
        }

        decodedMessages.removeAll(keepingCapacity: true)
        receivePipeline.reset()
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
        updateStatus(.connecting)
        self.peripheral = peripheral
        peripheral.delegate = self
        central.connect(peripheral, options: nil)
    }

    func disconnect() {
        guard let peripheral else {
            clearOutboundWrites()
            endLogSession()
            return
        }

        guard status == .connected, writeCharacteristic != nil else {
            clearOutboundWrites()
            endLogSession()
            central.cancelPeripheralConnection(peripheral)
            return
        }

        disconnectAfterStop = true
        sendWheelSpeeds([0, 0, 0, 0])
        endLogSession()
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
        let data = SmartCarProtocol.encode(type: .wheelSpeedCommand,
                                           payload: Self.floatPayload(speeds))
        let kind: BLEOutboundWriteKind = speeds.allSatisfy { $0 == 0.0 } ? .stop : .motion
        _ = sendFrame(data, kind: kind)
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
    private func sendFrame(_ data: Data, kind: BLEOutboundWriteKind = .normal) -> Bool {
        guard status == .connected,
              let peripheral,
              let writeCharacteristic else {
            logDiagnostic("WRITE_REJECT status=\(status.displayText) rx=\(self.writeCharacteristic != nil ? 1 : 0)")
            setError(.notConnected)
            return false
        }

        let outbound = BLEOutboundWrite(data: data, kind: kind)
        if kind == .stop {
            stopBarrierPending = true
            pendingMotionAfterStop = nil
            outboundWriteQueue.removeAll { $0.kind == .motion || $0.kind == .stop }
            if inFlightWrite?.kind != .stop {
                if outboundWriteQueue.count >= Self.maxOutboundWriteQueueDepth,
                   let normalIndex = outboundWriteQueue.firstIndex(where: { $0.kind == .normal }) {
                    outboundWriteQueue.remove(at: normalIndex)
                }
                outboundWriteQueue.insert(outbound, at: 0)
            }
        } else if kind == .motion {
            if stopBarrierPending {
                pendingMotionAfterStop = outbound
                return true
            } else {
                outboundWriteQueue.removeAll { $0.kind == .motion }
                return enqueueOutboundWrite(outbound)
            }
        } else {
            return enqueueOutboundWrite(outbound)
        }
        pumpWriteQueue(peripheral: peripheral, characteristic: writeCharacteristic)
        return true
    }

    @discardableResult
    private func enqueueOutboundWrite(_ outbound: BLEOutboundWrite) -> Bool {
        if outboundWriteQueue.count >= Self.maxOutboundWriteQueueDepth {
            if let normalIndex = outboundWriteQueue.firstIndex(where: { $0.kind == .normal }) {
                outboundWriteQueue.remove(at: normalIndex)
            } else {
                logDiagnostic("WRITE_REJECT queue_full kind=\(outbound.kind)")
                return false
            }
        }
        outboundWriteQueue.append(outbound)
        pumpWriteQueueIfPossible()
        return true
    }

    private func pumpWriteQueueIfPossible() {
        guard let peripheral, let writeCharacteristic else { return }
        pumpWriteQueue(peripheral: peripheral, characteristic: writeCharacteristic)
    }

    private func pumpWriteQueue(peripheral: CBPeripheral, characteristic: CBCharacteristic) {
        guard inFlightWrite == nil, !outboundWriteQueue.isEmpty else { return }
        let outbound = outboundWriteQueue.removeFirst()
        inFlightWrite = outbound
        submittedWriteCount &+= 1
        if shouldLogWrite(submittedWriteCount, kind: outbound.kind) {
            let frameType = outbound.data.count > 2 ? String(format: "0x%02X", outbound.data[2]) : "unknown"
            logDiagnostic("WRITE_SUBMIT count=\(submittedWriteCount) kind=\(outbound.kind) type=\(frameType) bytes=\(outbound.data.count) queue=\(outboundWriteQueue.count)")
        }
        peripheral.writeValue(outbound.data, for: characteristic, type: .withResponse)
    }

    private func clearOutboundWrites() {
        outboundWriteQueue.removeAll(keepingCapacity: true)
        inFlightWrite = nil
        pendingMotionAfterStop = nil
        stopBarrierPending = false
        disconnectAfterStop = false
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
                decodedMessages.append(record)
                telemetryStore.ingest(record.message, at: record.receivedAt)
                onDecodedMessage?(record)
            case .decodeFailure(let reason):
                setError(.protocolDecodeFailed(reason))
            }
        }
        if decodedMessages.count > 200 {
            decodedMessages.removeFirst(decodedMessages.count - 200)
        }
    }

    private func handleReceivedLogEvents(_ events: [BLELogReceiveEvent]) {
        for event in events {
            switch event {
            case .record(let sessionGeneration, let record):
                guard activeLogSessionGeneration == sessionGeneration else { continue }
                sessionLogWriter.append(record)
                switch record.source {
                case .stm32:
                    stmLogStore.append(record)
                case .s3:
                    s3LogStore.append(record)
                }
            }
        }
    }

    private func beginLogSession() {
        let descriptor = SessionLogWriter.makeDescriptor(connectedAt: Date())
        activeLogSessionGeneration = logReceivePipeline.beginSession()
        stmLogStore.clear()
        s3LogStore.clear()
        stmLogStore.setRecordingFileName(descriptor.fileName)
        s3LogStore.setRecordingFileName(descriptor.fileName)
        sessionLogWriter.beginSession(descriptor)
    }

    private func endLogSession() {
        activeLogSessionGeneration = nil
        logReceivePipeline.invalidateSession()
        stmLogStore.flush()
        s3LogStore.flush()
        stmLogStore.setRecordingFileName(nil)
        s3LogStore.setRecordingFileName(nil)
        sessionLogWriter.closeAndSynchronize()
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
                }
            case .poweredOff, .unauthorized, .unsupported, .resetting, .unknown:
                self.clearOutboundWrites()
                self.endLogSession()
                self.updateStatus(.unavailable)
            @unknown default:
                self.clearOutboundWrites()
                self.endLogSession()
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
            self.onDeviceNameChange?(self.discoveredDeviceName)
            self.setError(nil)
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.peripheral = peripheral
            peripheral.delegate = self
            self.writeCharacteristic = nil
            self.notifyCharacteristic = nil
            self.loggerCharacteristic = nil
            self.txNotifyConfirmed = false
            self.clearOutboundWrites()
            self.beginLogSession()
            self.updateStatus(.connecting)
            self.logDiagnostic("LINK_CONNECTED; discovering service=\(Self.serviceUUID.uuidString)")
            peripheral.discoverServices([Self.serviceUUID])
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            let message = error?.localizedDescription ?? "Connection failed"
            self.logDiagnostic("CONNECT_FAILED error=\(message)")
            self.endLogSession()
            self.setError(.connectionFailed(message))
            self.updateStatus(.failed(message))
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.writeCharacteristic = nil
            self.notifyCharacteristic = nil
            self.loggerCharacteristic = nil
            self.clearOutboundWrites()
            self.receivePipeline.reset()
            self.endLogSession()
            self.updateStatus(.disconnected)
            self.logDiagnostic("DISCONNECTED error=\(error?.localizedDescription ?? "none")")
            if let error {
                self.setError(.system(error.localizedDescription))
            }
        }
    }
}

extension BLEManager: CBPeripheralDelegate {
    nonisolated func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        let errorDescription = error?.localizedDescription
        Task { @MainActor [weak self] in
            guard let self else { return }
            guard errorDescription == nil else {
                self.logDiagnostic("SERVICE_DISCOVERY_FAILED error=\(errorDescription ?? "unknown")")
                self.setError(.system(errorDescription ?? "Unknown error"))
                self.updateStatus(.failed(errorDescription ?? "Service discovery failed"))
                return
            }
            guard let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID }) else {
                self.logDiagnostic("SERVICE_DISCOVERY_FAILED missing=\(Self.serviceUUID.uuidString)")
                self.setError(.serviceNotFound)
                self.updateStatus(.failed("Required BLE service not found"))
                return
            }
            self.logDiagnostic("SERVICE_DISCOVERED uuid=\(service.uuid.uuidString)")
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
                self.logDiagnostic("CHARACTERISTIC_DISCOVERY_FAILED error=\(errorDescription ?? "unknown")")
                self.setError(.system(errorDescription ?? "Unknown error"))
                self.updateStatus(.failed(errorDescription ?? "Characteristic discovery failed"))
                return
            }
            for characteristic in service.characteristics ?? [] {
                if characteristic.uuid == Self.rxCharacteristicUUID {
                    self.writeCharacteristic = characteristic
                } else if characteristic.uuid == Self.txCharacteristicUUID {
                    self.notifyCharacteristic = characteristic
                    peripheral.setNotifyValue(true, for: characteristic)
                } else if characteristic.uuid == Self.loggerCharacteristicUUID {
                    self.loggerCharacteristic = characteristic
                    peripheral.setNotifyValue(true, for: characteristic)
                }
            }
            let discovered = [
                self.writeCharacteristic != nil ? Self.rxCharacteristicUUID.uuidString : nil,
                self.notifyCharacteristic != nil ? Self.txCharacteristicUUID.uuidString : nil,
                self.loggerCharacteristic != nil ? Self.loggerCharacteristicUUID.uuidString : nil
            ].compactMap { $0 }.joined(separator: ",")
            self.logDiagnostic("CHARACTERISTICS_DISCOVERED values=\(discovered) tx_notify_pending=\(!self.txNotifyConfirmed)")
            guard self.writeCharacteristic != nil,
                  self.notifyCharacteristic != nil,
                  self.loggerCharacteristic != nil else {
                self.logDiagnostic("GATT_NOT_READY missing_required_characteristic")
                self.setError(.system("Required BLE characteristics are missing"))
                self.updateStatus(.failed("Required BLE characteristics are missing"))
                return
            }
            self.tryFinalizeGattReady(peripheral: peripheral)
        }
    }

    nonisolated func peripheral(
        _ peripheral: CBPeripheral,
        didUpdateNotificationStateFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        let errorDescription = error?.localizedDescription
        Task { @MainActor [weak self] in
            guard let self, self.peripheral === peripheral else { return }
            if characteristic.uuid == Self.txCharacteristicUUID {
                self.txNotifyConfirmed = errorDescription == nil && characteristic.isNotifying
                self.logDiagnostic("FFE2_NOTIFY_STATE enabled=\(characteristic.isNotifying ? 1 : 0) error=\(errorDescription ?? "none")")
                if let errorDescription {
                    self.setError(.system("FFE2 notification setup failed: \(errorDescription)"))
                    self.updateStatus(.failed(errorDescription))
                    return
                }
            } else if characteristic.uuid == Self.loggerCharacteristicUUID {
                self.logDiagnostic("FFE3_NOTIFY_STATE enabled=\(characteristic.isNotifying ? 1 : 0) error=\(errorDescription ?? "none")")
            } else {
                return
            }
            self.tryFinalizeGattReady(peripheral: peripheral)
        }
    }

    nonisolated func peripheral(
        _ peripheral: CBPeripheral,
        didWriteValueFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        let errorDescription = error?.localizedDescription
        Task { @MainActor [weak self] in
            guard let self else { return }
            guard self.peripheral === peripheral,
                  characteristic.uuid == Self.rxCharacteristicUUID else { return }

            let completed = self.inFlightWrite
            self.inFlightWrite = nil
            self.completedWriteCount &+= 1
            if let errorDescription {
                self.failedWriteCount &+= 1
                self.logDiagnostic("WRITE_COMPLETE result=FAIL count=\(self.completedWriteCount) failed=\(self.failedWriteCount) error=\(errorDescription)")
                let shouldDisconnect = self.disconnectAfterStop
                self.clearOutboundWrites()
                self.setError(.system(errorDescription))
                if shouldDisconnect {
                    self.endLogSession()
                    self.central.cancelPeripheralConnection(peripheral)
                }
                return
            }

            if let completed, self.shouldLogWrite(self.completedWriteCount, kind: completed.kind) {
                self.logDiagnostic("WRITE_COMPLETE result=OK count=\(self.completedWriteCount) kind=\(completed.kind) bytes=\(completed.data.count)")
            }

            if completed?.kind == .stop {
                self.stopBarrierPending = false
                if self.disconnectAfterStop {
                    // A disconnect stop is a terminal barrier: do not release
                    // any motion captured while the stop write was in flight.
                    self.clearOutboundWrites()
                    self.endLogSession()
                    self.central.cancelPeripheralConnection(peripheral)
                    return
                }
                if let pendingMotionAfterStop = self.pendingMotionAfterStop {
                    self.pendingMotionAfterStop = nil
                    self.enqueueOutboundWrite(pendingMotionAfterStop)
                }
            }

            if let writeCharacteristic = self.writeCharacteristic {
                self.pumpWriteQueue(peripheral: peripheral, characteristic: writeCharacteristic)
            }
        }
    }

    private func tryFinalizeGattReady(peripheral: CBPeripheral) {
        guard self.peripheral === peripheral,
              status == .connecting,
              writeCharacteristic != nil,
              notifyCharacteristic != nil,
              loggerCharacteristic != nil,
              txNotifyConfirmed else {
            return
        }
        updateStatus(.connected)
        logDiagnostic("GATT_READY rx=1 tx=1 ffe3=1 ffe2_notify=1")
        pumpWriteQueueIfPossible()
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
            receivePipeline.submit(data) { [weak self] events in
                Task { @MainActor [weak self] in
                    self?.handleReceivedEvents(events)
                }
            }
        } else if characteristic.uuid == Self.loggerCharacteristicUUID {
            logReceivePipeline.submit(data) { [weak self] events in
                Task { @MainActor [weak self] in
                    self?.handleReceivedLogEvents(events)
                }
            }
        }
    }
}
