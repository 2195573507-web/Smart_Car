import CoreBluetooth
import Foundation

private enum BLEReceiveEvent {
    case message(DecodedMessageRecord)
    case decodeFailure(String)
}

private enum BLELogReceiveEvent {
    case record(SmartCarLogRecord)
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
    private var peripheral: CBPeripheral?
    private var writeCharacteristic: CBCharacteristic?
    private var notifyCharacteristic: CBCharacteristic?
    private var loggerCharacteristic: CBCharacteristic?
    private var scanWhenPoweredOn = false

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

    func startScanning() {
        guard central.state == .poweredOn else {
            scanWhenPoweredOn = true
            setError(nil)
            return
        }

        decodedMessages.removeAll(keepingCapacity: true)
        receivePipeline.reset()
        logReceivePipeline.reset()
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

    private func sendFrame(_ data: Data) {
        guard let peripheral, let writeCharacteristic else {
            setError(.notConnected)
            return
        }
        peripheral.writeValue(data, for: writeCharacteristic, type: .withResponse)
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
            self.onDeviceNameChange?(self.discoveredDeviceName)
            self.setError(nil)
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.peripheral = peripheral
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
        }
    }

    nonisolated func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.writeCharacteristic = nil
            self.notifyCharacteristic = nil
            self.loggerCharacteristic = nil
            self.receivePipeline.reset()
            self.logReceivePipeline.reset()
            self.updateStatus(.disconnected)
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
                } else if characteristic.uuid == Self.txCharacteristicUUID {
                    self.notifyCharacteristic = characteristic
                    peripheral.setNotifyValue(true, for: characteristic)
                } else if characteristic.uuid == Self.loggerCharacteristicUUID {
                    self.loggerCharacteristic = characteristic
                    peripheral.setNotifyValue(true, for: characteristic)
                }
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
