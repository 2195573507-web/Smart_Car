import Foundation

#if canImport(CoreBluetooth)
import CoreBluetooth

public final class BLEManager: NSObject, VehicleTransport, @unchecked Sendable, CBCentralManagerDelegate, CBPeripheralDelegate {
    public let serviceUUID: CBUUID?
    public let writeCharacteristicUUID: CBUUID?
    public let notifyCharacteristicUUID: CBUUID?
    public private(set) var state: BLEState = .idle
    public private(set) var devices: [BLEDevice] = []
    public let incomingPackets: AsyncStream<Packet>
    private let central: CBCentralManager
    private var continuation: AsyncStream<Packet>.Continuation?
    private var peripheral: CBPeripheral?
    private var peripherals: [UUID: CBPeripheral] = [:]
    private var writeCharacteristic: CBCharacteristic?

    public init(serviceUUID: String? = nil, writeCharacteristicUUID: String? = nil, notifyCharacteristicUUID: String? = nil) {
        self.serviceUUID = serviceUUID.map(CBUUID.init(string:))
        self.writeCharacteristicUUID = writeCharacteristicUUID.map(CBUUID.init(string:))
        self.notifyCharacteristicUUID = notifyCharacteristicUUID.map(CBUUID.init(string:))
        var streamContinuation: AsyncStream<Packet>.Continuation?
        incomingPackets = AsyncStream { streamContinuation = $0 }
        continuation = streamContinuation
        central = CBCentralManager(delegate: nil, queue: nil)
        super.init()
        central.delegate = self
    }

    public func startScanning() { guard central.state == .poweredOn else { return }; state = .scanning; central.scanForPeripherals(withServices: serviceUUID.map { [$0] }, options: nil) }
    public func stopScanning() { central.stopScan(); if state == .scanning { state = .idle } }
    public func connect(to device: BLEDevice) { guard let peripheral = peripherals[device.id] ?? central.retrievePeripherals(withIdentifiers: [device.id]).first else { state = .failed("Device is no longer available") ; return }; state = .connecting; self.peripheral = peripheral; peripheral.delegate = self; central.connect(peripheral) }
    public func disconnect() { if let peripheral { central.cancelPeripheralConnection(peripheral) } }
    public func send(_ packet: Packet) async throws { guard let characteristic = writeCharacteristic, let peripheral else { throw BLEError.notReady }; peripheral.writeValue(packet.encoded(), for: characteristic, type: .withResponse) }

    public func centralManagerDidUpdateState(_ central: CBCentralManager) { if central.state != .poweredOn { state = central.state == .poweredOff ? .poweredOff : .unavailable } }
    public func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) { peripherals[peripheral.identifier] = peripheral; let device = BLEDevice(id: peripheral.identifier, name: peripheral.name ?? "Smart Car", rssi: RSSI.intValue); if let index = devices.firstIndex(where: { $0.id == device.id }) { devices[index] = device } else { devices.append(device) } }
    public func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) { state = .connected; peripheral.discoverServices(serviceUUID.map { [$0] }) }
    public func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) { state = .disconnected; writeCharacteristic = nil }
    public func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) { state = .failed(error?.localizedDescription ?? "Connection failed") }
    public func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) { peripheral.services?.forEach { peripheral.discoverCharacteristics([writeCharacteristicUUID, notifyCharacteristicUUID].compactMap { $0 }, for: $0) } }
    public func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) { for characteristic in service.characteristics ?? [] { if characteristic.uuid == writeCharacteristicUUID { writeCharacteristic = characteristic }; if characteristic.uuid == notifyCharacteristicUUID { peripheral.setNotifyValue(true, for: characteristic) } } }
    public func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) { guard let data = characteristic.value, let packet = try? PacketDecoder.decode(data) else { return }; continuation?.yield(packet) }
}

public enum BLEError: Error { case notReady }
#else
public final class BLEManager: VehicleTransport, @unchecked Sendable {
    public private(set) var state: BLEState = .unavailable
    public private(set) var devices: [BLEDevice] = []
    public let incomingPackets: AsyncStream<Packet> = AsyncStream { $0.finish() }
    public init(serviceUUID: String? = nil, writeCharacteristicUUID: String? = nil, notifyCharacteristicUUID: String? = nil) {}
    public func startScanning() {}
    public func stopScanning() {}
    public func connect(to device: BLEDevice) {}
    public func disconnect() {}
    public func send(_ packet: Packet) async throws { throw BLEError.notReady }
}
public enum BLEError: Error { case notReady }
#endif
