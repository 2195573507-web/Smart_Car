import Foundation

public protocol VehicleTransport: AnyObject, Sendable {
    var state: BLEState { get }
    var devices: [BLEDevice] { get }
    func startScanning()
    func stopScanning()
    func connect(to device: BLEDevice)
    func disconnect()
    func send(_ packet: Packet) async throws
    var incomingPackets: AsyncStream<Packet> { get }
}
