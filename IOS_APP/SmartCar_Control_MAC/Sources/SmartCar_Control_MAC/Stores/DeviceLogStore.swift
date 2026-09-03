import Combine
import Foundation

private struct BoundedLogRingBuffer<Element> {
    let capacity: Int
    private var storage: [Element?]
    private var writeIndex = 0
    private(set) var count = 0

    init(capacity: Int) {
        self.capacity = max(1, capacity)
        self.storage = Array(repeating: nil, count: max(1, capacity))
    }

    @discardableResult
    mutating func append(_ element: Element) -> Bool {
        let dropped = count == capacity
        storage[writeIndex] = element
        writeIndex = (writeIndex + 1) % capacity
        if !dropped {
            count += 1
        }
        return dropped
    }

    var elements: [Element] {
        guard count > 0 else { return [] }
        let start = count == capacity ? writeIndex : 0
        return (0..<count).compactMap { storage[(start + $0) % capacity] }
    }

    mutating func removeAll() {
        storage = Array(repeating: nil, count: capacity)
        writeIndex = 0
        count = 0
    }
}

@MainActor
final class DeviceLogStore: ObservableObject {
    static let capacity = 500

    let source: SmartCarLogSource
    @Published private(set) var records: [SmartCarLogRecord] = []
    @Published private(set) var recordingFileName: String?
    private var buffer = BoundedLogRingBuffer<SmartCarLogRecord>(capacity: DeviceLogStore.capacity)
    private var pendingPublicationCount = 0

    init(source: SmartCarLogSource) {
        self.source = source
    }

    func append(_ record: SmartCarLogRecord) {
        guard record.source == source else { return }
        pendingPublicationCount += 1
        _ = buffer.append(record)
        if records.isEmpty || pendingPublicationCount >= 8 || record.level >= .warn {
            publish()
        }
    }

    func clear() {
        buffer.removeAll()
        pendingPublicationCount = 0
        records.removeAll(keepingCapacity: true)
    }

    func flush() {
        publish()
    }

    func setRecordingFileName(_ fileName: String?) {
        recordingFileName = fileName
    }

    private func publish() {
        records = buffer.elements
        pendingPublicationCount = 0
    }
}
