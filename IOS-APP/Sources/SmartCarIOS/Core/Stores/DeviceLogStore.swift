import Combine
import Foundation
import SmartCarAppCore

@MainActor
final class DeviceLogStore: ObservableObject {
    static let capacity = 500

    let source: SmartCarLogSource
    @Published private(set) var records: [SmartCarLogRecord] = []
    private var buffer = AppBLEBoundedRingBuffer<SmartCarLogRecord>(capacity: DeviceLogStore.capacity)
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

    private func publish() {
        records = buffer.elements
        pendingPublicationCount = 0
    }
}
