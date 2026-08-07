import Combine
import Foundation

@MainActor
final class DeviceLogStore: ObservableObject {
    static let capacity = 500

    let source: SmartCarLogSource
    @Published private(set) var records: [SmartCarLogRecord] = []

    init(source: SmartCarLogSource) {
        self.source = source
    }

    func append(_ record: SmartCarLogRecord) {
        guard record.source == source else { return }
        records.append(record)
        if records.count > Self.capacity {
            records.removeFirst(records.count - Self.capacity)
        }
    }

    func clear() {
        records.removeAll(keepingCapacity: true)
    }
}
