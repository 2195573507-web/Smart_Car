import Foundation

enum LogLevel: String, CaseIterable, Sendable {
    case off = "OFF"
    case error = "ERROR"
    case warn = "WARN"
    case info = "INFO"
    case debug = "DEBUG"
    case trace = "TRACE"

    /// Lower-ranked entries are at least as important as the selected level.
    /// `OFF` is handled explicitly so it never admits an entry.
    func includes(_ entryLevel: Self) -> Bool {
        guard self != .off else { return false }
        return entryLevel != .off && entryLevel.rank <= rank
    }

    private var rank: Int {
        switch self {
        case .off:  Int.max
        case .error: 0
        case .warn: 1
        case .info: 2
        case .debug: 3
        case .trace: 4
        }
    }

    static func inferred(from message: String) -> Self {
        let prefix = message.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        if prefix.hasPrefix("[ERROR]") || prefix.hasPrefix("ERROR:") { return .error }
        if prefix.hasPrefix("[WARN]") || prefix.hasPrefix("[WARNING]") || prefix.hasPrefix("WARN:") || prefix.hasPrefix("WARNING:") { return .warn }
        if prefix.hasPrefix("[TRACE]") || prefix.hasPrefix("TRACE:") { return .trace }
        if prefix.hasPrefix("[DEBUG]") || prefix.hasPrefix("DEBUG:") { return .debug }
        if prefix.hasPrefix("[INFO]") || prefix.hasPrefix("INFO:") { return .info }
        return .info
    }
}

struct LogEntry: Identifiable, Equatable, Sendable {
    let id: UInt64
    let level: LogLevel
    let message: String
}

struct LoggerStatistics: Equatable, Sendable {
    let capacity: Int
    let storedLineCount: Int
    let droppedLineCount: UInt64

    var utilization: Double {
        guard capacity > 0 else { return 0 }
        return Double(storedLineCount) / Double(capacity)
    }
}

struct LogRingBuffer<Element> {
    let capacity: Int
    private var storage: [Element?]
    private var oldestIndex = 0
    private(set) var count = 0
    private(set) var droppedCount: UInt64 = 0

    init(capacity: Int) {
        precondition(capacity > 0, "Log Ring Buffer capacity must be positive")
        self.capacity = capacity
        storage = Array(repeating: nil, count: capacity)
    }

    mutating func append(_ element: Element) {
        if count < capacity {
            storage[(oldestIndex + count) % capacity] = element
            count += 1
            return
        }

        storage[oldestIndex] = element
        oldestIndex = (oldestIndex + 1) % capacity
        droppedCount += 1
    }

    mutating func removeAll() {
        storage = Array(repeating: nil, count: capacity)
        oldestIndex = 0
        count = 0
        droppedCount = 0
    }

    var elements: [Element] {
        (0..<count).compactMap { storage[(oldestIndex + $0) % capacity] }
    }
}

struct LogLineAssembler {
    private(set) var pendingLine = ""
    private let maximumPendingLineCharacters: Int

    init(maximumPendingLineCharacters: Int = 16_384) {
        precondition(maximumPendingLineCharacters > 0, "Maximum pending line length must be positive")
        self.maximumPendingLineCharacters = maximumPendingLineCharacters
    }

    mutating func reset() {
        pendingLine.removeAll(keepingCapacity: true)
    }

    mutating func append(_ text: String) -> [String] {
        pendingLine.append(text)
        var lines: [String] = []

        while pendingLine.utf8.contains(0x0A) {
            let bytes = Array(pendingLine.utf8)
            guard let newlineOffset = bytes.firstIndex(of: 0x0A) else { break }
            var line = String(decoding: bytes[..<newlineOffset], as: UTF8.self)
            if line.last == "\r" {
                line.removeLast()
            }
            lines.append(line)
            pendingLine = String(decoding: bytes.dropFirst(newlineOffset + 1), as: UTF8.self)
        }

        while pendingLine.count > maximumPendingLineCharacters {
            let split = pendingLine.index(pendingLine.startIndex, offsetBy: maximumPendingLineCharacters)
            lines.append(String(pendingLine[..<split]))
            pendingLine.removeSubrange(..<split)
        }
        return lines
    }
}
