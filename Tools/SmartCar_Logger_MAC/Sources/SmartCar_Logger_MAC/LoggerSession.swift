import Foundation
import AppKit
import Observation

@MainActor
@Observable
final class LoggerSession {
    private(set) var ports: [SerialPortDescriptor] = []
    var selectedPortID: String = ""
    var displayLevel: LogLevel = .info {
        didSet {
            publishVisibleLogEntries()
        }
    }
    private(set) var state: LoggerConnectionState = .disconnected
    private(set) var visibleLogEntries: [LogEntry] = []
    private(set) var loggerStatistics = LoggerStatistics(
        capacity: LoggerSession.defaultMaximumLogLines,
        storedLineCount: 0,
        droppedLineCount: 0
    )
    private(set) var receivedBytes = 0
    private(set) var lastReadByteCount = 0
    private(set) var lastReadAt: Date?
    private(set) var lastReadHex = ""
    private(set) var fileDescriptor: Int32?
    private(set) var readSourceActive = false
    private(set) var startedAt: Date?

    static let defaultMaximumLogLines = 500
    static let serialBaudRate = 115_200

    private let serial = SerialPortService()
    private var decoder = UTF8StreamDecoder()
    private var lineAssembler = LogLineAssembler()
    private var logBuffer = LogRingBuffer<LogEntry>(capacity: LoggerSession.defaultMaximumLogLines)
    private var activeConnectionID: UUID?
    private var displayFlushScheduled = false
    private var portRefreshTimer: Timer?
    private var nextLogID: UInt64 = 0

    init() {
        refreshPorts()
        startPortScanning()
    }

    func startPortScanning() {
        guard portRefreshTimer == nil else { return }
        portRefreshTimer = Timer.scheduledTimer(withTimeInterval: 2, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.refreshPorts() }
        }
    }

    func stopPortScanning() {
        portRefreshTimer?.invalidate()
        portRefreshTimer = nil
    }

    var selectedPort: SerialPortDescriptor? {
        ports.first { $0.id == selectedPortID }
    }

    func refreshPorts() {
        ports = SerialPortScanner.scan()
        if !ports.contains(where: { $0.id == selectedPortID }) {
            selectedPortID = ports.first?.id ?? ""
        }
    }

    func connect() {
        guard let port = selectedPort else { return }
        disconnect()
        state = .connecting
        let connectionID = UUID()
        // `SerialPortService.open` resumes its read source before it returns.
        // Establish this connection first so reset-time bytes are not dropped.
        decoder.reset()
        lineAssembler.reset()
        lastReadByteCount = 0
        lastReadAt = nil
        lastReadHex = ""
        fileDescriptor = nil
        readSourceActive = false
        activeConnectionID = connectionID
        do {
            let openInfo = try serial.open(
                path: port.path,
                onData: { [weak self] data in
                    Task { @MainActor in self?.append(data: data, for: connectionID) }
                },
                onDiagnostic: { [weak self] message in
                    Task { @MainActor in self?.appendHostDiagnostic(message, level: .debug, for: connectionID) }
                },
                onError: { [weak self] message in
                    Task { @MainActor in self?.handleError(message, for: connectionID) }
                }
            )
            fileDescriptor = openInfo.fileDescriptor
            readSourceActive = openInfo.readSourceActive
            startedAt = Date()
            state = .connected
            appendHostDiagnostic("OPEN fd=\(openInfo.fileDescriptor) read_source_active=\(openInfo.readSourceActive)", level: .info)
        } catch {
            activeConnectionID = nil
            state = .failed(error.localizedDescription)
        }
    }

    func disconnect() {
        activeConnectionID = nil
        serial.close()
        fileDescriptor = nil
        readSourceActive = false
        if state != .disconnected {
            state = .disconnected
        }
    }

    func clearLog() {
        logBuffer.removeAll()
        visibleLogEntries = []
        updateLoggerStatistics()
        receivedBytes = 0
        lastReadByteCount = 0
        lastReadAt = nil
        lastReadHex = ""
        decoder.reset()
        lineAssembler.reset()
    }

    /// Copies one bounded snapshot of the Ring Buffer to the macOS clipboard.
    /// The exported text is intentionally assembled locally and is not retained.
    func copyAllLogs() {
        let entries = logBuffer.elements
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        let header = [
            "时间: \(formatter.string(from: Date()))",
            "串口: \(selectedPort?.path ?? "未选择")",
            "波特率: \(Self.serialBaudRate)",
            "当前buffer数量: \(entries.count)",
            "dropped数量: \(logBuffer.droppedCount)",
            ""
        ]
        let content = header + entries.map(\.message)
        let exportText = content.joined(separator: "\n")

        let pasteboard = NSPasteboard.general
        pasteboard.clearContents()
        _ = pasteboard.setString(exportText, forType: .string)
    }

    private func append(data: Data, for connectionID: UUID) {
        guard activeConnectionID == connectionID else { return }
        receivedBytes += data.count
        lastReadByteCount = data.count
        lastReadAt = Date()
        lastReadHex = Self.hexPreview(data)
        let lines = lineAssembler.append(decoder.decode(data))
        for line in lines {
            appendLine(line, level: LogLevel.inferred(from: line))
        }
        scheduleDisplayFlush()
    }

    private func appendHostDiagnostic(_ message: String, level: LogLevel, for connectionID: UUID? = nil) {
        if let connectionID, activeConnectionID != connectionID { return }
        flushDisplay()
        appendLine("[LOGGER] \(message)", level: level)
        publishVisibleLogEntries()
    }

    private static func hexPreview(_ data: Data) -> String {
        let preview = data.prefix(64)
            .map { String(format: "%02X", $0) }
            .joined(separator: " ")
        return data.count > 64 ? "\(preview) ..." : preview
    }

    private func handleError(_ message: String, for connectionID: UUID) {
        guard activeConnectionID == connectionID else { return }
        activeConnectionID = nil
        serial.close()
        fileDescriptor = nil
        readSourceActive = false
        appendHostDiagnostic("READ_ERROR \(message)", level: .error)
        state = .failed(message)
    }

    private func scheduleDisplayFlush() {
        guard !displayFlushScheduled else { return }
        displayFlushScheduled = true
        Task { [weak self] in
            try? await Task.sleep(for: .milliseconds(100))
            self?.flushDisplay()
        }
    }

    private func flushDisplay() {
        displayFlushScheduled = false
        publishVisibleLogEntries()
    }

    private func appendLine(_ message: String, level: LogLevel) {
        nextLogID &+= 1
        logBuffer.append(LogEntry(id: nextLogID, level: level, message: message))
    }

    private func publishVisibleLogEntries() {
        visibleLogEntries = logBuffer.elements.filter { displayLevel.includes($0.level) }
        updateLoggerStatistics()
    }

    private func updateLoggerStatistics() {
        loggerStatistics = LoggerStatistics(
            capacity: logBuffer.capacity,
            storedLineCount: logBuffer.count,
            droppedLineCount: logBuffer.droppedCount
        )
    }
}

struct UTF8StreamDecoder {
    private var pending = Data()

    mutating func reset() {
        pending.removeAll(keepingCapacity: true)
    }

    mutating func decode(_ data: Data) -> String {
        pending.append(data)
        guard !pending.isEmpty else { return "" }

        let bytes = [UInt8](pending)
        var completeCount = bytes.count

        // Keep a valid but incomplete UTF-8 sequence for the next serial chunk.
        var continuationCount = 0
        while completeCount > 0, (bytes[completeCount - 1] & 0xC0) == 0x80 {
            continuationCount += 1
            completeCount -= 1
        }
        if completeCount == 0 {
            // There is no lead byte in this chunk, so these are malformed
            // continuations rather than an incomplete sequence.
            completeCount = bytes.count
        } else {
            let lead = bytes[completeCount - 1]
            let expectedLength: Int
            switch lead {
            case 0xC2...0xDF: expectedLength = 2
            case 0xE0...0xEF: expectedLength = 3
            case 0xF0...0xF4: expectedLength = 4
            default: expectedLength = 1
            }
            if expectedLength > continuationCount + 1 {
                completeCount -= 1
            }
        }

        let decoded = String(decoding: bytes[..<completeCount], as: UTF8.self)
        pending = Data(bytes[completeCount...])
        return decoded
    }
}
