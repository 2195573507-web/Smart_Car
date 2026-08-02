import Foundation
import Observation

@MainActor
@Observable
final class LoggerSession {
    private(set) var ports: [SerialPortDescriptor] = []
    var selectedPortID: String = ""
    private(set) var state: LoggerConnectionState = .disconnected
    private(set) var logText = ""
    private(set) var receivedBytes = 0
    private(set) var startedAt: Date?
    var saveRequest = SaveRequest()

    private let serial = SerialPortService()
    private var decoder = UTF8StreamDecoder()
    private var activeConnectionID: UUID?
    private var pendingDisplayText = ""
    private var displayFlushScheduled = false
    private var portRefreshTimer: Timer?

    private let displayCharacterLimit = 2_000_000

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
        do {
            try serial.open(
                path: port.path,
                onData: { [weak self] data in
                    Task { @MainActor in self?.append(data: data, for: connectionID) }
                },
                onError: { [weak self] message in
                    Task { @MainActor in self?.handleError(message, for: connectionID) }
                }
            )
            decoder.reset()
            startedAt = Date()
            activeConnectionID = connectionID
            state = .connected
        } catch {
            state = .failed(error.localizedDescription)
        }
    }

    func disconnect() {
        activeConnectionID = nil
        serial.close()
        if state != .disconnected {
            state = .disconnected
        }
    }

    func clearLog() {
        logText = ""
        receivedBytes = 0
        pendingDisplayText = ""
        decoder.reset()
    }

    func requestSave() {
        flushDisplay()
        saveRequest = SaveRequest(isPresented: true)
    }

    func save(to url: URL) throws {
        flushDisplay()
        try Data(logText.utf8).write(to: url, options: .atomic)
        saveRequest = SaveRequest()
    }

    private func append(data: Data, for connectionID: UUID) {
        guard activeConnectionID == connectionID else { return }
        receivedBytes += data.count
        pendingDisplayText.append(decoder.decode(data))
        scheduleDisplayFlush()
    }

    private func handleError(_ message: String, for connectionID: UUID) {
        guard activeConnectionID == connectionID else { return }
        activeConnectionID = nil
        serial.close()
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
        guard !pendingDisplayText.isEmpty else { return }
        logText.append(pendingDisplayText)
        pendingDisplayText = ""

        guard logText.count > displayCharacterLimit else { return }
        let retainedCount = displayCharacterLimit - 96
        let start = logText.index(logText.endIndex, offsetBy: -retainedCount)
        logText = "[Earlier log output discarded to keep the viewer responsive.]\n" + String(logText[start...])
    }
}

struct SaveRequest: Equatable {
    var isPresented = false
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
