import Foundation
import SmartCarAppCore

struct SessionLogDescriptor: Sendable {
    let fileName: String
    let fileURL: URL
    let connectedAt: Date
}

final class SessionLogWriter: @unchecked Sendable {
    static let logDirectoryURL = URL(
        fileURLWithPath: "/Users/zhiqin/Projects/Smart_Car/LOG",
        isDirectory: true
    )

    private let queue = DispatchQueue(
        label: "SmartCar_Control_MAC.session-log.writer",
        qos: .utility
    )
    private var fileHandle: FileHandle?
    private lazy var timestampFormatter = Self.makeFormatter("yyyy-MM-dd HH:mm:ss.SSS ZZZZZ")

    static func makeDescriptor(connectedAt: Date) -> SessionLogDescriptor {
        let fileName = "smartcar_log_\(format(connectedAt, as: "yyyy-MM-dd_HH-mm-ss")).md"
        return SessionLogDescriptor(
            fileName: fileName,
            fileURL: logDirectoryURL.appendingPathComponent(fileName, isDirectory: false),
            connectedAt: connectedAt
        )
    }

    static func ensureLogDirectory() throws {
        try FileManager.default.createDirectory(
            at: logDirectoryURL,
            withIntermediateDirectories: true
        )
    }

    func beginSession(_ descriptor: SessionLogDescriptor) {
        queue.async {
            self.openSession(descriptor)
        }
    }

    func append(_ record: SmartCarLogRecord) {
        queue.async {
            self.appendRecord(record)
        }
    }

    func closeAndSynchronize() {
        queue.async {
            self.closeCurrentFile()
        }
    }

    private func openSession(_ descriptor: SessionLogDescriptor) {
        closeCurrentFile()

        do {
            try Self.ensureLogDirectory()
            guard FileManager.default.createFile(atPath: descriptor.fileURL.path, contents: nil) else {
                throw CocoaError(.fileWriteUnknown)
            }

            let handle = try FileHandle(forWritingTo: descriptor.fileURL)
            let header = """
            # SmartCar Session Log
            - **Connected Time**: \(timestampFormatter.string(from: descriptor.connectedAt))
            - **Device**: ESP32-S3 Gateway
            ---

            """
            try handle.write(contentsOf: Data(header.utf8))
            try handle.synchronize()
            fileHandle = handle
        } catch {
            print("Session log creation failed: \(error.localizedDescription)")
        }
    }

    private func appendRecord(_ record: SmartCarLogRecord) {
        guard let fileHandle else { return }

        let line = "\(timestampFormatter.string(from: record.receivedAt)) [\(record.source.displayName)][\(record.level.displayName)] \(record.message)\n"
        do {
            try fileHandle.write(contentsOf: Data(line.utf8))
        } catch {
            print("Session log append failed: \(error.localizedDescription)")
        }
    }

    private func closeCurrentFile() {
        guard let fileHandle else { return }
        defer { self.fileHandle = nil }

        do {
            try fileHandle.synchronize()
        } catch {
            print("Session log synchronization failed: \(error.localizedDescription)")
        }

        do {
            try fileHandle.close()
        } catch {
            print("Session log close failed: \(error.localizedDescription)")
        }
    }

    private static func format(_ date: Date, as format: String) -> String {
        makeFormatter(format).string(from: date)
    }

    private static func makeFormatter(_ format: String) -> DateFormatter {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.timeZone = .current
        formatter.dateFormat = format
        return formatter
    }
}
