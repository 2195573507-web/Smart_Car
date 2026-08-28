import Foundation

public enum SmartCarLogSource: UInt8, CaseIterable, Sendable {
    case stm32 = 0
    case s3 = 1

    public var displayName: String {
        switch self {
        case .stm32: return "STM32"
        case .s3: return "S3"
        }
    }
}

public enum SmartCarLogLevel: UInt8, CaseIterable, Comparable, Sendable {
    case debug = 0
    case info = 1
    case warn = 2
    case error = 3

    public static func < (lhs: SmartCarLogLevel, rhs: SmartCarLogLevel) -> Bool {
        lhs.rawValue < rhs.rawValue
    }

    public var displayName: String {
        switch self {
        case .debug: return "DEBUG"
        case .info: return "INFO"
        case .warn: return "WARN"
        case .error: return "ERROR"
        }
    }
}

public struct SmartCarLogRecord: Identifiable, Equatable, Sendable {
    public let id: UUID
    public let source: SmartCarLogSource
    public let level: SmartCarLogLevel
    public let timestampMilliseconds: UInt32
    public let message: String
    public let receivedAt: Date

    public init(
        source: SmartCarLogSource,
        level: SmartCarLogLevel,
        timestampMilliseconds: UInt32,
        message: String,
        receivedAt: Date
    ) {
        self.id = UUID()
        self.source = source
        self.level = level
        self.timestampMilliseconds = timestampMilliseconds
        self.message = message
        self.receivedAt = receivedAt
    }
}

public struct SmartCarLogParser: Sendable {
    private static let head: [UInt8] = [0xAA, 0x55]
    private static let version: UInt8 = 0x01
    private static let headerSize = 10
    private static let crcSize = 2
    private static let maxPayload = 96
    private static let maxBuffer = 2_048

    private var buffer: [UInt8] = []
    private var readIndex = 0

    public init() {}

    public mutating func reset() {
        buffer.removeAll(keepingCapacity: true)
        readIndex = 0
    }

    public mutating func feed(_ data: Data, receivedAt: Date) -> [SmartCarLogRecord] {
        buffer.append(contentsOf: data)
        if buffer.count > Self.maxBuffer {
            buffer = Array(buffer.suffix(Self.maxBuffer))
            readIndex = 0
        }
        var records: [SmartCarLogRecord] = []

        while true {
            while readIndex + 1 < buffer.count,
                  !(buffer[readIndex] == Self.head[0] && buffer[readIndex + 1] == Self.head[1]) {
                readIndex += 1
            }
            guard readIndex + Self.headerSize <= buffer.count else { break }

            guard buffer[readIndex + 2] == Self.version,
                  let source = SmartCarLogSource(rawValue: buffer[readIndex + 3]),
                  let level = SmartCarLogLevel(rawValue: buffer[readIndex + 4]) else {
                readIndex += 1
                continue
            }

            let payloadLength = Int(buffer[readIndex + 9])
            guard payloadLength <= Self.maxPayload else {
                readIndex += 1
                continue
            }
            let frameLength = Self.headerSize + payloadLength + Self.crcSize
            guard buffer.count - readIndex >= frameLength else { break }

            let crcOffset = readIndex + Self.headerSize + payloadLength
            let receivedCRC = UInt16(buffer[crcOffset]) | (UInt16(buffer[crcOffset + 1]) << 8)
            let calculatedCRC = Self.crc16Modbus(buffer[(readIndex + 2)..<crcOffset])
            guard receivedCRC == calculatedCRC else {
                readIndex += 1
                continue
            }

            let timestamp = UInt32(buffer[readIndex + 5]) |
                (UInt32(buffer[readIndex + 6]) << 8) |
                (UInt32(buffer[readIndex + 7]) << 16) |
                (UInt32(buffer[readIndex + 8]) << 24)
            let message = String(decoding: buffer[(readIndex + Self.headerSize)..<crcOffset], as: UTF8.self)
            records.append(
                SmartCarLogRecord(
                    source: source,
                    level: level,
                    timestampMilliseconds: timestamp,
                    message: message,
                    receivedAt: receivedAt
                )
            )
            readIndex += frameLength
        }
        compactIfNeeded()
        return records
    }

    private mutating func compactIfNeeded() {
        guard readIndex > 0 else { return }
        if readIndex >= buffer.count {
            buffer.removeAll(keepingCapacity: true)
            readIndex = 0
        } else if readIndex >= 256 || readIndex * 2 >= buffer.count {
            buffer = Array(buffer[readIndex...])
            readIndex = 0
        }
    }

    private static func crc16Modbus<S: Sequence>(_ bytes: S) -> UInt16 where S.Element == UInt8 {
        var crc: UInt16 = 0xFFFF
        for byte in bytes {
            crc ^= UInt16(byte)
            for _ in 0..<8 {
                crc = (crc & 1) == 1 ? (crc >> 1) ^ 0xA001 : crc >> 1
            }
        }
        return crc
    }
}
