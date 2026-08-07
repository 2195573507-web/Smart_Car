import Foundation

enum SmartCarLogSource: UInt8, CaseIterable, Sendable {
    case stm32 = 0
    case s3 = 1

    var displayName: String {
        switch self {
        case .stm32: return "STM32"
        case .s3: return "S3"
        }
    }
}

enum SmartCarLogLevel: UInt8, CaseIterable, Comparable, Sendable {
    case debug = 0
    case info = 1
    case warn = 2
    case error = 3

    static func < (lhs: SmartCarLogLevel, rhs: SmartCarLogLevel) -> Bool {
        lhs.rawValue < rhs.rawValue
    }

    var displayName: String {
        switch self {
        case .debug: return "DEBUG"
        case .info: return "INFO"
        case .warn: return "WARN"
        case .error: return "ERROR"
        }
    }
}

struct SmartCarLogRecord: Identifiable, Equatable, Sendable {
    let id: UUID
    let source: SmartCarLogSource
    let level: SmartCarLogLevel
    let timestampMilliseconds: UInt32
    let message: String
    let receivedAt: Date

    init(
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

struct SmartCarLogParser: Sendable {
    private static let head: [UInt8] = [0xAA, 0x55]
    private static let version: UInt8 = 0x01
    private static let headerSize = 10
    private static let crcSize = 2
    private static let maxPayload = 96

    private var buffer: [UInt8] = []

    mutating func reset() {
        buffer.removeAll(keepingCapacity: true)
    }

    mutating func feed(_ data: Data, receivedAt: Date) -> [SmartCarLogRecord] {
        buffer.append(contentsOf: data)
        var records: [SmartCarLogRecord] = []

        while true {
            guard let headIndex = buffer.indices.first(where: {
                $0 + 1 < buffer.count && buffer[$0] == Self.head[0] && buffer[$0 + 1] == Self.head[1]
            }) else {
                buffer.removeAll(keepingCapacity: true)
                break
            }
            if headIndex > 0 {
                buffer.removeFirst(headIndex)
            }
            guard buffer.count >= Self.headerSize else { break }

            guard buffer[2] == Self.version,
                  let source = SmartCarLogSource(rawValue: buffer[3]),
                  let level = SmartCarLogLevel(rawValue: buffer[4]) else {
                buffer.removeFirst()
                continue
            }

            let payloadLength = Int(buffer[9])
            guard payloadLength <= Self.maxPayload else {
                buffer.removeFirst()
                continue
            }
            let frameLength = Self.headerSize + payloadLength + Self.crcSize
            guard buffer.count >= frameLength else { break }

            let crcOffset = Self.headerSize + payloadLength
            let receivedCRC = UInt16(buffer[crcOffset]) | (UInt16(buffer[crcOffset + 1]) << 8)
            let calculatedCRC = Self.crc16Modbus(Array(buffer[2..<crcOffset]))
            guard receivedCRC == calculatedCRC else {
                buffer.removeFirst()
                continue
            }

            let timestamp = UInt32(buffer[5]) |
                (UInt32(buffer[6]) << 8) |
                (UInt32(buffer[7]) << 16) |
                (UInt32(buffer[8]) << 24)
            let message = String(decoding: buffer[Self.headerSize..<crcOffset], as: UTF8.self)
            records.append(
                SmartCarLogRecord(
                    source: source,
                    level: level,
                    timestampMilliseconds: timestamp,
                    message: message,
                    receivedAt: receivedAt
                )
            )
            buffer.removeFirst(frameLength)
        }
        return records
    }

    private static func crc16Modbus(_ bytes: [UInt8]) -> UInt16 {
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
