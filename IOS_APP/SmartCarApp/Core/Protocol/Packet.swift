import Foundation

public struct Packet: Equatable, Sendable {
    public static let sof: [UInt8] = [0xA5, 0x5A]
    public static let currentVersion: UInt8 = 0x10
    public static let fixedHeaderLength: UInt8 = 24

    public var version: UInt8 = Packet.currentVersion
    public var flags: UInt8 = 0
    public var type: UInt8
    public var source: UInt8 = 0x01
    public var authority: UInt8
    public var sessionID: UInt32
    public var sequence: UInt32
    public var acknowledgement: UInt32 = 0
    public var timestampMilliseconds: UInt32
    public var payload: [UInt8]

    public init(version: UInt8 = Packet.currentVersion, flags: UInt8 = 0, type: UInt8, source: UInt8 = 0x01,
                authority: UInt8, sessionID: UInt32, sequence: UInt32, acknowledgement: UInt32 = 0,
                timestampMilliseconds: UInt32, payload: [UInt8] = []) {
        self.version = version
        self.flags = flags
        self.type = type
        self.source = source
        self.authority = authority
        self.sessionID = sessionID
        self.sequence = sequence
        self.acknowledgement = acknowledgement
        self.timestampMilliseconds = timestampMilliseconds
        self.payload = payload
    }

    public func encoded() -> Data {
        var bytes = Packet.sof
        bytes.append(contentsOf: [version, flags, Packet.fixedHeaderLength, type, source, authority])
        bytes.append(contentsOf: sessionID.littleEndianBytes)
        bytes.append(contentsOf: sequence.littleEndianBytes)
        bytes.append(contentsOf: acknowledgement.littleEndianBytes)
        bytes.append(contentsOf: timestampMilliseconds.littleEndianBytes)
        bytes.append(contentsOf: UInt16(payload.count).littleEndianBytes)
        bytes.append(contentsOf: payload)
        let crc = CRC16.ccittFalse(Data(bytes.dropFirst(2)))
        bytes.append(contentsOf: crc.littleEndianBytes)
        return Data(bytes)
    }
}

public enum PacketError: Error, Equatable {
    case tooShort
    case invalidSOF
    case invalidHeaderLength
    case invalidPayloadLength
    case unsupportedVersion
    case invalidCRC
}

public enum PacketDecoder {
    public static func decode(_ data: Data) throws -> Packet {
        let bytes = Array(data)
        if bytes.first == 0xAA {
            return try decodeSmartCarV1(bytes)
        }
        guard bytes.count >= 2 + Int(Packet.fixedHeaderLength) + 2 else { throw PacketError.tooShort }
        guard Array(bytes.prefix(2)) == Packet.sof else { throw PacketError.invalidSOF }
        let version = bytes[2]
        guard version >> 4 == Packet.currentVersion >> 4 else { throw PacketError.unsupportedVersion }
        let headerLength = Int(bytes[4])
        guard headerLength == Int(Packet.fixedHeaderLength) else { throw PacketError.invalidHeaderLength }
        let payloadLength = Int(UInt16(littleEndianBytes: Array(bytes[24..<26])))
        let expectedLength = 2 + headerLength + payloadLength + 2
        guard bytes.count == expectedLength else { throw PacketError.invalidPayloadLength }
        let receivedCRC = UInt16(littleEndianBytes: Array(bytes.suffix(2)))
        guard CRC16.ccittFalse(Data(bytes[2..<(bytes.count - 2)])) == receivedCRC else { throw PacketError.invalidCRC }
        return Packet(version: version, flags: bytes[3], type: bytes[5], source: bytes[6], authority: bytes[7], sessionID: UInt32(littleEndianBytes: Array(bytes[8..<12])), sequence: UInt32(littleEndianBytes: Array(bytes[12..<16])), acknowledgement: UInt32(littleEndianBytes: Array(bytes[16..<20])), timestampMilliseconds: UInt32(littleEndianBytes: Array(bytes[20..<24])), payload: Array(bytes[26..<(26 + payloadLength)]))
    }

    private static func decodeSmartCarV1(_ bytes: [UInt8]) throws -> Packet {
        guard bytes.count >= 8 else { throw PacketError.tooShort }
        guard bytes[0] == 0xAA, bytes[1] == 0x01, bytes.last == 0x55 else {
            throw PacketError.invalidSOF
        }
        let payloadLength = Int(UInt16(bytes[3]) | (UInt16(bytes[4]) << 8))
        guard payloadLength <= 128, bytes.count == payloadLength + 8 else {
            throw PacketError.invalidPayloadLength
        }
        let crcOffset = 5 + payloadLength
        let receivedCRC = UInt16(bytes[crcOffset]) | (UInt16(bytes[crcOffset + 1]) << 8)
        guard CRC16.modbus(bytes[1..<(5 + payloadLength)]) == receivedCRC else {
            throw PacketError.invalidCRC
        }
        return Packet(version: bytes[1], type: bytes[2], source: 0, authority: 0,
                      sessionID: 0, sequence: 0, timestampMilliseconds: 0,
                      payload: Array(bytes[5..<crcOffset]))
    }
}

public enum CRC16 {
    public static func ccittFalse(_ data: Data) -> UInt16 {
        var crc: UInt16 = 0xFFFF
        for byte in data {
            crc ^= UInt16(byte) << 8
            for _ in 0..<8 { crc = (crc & 0x8000) != 0 ? (crc << 1) ^ 0x1021 : crc << 1 }
        }
        return crc
    }

    public static func modbus(_ bytes: ArraySlice<UInt8>) -> UInt16 {
        var crc: UInt16 = 0xFFFF
        for byte in bytes {
            crc ^= UInt16(byte)
            for _ in 0..<8 {
                crc = (crc & 1) == 0 ? (crc >> 1) : (crc >> 1) ^ 0xA001
            }
        }
        return crc
    }
}

private extension FixedWidthInteger {
    var littleEndianBytes: [UInt8] {
        withUnsafeBytes(of: littleEndian) { Array($0) }
    }

    init(littleEndianBytes bytes: [UInt8]) {
        self = bytes.enumerated().reduce(into: Self(0)) { value, item in
            value |= Self(item.element) << Self(item.offset * 8)
        }
    }
}
