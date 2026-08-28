import Foundation

public enum AppBLEVersion: UInt8, Sendable {
    case v1 = 0x01
    case v2 = 0x02
}

public enum AppBLEV2Type: UInt8, Sendable {
    case hello = 0x70
    case helloAck = 0x71
    case heartbeat = 0x72
    case heartbeatAck = 0x73
    case commandAck = 0x74
    case command = 0x75
}

public enum AppBLEV2Result: UInt8, Sendable {
    case ok = 0x00
    case rejected = 0x01
    case sessionInvalid = 0x02
    case expired = 0x03
    case staleSequence = 0x04
    case busy = 0x05
}

public enum AppBLEV2AckStage: UInt8, Sendable {
    case gatewayAdmitted = 0x00
    case stm32Accepted = 0x01
    case stopQueued = 0x02
}

public struct AppBLEFrame: Identifiable, Equatable, Sendable {
    public let id: UUID
    public let raw: Data
    public let version: UInt8
    public let type: UInt8
    public let payload: Data

    public init(raw: Data, version: UInt8, type: UInt8, payload: Data) {
        self.id = UUID()
        self.raw = raw
        self.version = version
        self.type = type
        self.payload = payload
    }
}

public enum AppBLEFrameCodec {
    public static let head: UInt8 = 0xAA
    public static let tail: UInt8 = 0x55
    public static let maxPayload = 128

    public static func encode(
        version: AppBLEVersion,
        type: UInt8,
        payload: Data = Data()
    ) -> Data {
        precondition(payload.count <= maxPayload, "App-BLE payload exceeds 128 bytes")
        var frame = Data([head, version.rawValue, type,
                          UInt8(payload.count & 0xFF),
                          UInt8((payload.count >> 8) & 0xFF)])
        frame.append(payload)
        let crc = crc16Modbus(frame.dropFirst())
        frame.append(UInt8(crc & 0xFF))
        frame.append(UInt8(crc >> 8))
        frame.append(tail)
        return frame
    }

    public static func crc16Modbus<S: Sequence>(_ bytes: S) -> UInt16 where S.Element == UInt8 {
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

/// Source-compatible V1 API used by the existing app model layer. The parser
/// now accepts both V1 telemetry and V2 session frames while preserving the
/// historical V1 encoder and type names.
public enum SmartCarProtocol {
    public static let head: UInt8 = AppBLEFrameCodec.head
    public static let version: UInt8 = AppBLEVersion.v1.rawValue
    public static let tail: UInt8 = AppBLEFrameCodec.tail
    public static let maxPayload = AppBLEFrameCodec.maxPayload

    public enum FrameType: UInt8, Sendable {
        case control = 0x01
        case status = 0x02
        case ping = 0x05
        case ack = 0x06
        case imuStatus = 0x10
        case attitude = 0x11
        case imuCalibrationStatus = 0x12
        case imuCalibrationBias = 0x13
        case wheelSpeedCommand = 0x15
        case wheelSpeedStatus = 0x16
        case radarStatus = 0x1A
        case radarPWMControl = 0x1B
        case powerStatus = 0x1C
        case pidParams = 0x1D
        case imuCalibrationResult = 0x25
        case imuTelemetry = 0x27
        case dualIMUStatus = 0x28
        case chassisState = 0x29
        case wheelSpeedSingleCommand = 0x2A
        case masterSpeedCommand = 0x2B
        case wheelControlStatus = 0x2C
        case chassisSpeedCommand = 0x2D
        case chassisHeadingCommand = 0x2E

        public var displayName: String {
            switch self {
            case .control: return "CONTROL"
            case .status: return "STATUS"
            case .ping: return "PING"
            case .ack: return "ACK"
            case .imuStatus: return "IMU_STATUS"
            case .attitude: return "ATTITUDE"
            case .imuCalibrationStatus: return "IMU_CAL_STATUS"
            case .imuCalibrationBias: return "IMU_CAL_BIAS"
            case .wheelSpeedCommand: return "WHEEL_SPEED_CMD"
            case .wheelSpeedStatus: return "WHEEL_SPEED_STATUS"
            case .radarStatus: return "RADAR_STATUS"
            case .powerStatus: return "POWER_STATUS"
            case .radarPWMControl: return "RADAR_PWM_SET"
            case .imuCalibrationResult: return "IMU_CAL_RESULT"
            case .imuTelemetry: return "IMU_TELEMETRY"
            case .dualIMUStatus: return "DUAL_IMU_STATUS"
            case .chassisState: return "CHASSIS_STATE"
            case .wheelSpeedSingleCommand: return "WHEEL_SPEED_SINGLE_CMD"
            case .masterSpeedCommand: return "MASTER_SPEED_CMD"
            case .wheelControlStatus: return "WHEEL_CONTROL_STATUS"
            case .chassisSpeedCommand: return "CHASSIS_SPEED_CMD"
            case .chassisHeadingCommand: return "CHASSIS_HEADING_CMD"
            case .pidParams: return "PID_PARAMS_CMD"
            }
        }
    }

    public enum ControlCommand: UInt8, Sendable {
        case stop = 0x01
        case moveForward = 0x02
        case moveBack = 0x03
        case turnLeft = 0x04
        case turnRight = 0x05
        case speedControl = 0x06

        public var displayName: String {
            switch self {
            case .stop: return "STOP"
            case .moveForward: return "MOVE_FORWARD"
            case .moveBack: return "MOVE_BACK"
            case .turnLeft: return "TURN_LEFT"
            case .turnRight: return "TURN_RIGHT"
            case .speedControl: return "SPEED_CONTROL"
            }
        }
    }

    public struct Frame: Identifiable, Equatable, Sendable {
        public let id: UUID
        public let raw: Data
        public let version: UInt8
        public let type: UInt8
        public let payload: Data

        public init(raw: Data, version: UInt8 = SmartCarProtocol.version,
                    type: UInt8, payload: Data) {
            self.id = UUID()
            self.raw = raw
            self.version = version
            self.type = type
            self.payload = payload
        }

        public var typeName: String {
            FrameType(rawValue: type)?.displayName ?? String(format: "TYPE 0x%02X", type)
        }
    }

    public struct Parser: Sendable {
        private var parser = AppBLEFrameParser()

        public init() {}

        public mutating func reset() {
            parser.reset()
        }

        public mutating func feed(_ data: Data) -> [Frame] {
            parser.feed(data).map {
                Frame(raw: $0.raw, version: $0.version, type: $0.type, payload: $0.payload)
            }
        }
    }

    public static func encode(type: FrameType, payload: Data = Data()) -> Data {
        encode(type: type.rawValue, payload: payload)
    }

    public static func encode(type: UInt8, payload: Data = Data()) -> Data {
        AppBLEFrameCodec.encode(version: .v1, type: type, payload: payload)
    }

    public static func control(_ command: ControlCommand, data: Data = Data()) -> Data {
        encode(type: .control, payload: Data([command.rawValue]) + data)
    }

    public static func ping() -> Data { encode(type: .ping) }

    /// Builds the fixed 12-byte Target Yaw payload used by App-BLE 0x2E.
    /// Values are serialized explicitly to keep the wire format independent
    /// of host alignment and native endianness.
    public static func chassisHeadingPayload(vMmS: Float,
                                             targetYawDeg: Float,
                                             flags: UInt32 = 0) -> Data? {
        guard vMmS.isFinite,
              targetYawDeg.isFinite,
              abs(vMmS) <= 1_000.0,
              targetYawDeg >= -180.0,
              targetYawDeg <= 180.0,
              flags == 0 else {
            return nil
        }
        var payload = Data(capacity: 12)
        payload.appendFloat32LE(vMmS)
        payload.appendFloat32LE(targetYawDeg)
        payload.appendUInt32LE(flags)
        return payload
    }

    public static func crc16Modbus(_ bytes: [UInt8]) -> UInt16 {
        AppBLEFrameCodec.crc16Modbus(bytes)
    }
}

/// Parses a fragmented notification stream without repeatedly moving bytes at
/// the beginning of an Array. Invalid traffic is bounded to `capacity`.
public struct AppBLEFrameParser: Sendable {
    private var storage: [UInt8] = []
    private var readIndex = 0
    public let capacity: Int

    public init(capacity: Int = 512) {
        self.capacity = max(capacity, AppBLEFrameCodec.maxPayload + 8)
        storage.reserveCapacity(self.capacity)
    }

    public mutating func reset() {
        storage.removeAll(keepingCapacity: true)
        readIndex = 0
    }

    public mutating func feed(_ data: Data) -> [AppBLEFrame] {
        appendBounded(data)
        var frames: [AppBLEFrame] = []

        while true {
            while readIndex < storage.count, storage[readIndex] != AppBLEFrameCodec.head {
                readIndex += 1
            }
            guard storage.count - readIndex >= 5 else { break }

            let versionRaw = storage[readIndex + 1]
            guard AppBLEVersion(rawValue: versionRaw) != nil else {
                readIndex += 1
                continue
            }
            let payloadLength = Int(storage[readIndex + 3]) |
                (Int(storage[readIndex + 4]) << 8)
            guard payloadLength <= AppBLEFrameCodec.maxPayload else {
                readIndex += 1
                continue
            }
            let frameLength = payloadLength + 8
            guard storage.count - readIndex >= frameLength else { break }

            let end = readIndex + frameLength
            let crcIndex = readIndex + 5 + payloadLength
            guard storage[end - 1] == AppBLEFrameCodec.tail else {
                readIndex += 1
                continue
            }
            let receivedCRC = UInt16(storage[crcIndex]) | (UInt16(storage[crcIndex + 1]) << 8)
            let calculatedCRC = AppBLEFrameCodec.crc16Modbus(storage[(readIndex + 1)..<crcIndex])
            guard receivedCRC == calculatedCRC else {
                readIndex += 1
                continue
            }

            let raw = Data(storage[readIndex..<end])
            let payload = Data(storage[(readIndex + 5)..<crcIndex])
            frames.append(AppBLEFrame(raw: raw, version: versionRaw,
                                      type: storage[readIndex + 2], payload: payload))
            readIndex = end
        }
        compactIfNeeded()
        return frames
    }

    private mutating func appendBounded(_ data: Data) {
        guard !data.isEmpty else { return }
        if data.count >= capacity {
            storage = Array(data.suffix(capacity))
            readIndex = 0
            return
        }
        compactIfNeeded(force: storage.count + data.count > capacity)
        storage.append(contentsOf: data)
        if storage.count - readIndex > capacity {
            readIndex = storage.count - capacity
        }
    }

    private mutating func compactIfNeeded(force: Bool = false) {
        guard readIndex > 0, force || readIndex >= capacity / 2 else { return }
        storage = Array(storage[readIndex...])
        readIndex = 0
    }
}

public struct AppBLEV2CommandAck: Equatable, Sendable {
    public let sessionID: UInt32
    public let commandSequence: UInt32
    public let acknowledgedType: UInt8
    public let result: AppBLEV2Result
    public let stage: AppBLEV2AckStage
}

public enum AppBLESessionEvent: Equatable, Sendable {
    case v2Ready(sessionID: UInt32)
    case heartbeatAcknowledged(sequence: UInt32)
    case commandAcknowledged(AppBLEV2CommandAck)
    case ignored
}

public enum AppBLESessionMode: Equatable, Sendable {
    case idle
    case negotiating(deadline: Date)
    case v2Ready(sessionID: UInt32)
    case v1Fallback
    case expired
}

/// Owns only protocol timing and frame construction. BLE and UI lifecycle stay
/// in their platform targets. Validity duration is receiver-relative.
public struct AppBLESession: Sendable {
    public static let requestedCapabilities: UInt32 = 0x0000_0007
    public static let defaultNegotiationTimeout: TimeInterval = 1.2
    public static let defaultHeartbeatInterval: TimeInterval = 0.5
    public static let defaultCommandValidityMs: UInt16 = 300

    public private(set) var mode: AppBLESessionMode = .idle
    public private(set) var heartbeatInterval: TimeInterval = defaultHeartbeatInterval
    public private(set) var sessionTTL: TimeInterval = 3.0
    private var commandSequence: UInt32 = 0
    private var heartbeatSequence: UInt32 = 0
    private var lastHeartbeatAt: Date?

    public init() {}

    public var isCommandReady: Bool {
        switch mode {
        case .v2Ready, .v1Fallback: return true
        case .idle, .negotiating, .expired: return false
        }
    }

    public var usesV2: Bool {
        if case .v2Ready = mode { return true }
        return false
    }

    public mutating func begin(now: Date) -> Data {
        let deadline = now.addingTimeInterval(Self.defaultNegotiationTimeout)
        mode = .negotiating(deadline: deadline)
        commandSequence = 0
        heartbeatSequence = 0
        lastHeartbeatAt = nil
        var payload = Data([AppBLEVersion.v2.rawValue, AppBLEVersion.v2.rawValue])
        payload.appendUInt32LE(Self.requestedCapabilities)
        return AppBLEFrameCodec.encode(version: .v2, type: AppBLEV2Type.hello.rawValue, payload: payload)
    }

    public mutating func reset() {
        mode = .idle
        commandSequence = 0
        heartbeatSequence = 0
        lastHeartbeatAt = nil
    }

    public mutating func markExpired() {
        mode = .expired
        commandSequence = 0
        heartbeatSequence = 0
        lastHeartbeatAt = nil
    }

    public mutating func fallbackIfNeeded(now: Date) -> Bool {
        guard case .negotiating(let deadline) = mode, now >= deadline else { return false }
        mode = .v1Fallback
        return true
    }

    public mutating func handle(_ frame: AppBLEFrame, now: Date) -> AppBLESessionEvent {
        guard frame.version == AppBLEVersion.v2.rawValue,
              let type = AppBLEV2Type(rawValue: frame.type) else {
            return .ignored
        }
        switch type {
        case .helloAck:
            guard frame.payload.count == 13, frame.payload[0] == AppBLEVersion.v2.rawValue,
                  let sessionID = frame.payload.uint32LE(at: 1), sessionID != 0,
                  let heartbeatMs = frame.payload.uint16LE(at: 5),
                  let ttlMs = frame.payload.uint16LE(at: 7),
                  heartbeatMs >= 100, heartbeatMs <= 2_000, ttlMs >= 1_000 else {
                return .ignored
            }
            heartbeatInterval = TimeInterval(heartbeatMs) / 1_000
            sessionTTL = TimeInterval(ttlMs) / 1_000
            mode = .v2Ready(sessionID: sessionID)
            lastHeartbeatAt = now
            return .v2Ready(sessionID: sessionID)
        case .heartbeatAck:
            guard let sessionID = frame.payload.uint32LE(at: 0),
                  let sequence = frame.payload.uint32LE(at: 4),
                  frame.payload.count == 9, frame.payload[8] == AppBLEV2Result.ok.rawValue,
                  case .v2Ready(let expectedSessionID) = mode,
                  sessionID == expectedSessionID else { return .ignored }
            return .heartbeatAcknowledged(sequence: sequence)
        case .commandAck:
            guard frame.payload.count == 11,
                  let sessionID = frame.payload.uint32LE(at: 0),
                  let sequence = frame.payload.uint32LE(at: 4),
                  let result = AppBLEV2Result(rawValue: frame.payload[9]),
                  let stage = AppBLEV2AckStage(rawValue: frame.payload[10]),
                  case .v2Ready(let expectedSessionID) = mode,
                  sessionID == expectedSessionID else { return .ignored }
            return .commandAcknowledged(AppBLEV2CommandAck(
                sessionID: sessionID, commandSequence: sequence,
                acknowledgedType: frame.payload[8], result: result, stage: stage))
        case .hello, .heartbeat, .command:
            return .ignored
        }
    }

    public mutating func heartbeatIfDue(now: Date) -> Data? {
        guard case .v2Ready(let sessionID) = mode else { return nil }
        if let lastHeartbeatAt, now.timeIntervalSince(lastHeartbeatAt) < heartbeatInterval {
            return nil
        }
        heartbeatSequence &+= 1
        lastHeartbeatAt = now
        var payload = Data()
        payload.appendUInt32LE(sessionID)
        payload.appendUInt32LE(heartbeatSequence)
        return AppBLEFrameCodec.encode(version: .v2, type: AppBLEV2Type.heartbeat.rawValue, payload: payload)
    }

    public mutating func command(
        type: UInt8,
        payload: Data,
        validForMs: UInt16 = defaultCommandValidityMs
    ) -> Data? {
        switch mode {
        case .v1Fallback:
            return AppBLEFrameCodec.encode(version: .v1, type: type, payload: payload)
        case .v2Ready(let sessionID):
            guard payload.count <= AppBLEFrameCodec.maxPayload - 11,
                  validForMs >= 20, validForMs <= 1_000 else { return nil }
            commandSequence &+= 1
            var wrapped = Data()
            wrapped.appendUInt32LE(sessionID)
            wrapped.appendUInt32LE(commandSequence)
            wrapped.appendUInt16LE(validForMs)
            wrapped.append(type)
            wrapped.append(payload)
            return AppBLEFrameCodec.encode(version: .v2, type: AppBLEV2Type.command.rawValue, payload: wrapped)
        case .idle, .negotiating, .expired:
            return nil
        }
    }
}

public enum AppBLEOutboundLane: Sendable {
    case reliable
    case motion
    case stop
}

public struct AppBLEOutboundFrame: Equatable, Sendable {
    public let data: Data
    public let lane: AppBLEOutboundLane
    public let legacyType: UInt8?

    public init(data: Data, lane: AppBLEOutboundLane, legacyType: UInt8? = nil) {
        self.data = data
        self.lane = lane
        self.legacyType = legacyType
    }
}

/// Fixed-capacity scheduler. A latest motion state replaces unsent motion, and
/// an explicit stop always wins over ordinary traffic.
public struct AppBLEOutboundScheduler: Sendable {
    public let reliableCapacity: Int
    private var reliable: [AppBLEOutboundFrame] = []
    private var reliableHead = 0
    private var latestMotion: AppBLEOutboundFrame?
    private var stop: AppBLEOutboundFrame?

    public init(reliableCapacity: Int = 12) {
        self.reliableCapacity = max(1, reliableCapacity)
        reliable.reserveCapacity(self.reliableCapacity)
    }

    public var reliableCount: Int { reliable.count - reliableHead }
    public var hasPending: Bool { stop != nil || latestMotion != nil || reliableCount > 0 }
    public var hasPendingStop: Bool { stop != nil }

    @discardableResult
    public mutating func enqueueReliable(_ data: Data, legacyType: UInt8? = nil) -> Bool {
        guard reliableCount < reliableCapacity else { return false }
        reliable.append(AppBLEOutboundFrame(data: data, lane: .reliable, legacyType: legacyType))
        return true
    }

    public mutating func replaceMotion(_ data: Data, legacyType: UInt8? = nil, isStop: Bool) {
        let frame = AppBLEOutboundFrame(data: data, lane: isStop ? .stop : .motion, legacyType: legacyType)
        if isStop {
            latestMotion = nil
            stop = frame
        } else if stop == nil {
            latestMotion = frame
        }
    }

    public mutating func cancelMotion() {
        latestMotion = nil
    }

    public mutating func removeAll() {
        reliable.removeAll(keepingCapacity: true)
        reliableHead = 0
        latestMotion = nil
        stop = nil
    }

    public mutating func dequeueNext() -> AppBLEOutboundFrame? {
        if let stop {
            self.stop = nil
            return stop
        }
        if reliableHead < reliable.count {
            let frame = reliable[reliableHead]
            reliableHead += 1
            if reliableHead >= reliable.count || reliableHead >= reliableCapacity / 2 {
                reliable.removeSubrange(0..<reliableHead)
                reliableHead = 0
            }
            return frame
        }
        defer { latestMotion = nil }
        return latestMotion
    }
}

/// Fixed-capacity FIFO for high-rate telemetry and log records. Appending is
/// O(1); callers take an ordered snapshot only when publishing UI state.
public struct AppBLEBoundedRingBuffer<Element: Sendable>: Sendable {
    private var storage: [Element?]
    private var readIndex = 0
    private(set) public var count = 0

    public let capacity: Int

    public init(capacity: Int) {
        self.capacity = max(1, capacity)
        self.storage = Array(repeating: nil, count: max(1, capacity))
    }

    @discardableResult
    public mutating func append(_ element: Element) -> Bool {
        let writeIndex = (readIndex + count) % capacity
        let dropped = count == capacity
        if dropped {
            storage[readIndex] = element
            readIndex = (readIndex + 1) % capacity
        } else {
            storage[writeIndex] = element
            count += 1
        }
        return dropped
    }

    public mutating func removeAll() {
        storage = Array(repeating: nil, count: capacity)
        readIndex = 0
        count = 0
    }

    public var elements: [Element] {
        var result: [Element] = []
        result.reserveCapacity(count)
        for offset in 0..<count {
            if let element = storage[(readIndex + offset) % capacity] {
                result.append(element)
            }
        }
        return result
    }
}

private extension Data {
    mutating func appendFloat32LE(_ value: Float) {
        appendUInt32LE(value.bitPattern)
    }

    mutating func appendUInt16LE(_ value: UInt16) {
        append(UInt8(value & 0xFF))
        append(UInt8(value >> 8))
    }

    mutating func appendUInt32LE(_ value: UInt32) {
        append(UInt8(value & 0xFF))
        append(UInt8((value >> 8) & 0xFF))
        append(UInt8((value >> 16) & 0xFF))
        append(UInt8((value >> 24) & 0xFF))
    }

    func uint16LE(at offset: Int) -> UInt16? {
        guard offset >= 0, offset + 2 <= count else { return nil }
        return UInt16(self[startIndex + offset]) | (UInt16(self[startIndex + offset + 1]) << 8)
    }

    func uint32LE(at offset: Int) -> UInt32? {
        guard offset >= 0, offset + 4 <= count else { return nil }
        return UInt32(self[startIndex + offset]) |
            (UInt32(self[startIndex + offset + 1]) << 8) |
            (UInt32(self[startIndex + offset + 2]) << 16) |
            (UInt32(self[startIndex + offset + 3]) << 24)
    }
}
