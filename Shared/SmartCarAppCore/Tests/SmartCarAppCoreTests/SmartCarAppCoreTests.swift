import XCTest
@testable import SmartCarAppCore

final class SmartCarAppCoreTests: XCTestCase {
    func testParserAcceptsFragmentedV1AndV2Frames() {
        let v1 = AppBLEFrameCodec.encode(version: .v1, type: 0x15, payload: Data([1, 2]))
        let v2 = AppBLEFrameCodec.encode(version: .v2, type: AppBLEV2Type.hello.rawValue, payload: Data([2, 2, 0, 0, 0, 0]))
        var parser = AppBLEFrameParser()
        XCTAssertTrue(parser.feed(v1.prefix(3)).isEmpty)
        let first = parser.feed(v1.dropFirst(3))
        XCTAssertEqual(first.count, 1)
        XCTAssertEqual(first[0].version, AppBLEVersion.v1.rawValue)
        let second = parser.feed(v2)
        XCTAssertEqual(second.count, 1)
        XCTAssertEqual(second[0].version, AppBLEVersion.v2.rawValue)
    }

    func testParserRecoversAfterNoiseAndBadCRC() {
        var invalid = Array(AppBLEFrameCodec.encode(version: .v1, type: 0x15, payload: Data([1])))
        invalid[5] ^= 0xFF
        let valid = AppBLEFrameCodec.encode(version: .v1, type: 0x16, payload: Data([2]))
        var parser = AppBLEFrameParser()
        let frames = parser.feed(Data([0, 1, 2] + invalid + Array(valid)))
        XCTAssertEqual(frames.map(\.type), [0x16])
    }

    func testV2SessionNegotiatesAndWrapsCommands() {
        var session = AppBLESession()
        let now = Date(timeIntervalSince1970: 10)
        let hello = session.begin(now: now)
        XCTAssertEqual(hello[1], AppBLEVersion.v2.rawValue)

        var ackPayload = Data([AppBLEVersion.v2.rawValue])
        ackPayload.append(contentsOf: [0x78, 0x56, 0x34, 0x12])
        ackPayload.append(contentsOf: [0xF4, 0x01, 0xB8, 0x0B])
        ackPayload.append(contentsOf: [0x07, 0x00, 0x00, 0x00])
        let ack = AppBLEFrame(
            raw: AppBLEFrameCodec.encode(version: .v2, type: AppBLEV2Type.helloAck.rawValue, payload: ackPayload),
            version: AppBLEVersion.v2.rawValue, type: AppBLEV2Type.helloAck.rawValue, payload: ackPayload)
        XCTAssertEqual(session.handle(ack, now: now), .v2Ready(sessionID: 0x1234_5678))
        let command = session.command(type: 0x15, payload: Data(repeating: 0, count: 16))
        XCTAssertEqual(command?[1], AppBLEVersion.v2.rawValue)
        XCTAssertEqual(command?[2], AppBLEV2Type.command.rawValue)
    }

    func testV1FallbackAfterNegotiationDeadline() {
        var session = AppBLESession()
        let now = Date(timeIntervalSince1970: 10)
        _ = session.begin(now: now)
        XCTAssertFalse(session.fallbackIfNeeded(now: now))
        XCTAssertTrue(session.fallbackIfNeeded(now: now.addingTimeInterval(2)))
        XCTAssertTrue(session.isCommandReady)
        XCTAssertEqual(session.command(type: 0x1D, payload: Data())?[1], AppBLEVersion.v1.rawValue)
    }

    func testChassisHeadingPayloadUsesLittleEndianAndZeroFlags() {
        let payload = SmartCarProtocol.chassisHeadingPayload(
            vMmS: 420.0, targetYawDeg: -179.5)
        XCTAssertEqual(payload?.count, 12)
        XCTAssertEqual(payload.map(Array.init), [
            0x00, 0x00, 0xD2, 0x43,
            0x00, 0x80, 0x33, 0xC3,
            0x00, 0x00, 0x00, 0x00
        ])

        guard let payload else {
            return XCTFail("expected valid heading payload")
        }
        let frame = SmartCarProtocol.encode(type: .chassisHeadingCommand,
                                            payload: payload)
        XCTAssertEqual(frame.count, 20)
        XCTAssertEqual(frame[2], 0x2E)
        XCTAssertEqual(frame[3], 12)
        XCTAssertEqual(frame[4], 0)
    }

    func testChassisHeadingPayloadRejectsInvalidValues() {
        XCTAssertNil(SmartCarProtocol.chassisHeadingPayload(
            vMmS: .infinity, targetYawDeg: 0))
        XCTAssertNil(SmartCarProtocol.chassisHeadingPayload(
            vMmS: 0, targetYawDeg: 180.1))
        XCTAssertNil(SmartCarProtocol.chassisHeadingPayload(
            vMmS: 0, targetYawDeg: -180.1))
        XCTAssertNil(SmartCarProtocol.chassisHeadingPayload(
            vMmS: 0, targetYawDeg: 0, flags: 1))
        XCTAssertNil(SmartCarProtocol.chassisHeadingPayload(
            vMmS: 1_000.1, targetYawDeg: 0))
    }

    func testStopPreemptsReliableAndMotionTraffic() {
        var scheduler = AppBLEOutboundScheduler(reliableCapacity: 2)
        XCTAssertTrue(scheduler.enqueueReliable(Data([1])))
        scheduler.replaceMotion(Data([2]), legacyType: 0x15, isStop: false)
        scheduler.replaceMotion(Data([0]), legacyType: 0x15, isStop: true)
        XCTAssertEqual(scheduler.dequeueNext()?.data, Data([0]))
        XCTAssertEqual(scheduler.dequeueNext()?.data, Data([1]))
        XCTAssertNil(scheduler.dequeueNext())
    }

    func testBoundedRingBufferKeepsNewestElements() {
        var buffer = AppBLEBoundedRingBuffer<Int>(capacity: 3)
        XCTAssertFalse(buffer.append(1))
        XCTAssertFalse(buffer.append(2))
        XCTAssertFalse(buffer.append(3))
        XCTAssertTrue(buffer.append(4))
        XCTAssertEqual(buffer.elements, [2, 3, 4])
        buffer.removeAll()
        XCTAssertEqual(buffer.count, 0)
        XCTAssertTrue(buffer.elements.isEmpty)
    }

    func testSessionExpiryBlocksCommandsUntilRenegotiation() {
        var session = AppBLESession()
        session.markExpired()
        XCTAssertEqual(session.mode, .expired)
        XCTAssertFalse(session.isCommandReady)
        XCTAssertNil(session.command(type: 0x15, payload: Data(repeating: 0, count: 16)))
    }

    func testSessionIgnoresAckForAnotherSession() {
        var session = AppBLESession()
        let now = Date(timeIntervalSince1970: 10)
        _ = session.begin(now: now)
        var helloAck = Data([2])
        helloAck.append(contentsOf: [1, 0, 0, 0])
        helloAck.append(contentsOf: [0xF4, 0x01, 0xB8, 0x0B])
        helloAck.append(contentsOf: [7, 0, 0, 0])
        let hello = AppBLEFrame(raw: Data(), version: 2,
                                type: AppBLEV2Type.helloAck.rawValue,
                                payload: helloAck)
        XCTAssertEqual(session.handle(hello, now: now), .v2Ready(sessionID: 1))
        let heartbeat = Data([2, 0, 0, 0, 1, 0, 0, 0, 0])
        let frame = AppBLEFrame(raw: Data(), version: 2,
                                type: AppBLEV2Type.heartbeatAck.rawValue,
                                payload: heartbeat)
        XCTAssertEqual(session.handle(frame, now: now), .ignored)
    }

    func testInboundWorkerCoalescesTelemetryAndRetainsCriticalFrames() async {
        let worker = AppBLEInboundWorker(criticalCapacity: 2, otherCapacity: 2)
        let ack1 = AppBLEFrameCodec.encode(version: .v1, type: SmartCarProtocol.FrameType.ack.rawValue,
                                           payload: Data([1]))
        let ack2 = AppBLEFrameCodec.encode(version: .v1, type: SmartCarProtocol.FrameType.ack.rawValue,
                                           payload: Data([2]))
        let telemetry1 = AppBLEFrameCodec.encode(version: .v1, type: SmartCarProtocol.FrameType.powerStatus.rawValue,
                                                 payload: Data([0, 0, 72, 66]))
        let telemetry2 = AppBLEFrameCodec.encode(version: .v1, type: SmartCarProtocol.FrameType.powerStatus.rawValue,
                                                 payload: Data([0, 0, 74, 66]))

        await worker.submit(ack1)
        await worker.submit(telemetry1)
        await worker.submit(telemetry2)
        await worker.submit(ack2)
        let batch = await worker.drain()

        XCTAssertEqual(batch.critical.map { $0.frame.payload }, [Data([1]), Data([2])])
        XCTAssertEqual(batch.telemetry.count, 1)
        XCTAssertEqual(batch.telemetry.first?.frame.payload, Data([0, 0, 74, 66]))
        XCTAssertEqual(batch.droppedTelemetry, 1)
    }

    func testInboundWorkerBackpressuresCriticalWithoutDroppingAndBoundsOtherTraffic() async {
        let worker = AppBLEInboundWorker(criticalCapacity: 1, otherCapacity: 2)
        let ack1 = AppBLEFrameCodec.encode(version: .v1, type: SmartCarProtocol.FrameType.ack.rawValue,
                                           payload: Data([1]))
        let ack2 = AppBLEFrameCodec.encode(version: .v1, type: SmartCarProtocol.FrameType.ack.rawValue,
                                           payload: Data([2]))
        let ping = AppBLEFrameCodec.encode(version: .v1, type: SmartCarProtocol.FrameType.ping.rawValue)

        await worker.submit(ack1)
        let blocked = Task { await worker.submit(ack2) }
        for _ in 0..<4 { await worker.submit(ping) }

        let first = await worker.drain()
        XCTAssertEqual(first.critical.map { $0.frame.payload }, [Data([1])])
        XCTAssertEqual(first.other.count, 2)
        XCTAssertEqual(first.droppedOther, 2)

        await blocked.value
        let second = await worker.drain()
        XCTAssertEqual(second.critical.map { $0.frame.payload }, [Data([2])])
    }

    func testTelemetryReducerPublishesLatestTelemetryAsOneSnapshot() async throws {
        let reducer = AppTelemetryReducer(otherCapacity: 2)
        let first = DecodedMessageRecord(
            message: .powerStatus(PowerStatus(voltage: 12.0)),
            receivedAt: Date(timeIntervalSince1970: 1)
        )
        let latest = DecodedMessageRecord(
            message: .powerStatus(PowerStatus(voltage: 12.5)),
            receivedAt: Date(timeIntervalSince1970: 2)
        )
        await reducer.submit(first)
        await reducer.submit(latest)
        let snapshot = await reducer.drain()
        XCTAssertEqual(snapshot.telemetry.count, 1)
        guard case .powerStatus(let status) = snapshot.telemetry[0].message else {
            return XCTFail("expected power telemetry")
        }
        XCTAssertEqual(status.voltage, 12.5, accuracy: 0.001)
        let emptySnapshot = await reducer.drain()
        XCTAssertTrue(emptySnapshot.isEmpty)
    }

    func testSmartCarLogParserHandlesFragmentationWithoutArrayFrontRemoval() {
        var parser = SmartCarLogParser()
        let payload = Data("hello".utf8)
        var frame = Data([0xAA, 0x55, 0x01, SmartCarLogSource.s3.rawValue,
                          SmartCarLogLevel.warn.rawValue, 1, 0, 0, 0, UInt8(payload.count)])
        frame.append(payload)
        let crc = AppBLEFrameCodec.crc16Modbus(frame.dropFirst(2))
        frame.append(UInt8(crc & 0xFF))
        frame.append(UInt8(crc >> 8))
        XCTAssertTrue(parser.feed(frame.prefix(4), receivedAt: Date()).isEmpty)
        let records = parser.feed(frame.dropFirst(4), receivedAt: Date())
        XCTAssertEqual(records.map(\.message), ["hello"])
    }
}
