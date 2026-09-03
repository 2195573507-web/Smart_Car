import XCTest
@testable import SmartCarIOS

@MainActor
final class SmartCarViewModelTests: XCTestCase {
    func testWriteResponseWatchdogRejectsAStaleTimeoutAfterCompletion() {
        var watchdog = BLEWriteResponseWatchdog()
        let timedOutWrite = watchdog.arm()

        watchdog.invalidate()

        XCTAssertFalse(watchdog.isCurrent(timedOutWrite))
    }

    func testWriteResponseWatchdogKeepsOnlyTheMostRecentWriteTimeoutCurrent() {
        var watchdog = BLEWriteResponseWatchdog()
        let firstWrite = watchdog.arm()
        let secondWrite = watchdog.arm()

        XCTAssertFalse(watchdog.isCurrent(firstWrite))
        XCTAssertTrue(watchdog.isCurrent(secondWrite))
    }

    func testNonzeroLatestMotionCommandsUseWriteWithoutResponseWhenSupported() {
        let cases: [(SmartCarProtocol.FrameType, Int)] = [
            (.wheelSpeedCommand, 16),
            (.chassisSpeedCommand, 16),
            (.chassisHeadingCommand, 12)
        ]

        for (type, payloadLength) in cases {
            var payload = Data(repeating: 0, count: payloadLength)
            payload[0] = 1
            let frame = SmartCarProtocol.encode(type: type, payload: payload)

            XCTAssertEqual(
                BLEWriteTransportPolicy.writeType(
                    for: frame,
                    motionType: type.rawValue,
                    supportsWriteWithoutResponse: true
                ),
                .withoutResponse,
                "expected no-response transport for \(type)"
            )
        }
    }

    func testZeroAndNonMotionCommandsKeepWriteResponseTransport() {
        let zeroMotionCases: [(SmartCarProtocol.FrameType, Int)] = [
            (.wheelSpeedCommand, 16),
            (.chassisSpeedCommand, 16),
            (.chassisHeadingCommand, 12)
        ]

        for (type, payloadLength) in zeroMotionCases {
            let frame = SmartCarProtocol.encode(
                type: type,
                payload: Data(repeating: 0, count: payloadLength)
            )
            XCTAssertEqual(
                BLEWriteTransportPolicy.writeType(
                    for: frame,
                    motionType: type.rawValue,
                    supportsWriteWithoutResponse: true
                ),
                .withResponse,
                "expected acknowledged zero-speed command for \(type)"
            )
        }

        let signedZero = Data([0, 0, 0, 0x80])
        let signedZeroWheelFrame = SmartCarProtocol.encode(
            type: .wheelSpeedCommand,
            payload: signedZero + signedZero + signedZero + signedZero
        )
        XCTAssertEqual(
            BLEWriteTransportPolicy.writeType(
                for: signedZeroWheelFrame,
                motionType: SmartCarProtocol.FrameType.wheelSpeedCommand.rawValue,
                supportsWriteWithoutResponse: true
            ),
            .withResponse
        )

        let singleWheel = SmartCarProtocol.encode(
            type: .wheelSpeedSingleCommand,
            payload: Data([0, 1, 0, 0, 0])
        )
        XCTAssertEqual(
            BLEWriteTransportPolicy.writeType(
                for: singleWheel,
                motionType: SmartCarProtocol.FrameType.wheelSpeedSingleCommand.rawValue,
                supportsWriteWithoutResponse: true
            ),
            .withResponse
        )

        let ping = SmartCarProtocol.ping()
        XCTAssertEqual(
            BLEWriteTransportPolicy.writeType(
                for: ping,
                motionType: nil,
                supportsWriteWithoutResponse: true
            ),
            .withResponse
        )
    }

    func testMotionFallsBackToWriteResponseWithoutS3NoResponseCapability() {
        var payload = Data(repeating: 0, count: 16)
        payload[0] = 1
        let frame = SmartCarProtocol.encode(type: .wheelSpeedCommand, payload: payload)

        XCTAssertEqual(
            BLEWriteTransportPolicy.writeType(
                for: frame,
                motionType: SmartCarProtocol.FrameType.wheelSpeedCommand.rawValue,
                supportsWriteWithoutResponse: false
            ),
            .withResponse
        )
    }

    func testActiveJoystickKeepsControlTargetsBeforeTelemetryConfirmation() {
        let intent = JoystickIntent(horizontal: 0.25, vertical: -0.75)

        XCTAssertFalse(SmartCarViewModel.shouldApplyWheelControlTelemetry(
            activeJoystickIntent: intent
        ))
    }

    func testActiveJoystickKeepsControlTargetsAfterConfirmationWindow() {
        let intent = JoystickIntent(horizontal: -0.6, vertical: 0.4)

        XCTAssertFalse(SmartCarViewModel.shouldApplyWheelControlTelemetry(
            activeJoystickIntent: intent
        ))
    }

    func testReleasedJoystickAllowsTelemetryToRefreshIdleControlState() {
        XCTAssertTrue(SmartCarViewModel.shouldApplyWheelControlTelemetry(
            activeJoystickIntent: nil
        ))
    }
}
