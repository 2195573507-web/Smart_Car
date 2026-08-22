import Foundation

enum BLEUserFacingError: Equatable {
    case deviceNotFound
    case notConnected
    case protocolDecodeFailed(String)
    case connectionFailed(String)
    case serviceNotFound
    case system(String)

    func localizedDescription(locale: Locale) -> String {
        switch self {
        case .deviceNotFound:
            return AppStrings.text("error.device_not_found", locale: locale)
        case .notConnected:
            return AppStrings.text("error.not_connected", locale: locale)
        case .protocolDecodeFailed(let reason):
            return AppStrings.format("error.protocol_decode_failed", locale: locale, reason)
        case .connectionFailed(let reason):
            return AppStrings.format("error.connection_failed", locale: locale, reason)
        case .serviceNotFound:
            return AppStrings.text("error.service_not_found", locale: locale)
        case .system(let reason):
            return AppStrings.format("error.system", locale: locale, reason)
        }
    }
}

enum AppPresentationStrings {
    static func connectionStatus(_ status: BLEConnectionStatus, locale: Locale) -> String {
        switch status {
        case .disconnected:
            return AppStrings.text("status.disconnected", locale: locale)
        case .scanning:
            return AppStrings.text("status.scanning", locale: locale)
        case .connecting:
            return AppStrings.text("status.connecting", locale: locale)
        case .connected:
            return AppStrings.text("status.connected", locale: locale)
        case .unavailable:
            return AppStrings.text("status.bluetooth_unavailable", locale: locale)
        case .failed(let reason):
            return AppStrings.format("status.failed", locale: locale, reason)
        }
    }

    static func smartCarStatus(_ status: String, locale: Locale) -> String {
        switch status {
        case "ONLINE": return AppStrings.text("status.online", locale: locale)
        case "OFFLINE": return AppStrings.text("status.offline", locale: locale)
        case "WAITING": return AppStrings.text("status.waiting", locale: locale)
        case "STALE": return AppStrings.text("status.stale", locale: locale)
        default: return status
        }
    }

    static func availability(_ isOnline: Bool, locale: Locale) -> String {
        AppStrings.text(isOnline ? "status.online" : "status.offline", locale: locale)
    }

    static func packetType(_ type: String, locale: Locale) -> String {
        let key: String
        switch type {
        case "CONTROL": key = "protocol.control"
        case "STATUS": key = "protocol.status"
        case "PING": key = "protocol.ping"
        case "ACK": key = "protocol.ack"
        case "IMU_STATUS": key = "protocol.imu_status"
        case "ATTITUDE": key = "protocol.attitude"
        case "IMU_CAL_STATUS": key = "protocol.imu_cal_status"
        case "IMU_CAL_BIAS": key = "protocol.imu_cal_bias"
        case "IMU_CAL_RESULT": key = "protocol.imu_cal_result"
        case "IMU_TELEMETRY": key = "protocol.imu_telemetry"
        case "DUAL_IMU_STATUS": return "DUAL_IMU_STATUS"
        case "RADAR_STATUS": key = "protocol.radar_status"
        case "WHEEL_SPEED_STATUS": return "WHEEL_SPEED_STATUS"
        case "POWER_STATUS": return "POWER_STATUS"
        default: return type
        }
        return AppStrings.text(key, locale: locale)
    }

    static func decodedMessage(_ record: DecodedMessageRecord, locale: Locale,
                               angleUnit: AngleUnit = .degree) -> String {
        let timestamp = record.receivedAt.formatted(date: .omitted, time: .standard)
        let summary: String
        switch record.message {
        case .control(let command, let speed):
            let commandKey: String
            switch command {
            case .stop: commandKey = "command.stop"
            case .moveForward: commandKey = "command.move_forward"
            case .moveBack: commandKey = "command.move_back"
            case .turnLeft: commandKey = "command.turn_left"
            case .turnRight: commandKey = "command.turn_right"
            case .speedControl: commandKey = "command.speed_control"
            }
            let commandName = AppStrings.text(commandKey, locale: locale)
            if let speed {
                summary = AppStrings.format("debug.control_speed", locale: locale, commandName, Int(speed))
            } else {
                summary = AppStrings.format("debug.control", locale: locale, commandName)
            }
        case .status(let status):
            summary = AppStrings.format(
                "debug.status",
                locale: locale,
                Int(status.battery),
                Int(status.motorState),
                String(format: "%04X", status.errorCode)
            )
        case .ping:
            summary = AppStrings.text("debug.ping", locale: locale)
        case .ack(let data):
            let bytes = data.map { String(format: "%02X", $0) }.joined(separator: " ")
            summary = AppStrings.format("debug.ack", locale: locale, bytes)
        case .imuStatus(let imu):
            let sensor = imu.sensorID == .bmi323 ? "BMI323" : "LSM303"
            let availability = AppPresentationStrings.availability(imu.data.online, locale: locale)
            summary = AppStrings.format("debug.imu_status", locale: locale, sensor, availability)
        case .attitude(let attitude):
            summary = AppStrings.format(
                "debug.attitude",
                locale: locale,
                angleUnit.format(attitude.value(for: .roll, unit: angleUnit), precision: 2),
                angleUnit.format(attitude.value(for: .pitch, unit: angleUnit), precision: 2),
                angleUnit.format(attitude.value(for: .yaw, unit: angleUnit), precision: 2)
            )
        case .dualAttitude(let dual):
            summary = String(format: "DUAL_ATTITUDE dR=%.2f dP=%.2f dY=%.2f",
                             angleUnit == .degree ? dual.deltaRad.x * 57.29578 : dual.deltaRad.x,
                             angleUnit == .degree ? dual.deltaRad.y * 57.29578 : dual.deltaRad.y,
                             angleUnit == .degree ? dual.deltaRad.z * 57.29578 : dual.deltaRad.z)
        case .imuCalibrationStatus(let status):
            summary = AppStrings.format(
                "debug.imu_cal_status",
                locale: locale,
                Int(status.state.rawValue),
                Int(status.totalProgress)
            )
        case .imuCalibrationBias(let bias):
            summary = AppStrings.format(
                "debug.imu_cal_bias",
                locale: locale,
                bias.x.displayValue,
                bias.y.displayValue,
                bias.z.displayValue
            )
        case .imuCalibrationResult(let result):
            let sensor = result.sensorID == .bmi323 ? "BMI323" : "LSM303"
            let accel = result.accelBias.map { "accel=(\($0.x.displayValue), \($0.y.displayValue), \($0.z.displayValue))" } ?? "accel=n/a"
            let gyro = result.gyroBias.map { "gyro=(\($0.x.displayValue), \($0.y.displayValue), \($0.z.displayValue))" } ?? "gyro=n/a"
            summary = "IMU_CAL_RESULT \(sensor) \(accel) \(gyro)"
        case .calibrationEvent(let event):
            summary = "CAL_EVENT id=\(event.rawValue) \(event.displayName)"
        case .imuTelemetry(let telemetry):
            switch telemetry {
            case .lsm303(let value):
                summary = String(format: "IMU_TELEMETRY LSM303 accel=(%.3f, %.3f, %.3f) mag=(%.3f, %.3f, %.3f)",
                                 value.accel.x, value.accel.y, value.accel.z,
                                 value.mag.x, value.mag.y, value.mag.z)
            case .bmi323(let value):
                summary = String(format: "IMU_TELEMETRY BMI323 accel=(%.3f, %.3f, %.3f) gyro=(%.3f, %.3f, %.3f)",
                                 value.accel.x, value.accel.y, value.accel.z,
                                 value.gyro.x, value.gyro.y, value.gyro.z)
            }
        case .dualIMUStatus(let status):
            summary = "DUAL_IMU_STATUS phase=\(status.phase.displayName) lsm=\(status.lsmProgress)% bmi=\(status.bmiProgress)% overall=\(status.overallProgress)% error=\(status.errorCode)"
        case .radarStatus(let status):
            summary = AppStrings.format(
                "debug.radar_status",
                locale: locale,
                AppPresentationStrings.availability(status.online, locale: locale),
                Int(status.speedPercent)
            )
        case .wheelSpeedStatus(let status):
            summary = "WHEEL_SPEED_STATUS " + status.speeds.map { String(format: "%.1f", $0) }.joined(separator: ",")
        case .powerStatus(let status):
            summary = String(format: "POWER_STATUS %.2fV", status.voltage)
        }
        return "\(timestamp)  \(summary)"
    }
}
