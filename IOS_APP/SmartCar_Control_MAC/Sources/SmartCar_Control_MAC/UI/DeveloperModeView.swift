import SwiftUI

struct DeveloperModeView: View {
    @ObservedObject var viewModel: SmartCarViewModel
    let telemetryStore: TelemetryStore
    let angleUnit: AngleUnit
    @Environment(\.locale) private var locale

    var body: some View {
        Group {
            switch viewModel.developerPage {
            case .overview:
                overview
            case .loggerSTM:
                LoggerSTMView(store: viewModel.stmLogStore) {
                    viewModel.developerPage = .overview
                }
            case .loggerS3:
                LoggerS3View(store: viewModel.s3LogStore) {
                    viewModel.developerPage = .overview
                }
            }
        }
    }

    private var overview: some View {
        ScrollView {
            HStack(alignment: .top, spacing: 18) {
                VStack(spacing: 16) {
                    DualIMUCard(sensor: .bmi323,
                                data: telemetryStore.imu.snapshot.model.bmi323)
                    DualIMUCard(sensor: .lsm303,
                                data: telemetryStore.imu.snapshot.model.lsm303)
                }.frame(maxWidth: 440)
                VStack(spacing: 16) {
                    DualIMULifecycleCard(store: telemetryStore.dualIMU)
                    DualAttitudeComparisonView(store: telemetryStore.dualAttitude,
                                                angleUnit: angleUnit)
                    DualAttitudeLogConsole(store: telemetryStore.dualAttitude)
                    DeveloperCalibrationCard(viewModel: viewModel.calibrationViewModel)
                    RadarControlCard(viewModel: viewModel, store: telemetryStore.radar)
                    StaticCalibrationAnalysisCard(
                        staticCalibration: telemetryStore.staticCalibration
                    )
                    ProtocolMonitor(viewModel: viewModel)
                }.frame(maxWidth: 500)
            }
            .frame(maxWidth: .infinity, alignment: .center)
            .padding(.horizontal, 24).padding(.bottom, 24)
        }
    }
}

private struct DualAttitudeLogConsole: View {
    @ObservedObject var store: DualAttitudeState

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("ATT_DUAL LOG")
                .font(.headline.monospaced())
            ScrollView {
                LazyVStack(alignment: .leading, spacing: 6) {
                    if store.snapshot.logLines.isEmpty {
                        Text("Waiting for DualAttitude frames")
                            .foregroundStyle(.secondary)
                    }
                    ForEach(Array(store.snapshot.logLines.enumerated().reversed()), id: \.offset) { _, line in
                        Text(line)
                            .font(.caption.monospaced())
                            .textSelection(.enabled)
                            .frame(maxWidth: .infinity, alignment: .leading)
                    }
                }
            }
            .frame(minHeight: 120, maxHeight: 220)
            .padding(10)
            .background(.black.opacity(0.06), in: RoundedRectangle(cornerRadius: 6))
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}

private struct DualIMULifecycleCard: View {
    @ObservedObject var store: DualIMUState

    var body: some View {
        let status = store.snapshot.status
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text("DUAL_IMU_BOOT")
                    .font(.headline.monospaced())
                Spacer()
                Text(status?.phase.displayName ?? "WAITING")
                    .font(.caption.monospaced().weight(.bold))
                    .foregroundStyle(phaseColor(status?.phase))
            }
            progressRow(label: "LSM303", value: status?.lsmProgress ?? 0)
            progressRow(label: "BMI323", value: status?.bmiProgress ?? 0)
            progressRow(label: "TOTAL", value: status?.overallProgress ?? 0)
            if let status {
                KeyValueRow(label: "WINDOW", value: "\(status.phaseStartTimeMs) / \(status.phaseEndTimeMs)")
                if status.errorCode != 0 {
                    KeyValueRow(label: "ERROR", value: "\(status.errorCode)")
                        .foregroundStyle(.red)
                }
            }
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    @ViewBuilder
    private func progressRow(label: String, value: UInt8) -> some View {
        HStack(spacing: 10) {
            Text(label).font(.caption.monospaced().weight(.bold)).frame(width: 52, alignment: .leading)
            ProgressView(value: Double(value), total: 100)
            Text("\(value)%").font(.caption.monospaced()).frame(width: 36, alignment: .trailing)
        }
    }

    private func phaseColor(_ phase: DualIMUPhase?) -> Color {
        switch phase {
        case .some(.ready): return .green
        case .some(.failed): return .red
        case .none, .some(.idle): return .secondary
        default: return .orange
        }
    }
}

private struct RadarControlCard: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @ObservedObject var store: RadarState
    @Environment(\.locale) private var locale

    var body: some View {
        let snapshot = store.snapshot
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text(AppStrings.text("label.radar_control", locale: locale))
                    .font(.headline.monospaced())
                Spacer()
                Text(statusText(snapshot.availability))
                    .font(.caption.monospaced().weight(.bold))
                    .foregroundStyle(statusColor(snapshot.availability))
            }
            HStack {
                Text(AppStrings.text("label.radar_current_speed", locale: locale))
                    .font(.caption.weight(.bold))
                Spacer()
                Text("\(Int(snapshot.speedPercent))%")
                    .font(.caption.monospaced())
            }
            KeyValueRow(label: AppStrings.text("label.target_speed", locale: locale), value: "\(Int(viewModel.radarSpeed))%")
            Slider(
                value: $viewModel.radarSpeed,
                in: 0...100,
                step: 1,
                onEditingChanged: { editing in
                    if !editing {
                        viewModel.updateRadarSpeed()
                    }
                }
            )
            .disabled(viewModel.status != .connected)
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private func statusText(_ availability: RadarAvailability) -> String {
        switch availability {
        case .online:
            return AppStrings.text("status.online", locale: locale)
        case .offline:
            return "ERROR"
        case .waiting:
            return AppStrings.text("status.waiting", locale: locale)
        }
    }

    private func statusColor(_ availability: RadarAvailability) -> Color {
        switch availability {
        case .online:
            return .green
        case .offline:
            return .red
        case .waiting:
            return .secondary
        }
    }
}

private enum DebugIMUSensor {
    case lsm303
    case bmi323
}

private struct DualIMUCard: View {
    let sensor: DebugIMUSensor
    let data: Any
    @Environment(\.locale) private var locale

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text(title).font(.headline.monospaced())
                Spacer()
                Text(AppPresentationStrings.availability(online, locale: locale))
                    .font(.caption.monospaced().weight(.bold))
                    .foregroundStyle(online ? .green : .red)
            }
            VectorReadout(titleKey: "label.accel", vector: accel)
            VectorReadout(titleKey: sensor == .bmi323 ? "label.gyro" : "label.mag",
                          vector: secondary)
            Text("t=\(timestamp) ms")
                .font(.caption2.monospaced())
                .foregroundStyle(.secondary)
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private var title: String { sensor == .bmi323 ? "BMI323" : "LSM303" }

    private var online: Bool {
        switch sensor {
        case .lsm303: return (data as? LSM303Data)?.online ?? false
        case .bmi323: return (data as? BMI323Data)?.online ?? false
        }
    }

    private var accel: Vector3 {
        switch sensor {
        case .lsm303: return (data as? LSM303Data)?.accel ?? Vector3()
        case .bmi323: return (data as? BMI323Data)?.accel ?? Vector3()
        }
    }

    private var secondary: Vector3 {
        switch sensor {
        case .lsm303: return (data as? LSM303Data)?.mag ?? Vector3()
        case .bmi323: return (data as? BMI323Data)?.gyro ?? Vector3()
        }
    }

    private var timestamp: UInt32 {
        switch sensor {
        case .lsm303: return (data as? LSM303Data)?.timestamp ?? 0
        case .bmi323: return (data as? BMI323Data)?.timestamp ?? 0
        }
    }
}

private struct VectorReadout: View {
    let titleKey: String
    let vector: Vector3
    @Environment(\.locale) private var locale

    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            Text(AppStrings.text(titleKey, locale: locale)).font(.caption.weight(.bold)).foregroundStyle(.secondary)
            HStack {
                ValueReadout(label: "X", value: vector.x)
                ValueReadout(label: "Y", value: vector.y)
                ValueReadout(label: "Z", value: vector.z)
            }
        }
    }
}

private struct AttitudeDeveloperCard: View {
    @ObservedObject var store: AttitudeState
    @ObservedObject var calibration: CalibrationViewModel
    let angleUnit: AngleUnit
    @Environment(\.locale) private var locale

    var body: some View {
        let snapshot = store.snapshot
        let attitude = snapshot.data
        VStack(alignment: .leading, spacing: 12) {
            Text(AppStrings.text("label.attitude_rate", locale: locale)).font(.headline.monospaced())
            Text(statusText)
                .font(.caption.weight(.semibold))
                .foregroundStyle(statusColor)
            HStack {
                AngleReadout(label: AppStrings.text("label.roll", locale: locale), attitude: attitude, axis: .roll, unit: angleUnit, precision: 3)
                AngleReadout(label: AppStrings.text("label.pitch", locale: locale), attitude: attitude, axis: .pitch, unit: angleUnit, precision: 3)
                AngleReadout(label: AppStrings.text("label.yaw", locale: locale), attitude: attitude, axis: .yaw, unit: angleUnit, precision: 3)
            }
            KeyValueRow(label: AppStrings.text("label.imu_source", locale: locale), value: attitude.source.displayName)
        }
        .padding(16).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private var statusText: String {
        calibration.attitudeStatus.rawValue
    }

    private var statusColor: Color {
        switch calibration.attitudeStatus {
        case .waitCal: return .secondary
        case .calibrating: return .orange
        case .ready: return .green
        case .error: return .red
        }
    }
}

private struct DeveloperCalibrationCard: View {
    @ObservedObject var viewModel: CalibrationViewModel
    @Environment(\.locale) private var locale

    var body: some View {
        TimelineView(.periodic(from: .now, by: 1)) { context in
            let status = viewModel.status
            VStack(alignment: .leading, spacing: 10) {
                Text(AppStrings.text("label.imu_calibration", locale: locale))
                    .font(.headline.monospaced())
                KeyValueRow(label: AppStrings.text("label.state", locale: locale), value: stageName(for: status))
                KeyValueRow(label: AppStrings.text("label.pwm", locale: locale), value: "\(Int(status.currentPWM))%")
                KeyValueRow(label: AppStrings.text("label.progress", locale: locale), value: "\(status.sampleCount) / \(status.totalSample)")
                KeyValueRow(label: AppStrings.text("label.elapsed", locale: locale), value: elapsedText(viewModel.elapsed(at: context.date)))
                ProgressView(value: Double(status.totalProgress), total: 100)
                    .tint(stageColor(for: status))
                    .accessibilityLabel("Calibration progress")
                if status.state == .error {
                    Text("ERROR CODE \(status.errorCode)")
                        .font(.caption.monospaced().weight(.semibold))
                        .foregroundStyle(.red)
                }
            }
            .padding(16)
            .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
        }
    }

    private func stageName(for status: IMUCalibrationStatus) -> String {
        status.stage.displayName
    }

    private func stageColor(for status: IMUCalibrationStatus) -> Color {
        switch status.state {
        case .complete: return .green
        case .error: return .red
        case .idle: return .secondary
        default: return .orange
        }
    }

    private func elapsedText(_ elapsed: TimeInterval) -> String {
        let seconds = max(0, Int(elapsed.rounded(.down)))
        return "\(seconds / 60)m \(seconds % 60)s"
    }
}

private struct StaticCalibrationAnalysisCard: View {
    @ObservedObject var staticCalibration: StaticCalibrationState
    @Environment(\.locale) private var locale

    var body: some View {
        let staticResult = staticCalibration.snapshot.result
        VStack(alignment: .leading, spacing: 10) {
            Text(AppStrings.text("label.imu_calibration_analysis", locale: locale))
                .font(.headline.monospaced())
            StaticCalibrationSection(result: staticResult)
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

}

private struct StaticCalibrationSection: View {
    let result: StaticCalibrationResult
    @Environment(\.locale) private var locale

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text(AppStrings.text("label.static_calibration", locale: locale))
                .font(.subheadline.monospaced().weight(.bold))
            KeyValueRow(label: AppStrings.text("label.state", locale: locale),
                        value: statusText)
                .foregroundStyle(statusColor)
            KeyValueRow(label: AppStrings.text("label.samples", locale: locale),
                        value: sampleText)

            Text(AppStrings.text("label.acc_offset", locale: locale))
                .font(.caption.weight(.bold))
                .foregroundStyle(.secondary)
            HStack {
                StaticCalibrationValue(label: "X", value: result.accelOffsetX)
                StaticCalibrationValue(label: "Y", value: result.accelOffsetY)
                StaticCalibrationValue(label: "Z", value: result.accelOffsetZ)
            }

            Text("LSM303 bias").font(.caption.weight(.bold)).foregroundStyle(.secondary)
            CalibrationVectorReadout(result.lsmAccelBias)
            Text("BMI323 accel bias").font(.caption.weight(.bold)).foregroundStyle(.secondary)
            CalibrationVectorReadout(result.bmiAccelBias)
            Text("BMI323 gyro bias").font(.caption.weight(.bold)).foregroundStyle(.secondary)
            CalibrationVectorReadout(result.bmiGyroBias, unit: .angularVelocityDps)

        }
    }

    private var statusText: String {
        switch result.phase {
        case .waiting: return "Waiting..."
        case .sampling:
            let samples = result.sampleTotal > 0
                ? "\(result.sampleCount)/\(result.sampleTotal)"
                : "\(result.sampleCount)"
            return "Sampling \(samples)"
        case .completed: return "Completed"
        case .error: return "Calibration Error"
        }
    }

    private var sampleText: String {
        guard result.sampleTotal > 0 else { return "--" }
        return "\(result.sampleCount) / \(result.sampleTotal)"
    }

    private var statusColor: Color {
        switch result.phase {
        case .waiting: return .secondary
        case .sampling: return .orange
        case .completed: return .green
        case .error: return .red
        }
    }

    private func calibrationValue(_ value: Float?) -> String {
        guard let value, value.isFinite else { return "--" }
        return String(format: "%.4f g", value / 9.80665)
    }
}

private enum CalibrationValueUnit {
    case accelerationG
    case angularVelocityDps
}

private struct CalibrationVectorReadout: View {
    let vector: Vector3?
    let unit: CalibrationValueUnit

    init(_ vector: Vector3?, unit: CalibrationValueUnit = .accelerationG) {
        self.vector = vector
        self.unit = unit
    }

    var body: some View {
        HStack {
            StaticCalibrationValue(label: "X", value: vector?.x, unit: unit)
            StaticCalibrationValue(label: "Y", value: vector?.y, unit: unit)
            StaticCalibrationValue(label: "Z", value: vector?.z, unit: unit)
        }
    }
}

private struct StaticCalibrationValue: View {
    let label: String
    let value: Float?
    let unit: CalibrationValueUnit

    init(label: String, value: Float?, unit: CalibrationValueUnit = .accelerationG) {
        self.label = label
        self.value = value
        self.unit = unit
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(label)
                .font(.caption2.weight(.bold))
                .foregroundStyle(.secondary)
            Text(calibrationValue)
                .font(.caption.monospaced())
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private var calibrationValue: String {
        guard let value, value.isFinite else { return "--" }
        switch unit {
        case .accelerationG:
            return String(format: "%.4f g", value / 9.80665)
        case .angularVelocityDps:
            return String(format: "%.4f dps", value)
        }
    }
}

private struct ProtocolMonitor: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @Environment(\.locale) private var locale

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text(AppStrings.text("label.protocol_monitor", locale: locale)).font(.headline.monospaced())
            MonitorRow(label: AppStrings.text("label.rx_count", locale: locale), value: "\(viewModel.receivedFrameCount)")
            MonitorRow(label: AppStrings.text("label.tx_count", locale: locale), value: "\(viewModel.transmittedFrameCount)")
            MonitorRow(label: AppStrings.text("label.crc_error", locale: locale), value: AppStrings.text("value.not_available", locale: locale))
                .help(AppStrings.text("help.crc_error", locale: locale))
            MonitorRow(label: AppStrings.text("label.last_packet_type", locale: locale), value: AppPresentationStrings.packetType(viewModel.lastPacketType, locale: locale))
        }
        .padding(16).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}

private struct MonitorRow: View {
    let label: String
    let value: String

    var body: some View {
        HStack {
            Text(label).foregroundStyle(.secondary)
            Spacer()
            Text(value).font(.body.monospaced()).fontWeight(.semibold)
        }
    }
}

private struct DebugConsole: View {
    let messages: [DecodedMessageRecord]
    let refresh: () -> Void
    @Environment(\.locale) private var locale

    var body: some View {
        DisclosureGroup(AppStrings.text("label.debug_console", locale: locale)) {
            HStack {
                Spacer()
                Button(action: refresh) {
                    Label(AppStrings.text("action.refresh_logs", locale: locale), systemImage: "arrow.clockwise")
                }
                .buttonStyle(.bordered)
            }
            ScrollView {
                LazyVStack(alignment: .leading, spacing: 6) {
                    if messages.isEmpty { Text(AppStrings.text("console.no_frames", locale: locale)).foregroundStyle(.secondary) }
                    ForEach(messages.reversed()) { message in
                        Text(AppPresentationStrings.decodedMessage(message, locale: locale))
                            .font(.caption.monospaced())
                            .textSelection(.enabled)
                            .frame(maxWidth: .infinity, alignment: .leading)
                    }
                }
            }
            .frame(minHeight: 190, maxHeight: 290).padding(10).background(.black.opacity(0.06), in: RoundedRectangle(cornerRadius: 6))
        }
        .padding(16).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}
