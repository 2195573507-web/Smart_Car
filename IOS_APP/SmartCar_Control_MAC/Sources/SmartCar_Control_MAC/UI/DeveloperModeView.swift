import SwiftUI

struct DeveloperModeView: View {
    @ObservedObject var viewModel: SmartCarViewModel
    let telemetryStore: TelemetryStore
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
                    IMUCard(title: "BMI323", sensorID: .bmi323, store: telemetryStore.imu, secondaryTitleKey: "label.gyro")
                    IMUCard(title: "LSM303", sensorID: .lsm303, store: telemetryStore.imu, secondaryTitleKey: "label.mag")
                }.frame(maxWidth: 440)
                VStack(spacing: 16) {
                    AttitudeDeveloperCard(store: telemetryStore.attitude)
                    CalibrationCard(viewModel: viewModel.calibrationViewModel)
                    RadarControlCard(viewModel: viewModel, store: telemetryStore.radar)
                    ProtocolMonitor(viewModel: viewModel)
                }.frame(maxWidth: 500)
            }
            .frame(maxWidth: .infinity, alignment: .center)
            .padding(.horizontal, 24).padding(.bottom, 24)
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
                Text(AppStrings.text("label.radar_speed", locale: locale))
                    .font(.caption.weight(.bold))
                Spacer()
                Text("\(Int(viewModel.radarSpeed))%")
                    .font(.caption.monospaced())
            }
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
            KeyValueRow(
                label: AppStrings.text("label.target_speed", locale: locale),
                value: "\(snapshot.speedPercent)%"
            )
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private func statusText(_ availability: RadarAvailability) -> String {
        switch availability {
        case .online:
            return AppStrings.text("status.online", locale: locale)
        case .offline:
            return AppStrings.text("status.offline", locale: locale)
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

private struct IMUCard: View {
    let title: String
    let sensorID: IMUSensorID
    @ObservedObject var store: IMUState
    let secondaryTitleKey: String
    @Environment(\.locale) private var locale

    var body: some View {
        let data = sensorID == .bmi323 ? store.snapshot.bmi323 : store.snapshot.lsm303
        let secondaryVector = sensorID == .bmi323 ? data.gyro : data.mag
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text(title).font(.headline.monospaced())
                Spacer()
                Text(AppPresentationStrings.availability(data.online, locale: locale))
                    .font(.caption.monospaced().weight(.bold))
                    .foregroundStyle(data.online ? .green : .red)
            }
            VectorReadout(titleKey: "label.accel", vector: data.accel)
            VectorReadout(titleKey: secondaryTitleKey, vector: secondaryVector)
        }
        .padding(16).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
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
    @Environment(\.locale) private var locale

    var body: some View {
        let snapshot = store.snapshot
        let attitude = snapshot.data
        let status = snapshot.displayStatus
        VStack(alignment: .leading, spacing: 12) {
            Text(AppStrings.text("label.attitude_rate", locale: locale)).font(.headline.monospaced())
            Text(statusText)
                .font(.caption.weight(.semibold))
                .foregroundStyle(status == .valid ? .green : .red)
            HStack {
                AngleReadout(label: AppStrings.text("label.roll", locale: locale), value: attitude.roll, precision: 3)
                AngleReadout(label: AppStrings.text("label.pitch", locale: locale), value: attitude.pitch, precision: 3)
                AngleReadout(label: AppStrings.text("label.yaw", locale: locale), value: attitude.yaw, precision: 3)
            }
            KeyValueRow(label: AppStrings.text("label.imu_source", locale: locale), value: attitude.source.displayName)
            KeyValueRow(label: "BMI323", value: AppStrings.text("status.offline", locale: locale))
        }
        .padding(16).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private var statusText: String {
        switch store.snapshot.displayStatus {
        case .valid: return AppStrings.text("attitude.valid", locale: locale)
        case .invalid: return AppStrings.text("attitude.invalid", locale: locale)
        case .timeout: return AppStrings.text("attitude.timeout", locale: locale)
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
