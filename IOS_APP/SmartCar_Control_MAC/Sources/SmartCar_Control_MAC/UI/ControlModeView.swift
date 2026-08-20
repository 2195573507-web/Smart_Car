import SwiftUI

struct ControlModeView: View {
    @ObservedObject var viewModel: SmartCarViewModel
    let telemetryStore: TelemetryStore
    let angleUnit: AngleUnit
    @Environment(\.locale) private var locale

    var body: some View {
        ScrollView {
            HStack(alignment: .top, spacing: 22) {
            VStack(alignment: .leading, spacing: 16) {
                ConnectionPanel(viewModel: viewModel)
                VehicleCard(statusStore: telemetryStore.status, imuStore: telemetryStore.imu)
                AttitudeCard(store: telemetryStore.attitude, angleUnit: angleUnit)
                CalibrationCard(
                    viewModel: viewModel.calibrationViewModel,
                    staticCalibration: telemetryStore.staticCalibration
                )
            }
            .frame(maxWidth: 365)

            VStack(spacing: 20) {
                Text(AppStrings.text("label.drive_control", locale: locale)).font(.headline.monospaced()).frame(maxWidth: .infinity, alignment: .leading)
                VirtualJoystick(viewModel: viewModel)
                    .disabled(viewModel.status != .connected)
                VStack(alignment: .leading, spacing: 8) {
                    HStack { Text(AppStrings.text("label.speed", locale: locale)).font(.caption.weight(.bold)); Spacer(); Text("\(Int(viewModel.speed))%").font(.caption.monospaced()) }
                    Slider(value: $viewModel.speed, in: 0...100, step: 1, onEditingChanged: { editing in if !editing { viewModel.updateSpeed() } })
                        .disabled(viewModel.status != .connected)
                }
                .padding(16)
                .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
                Button(action: viewModel.emergencyStop) {
                    Label(AppStrings.text("action.stop", locale: locale), systemImage: "hand.raised.fill")
                        .font(.title3.weight(.bold)).frame(maxWidth: .infinity).padding(.vertical, 13)
                }
                .buttonStyle(.borderedProminent).tint(.red)
            }
            .frame(maxWidth: 420)
            .padding(20)
            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 8))
            }
            .frame(maxWidth: .infinity, alignment: .center)
            .padding(.horizontal, 24).padding(.bottom, 24)
        }
    }
}

struct CalibrationCard: View {
    @ObservedObject var viewModel: CalibrationViewModel
    @ObservedObject var staticCalibration: StaticCalibrationState
    @Environment(\.locale) private var locale

    var body: some View {
        TimelineView(.periodic(from: .now, by: 1)) { context in
            if viewModel.status.isWaitingForSTM(at: context.date) {
                VStack(spacing: 10) {
                    Image(systemName: "antenna.radiowaves.left.and.right")
                        .foregroundStyle(.secondary)
                    Text("Waiting STM response")
                        .foregroundStyle(.secondary)
                }
                .frame(maxWidth: .infinity, minHeight: 120)
            } else {
            VStack(alignment: .leading, spacing: 12) {
            Text(AppStrings.text("calibration.title", locale: locale))
                .font(.headline.monospaced())
            Text(AppStrings.text("calibration.keep_still", locale: locale))
                .font(.caption)
                .foregroundStyle(.secondary)

            HStack {
                Text(AppStrings.text("calibration.status", locale: locale))
                    .foregroundStyle(.secondary)
                Spacer()
                Text(statusText)
                    .font(.body.weight(.semibold))
                    .foregroundStyle(statusColor)
            }

            ProgressView(value: Double(viewModel.status.totalProgress), total: 100) {
                Text(AppStrings.text("calibration.progress", locale: locale))
            } currentValueLabel: {
                Text("\(viewModel.status.totalProgress)%")
                    .font(.caption.monospaced())
            }
            .tint(statusColor)

            if viewModel.status.state == .sample {
                KeyValueRow(label: "Sample",
                            value: "\(viewModel.status.sampleProgress)")
            }
            if viewModel.status.state == .error {
                Text("PWM error (\(viewModel.status.errorCode))")
                    .font(.caption.weight(.semibold))
                    .foregroundStyle(.red)
            }

            Divider()
            Text(AppStrings.text("calibration.bias", locale: locale))
                .font(.caption.weight(.bold))
                .foregroundStyle(.secondary)
            HStack {
                CalibrationBiasValue(label: "X", value: viewModel.bias?.x)
                CalibrationBiasValue(label: "Y", value: viewModel.bias?.y)
                CalibrationBiasValue(label: "Z", value: viewModel.bias?.z)
            }

            let staticResult = staticCalibration.snapshot.result
            Divider()
            Text("IMU Calibration Result")
                .font(.caption.weight(.bold))
                .foregroundStyle(.secondary)
            CalibrationVectorRow(title: "LSM303 Accel Bias", vector: staticResult.lsmAccelBias)
            CalibrationVectorRow(title: "BMI323 Accel Bias", vector: staticResult.bmiAccelBias)
            CalibrationVectorRow(title: "BMI323 Gyro Bias", vector: staticResult.bmiGyroBias)

            if viewModel.status.state == .complete {
                Label(AppStrings.text("calibration.completed", locale: locale),
                      systemImage: "checkmark.circle.fill")
                    .font(.callout.weight(.semibold))
                    .foregroundStyle(.green)
            }
            }
            }
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private var statusText: String {
        switch viewModel.status.state {
        case .idle: return AppStrings.text("calibration.idle", locale: locale)
        case .setPWM: return AppStrings.text("calibration.set_pwm", locale: locale)
        case .waitStable: return AppStrings.text("calibration.stabilize", locale: locale)
        case .sample:
            return AppStrings.text("calibration.static_sampling", locale: locale)
        case .complete: return AppStrings.text("calibration.done", locale: locale)
        case .error: return AppStrings.text("calibration.failed", locale: locale)
        }
    }

    private var statusColor: Color {
        switch viewModel.status.state {
        case .complete: return .green
        case .error: return .red
        case .setPWM, .waitStable, .sample: return .orange
        case .idle: return .secondary
        }
    }
}

private struct CalibrationVectorRow: View {
    let title: String
    let vector: Vector3?

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title)
                .font(.caption2.weight(.bold))
                .foregroundStyle(.secondary)
            HStack {
                CalibrationBiasValue(label: "X", value: vector?.x)
                CalibrationBiasValue(label: "Y", value: vector?.y)
                CalibrationBiasValue(label: "Z", value: vector?.z)
            }
        }
    }
}

private struct CalibrationBiasValue: View {
    let label: String
    let value: Float?

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text("Bias \(label)")
                .font(.caption2.weight(.bold))
                .foregroundStyle(.secondary)
            Text(value?.displayValue ?? "--")
                .font(.caption.monospaced())
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct ConnectionPanel: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @Environment(\.locale) private var locale

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text(AppStrings.text("label.connection", locale: locale)).font(.headline.monospaced())
            KeyValueRow(label: AppStrings.text("label.ble", locale: locale), value: AppPresentationStrings.connectionStatus(viewModel.status, locale: locale))
            KeyValueRow(label: AppStrings.text("label.device", locale: locale), value: viewModel.discoveredDeviceName)
            if let lastError = viewModel.lastError {
                Text(lastError.localizedDescription(locale: locale))
                    .font(.caption).foregroundStyle(.red).fixedSize(horizontal: false, vertical: true)
            }
        }
        .padding(16).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}

private struct VehicleCard: View {
    @ObservedObject var statusStore: VehicleStatusState
    @ObservedObject var imuStore: IMUState
    @Environment(\.locale) private var locale

    var body: some View {
        let state = statusStore.snapshot
        let imu = imuStore.snapshot
        VStack(alignment: .leading, spacing: 12) {
            Text(AppStrings.text("label.vehicle", locale: locale)).font(.headline.monospaced())
            HStack {
                VStack(alignment: .leading) { Text(AppStrings.text("label.battery", locale: locale)).font(.caption.weight(.bold)); Text("\(state.battery)%").font(.system(.title2, design: .rounded, weight: .bold)) }
                Spacer()
                VStack(alignment: .trailing) { Text(AppStrings.text("label.motor", locale: locale)).font(.caption.weight(.bold)); Text("\(state.motorState)").font(.title3.monospaced()) }
            }
            Divider()
            KeyValueRow(label: "LSM303", value: AppPresentationStrings.availability(imu.lsm303.online, locale: locale))
            KeyValueRow(label: "BMI323", value: AppPresentationStrings.availability(imu.model.bmi323.online, locale: locale))
            KeyValueRow(label: AppStrings.text("label.error", locale: locale), value: String(format: "0x%04X", state.errorCode))
        }
        .padding(16).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}

private struct AttitudeCard: View {
    @ObservedObject var store: AttitudeState
    let angleUnit: AngleUnit
    @Environment(\.locale) private var locale

    var body: some View {
        let snapshot = store.snapshot
        let attitude = snapshot.data
        let status = snapshot.displayStatus
        VStack(alignment: .leading, spacing: 12) {
            Text(AppStrings.text("label.vehicle_attitude", locale: locale)).font(.headline.monospaced())
            Text(statusText)
                .font(.caption.weight(.semibold))
                .foregroundStyle(status == .valid ? .green : .red)
            ZStack {
                RoundedRectangle(cornerRadius: 5).fill(.blue.gradient).frame(width: 150, height: 82)
                    .overlay { Capsule().fill(.white.opacity(0.8)).frame(width: 48, height: 12).offset(y: -18) }
                    .rotation3DEffect(rotation(attitude, axis: .pitch), axis: (x: 1, y: 0, z: 0))
                    .rotation3DEffect(rotation(attitude, axis: .roll), axis: (x: 0, y: 1, z: 0))
                    .rotationEffect(rotation(attitude, axis: .yaw))
            }
            .frame(maxWidth: .infinity, minHeight: 118)
            HStack {
                AngleReadout(label: AppStrings.text("label.roll", locale: locale), attitude: attitude, axis: .roll, unit: angleUnit, precision: 1)
                AngleReadout(label: AppStrings.text("label.pitch", locale: locale), attitude: attitude, axis: .pitch, unit: angleUnit, precision: 1)
                AngleReadout(label: AppStrings.text("label.yaw", locale: locale), attitude: attitude, axis: .yaw, unit: angleUnit, precision: 1)
            }
        }
        .padding(16).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private func rotation(_ attitude: AttitudeData, axis: AttitudeAxis) -> Angle {
        let value = attitude.value(for: axis, unit: angleUnit)
        return angleUnit == .degree ? .degrees(Double(value)) : .radians(Double(value))
    }

    private var statusText: String {
        switch store.snapshot.displayStatus {
        case .valid: return AppStrings.text("attitude.valid", locale: locale)
        case .invalid: return AppStrings.text("attitude.invalid", locale: locale)
        case .timeout: return AppStrings.text("attitude.timeout", locale: locale)
        }
    }
}

private struct VirtualJoystick: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @Environment(\.locale) private var locale
    @State private var translation: CGSize = .zero
    @State private var activeCommand: SmartCarProtocol.ControlCommand?

    var body: some View {
        GeometryReader { geometry in
            let side = min(geometry.size.width, geometry.size.height)
            let limit = side * 0.30
            ZStack {
                Circle().fill(.quaternary.opacity(0.7))
                Circle().strokeBorder(.secondary.opacity(0.35), lineWidth: 1)
                Crosshair()
                Circle()
                    .fill(.blue.gradient)
                    .frame(width: side * 0.34, height: side * 0.34)
                    .overlay(Image(systemName: "arrow.up").font(.title2.weight(.bold)).foregroundStyle(.white))
                    .offset(clamped(translation, limit: limit))
            }
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { value in
                        translation = value.translation
                        let command = command(for: value.translation)
                        if command != activeCommand {
                            activeCommand = command
                            command.map(viewModel.send)
                        }
                    }
                    .onEnded { _ in
                        translation = .zero
                        activeCommand = nil
                        viewModel.stop()
                    }
            )
        }
        .frame(width: 230, height: 230)
        .accessibilityLabel(AppStrings.text("accessibility.drive_joystick", locale: locale))
    }

    private func command(for translation: CGSize) -> SmartCarProtocol.ControlCommand? {
        let threshold: CGFloat = 18
        guard max(abs(translation.width), abs(translation.height)) >= threshold else { return nil }
        if abs(translation.width) > abs(translation.height) {
            return translation.width < 0 ? .turnLeft : .turnRight
        }
        return translation.height < 0 ? .moveForward : .moveBack
    }
}

private struct Crosshair: View {
    var body: some View {
        GeometryReader { geometry in
            Path { path in
                let center = CGPoint(x: geometry.size.width / 2, y: geometry.size.height / 2)
                path.move(to: CGPoint(x: 0, y: center.y))
                path.addLine(to: CGPoint(x: geometry.size.width, y: center.y))
                path.move(to: CGPoint(x: center.x, y: 0))
                path.addLine(to: CGPoint(x: center.x, y: geometry.size.height))
            }
            .stroke(.secondary.opacity(0.28), style: StrokeStyle(lineWidth: 1, dash: [4, 5]))
        }
    }
}

private func clamped(_ value: CGSize, limit: CGFloat) -> CGSize {
    CGSize(width: min(max(value.width, -limit), limit), height: min(max(value.height, -limit), limit))
}

struct KeyValueRow: View {
    let label: String
    let value: String
    var body: some View { HStack { Text(label).foregroundStyle(.secondary); Spacer(); Text(value).font(.body.monospaced()).fontWeight(.semibold) } }
}

struct ValueReadout: View {
    let label: String
    let value: Float
    var body: some View { VStack(alignment: .leading, spacing: 2) { Text(label).font(.caption2.weight(.bold)).foregroundStyle(.secondary); Text(value.displayValue).font(.caption.monospaced()) }.frame(maxWidth: .infinity, alignment: .leading) }
}

struct AngleReadout: View {
    let label: String
    let attitude: AttitudeData
    let axis: AttitudeAxis
    let unit: AngleUnit
    let precision: Int

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(label).font(.caption2.weight(.bold)).foregroundStyle(.secondary)
            Text(unit.format(attitude.value(for: axis, unit: unit), precision: precision))
                .font(.caption.monospaced())
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}
