import SwiftUI

struct ControlDashboardView: View {
    @ObservedObject var viewModel: SmartCarViewModel

    private var telemetry: TelemetryStore { viewModel.telemetryStore }

    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                ConnectionStatusCard(viewModel: viewModel)
                    .frame(maxWidth: .infinity)
                PosePanel(
                    attitude: telemetry.attitude.snapshot.data,
                    targetYawDegrees: telemetry.chassis.snapshot.state?.yawDeg
                )
                .frame(maxWidth: .infinity)
                SpeedControlPanel(
                    viewModel: viewModel,
                    store: telemetry.wheelSpeed,
                    attitude: telemetry.attitude
                )
                    .frame(maxWidth: .infinity)
                WheelActualPanel(viewModel: viewModel, store: telemetry.wheelSpeed)
                    .frame(maxWidth: .infinity)
            }
            .frame(maxWidth: .infinity)
            .padding(.horizontal, 14)
            .padding(.top, 14)
            .padding(.bottom, 80)
        }
        .scrollIndicators(.hidden)
        .safeAreaInset(edge: .top, spacing: 0) {
            EStopButton {
                Haptics.impact(.heavy)
                viewModel.emergencyWheelBrake()
            }
            .padding(.horizontal, 14)
            .padding(.vertical, 7)
            .background(.regularMaterial)
        }
        .navigationTitle("操控")
    }
}

struct PosePanel: View {
    let attitude: AttitudeData
    let targetYawDegrees: Float?

    private var statusText: String {
        attitude.valid ? "AHRS VALID" : "AHRS WAITING"
    }

    var body: some View {
        Surface {
            VStack(alignment: .leading, spacing: 10) {
                SectionTitle(title: "LIVE POSE", systemImage: "cube.transparent", trailing: statusText)
                SceneKitPoseView(attitude: attitude, targetYawDegrees: targetYawDegrees)
                    .frame(maxWidth: .infinity)
                    .frame(height: 230)
                HStack(spacing: 8) {
                    MetricTile(label: "ROLL", value: degreeText(attitude.rollRad), tint: .teal)
                    MetricTile(label: "PITCH", value: degreeText(attitude.pitchRad), tint: .teal)
                    MetricTile(label: "YAW", value: degreeText(attitude.yawRad), tint: .teal)
                }
                HStack(alignment: .center, spacing: 12) {
                    HeadingCompass(headingDegrees: attitude.yawDeg, targetDegrees: targetYawDegrees)
                        .frame(width: 92, height: 92)
                    VStack(alignment: .leading, spacing: 7) {
                        StatusPill(title: "HEADING LOCK", tint: .orange)
                        Text(targetYawDegrees.map { "TARGET \(String(format: "%.1f°", $0))" } ?? "TARGET --")
                            .font(.caption2.monospaced())
                            .foregroundStyle(.secondary)
                        Text("NORTH-UP COMPASS")
                            .font(.caption2.monospaced())
                            .foregroundStyle(.secondary)
                    }
                }
            }
        }
    }
}

struct HeadingCompass: View {
    let headingDegrees: Float
    let targetDegrees: Float?

    var body: some View {
        GeometryReader { proxy in
            let diameter = min(proxy.size.width, proxy.size.height)
            ZStack {
                Circle()
                    .stroke(.secondary.opacity(0.45), lineWidth: 1)
                Circle()
                    .stroke(.secondary.opacity(0.2), style: StrokeStyle(lineWidth: 1, dash: [2, 4]))
                    .padding(diameter * 0.12)
                ForEach(Array(["N", "E", "S", "W"].enumerated()), id: \.offset) { index, label in
                    Text(label)
                        .font(.caption2.weight(.bold).monospaced())
                        .foregroundStyle(index == 0 ? .red : .secondary)
                        .position(cardinalPosition(index: index, diameter: diameter))
                }
                Image(systemName: "location.north.fill")
                    .font(.title3.weight(.bold))
                    .foregroundStyle(.teal)
                    .rotationEffect(.degrees(Double(headingDegrees)))
                if let targetDegrees {
                    Circle()
                        .fill(.orange)
                        .frame(width: 7, height: 7)
                        .offset(y: -diameter * 0.39)
                        .rotationEffect(.degrees(Double(targetDegrees)))
                }
            }
            .frame(width: diameter, height: diameter)
            .frame(maxWidth: .infinity, maxHeight: .infinity)
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("航向罗盘")
        .accessibilityValue("当前 \(String(format: "%.1f°", headingDegrees))，目标 \(targetDegrees.map { String(format: "%.1f°", $0) } ?? "未设置")")
    }

    private func cardinalPosition(index: Int, diameter: CGFloat) -> CGPoint {
        let radius = diameter * 0.37
        let angle = (Double(index) * .pi / 2) - .pi / 2
        return CGPoint(
            x: diameter / 2 + CGFloat(cos(angle)) * radius,
            y: diameter / 2 + CGFloat(sin(angle)) * radius
        )
    }
}

struct SpeedControlPanel: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @ObservedObject var store: WheelSpeedState
    @ObservedObject var attitude: AttitudeState

    @State private var showIndividual = true

    private let names = ["M1 RR", "M2 RF", "M3 LR", "M4 LF"]

    private var allTargetsMatch: Bool {
        guard let first = viewModel.wheelTargets.first else { return true }
        return viewModel.wheelTargets.allSatisfy { $0 == first }
    }

    private var allTargetLabel: String {
        allTargetsMatch ? speedText(viewModel.wheelTargets[0]) : "MIXED"
    }

    var body: some View {
        Surface {
            VStack(alignment: .leading, spacing: 13) {
                SectionTitle(
                    title: "WHEEL SPEED",
                    systemImage: "gauge.with.dots.needle.67percent",
                    trailing: store.snapshot.voltage.map { String(format: "%.2f V", $0) } ?? "-- V"
                )

                HStack {
                    Label("控制模式", systemImage: "switch.2")
                        .font(.caption.weight(.bold))
                    Spacer(minLength: 8)
                    Text(store.snapshot.mode == .chassisDiff ? "底盘差速" : "独立调试")
                        .font(.caption.weight(.bold))
                        .foregroundStyle(store.snapshot.mode == .chassisDiff ? .green : .yellow)
                }

                Picker("控制模式", selection: Binding(
                    get: { viewModel.controlMode },
                    set: { newMode in
                        Haptics.impact(.medium)
                        viewModel.setControlMode(newMode)
                    }
                )) {
                    Text("底盘差速直行").tag(ChassisControlMode.chassisDiff)
                    Text("独立调试").tag(ChassisControlMode.wheelIndependent)
                }
                .pickerStyle(.segmented)
                .disabled(viewModel.status != .connected)

                VStack(alignment: .leading, spacing: 7) {
                    HStack {
                        Label(viewModel.controlMode == .chassisDiff ? "直行目标速度" : "ALL WHEELS",
                              systemImage: viewModel.controlMode == .chassisDiff ? "arrow.forward" : "link")
                            .font(.caption.weight(.bold))
                        Spacer()
                        Text(viewModel.controlMode == .chassisDiff
                             ? "\(speedText(viewModel.chassisBaseSpeed)) mm/s"
                             : "\(allTargetLabel) mm/s")
                            .font(.caption.monospaced())
                    }
                    Slider(
                        value: Binding(
                            get: {
                                viewModel.controlMode == .chassisDiff
                                    ? Double(viewModel.chassisBaseSpeed)
                                    : (allTargetsMatch ? Double(viewModel.wheelTargets[0]) : 0)
                            },
                            set: {
                                if viewModel.controlMode == .chassisDiff {
                                    viewModel.setChassisBaseSpeed($0)
                                } else {
                                    viewModel.setAllWheelTargets($0)
                                }
                            }
                        ),
                        in: -800...800,
                        step: 10,
                        onEditingChanged: { editing in
                            if !editing { Haptics.impact(.light) }
                        }
                    )
                    .disabled(viewModel.status != .connected)
                    HStack {
                        Text("-800").frame(maxWidth: .infinity, alignment: .leading)
                        Text("0")
                        Text("+800").frame(maxWidth: .infinity, alignment: .trailing)
                    }
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
                }

                Divider()
                TargetYawControl(viewModel: viewModel, attitude: attitude)

                DisclosureGroup(isExpanded: $showIndividual) {
                    VStack(alignment: .leading, spacing: 8) {
                        ForEach(0..<4, id: \.self) { index in
                            WheelTargetRow(
                                name: names[index],
                                color: wheelColor(index),
                                target: viewModel.wheelTargets[index],
                                actual: store.snapshot.actual[index],
                                isEnabled: viewModel.status == .connected &&
                                    viewModel.controlMode == .wheelIndependent,
                                onChange: { value in viewModel.setWheelTarget(index: index, value: value) },
                                onStep: { delta in
                                    Haptics.impact(.light)
                                    viewModel.nudgeWheelTarget(
                                        index: index,
                                        delta: delta
                                    )
                                }
                            )
                        }
                    }
                    .padding(.top, 8)
                } label: {
                    Text("INDIVIDUAL TARGETS")
                        .font(.caption2.monospaced())
                        .foregroundStyle(.secondary)
                }
            }
        }
    }
}

private struct TargetYawControl: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @ObservedObject var attitude: AttitudeState
    @State private var yawText = "0.0"

    private var attitudeSnapshot: AttitudeStateSnapshot { attitude.snapshot }
    private var yawIsValid: Bool {
        attitudeSnapshot.displayStatus == .valid &&
            attitudeSnapshot.data.valid && attitudeSnapshot.data.yawDeg.isFinite
    }
    private var controlsEnabled: Bool {
        viewModel.status == .connected && viewModel.controlMode == .chassisDiff
    }
    private var currentYawText: String {
        yawIsValid ? String(format: "%.1f°", attitudeSnapshot.data.yawDeg) : "--"
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 8) {
                Label("定向巡航", systemImage: "location.north.line")
                    .font(.caption.weight(.bold))
                Text("TARGET YAW")
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
                Spacer(minLength: 8)
                Text(yawIsValid ? "AHRS VALID" : "AHRS WAITING")
                    .font(.caption2.monospaced())
                    .foregroundStyle(yawIsValid ? .green : .secondary)
            }
            HStack(spacing: 8) {
                Text("当前 (currentYawText)")
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
                Spacer(minLength: 8)
                Text("目标").font(.caption.weight(.semibold))
                TextField("角度", text: $yawText)
                    .textFieldStyle(.roundedBorder)
                    .multilineTextAlignment(.trailing)
                    .font(.caption.monospacedDigit())
                    .frame(width: 72)
                    .disabled(!controlsEnabled)
                    .onSubmit(commitYawText)
                Text("°").font(.caption.monospaced())
                Stepper("", value: Binding(
                    get: { Double(viewModel.targetYawDeg) },
                    set: { viewModel.setTargetYaw($0); syncYawText() }
                ), in: -180...180, step: 1)
                .labelsHidden()
                .disabled(!controlsEnabled)
            }
            HStack(spacing: 10) {
                Button(action: viewModel.alignTargetYawToCurrent) {
                    Label("当前航向对齐", systemImage: "scope")
                        .font(.caption.weight(.semibold))
                }
                .buttonStyle(.bordered)
                .disabled(!controlsEnabled || !yawIsValid)
                Spacer(minLength: 8)
                Toggle("巡航锁航", isOn: Binding(
                    get: { viewModel.headingLockEnabled },
                    set: { viewModel.setHeadingLockEnabled($0) }
                ))
                .font(.caption.weight(.semibold))
                .toggleStyle(.switch)
                .disabled(!controlsEnabled)
            }
        }
        .onAppear(perform: syncYawText)
        .onChange(of: viewModel.targetYawDeg) { syncYawText() }
    }

    private func commitYawText() {
        guard let value = Double(yawText), value.isFinite else {
            syncYawText()
            return
        }
        viewModel.setTargetYaw(value)
        syncYawText()
    }

    private func syncYawText() {
        yawText = String(format: "%.1f", Double(viewModel.targetYawDeg))
    }
}

private struct WheelTargetRow: View {
    let name: String
    let color: Color
    let target: Float
    let actual: Float
    let isEnabled: Bool
    let onChange: (Double) -> Void
    let onStep: (Double) -> Void

    var body: some View {
        VStack(alignment: .leading, spacing: 7) {
            HStack(spacing: 8) {
                Text(name)
                    .font(.caption.weight(.semibold))
                    .foregroundStyle(color)
                Spacer(minLength: 8)
                VStack(alignment: .trailing, spacing: 1) {
                    Text("T \(speedText(target))")
                    Text("A \(speedText(actual))")
                        .foregroundStyle(.secondary)
                }
                .font(.caption2.monospaced())
            }
            HStack(spacing: 8) {
                Button { onStep(-10) } label: {
                    Image(systemName: "minus.circle.fill")
                }
                .buttonStyle(.borderless)
                .disabled(!isEnabled)
                Slider(
                    value: Binding(
                        get: { Double(target) },
                        set: onChange
                    ),
                    in: -800...800,
                    step: 10,
                    onEditingChanged: { editing in
                        if !editing { Haptics.impact(.light) }
                    }
                )
                .disabled(!isEnabled)
                Button { onStep(10) } label: {
                    Image(systemName: "plus.circle.fill")
                }
                .buttonStyle(.borderless)
                .disabled(!isEnabled)
            }
            Text("TARGET \(speedText(target))  /  ACTUAL \(speedText(actual)) mm/s")
                .font(.caption2.monospaced())
                .foregroundStyle(.secondary)
        }
    }
}

struct WheelActualPanel: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @ObservedObject var store: WheelSpeedState

    private let names = ["RR", "RF", "LR", "LF"]

    var body: some View {
        Surface {
            VStack(alignment: .leading, spacing: 10) {
                SectionTitle(title: "WHEEL TARGET VS ACTUAL", systemImage: "chart.xyaxis.line", trailing: "20 Hz")
                ForEach(0..<4, id: \.self) { index in
                    VStack(alignment: .leading, spacing: 6) {
                        HStack(spacing: 8) {
                        Text(names[index])
                            .font(.caption.weight(.bold).monospaced())
                            .foregroundStyle(wheelColor(index))
                            Spacer(minLength: 8)
                            Text("TARGET \(speedText(viewModel.wheelTargets[index]))")
                            Text("ACTUAL \(speedText(store.snapshot.actual[index]))")
                                .foregroundStyle(.secondary)
                        }
                        .font(.caption2.monospaced())
                        WheelSparkline(values: store.snapshot.history[index], target: viewModel.wheelTargets[index], color: wheelColor(index))
                            .frame(maxWidth: .infinity)
                            .frame(height: 35)
                    }
                }
            }
        }
    }
}

private struct WheelSparkline: View {
    let values: [Float]
    let target: Float
    let color: Color

    var body: some View {
        GeometryReader { proxy in
            let peak = max(50, values.map { abs($0) }.max() ?? 0, abs(target)) * 1.2
            ZStack {
                Path { path in
                    path.move(to: CGPoint(x: 0, y: proxy.size.height / 2))
                    path.addLine(to: CGPoint(x: proxy.size.width, y: proxy.size.height / 2))
                }
                .stroke(.secondary.opacity(0.25), style: StrokeStyle(lineWidth: 1, dash: [3, 3]))
                SpeedHistoryPath(values: values, peak: peak)
                    .stroke(color, style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round))
                let y = proxy.size.height / 2 - CGFloat(max(-1, min(1, target / peak))) * proxy.size.height * 0.42
                Path { path in
                    path.move(to: CGPoint(x: 0, y: y))
                    path.addLine(to: CGPoint(x: proxy.size.width, y: y))
                }
                .stroke(.white.opacity(0.65), style: StrokeStyle(lineWidth: 1, dash: [4, 3]))
            }
        }
    }
}

private struct SpeedHistoryPath: Shape {
    let values: [Float]
    let peak: Float

    func path(in rect: CGRect) -> Path {
        guard values.count > 1, peak > 0 else { return Path() }
        var path = Path()
        let step = rect.width / CGFloat(values.count - 1)
        for (index, value) in values.enumerated() where value.isFinite {
            let x = CGFloat(index) * step
            let normalized = max(-1, min(1, value / peak))
            let y = rect.midY - CGFloat(normalized) * rect.height * 0.42
            if index == 0 { path.move(to: CGPoint(x: x, y: y)) } else { path.addLine(to: CGPoint(x: x, y: y)) }
        }
        return path
    }
}

func wheelColor(_ index: Int) -> Color {
    [.orange, .blue, .green, .purple][index % 4]
}
