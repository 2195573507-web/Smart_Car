import SwiftUI

struct WheelSpeedControlCard: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @ObservedObject var store: WheelSpeedState
    @ObservedObject var attitude: AttitudeState

    private let names = ["M1 RR", "M2 RF", "M3 LR", "M4 LF"]
    private let colors: [Color] = [.orange, .blue, .green, .purple]

    private var allTargetsMatch: Bool {
        guard let first = viewModel.wheelTargets.first else { return true }
        return viewModel.wheelTargets.allSatisfy { $0 == first }
    }

    private var allTargetLabel: String {
        allTargetsMatch
            ? String(format: "%.0f", Double(viewModel.wheelTargets[0]))
            : "MIXED"
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            HStack {
                Label("WHEEL SPEED", systemImage: "gauge.with.dots.needle.67percent")
                    .font(.headline.monospaced())
                Spacer()
                if let voltage = store.snapshot.voltage {
                    Text(String(format: "%.2f V", voltage))
                        .font(.headline.monospaced())
                        .foregroundStyle(.green)
                } else {
                    Text("-- V").font(.headline.monospaced()).foregroundStyle(.secondary)
                }
            }

            HStack {
                Label("控制模式", systemImage: "switch.2")
                    .font(.caption.weight(.bold))
                Spacer()
                Text(store.snapshot.mode == .chassisDiff ? "底盘差速" : "独立调试")
                    .font(.caption.weight(.bold))
                    .foregroundStyle(store.snapshot.mode == .chassisDiff ? .green : .yellow)
            }

            Picker("控制模式", selection: Binding(
                get: { viewModel.controlMode },
                set: { viewModel.setControlMode($0) }
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
                         ? String(format: "%.0f mm/s", Double(viewModel.chassisBaseSpeed))
                         : allTargetLabel)
                        .font(.caption.monospaced())
                        .frame(minWidth: 58, alignment: .trailing)
                }
                Slider(value: Binding(
                    get: {
                        if viewModel.controlMode == .chassisDiff {
                            return Double(viewModel.chassisBaseSpeed)
                        }
                        return allTargetsMatch ? Double(viewModel.wheelTargets[0]) : 0.0
                    },
                    set: {
                        if viewModel.controlMode == .chassisDiff {
                            viewModel.setChassisBaseSpeed($0)
                        } else {
                            viewModel.setAllWheelTargets($0)
                        }
                    }
                ), in: -800...800, step: 10)
            }
            .disabled(viewModel.status != .connected)

            Divider()
            TargetYawControl(viewModel: viewModel, attitude: attitude)

            VStack(alignment: .leading, spacing: 8) {
                Text("INDIVIDUAL TARGETS")
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
                ForEach(0..<4, id: \.self) { index in
                    HStack(spacing: 8) {
                        Text(names[index])
                            .font(.caption.weight(.semibold))
                            .foregroundStyle(colors[index])
                            .frame(width: 58, alignment: .leading)
                        Slider(value: Binding(
                            get: { Double(viewModel.wheelTargets[index]) },
                            set: { viewModel.setWheelTarget(index: index, value: $0) }
                        ), in: -800...800, step: 10)
                        VStack(alignment: .trailing, spacing: 1) {
                            Text(String(format: "%.0f", Double(viewModel.wheelTargets[index])))
                                .font(.caption.monospaced())
                            Text(String(format: "%.0f", Double(store.snapshot.actual[index])))
                                .font(.caption2.monospaced())
                                .foregroundStyle(.secondary)
                        }
                        .frame(width: 72, alignment: .trailing)
                    }
                }
            }
            .disabled(viewModel.status != .connected || viewModel.controlMode != .wheelIndependent)

            VStack(alignment: .leading, spacing: 7) {
                HStack {
                    Text("ACTUAL SPEED - mm/s")
                        .font(.caption2.monospaced())
                        .foregroundStyle(.secondary)
                    Spacer()
                    Text("-- target")
                        .font(.caption2.monospaced())
                        .foregroundStyle(.secondary)
                }
                VStack(spacing: 6) {
                    ForEach(0..<4, id: \.self) { index in
                        WheelSpeedLane(
                            name: names[index],
                            color: colors[index],
                            values: store.snapshot.history[index],
                            target: viewModel.wheelTargets[index]
                        )
                        .frame(height: 61)
                    }
                }
                .frame(height: 264)
                HStack {
                    Text("-20 s").frame(maxWidth: .infinity, alignment: .leading)
                    Text("now").frame(maxWidth: .infinity, alignment: .trailing)
                }
                .font(.caption2.monospaced())
                .foregroundStyle(.secondary)
            }
            .padding(.top, 2)

            HStack {
                Text("M1 orange - M2 blue - M3 green - M4 purple")
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
                Spacer()
                Button(action: viewModel.emergencyWheelBrake) {
                    Label("BRAKE", systemImage: "hand.raised.fill")
                        .font(.headline.weight(.black))
                        .frame(minWidth: 126)
                }
                .buttonStyle(.borderedProminent)
                .tint(.red)
                .disabled(viewModel.status != .connected)
            }
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
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

private struct WheelSpeedLane: View {
    let name: String
    let color: Color
    let values: [Float]
    let target: Float

    private var peak: Float {
        let finiteValues = values.filter(\.isFinite)
        let largest = max(finiteValues.map { abs($0) }.max() ?? 0.0, abs(target))
        return max(50.0, largest * 1.25)
    }

    var body: some View {
        HStack(spacing: 8) {
            Text(name)
                .font(.caption2.weight(.bold).monospaced())
                .foregroundStyle(color)
                .frame(width: 58, alignment: .leading)
            ZStack {
                SpeedLaneGrid()
                    .stroke(.secondary.opacity(0.22), lineWidth: 0.7)
                SpeedLaneLine(values: values, peak: peak)
                    .stroke(color, style: StrokeStyle(lineWidth: 2.0, lineCap: .round, lineJoin: .round))
                SpeedTargetGuide(target: target, peak: peak)
                    .stroke(.white.opacity(0.55), style: StrokeStyle(lineWidth: 1.0, dash: [5, 4]))
            }
            .frame(maxWidth: .infinity)
            .background(.black.opacity(0.08), in: RoundedRectangle(cornerRadius: 4))
            VStack {
                Text("+\(Int(peak))")
                Spacer(minLength: 0)
                Text("-\(Int(peak))")
            }
            .font(.caption2.monospaced())
            .foregroundStyle(.secondary)
            .frame(width: 48, alignment: .trailing)
        }
    }
}

private struct SpeedLaneGrid: Shape {
    func path(in rect: CGRect) -> Path {
        var path = Path()
        for fraction in [0.25, 0.5, 0.75] {
            let y = rect.minY + rect.height * fraction
            path.move(to: CGPoint(x: rect.minX, y: y))
            path.addLine(to: CGPoint(x: rect.maxX, y: y))
        }
        return path
    }
}

private struct SpeedLaneLine: Shape {
    let values: [Float]
    let peak: Float

    func path(in rect: CGRect) -> Path {
        guard values.count > 1, peak > 0 else { return Path() }
        let step = rect.width / CGFloat(values.count - 1)
        var path = Path()
        var hasPoint = false
        for (index, value) in values.enumerated() where value.isFinite {
            let x = CGFloat(index) * step
            let normalized = max(-1.0, min(1.0, value / peak))
            let y = rect.midY - CGFloat(normalized) * rect.height * 0.44
            if hasPoint { path.addLine(to: CGPoint(x: x, y: y)) }
            else { path.move(to: CGPoint(x: x, y: y)); hasPoint = true }
        }
        return path
    }
}

private struct SpeedTargetGuide: Shape {
    let target: Float
    let peak: Float

    func path(in rect: CGRect) -> Path {
        guard target.isFinite, peak > 0 else { return Path() }
        let normalized = max(-1.0, min(1.0, target / peak))
        let y = rect.midY - CGFloat(normalized) * rect.height * 0.44
        var path = Path()
        path.move(to: CGPoint(x: rect.minX, y: y))
        path.addLine(to: CGPoint(x: rect.maxX, y: y))
        return path
    }
}
