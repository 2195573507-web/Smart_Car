import SwiftUI

struct TelemetryView: View {
    @ObservedObject var viewModel: SmartCarViewModel
    private var telemetry: TelemetryStore { viewModel.telemetryStore }

    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                PosePanel(
                    attitude: telemetry.attitude.snapshot.data,
                    targetYawDegrees: telemetry.chassis.snapshot.state?.yawDeg
                )

                Surface {
                    VStack(alignment: .leading, spacing: 12) {
                        SectionTitle(title: "POWER", systemImage: "battery.75", trailing: batteryText)
                        HStack(spacing: 12) {
                            MetricTile(label: "VOLTAGE", value: telemetry.wheelSpeed.snapshot.voltage.map { String(format: "%.2f V", $0) } ?? "--", tint: .green)
                            MetricTile(label: "BATTERY", value: "\(telemetry.status.snapshot.battery)%", tint: batteryTint)
                            MetricTile(label: "MOTOR", value: String(format: "0x%02X", telemetry.status.snapshot.motorState))
                        }
                        ProgressView(value: Double(telemetry.status.snapshot.battery), total: 100)
                            .tint(batteryTint)
                    }
                }

                DualAHRSCard(snapshot: telemetry.dualAttitude.snapshot)
                IMUCard(snapshot: telemetry.imu.snapshot)
                CalibrationCard(snapshot: telemetry.calibration.snapshot)
                ChassisCard(snapshot: telemetry.chassis.snapshot)
            }
            .frame(maxWidth: .infinity)
            .padding(.horizontal, 14)
            .padding(.top, 14)
            .padding(.bottom, 80)
        }
        .scrollIndicators(.hidden)
        .navigationTitle("姿态 / 遥测")
    }

    private var batteryText: String { "\(telemetry.status.snapshot.battery)%" }
    private var batteryTint: Color { telemetry.status.snapshot.battery < 20 ? .red : .green }
}

private struct DualAHRSCard: View {
    let snapshot: DualAttitudeStateSnapshot

    var body: some View {
        Surface {
            VStack(alignment: .leading, spacing: 10) {
                SectionTitle(title: "DUAL AHRS", systemImage: "arrow.triangle.2.circlepath", trailing: snapshot.displayStatus.label)
                if let data = snapshot.data {
                    HStack(spacing: 10) {
                        MetricTile(label: "ΔROLL", value: degreeText(data.deltaRad.x), tint: deltaTint(data.deltaRad.x))
                        MetricTile(label: "ΔPITCH", value: degreeText(data.deltaRad.y), tint: deltaTint(data.deltaRad.y))
                        MetricTile(label: "ΔYAW", value: degreeText(data.deltaRad.z), tint: deltaTint(data.deltaRad.z))
                    }
                    Divider()
                    HStack {
                        Text("PRIMARY")
                        Spacer()
                        Text("R \(degreeText(data.primary.rollRad))  P \(degreeText(data.primary.pitchRad))  Y \(degreeText(data.primary.yawRad))")
                    }
                    HStack {
                        Text("REDUNDANT")
                        Spacer()
                        Text("R \(degreeText(data.redundant.rollRad))  P \(degreeText(data.redundant.pitchRad))  Y \(degreeText(data.redundant.yawRad))")
                    }
                    .foregroundStyle(.secondary)
                    .font(.caption.monospaced())
                } else {
                    Text("Waiting for 0x28 dual-AHRS telemetry")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        }
    }

    private func deltaTint(_ value: Float) -> Color {
        abs(value) > 0.15 ? .orange : .green
    }
}

private struct IMUCard: View {
    let snapshot: IMUStateSnapshot

    var body: some View {
        Surface {
            VStack(alignment: .leading, spacing: 10) {
                SectionTitle(title: "IMU SENSORS", systemImage: "gyroscope", trailing: "0x27")
                HStack(spacing: 10) {
                    SensorTile(name: "BMI323", online: snapshot.model.bmi323.online, detail: "Primary")
                    SensorTile(name: "LSM303", online: snapshot.model.lsm303.online, detail: "Redundant")
                }
            }
        }
    }
}

private struct SensorTile: View {
    let name: String
    let online: Bool
    let detail: String

    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            HStack {
                Text(name).font(.caption.weight(.bold))
                Spacer()
                Circle().fill(online ? .green : .secondary).frame(width: 7, height: 7)
            }
            Text(detail).font(.caption2).foregroundStyle(.secondary)
            Text(online ? "ONLINE" : "OFFLINE").font(.caption2.monospaced()).foregroundStyle(online ? .green : .secondary)
        }
        .padding(10)
        .background(.quaternary.opacity(0.45), in: RoundedRectangle(cornerRadius: 8))
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct CalibrationCard: View {
    let snapshot: CalibrationStateSnapshot

    var body: some View {
        Surface {
            VStack(alignment: .leading, spacing: 10) {
                SectionTitle(title: "STATIC CALIBRATION", systemImage: "scope", trailing: snapshot.status.stage.displayName)
                HStack {
                    Text(statusText)
                        .font(.caption.weight(.semibold))
                        .foregroundStyle(statusTint)
                    Spacer()
                    Text("\(snapshot.status.totalProgress)%")
                        .font(.caption.monospaced())
                }
                ProgressView(value: Double(snapshot.status.totalProgress), total: 100)
                    .tint(statusTint)
                Text("SAMPLE \(snapshot.status.sampleCount) / \(snapshot.status.totalSample == 0 ? "--" : String(snapshot.status.totalSample)) · PWM \(snapshot.status.currentPWM)%")
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
            }
        }
    }

    private var statusText: String {
        switch snapshot.status.state {
        case .idle: return "WAIT_CAL"
        case .setPWM, .waitStable, .sample: return "CALIBRATING"
        case .complete: return "READY"
        case .error: return "ERROR"
        }
    }

    private var statusTint: Color {
        switch snapshot.status.state {
        case .complete: return .green
        case .error: return .red
        case .idle: return .secondary
        case .setPWM, .waitStable, .sample: return .orange
        }
    }
}

private struct ChassisCard: View {
    let snapshot: ChassisTelemetrySnapshot

    var body: some View {
        Surface {
            VStack(alignment: .leading, spacing: 9) {
                SectionTitle(title: "CHASSIS STATE", systemImage: "location.north.line", trailing: fresh ? "LIVE" : "STALE")
                if let state = snapshot.state {
                    HStack(spacing: 7) {
                        StatusPill(title: state.safetyFused ? "FUSED" : "SAFE", tint: state.safetyFused ? .red : .green)
                        StatusPill(title: state.headingLocked ? "HEADING LOCK" : "FREE YAW", tint: state.headingLocked ? .blue : .secondary)
                        StatusPill(title: state.odometryValid ? "ODOM OK" : "ODOM WAIT", tint: state.odometryValid ? .green : .orange)
                    }
                    Text(String(format: "X %.0f mm · Y %.0f mm · YAW %.1f° · DIST %.3f m", state.xMm, state.yMm, state.yawDeg, state.totalDistanceM))
                        .font(.caption.monospaced())
                } else {
                    Text("Waiting for 0x29 chassis telemetry")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        }
    }

    private var fresh: Bool {
        guard let timestamp = snapshot.lastUpdatedAt else { return false }
        return Date().timeIntervalSince(timestamp) < 3
    }
}
