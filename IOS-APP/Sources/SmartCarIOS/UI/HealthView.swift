import SwiftUI

struct HealthView: View {
    @ObservedObject var viewModel: SmartCarViewModel
    private var telemetry: TelemetryStore { viewModel.telemetryStore }

    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                Surface {
                    VStack(alignment: .leading, spacing: 11) {
                        SectionTitle(title: "LINK HEALTH", systemImage: "antenna.radiowaves.left.and.right", trailing: viewModel.status.displayText)
                        HStack(spacing: 10) {
                            MetricTile(label: "RSSI", value: viewModel.discoveredRSSI.map { "\($0) dBm" } ?? "--", tint: .teal)
                            MetricTile(label: "TX FRAMES", value: "\(viewModel.transmittedFrameCount)", tint: .blue)
                            MetricTile(label: "RX FRAMES", value: "\(viewModel.receivedFrameCount)", tint: .green)
                        }
                        HStack(spacing: 10) {
                            MetricTile(label: "DECODE ERR", value: "\(viewModel.decodeFailureCount)", tint: viewModel.decodeFailureCount == 0 ? .green : .orange)
                            MetricTile(label: "DROPPED", value: "\(viewModel.droppedMessageCount)", tint: viewModel.droppedMessageCount == 0 ? .green : .orange)
                            MetricTile(label: "LAST TYPE", value: viewModel.lastPacketType)
                        }
                        if let lastPacket = viewModel.lastPacketReceivedAt {
                            Text("LAST RX \(lastPacket.formatted(date: .omitted, time: .standard))")
                                .font(.caption2.monospaced())
                                .foregroundStyle(.secondary)
                        }
                    }
                }

                Surface {
                    VStack(alignment: .leading, spacing: 11) {
                        SectionTitle(title: "RUNTIME STATE", systemImage: "cpu", trailing: "APP-BLE")
                        HealthRow(label: "SmartCar_S3", value: telemetry.status.snapshot.smartCarS3Status, tint: statusTint(telemetry.status.snapshot.smartCarS3Status))
                        HealthRow(label: "Wheel status 0x210", value: fresh(telemetry.wheelSpeed.snapshot.lastUpdatedAt) ? "LIVE" : "STALE", tint: fresh(telemetry.wheelSpeed.snapshot.lastUpdatedAt) ? .green : .orange)
                        HealthRow(label: "Chassis state 0x211", value: fresh(telemetry.chassis.snapshot.lastUpdatedAt) ? "LIVE" : "STALE", tint: fresh(telemetry.chassis.snapshot.lastUpdatedAt) ? .green : .orange)
                        HealthRow(label: "Dual AHRS", value: telemetry.dualAttitude.snapshot.displayStatus.label, tint: telemetry.dualAttitude.snapshot.displayStatus == .valid ? .green : .orange)
                    }
                }

                Surface {
                    VStack(alignment: .leading, spacing: 10) {
                        SectionTitle(title: "RTOS TASKS", systemImage: "list.bullet.rectangle", trailing: "TELEMETRY GAP")
                        Text("当前 App-BLE 活跃 0x29 / 0x2C payload 不包含任务栈深度字段；此处保留诊断槽位并明确标记，避免伪造 RTOS 数值。")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        HealthRow(label: "Task health", value: "NOT PRESENT IN ACTIVE SCHEMA", tint: .orange)
                        HealthRow(label: "Stack depth", value: "NOT PRESENT IN ACTIVE SCHEMA", tint: .orange)
                    }
                }

                Surface {
                    VStack(alignment: .leading, spacing: 10) {
                        SectionTitle(title: "SAFETY", systemImage: "shield.lefthalf.filled", trailing: "LIVE")
                        if let chassis = telemetry.chassis.snapshot.state {
                            HealthRow(label: "Safety fuse", value: chassis.safetyFused ? "FUSED" : "SAFE", tint: chassis.safetyFused ? .red : .green)
                            HealthRow(label: "Attitude ready", value: chassis.attitudeReady ? "READY" : "WAIT", tint: chassis.attitudeReady ? .green : .orange)
                            HealthRow(label: "Odometry", value: chassis.odometryValid ? "VALID" : "WAIT", tint: chassis.odometryValid ? .green : .orange)
                        } else {
                            HealthRow(label: "Chassis safety", value: "WAITING FOR 0x29", tint: .secondary)
                        }
                    }
                }
            }
            .frame(maxWidth: .infinity)
            .padding(.horizontal, 14)
            .padding(.top, 14)
            .padding(.bottom, 80)
        }
        .scrollIndicators(.hidden)
        .navigationTitle("健康诊断")
    }

    private func fresh(_ date: Date?) -> Bool {
        guard let date else { return false }
        return Date().timeIntervalSince(date) < 3
    }

    private func statusTint(_ value: String) -> Color {
        value == "ONLINE" ? .green : value == "STALE" ? .orange : .secondary
    }
}

private struct HealthRow: View {
    let label: String
    let value: String
    let tint: Color

    var body: some View {
        HStack {
            Text(label).font(.caption)
            Spacer()
            Text(value)
                .font(.caption2.monospaced().weight(.semibold))
                .foregroundStyle(tint)
                .multilineTextAlignment(.trailing)
        }
    }
}
