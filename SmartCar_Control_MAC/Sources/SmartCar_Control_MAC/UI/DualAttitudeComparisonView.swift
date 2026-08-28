import Foundation
import SwiftUI

struct DualAttitudeComparisonView: View {
    @ObservedObject var store: DualAttitudeState
    let angleUnit: AngleUnit

    var body: some View {
        let snapshot = store.snapshot
        let data = snapshot.data

        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text("DUAL_ATTITUDE")
                    .font(.headline.monospaced())
                Spacer()
                Text(statusText(snapshot.displayStatus))
                    .font(.caption.monospaced().weight(.bold))
                    .foregroundStyle(statusColor(snapshot.displayStatus))
            }

            ViewThatFits(in: .horizontal) {
                HStack(alignment: .top, spacing: 12) {
                    DualPoseCard(
                        title: "PRIMARY AHRS",
                        accent: .blue,
                        pose: data?.primary,
                        statusLabel: "9-DOF FUSED",
                        sourceDetail: "BMI323 · high dynamic",
                        angleUnit: angleUnit,
                        showsQuaternion: true
                    )
                    DualPoseCard(
                        title: "REDUNDANT AHRS",
                        accent: .orange,
                        pose: data?.redundant,
                        statusLabel: "6-DOF DAMPED",
                        sourceDetail: "LSM303 · static reference",
                        angleUnit: angleUnit,
                        showsQuaternion: false
                    )
                }
                VStack(spacing: 12) {
                    DualPoseCard(
                        title: "PRIMARY AHRS",
                        accent: .blue,
                        pose: data?.primary,
                        statusLabel: "9-DOF FUSED",
                        sourceDetail: "BMI323 · high dynamic",
                        angleUnit: angleUnit,
                        showsQuaternion: true
                    )
                    DualPoseCard(
                        title: "REDUNDANT AHRS",
                        accent: .orange,
                        pose: data?.redundant,
                        statusLabel: "6-DOF DAMPED",
                        sourceDetail: "LSM303 · static reference",
                        angleUnit: angleUnit,
                        showsQuaternion: false
                    )
                }
            }

            AttitudeDivergencePanel(data: data, angleUnit: angleUnit)
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private func statusText(_ status: AttitudeDisplayStatus) -> String {
        switch status {
        case .valid: return "TRACKING"
        case .invalid: return "WAIT_CAL"
        case .timeout: return "STALE"
        }
    }

    private func statusColor(_ status: AttitudeDisplayStatus) -> Color {
        switch status {
        case .valid: return .green
        case .invalid: return .orange
        case .timeout: return .secondary
        }
    }
}

private struct DualPoseCard: View {
    let title: String
    let accent: Color
    let pose: DualAttitudePose?
    let statusLabel: String
    let sourceDetail: String
    let angleUnit: AngleUnit
    let showsQuaternion: Bool

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text(title)
                    .font(.caption.monospaced().weight(.bold))
                    .foregroundStyle(accent)
                Spacer()
                Text(pose?.valid == true ? "VALID" : "WAIT_CAL")
                    .font(.caption2.monospaced().weight(.bold))
                    .foregroundStyle(pose?.valid == true ? .green : .secondary)
            }

            AttitudeModelView(pose: pose, accent: accent)
                .frame(maxWidth: .infinity)
                .frame(height: 102)

            HStack(spacing: 8) {
                AngleValue(label: "ROLL", value: pose?.rollRad, unit: angleUnit)
                AngleValue(label: "PITCH", value: pose?.pitchRad, unit: angleUnit)
                AngleValue(label: "YAW", value: pose?.yawRad, unit: angleUnit)
            }

            Text(statusLabel)
                .font(.caption2.monospaced().weight(.semibold))
                .foregroundStyle(accent)
            Text(sourceDetail)
                .font(.caption2.monospaced())
                .foregroundStyle(.secondary)

            if showsQuaternion {
                QuaternionReadout(quaternion: pose?.quaternion)
            }
        }
        .padding(12)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(accent.opacity(0.08), in: RoundedRectangle(cornerRadius: 7))
        .overlay {
            RoundedRectangle(cornerRadius: 7)
                .stroke(accent.opacity(0.4), lineWidth: 1)
        }
    }
}

private struct AttitudeModelView: View {
    let pose: DualAttitudePose?
    let accent: Color

    var body: some View {
        ZStack {
            Circle()
                .stroke(accent.opacity(0.28), lineWidth: 1)
            RoundedRectangle(cornerRadius: 6)
                .fill(accent.opacity(pose?.valid == true ? 0.72 : 0.18))
                .overlay {
                    RoundedRectangle(cornerRadius: 6)
                        .stroke(accent, lineWidth: 1.5)
                }
                .frame(width: 118, height: 66)
                .overlay {
                    Rectangle()
                        .fill(.primary.opacity(0.45))
                        .frame(width: 74, height: 1)
                }
                .rotation3DEffect(
                    .radians(Double(pose?.pitchRad ?? 0)),
                    axis: (x: 1, y: 0, z: 0)
                )
                .rotation3DEffect(
                    .radians(Double(pose?.rollRad ?? 0)),
                    axis: (x: 0, y: 1, z: 0)
                )
                .rotationEffect(.radians(Double(pose?.yawRad ?? 0)))
                .animation(.easeOut(duration: 0.12), value: pose?.rollRad ?? 0)
                .animation(.easeOut(duration: 0.12), value: pose?.pitchRad ?? 0)
                .animation(.easeOut(duration: 0.12), value: pose?.yawRad ?? 0)
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("\(pose?.valid == true ? "valid" : "waiting") attitude model")
    }
}

private struct AngleValue: View {
    let label: String
    let value: Float?
    let unit: AngleUnit

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(label)
                .font(.caption2.monospaced().weight(.bold))
                .foregroundStyle(.secondary)
            Text(value.map {
                let converted = unit == .degree ? $0 * 180 / Float.pi : $0
                return unit.format(converted, precision: 1)
            } ?? "--")
                .font(.caption.monospaced())
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct QuaternionReadout: View {
    let quaternion: Quaternion?

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text("QUATERNION")
                .font(.caption2.monospaced().weight(.bold))
                .foregroundStyle(.secondary)
            Text(quaternion.map {
                String(format: "q0 %.3f  q1 %.3f", $0.q0, $0.q1)
            } ?? "q0 --  q1 --")
            Text(quaternion.map {
                String(format: "q2 %.3f  q3 %.3f", $0.q2, $0.q3)
            } ?? "q2 --  q3 --")
        }
        .font(.caption2.monospaced())
        .foregroundStyle(.secondary)
    }
}

private struct AttitudeDivergencePanel: View {
    let data: DualAttitude?
    let angleUnit: AngleUnit

    private let normalLimitDegrees: Float = 3
    private let warningLimitDegrees: Float = 6

    var body: some View {
        let isValid = data?.primaryValid == true && data?.redundantValid == true
        let roll = degrees(data?.deltaRad.x)
        let pitch = degrees(data?.deltaRad.y)
        let yaw = degrees(data?.deltaRad.z)
        let maximum = max(roll ?? 0, max(pitch ?? 0, yaw ?? 0))

        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text("ATTITUDE DIVERGENCE")
                    .font(.caption.monospaced().weight(.bold))
                Spacer()
                Text(isValid ? severityText(maximum) : "WAIT_CAL")
                    .font(.caption2.monospaced().weight(.bold))
                    .foregroundStyle(isValid ? severityColor(maximum) : .secondary)
            }
            HStack(spacing: 8) {
                DivergenceValue(label: "ΔROLL", value: roll, color: isValid ? severityColor(roll ?? 0) : .secondary, unit: angleUnit)
                DivergenceValue(label: "ΔPITCH", value: pitch, color: isValid ? severityColor(pitch ?? 0) : .secondary, unit: angleUnit)
                DivergenceValue(label: "ΔYAW", value: yaw, color: isValid ? severityColor(yaw ?? 0) : .secondary, unit: angleUnit)
            }
        }
        .padding(12)
        .background(.quaternary.opacity(0.35), in: RoundedRectangle(cornerRadius: 7))
    }

    private func degrees(_ value: Float?) -> Float? {
        guard let value, value.isFinite else { return nil }
        return abs(value * 180 / Float.pi)
    }

    private func severityColor(_ degrees: Float) -> Color {
        if degrees < normalLimitDegrees { return .green }
        if degrees <= warningLimitDegrees { return .yellow }
        return .red
    }

    private func severityText(_ degrees: Float) -> String {
        if degrees < normalLimitDegrees { return "WITHIN LIMIT" }
        if degrees <= warningLimitDegrees { return "WARNING" }
        return "DIVERGENCE"
    }
}

private struct DivergenceValue: View {
    let label: String
    let value: Float?
    let color: Color
    let unit: AngleUnit

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(label)
                .font(.caption2.monospaced().weight(.bold))
                .foregroundStyle(.secondary)
            Text(value.map {
                let converted = unit == .degree ? $0 : $0 * Float.pi / 180
                return unit.format(converted, precision: 1)
            } ?? "--")
                .font(.caption.monospaced().weight(.semibold))
                .foregroundStyle(color)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}
