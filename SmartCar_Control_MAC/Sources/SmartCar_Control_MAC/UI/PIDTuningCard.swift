import SwiftUI

struct PIDTuningCard: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @ObservedObject var tuning: PIDTuningState

    var body: some View {
        DisclosureGroup(isExpanded: $tuning.isExpanded) {
            VStack(alignment: .leading, spacing: 12) {
                PIDInputRow(
                    title: "Kp",
                    text: $tuning.kpText,
                    rangeText: "0.00-4.00 - step 0.05",
                    value: Double(tuning.kpText) ?? 1.10,
                    bounds: PIDTuningState.kpRange,
                    step: PIDTuningState.kpStep,
                    setValue: tuning.setKp
                )
                PIDInputRow(
                    title: "Ki",
                    text: $tuning.kiText,
                    rangeText: "0.000-0.300 - step 0.005",
                    value: Double(tuning.kiText) ?? 0.08,
                    bounds: PIDTuningState.kiRange,
                    step: PIDTuningState.kiStep,
                    setValue: tuning.setKi
                )
                PIDInputRow(
                    title: "Kd",
                    text: $tuning.kdText,
                    rangeText: "0.000-0.100 - step 0.002",
                    value: Double(tuning.kdText) ?? 0.0,
                    bounds: PIDTuningState.kdRange,
                    step: PIDTuningState.kdStep,
                    setValue: tuning.setKd
                )
                PIDInputRow(
                    title: "Accel",
                    text: $tuning.accelText,
                    rangeText: "200-2000 mm/s^2 - step 50",
                    value: Double(tuning.accelText) ?? 800.0,
                    bounds: PIDTuningState.accelRange,
                    step: PIDTuningState.accelStep,
                    setValue: tuning.setAccel
                )

                if let validationMessage = tuning.validationMessage {
                    Text(validationMessage)
                        .font(.caption)
                        .foregroundStyle(.red)
                }

                HStack(spacing: 8) {
                    Button(action: viewModel.applyPIDParameters) {
                        Label("Apply", systemImage: "arrow.up.circle.fill")
                            .frame(maxWidth: .infinity)
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(viewModel.status != .connected || tuning.values == nil)

                    Button(action: tuning.restoreDefaults) {
                        Label("Defaults", systemImage: "arrow.counterclockwise")
                            .frame(maxWidth: .infinity)
                    }
                    .buttonStyle(.bordered)
                }

                Text(tuning.applyStatus.title)
                    .font(.caption2.monospaced())
                    .foregroundStyle(statusColor)
            }
            .padding(.top, 10)
        } label: {
            HStack {
                Label("PID TUNING", systemImage: "slider.horizontal.3")
                    .font(.headline.monospaced())
                Spacer()
                Text("ALL WHEELS")
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
            }
        }
        .padding(16)
        .background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private var statusColor: Color {
        switch tuning.applyStatus {
        case .applied: return .green
        case .rejected, .invalid: return .red
        case .sending: return .orange
        default: return .secondary
        }
    }
}

private struct PIDInputRow: View {
    let title: String
    @Binding var text: String
    let rangeText: String
    let value: Double
    let bounds: ClosedRange<Double>
    let step: Double
    let setValue: (Double) -> Void

    var body: some View {
        HStack(spacing: 8) {
            Text(title)
                .font(.caption.weight(.bold).monospaced())
                .frame(width: 48, alignment: .leading)
            TextField(title, text: $text)
                .textFieldStyle(.roundedBorder)
                .multilineTextAlignment(.trailing)
                .font(.body.monospacedDigit())
                .frame(width: 92)
                .onSubmit { }
            Stepper("", value: Binding(
                get: { value },
                set: { setValue(min(bounds.upperBound, max(bounds.lowerBound, $0))) }
            ), in: bounds, step: step)
            .labelsHidden()
            Text(rangeText)
                .font(.caption2.monospaced())
                .foregroundStyle(.secondary)
                .lineLimit(1)
                .frame(maxWidth: .infinity, alignment: .leading)
        }
    }
}
