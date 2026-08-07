import Foundation
import SwiftUI

public struct IMUCalibrationView: View {
    let calibration: IMUCalibrationStatus

    public init(calibration: IMUCalibrationStatus) {
        self.calibration = calibration
    }

    public var body: some View {
        TimelineView(.periodic(from: .now, by: 1)) { context in
            if calibration.isWaitingForSTM(at: context.date) {
                VStack(spacing: 12) {
                    Image(systemName: "antenna.radiowaves.left.and.right")
                        .font(.title2)
                        .foregroundStyle(.secondary)
                    Text("Waiting STM response")
                        .foregroundStyle(.secondary)
                }
                .frame(maxWidth: .infinity, minHeight: 180)
            } else {
                Form {
                    Section("Current Stage") {
                        Label(calibration.displayName, systemImage: stageIcon)
                        if calibration.isComplete {
                            Label("IMU Calibration Complete", systemImage: "checkmark.circle.fill")
                                .foregroundStyle(.green)
                        }
                    }

                    Section("Overall Progress") {
                        ProgressView(value: Double(calibration.totalProgress), total: 100)
                        Text("\(calibration.totalProgress)%")
                            .monospacedDigit()
                    }

                    Section("PWM and Sampling") {
                        LabeledContent("Current PWM") {
                            Text("\(calibration.currentPWM)%")
                                .monospacedDigit()
                        }
                        if calibration.state == .sample {
                            LabeledContent("Sample") {
                                Text("\(calibration.sampleProgress)")
                                    .monospacedDigit()
                            }
                        }
                    }

                    if let errorMessage = calibration.errorMessage {
                        Section("Error") {
                            Label(errorMessage, systemImage: "exclamationmark.triangle.fill")
                                .foregroundStyle(.red)
                        }
                    }
                }
            }
        }
        .navigationTitle("IMU Calibration")
    }

    private var stageIcon: String {
        switch calibration.state {
        case .idle: return "circle"
        case .setPWM: return "slider.horizontal.3"
        case .waitStable: return "hourglass"
        case .sample: return "waveform.path.ecg"
        case .complete: return "checkmark.circle.fill"
        case .error: return "exclamationmark.triangle.fill"
        }
    }
}
