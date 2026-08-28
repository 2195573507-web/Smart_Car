import Combine
import Foundation

enum IMUCalibrationDisplayStatus: String, Equatable {
    case waitCal = "WAIT_CAL"
    case calibrating = "CALIBRATING"
    case ready = "READY"
    case error = "ERROR"
}

@MainActor
final class CalibrationViewModel: ObservableObject {
    @Published private(set) var status = IMUCalibrationStatus(state: .idle,
                                                               sampleMode: .static,
                                                               totalProgress: 0,
                                                               currentPWM: 0,
                                                               sampleProgress: 0,
                                                               errorCode: 0,
                                                               lastUpdatedAt: nil)
    @Published private(set) var bias: IMUCalibrationBias?
    @Published private(set) var calibrationStartedAt: Date?

    var attitudeStatus: IMUCalibrationDisplayStatus {
        switch status.state {
        case .idle: return .waitCal
        case .complete: return .ready
        case .error: return .error
        case .setPWM, .waitStable, .sample: return .calibrating
        }
    }

    private var statusCancellable: AnyCancellable?
    private var biasCancellable: AnyCancellable?

    init(telemetryStore: TelemetryStore) {
        statusCancellable = telemetryStore.calibration.$snapshot
            .removeDuplicates()
            .sink { [weak self] snapshot in
                guard let self else { return }
                guard snapshot.availability == .current else {
                    self.status = IMUCalibrationStatus(state: .idle,
                                                       sampleMode: .static,
                                                       totalProgress: 0,
                                                       currentPWM: 0,
                                                       sampleProgress: 0,
                                                       errorCode: 0,
                                                       lastUpdatedAt: nil)
                    self.bias = nil
                    self.calibrationStartedAt = nil
                    return
                }

                let status = snapshot.status
                let previousState = self.status.state
                self.status = status

                switch status.state {
                case .idle:
                    if previousState == .complete || previousState == .error {
                        self.calibrationStartedAt = nil
                    } else if self.calibrationStartedAt == nil {
                        self.calibrationStartedAt = status.lastUpdatedAt ?? Date()
                    }
                case .setPWM, .waitStable, .sample:
                    if self.calibrationStartedAt == nil ||
                        previousState == .complete || previousState == .error {
                        self.calibrationStartedAt = status.lastUpdatedAt ?? Date()
                    }
                default:
                    break
                }
            }
        biasCancellable = telemetryStore.calibration.$snapshot
            .map(\.bias)
            .removeDuplicates()
            .sink { [weak self] bias in
                self?.bias = bias
            }
    }

    func elapsed(at date: Date = Date()) -> TimeInterval {
        guard let calibrationStartedAt else { return 0 }
        return max(0, date.timeIntervalSince(calibrationStartedAt))
    }
}
