import Combine
import Foundation

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

    private var statusCancellable: AnyCancellable?
    private var biasCancellable: AnyCancellable?

    init(telemetryStore: TelemetryStore) {
        statusCancellable = telemetryStore.calibration.$snapshot
            .map(\.status)
            .removeDuplicates()
            .sink { [weak self] status in
                self?.status = status
            }
        biasCancellable = telemetryStore.calibration.$snapshot
            .map(\.bias)
            .removeDuplicates()
            .sink { [weak self] bias in
                self?.bias = bias
            }
    }
}
