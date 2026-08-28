import Foundation

struct PIDParameterValues: Equatable {
    let kp: Float
    let ki: Float
    let kd: Float
    let maxAccel: Float
}

enum PIDApplyStatus: Equatable {
    case idle
    case sending
    case applied
    case rejected
    case unavailable
    case invalid(String)

    var title: String {
        switch self {
        case .idle: return "NOT APPLIED"
        case .sending: return "WAITING FOR STM"
        case .applied: return "APPLIED"
        case .rejected: return "REJECTED"
        case .unavailable: return "NOT CONNECTED"
        case .invalid(let message): return message
        }
    }
}

@MainActor
final class PIDTuningState: ObservableObject {
    static let kpRange = 0.0...4.0
    static let kiRange = 0.0...0.3
    static let kdRange = 0.0...0.1
    static let accelRange = 200.0...2000.0
    static let kpStep = 0.05
    static let kiStep = 0.005
    static let kdStep = 0.002
    static let accelStep = 50.0

    @Published var kpText = "1.10"
    @Published var kiText = "0.080"
    @Published var kdText = "0.000"
    @Published var accelText = "800"
    @Published var isExpanded = true
    @Published var applyStatus: PIDApplyStatus = .idle

    var values: PIDParameterValues? {
        guard let kp = Float(kpText), let ki = Float(kiText),
              let kd = Float(kdText), let maxAccel = Float(accelText),
              kp.isFinite, ki.isFinite, kd.isFinite, maxAccel.isFinite,
              Self.kpRange.contains(Double(kp)),
              Self.kiRange.contains(Double(ki)),
              Self.kdRange.contains(Double(kd)),
              Self.accelRange.contains(Double(maxAccel)),
              isAligned(Double(kp), to: Self.kpStep, in: Self.kpRange),
              isAligned(Double(ki), to: Self.kiStep, in: Self.kiRange),
              isAligned(Double(kd), to: Self.kdStep, in: Self.kdRange),
              isAligned(Double(maxAccel), to: Self.accelStep, in: Self.accelRange) else {
            return nil
        }
        return PIDParameterValues(kp: kp, ki: ki, kd: kd, maxAccel: maxAccel)
    }

    var validationMessage: String? {
        guard values == nil else { return nil }
        return "Enter values within the allowed range and step."
    }

    func restoreDefaults() {
        kpText = "1.10"
        kiText = "0.080"
        kdText = "0.000"
        accelText = "800"
        applyStatus = .idle
    }

    func setKp(_ value: Double) { kpText = format(value, decimals: 2) }
    func setKi(_ value: Double) { kiText = format(value, decimals: 3) }
    func setKd(_ value: Double) { kdText = format(value, decimals: 3) }
    func setAccel(_ value: Double) { accelText = format(value, decimals: 0) }

    func normalizeValidFields() {
        guard let values else { return }
        kpText = format(Double(values.kp), decimals: 2)
        kiText = format(Double(values.ki), decimals: 3)
        kdText = format(Double(values.kd), decimals: 3)
        accelText = format(Double(values.maxAccel), decimals: 0)
    }

    private func isAligned(_ value: Double, to step: Double, in range: ClosedRange<Double>) -> Bool {
        let scaled = (value - range.lowerBound) / step
        return abs(scaled.rounded() - scaled) < 0.0001
    }

    private func format(_ value: Double, decimals: Int) -> String {
        String(format: "%.*f", decimals, value)
    }
}
