import Foundation

public enum IMUCalibrationState: UInt8, Equatable, Sendable {
    case idle = 0
    case setPWM = 1
    case waitStable = 2
    case sample = 3
    case complete = 4
    case error = 5

    public var displayName: String {
        switch self {
        case .idle: return "Idle"
        case .setPWM: return "Setting PWM"
        case .waitStable: return "PWM Stabilizing"
        case .sample: return "Sampling"
        case .complete: return "Complete"
        case .error: return "Error"
        }
    }
}

public enum IMUCalibrationSampleMode: UInt8, Equatable, Sendable {
    case `static` = 0
    case vibration = 1
}

public struct IMUCalibrationStatus: Equatable, Sendable {
    public var state: IMUCalibrationState = .idle
    public var sampleMode: IMUCalibrationSampleMode = .static
    public var totalProgress: UInt8 = 0
    public var currentPWM: UInt8 = 0
    public var sampleProgress: UInt16 = 0
    public var errorCode: UInt8 = 0
    public var lastUpdatedAt: Date?

    public init(state: IMUCalibrationState = .idle,
                sampleMode: IMUCalibrationSampleMode = .static,
                totalProgress: UInt8 = 0,
                currentPWM: UInt8 = 0,
                sampleProgress: UInt16 = 0,
                errorCode: UInt8 = 0,
                lastUpdatedAt: Date? = nil) {
        self.state = state
        self.sampleMode = sampleMode
        self.totalProgress = totalProgress
        self.currentPWM = currentPWM
        self.sampleProgress = sampleProgress
        self.errorCode = errorCode
        self.lastUpdatedAt = lastUpdatedAt
    }

    public var isComplete: Bool { state == .complete }

    public var displayName: String {
        if state == .sample {
            return sampleMode == .static ? "Static Sampling" : "Vibration Sampling"
        }
        return state.displayName
    }

    public var errorMessage: String? {
        guard state == .error else { return nil }
        switch errorCode {
        case 1: return "PWM apply failed"
        case 2: return "PWM apply confirmation timeout"
        default: return "Calibration failed"
        }
    }

    public func isWaitingForSTM(at date: Date = Date()) -> Bool {
        guard let lastUpdatedAt else { return true }
        return date.timeIntervalSince(lastUpdatedAt) > 3.0
    }
}
