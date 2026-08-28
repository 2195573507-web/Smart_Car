import Foundation
#if canImport(UIKit)
import UIKit
#endif

enum Haptics {
    static func impact(_ style: ImpactStyle = .light) {
#if canImport(UIKit)
        let generator = UIImpactFeedbackGenerator(style: style.uiStyle)
        generator.prepare()
        generator.impactOccurred()
#endif
    }

    enum ImpactStyle {
        case light
        case medium
        case heavy

#if canImport(UIKit)
        var uiStyle: UIImpactFeedbackGenerator.FeedbackStyle {
            switch self {
            case .light: return .light
            case .medium: return .medium
            case .heavy: return .heavy
            }
        }
#endif
    }
}
