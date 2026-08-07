import Foundation
import SwiftUI

enum AppLanguage: String, CaseIterable, Identifiable {
    case english = "en"
    case simplifiedChinese = "zh-Hans"

    var id: String { rawValue }

    var locale: Locale { Locale(identifier: rawValue) }

    var titleKey: String {
        switch self {
        case .english: return "language.english"
        case .simplifiedChinese: return "language.simplified_chinese"
        }
    }
}

@MainActor
final class LanguageManager: ObservableObject {
    private static let userDefaultsKey = "SmartCar_Control_MAC.selectedLanguage"
    private let defaults: UserDefaults

    @Published private(set) var language: AppLanguage

    init(defaults: UserDefaults = .standard) {
        self.defaults = defaults
        let storedValue = defaults.string(forKey: Self.userDefaultsKey)
        self.language = storedValue.flatMap(AppLanguage.init(rawValue:)) ?? .english
    }

    var locale: Locale { language.locale }

    func setLanguage(_ language: AppLanguage) {
        guard self.language != language else { return }
        self.language = language
        defaults.set(language.rawValue, forKey: Self.userDefaultsKey)
    }

    func toggle() {
        setLanguage(language == .english ? .simplifiedChinese : .english)
    }
}

enum AppStrings {
    static func text(_ key: String, locale: Locale) -> String {
        String(localized: String.LocalizationValue(key), bundle: .module, locale: locale)
    }

    static func format(_ key: String, locale: Locale, _ arguments: CVarArg...) -> String {
        String(format: text(key, locale: locale), locale: locale, arguments: arguments)
    }
}
