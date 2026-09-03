import SwiftUI

struct SmartCarControlMACApp: App {
    @StateObject private var viewModel = SmartCarViewModel()
    @StateObject private var languageManager = LanguageManager()

    var body: some Scene {
        WindowGroup {
            ContentView(viewModel: viewModel)
                .environmentObject(languageManager)
                .environment(\.locale, languageManager.locale)
        }
    }
}

SmartCarControlMACApp.main()
