import SwiftUI

@main
struct SmartCarIOSApp: App {
    @StateObject private var viewModel = SmartCarViewModel()

    var body: some Scene {
        WindowGroup {
            RootView(viewModel: viewModel)
                .tint(.teal)
                .preferredColorScheme(.dark)
        }
    }
}
