import SwiftUI

@main
struct SmartCarApp: App {
    @StateObject private var viewModel = RemoteViewModel()

    var body: some Scene {
        WindowGroup {
            RemoteView(viewModel: viewModel)
        }
    }
}
