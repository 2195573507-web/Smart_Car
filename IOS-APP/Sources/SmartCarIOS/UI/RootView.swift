import SwiftUI

struct RootView: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @State private var selectedTab = AppTab.control

    var body: some View {
        ZStack {
            Color.black
                .ignoresSafeArea()

            NavigationStack {
                TabView(selection: $selectedTab) {
                    ControlDashboardView(viewModel: viewModel)
                        .tabItem { Label("操控", systemImage: "steeringwheel") }
                        .tag(AppTab.control)

                    WheelTuningView(viewModel: viewModel)
                        .tabItem { Label("四轮调速", systemImage: "gauge.with.dots.needle.67percent") }
                        .tag(AppTab.wheels)

                    TelemetryView(viewModel: viewModel)
                        .tabItem { Label("姿态/遥测", systemImage: "cube.transparent") }
                        .tag(AppTab.telemetry)

                    HealthView(viewModel: viewModel)
                        .tabItem { Label("健康", systemImage: "waveform.path.ecg") }
                        .tag(AppTab.health)

                    LogConsoleView(viewModel: viewModel)
                        .tabItem { Label("日志", systemImage: "terminal") }
                        .tag(AppTab.logs)
                }
                .background(Color.black.ignoresSafeArea())
            }
        }
        .tint(.teal)
    }
}

private enum AppTab: Hashable {
    case control, wheels, telemetry, health, logs
}
