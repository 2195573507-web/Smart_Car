import SwiftUI

struct WheelTuningView: View {
    @ObservedObject var viewModel: SmartCarViewModel

    private var store: WheelSpeedState { viewModel.telemetryStore.wheelSpeed }

    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                Text("四轮独立目标速度与实际回显")
                    .font(.subheadline.weight(.semibold))
                    .frame(maxWidth: .infinity, alignment: .leading)
                SpeedControlPanel(
                    viewModel: viewModel,
                    store: store,
                    attitude: viewModel.telemetryStore.attitude
                )
                    .frame(maxWidth: .infinity)
                WheelActualPanel(viewModel: viewModel, store: store)
                    .frame(maxWidth: .infinity)

                HStack(spacing: 10) {
                    Button {
                        Haptics.impact(.medium)
                        viewModel.sendZeroWheelSpeeds()
                    } label: {
                        Label("ZERO RESET", systemImage: "arrow.counterclockwise")
                            .frame(maxWidth: .infinity)
                    }
                    .buttonStyle(.bordered)
                    .disabled(viewModel.status != .connected)
                }
            }
            .frame(maxWidth: .infinity)
            .padding(.horizontal, 14)
            .padding(.top, 14)
            .padding(.bottom, 80)
        }
        .scrollIndicators(.hidden)
        .safeAreaInset(edge: .top, spacing: 0) {
            EStopButton {
                Haptics.impact(.heavy)
                viewModel.emergencyWheelBrake()
            }
            .padding(.horizontal, 14)
            .padding(.vertical, 7)
            .background(.regularMaterial)
        }
        .navigationTitle("四轮调速")
    }
}
