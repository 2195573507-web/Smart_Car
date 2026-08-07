import SwiftUI

public struct RemoteView: View {
    @ObservedObject var viewModel: RemoteViewModel
    public init(viewModel: RemoteViewModel) { self.viewModel = viewModel }
    public var body: some View {
        NavigationStack {
            VStack(spacing: 20) {
                StatusView(status: viewModel.status, bleState: viewModel.bleState)
                NavigationLink {
                    IMUCalibrationView(calibration: viewModel.imuCalibration)
                } label: {
                    Label("IMU Calibration", systemImage: "waveform.path.ecg")
                }
                HStack { JoystickView(viewModel: viewModel); DirectionPadView(send: viewModel.sendPad) }
                EmergencyStopView(action: viewModel.emergencyStop)
            }
            .padding()
            .navigationTitle("Smart Car")
        }
    }
}
