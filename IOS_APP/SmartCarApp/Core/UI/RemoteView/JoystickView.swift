import SwiftUI

public struct JoystickView: View {
    @ObservedObject var viewModel: RemoteViewModel
    @State private var knob = CGSize.zero
    public var body: some View {
        GeometryReader { proxy in
            let radius = min(proxy.size.width, proxy.size.height) / 2
            ZStack {
                Circle().fill(.blue.opacity(0.2))
                Circle().fill(.blue).frame(width: radius * 0.55, height: radius * 0.55).offset(knob)
            }
            .frame(width: radius * 2, height: radius * 2)
            .contentShape(Circle())
            .gesture(DragGesture().onChanged { value in
                let limit = radius * 0.45
                let x = max(-limit, min(limit, value.translation.width))
                let y = max(-limit, min(limit, value.translation.height))
                knob = CGSize(width: x, height: y)
                viewModel.joystick.update(linear: -Double(y / limit), turn: Double(x / limit))
                viewModel.sendJoystick()
            }.onEnded { _ in
                knob = .zero
                viewModel.joystick.reset()
                viewModel.stop()
            })
        }
        .frame(width: 160, height: 160)
    }
}
