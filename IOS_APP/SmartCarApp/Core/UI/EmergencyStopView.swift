import SwiftUI

public struct EmergencyStopView: View {
    let action: () -> Void
    public var body: some View { Button(role: .destructive, action: action) { Label("Emergency Stop", systemImage: "octagon.fill") }.buttonStyle(.borderedProminent) }
}
