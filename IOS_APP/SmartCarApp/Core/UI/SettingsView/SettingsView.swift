import SwiftUI

public struct SettingsView: View {
    @AppStorage("bleServiceUUID") private var serviceUUID = ""
    public var body: some View { Form { TextField("BLE Service UUID", text: $serviceUUID) } }
}
