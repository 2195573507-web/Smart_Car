import SwiftUI

public struct StatusView: View {
    let status: VehicleStatus
    let bleState: BLEState
    public var body: some View { VStack(alignment: .leading) { Text("BLE: \(String(describing: bleState))"); Text("Vehicle: \(String(describing: status.link))"); Text(status.hasFreshStatus ? "Status fresh" : "Status unavailable") } }
}
