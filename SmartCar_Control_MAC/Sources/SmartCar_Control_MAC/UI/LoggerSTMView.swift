import SwiftUI

struct LoggerSTMView: View {
    @ObservedObject var store: DeviceLogStore
    let returnToOverview: () -> Void

    var body: some View {
        DeviceLogView(
            title: "LOGGER-STM",
            store: store,
            returnToOverview: returnToOverview
        )
    }
}
