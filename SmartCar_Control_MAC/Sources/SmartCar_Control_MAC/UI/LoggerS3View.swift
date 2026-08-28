import SwiftUI

struct LoggerS3View: View {
    @ObservedObject var store: DeviceLogStore
    let returnToOverview: () -> Void

    var body: some View {
        DeviceLogView(
            title: "LOGGER-S3",
            store: store,
            returnToOverview: returnToOverview
        )
    }
}
