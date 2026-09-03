import SwiftUI

struct Surface<Content: View>: View {
    @ViewBuilder var content: () -> Content

    var body: some View {
        content()
            .padding(14)
            .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 12))
            .overlay {
                RoundedRectangle(cornerRadius: 12)
                    .stroke(.white.opacity(0.08), lineWidth: 1)
            }
            .frame(maxWidth: .infinity, alignment: .leading)
    }
}

struct SectionTitle: View {
    let title: String
    let systemImage: String
    var trailing: String?

    var body: some View {
        HStack(spacing: 8) {
            Label(title, systemImage: systemImage)
                .font(.headline.monospaced())
            Spacer()
            if let trailing {
                Text(trailing)
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
            }
        }
    }
}

struct MetricTile: View {
    let label: String
    let value: String
    var tint: Color = .primary

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(label)
                .font(.caption2.weight(.bold))
                .foregroundStyle(.secondary)
            Text(value)
                .font(.callout.monospaced().weight(.semibold))
                .foregroundStyle(tint)
                .lineLimit(1)
                .minimumScaleFactor(0.75)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

struct StatusPill: View {
    let title: String
    var tint: Color = .secondary

    var body: some View {
        Text(title)
            .font(.caption2.weight(.bold))
            .foregroundStyle(tint)
            .padding(.horizontal, 8)
            .padding(.vertical, 5)
            .background(tint.opacity(0.14), in: Capsule())
    }
}

struct ConnectionStatusCard: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @Environment(\.locale) private var locale

    private var isConnected: Bool { viewModel.status == .connected }

    var body: some View {
        Surface {
            HStack(spacing: 10) {
                Circle()
                    .fill(isConnected ? .green : .orange)
                    .frame(width: 8, height: 8)
                VStack(alignment: .leading, spacing: 2) {
                    Text(viewModel.discoveredDeviceName)
                        .font(.caption.weight(.semibold))
                        .lineLimit(1)
                Text(viewModel.status.displayText)
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
                if let error = viewModel.lastError {
                    Text(error.localizedDescription(locale: locale))
                        .font(.caption2)
                        .foregroundStyle(.red)
                        .lineLimit(2)
                }
                }
                if let rssi = viewModel.discoveredRSSI {
                    Text("\(rssi) dBm")
                        .font(.caption2.monospaced())
                        .foregroundStyle(.secondary)
                }
                let battery = viewModel.telemetryStore.status.snapshot.battery
                Label("\(battery)%", systemImage: battery < 20 ? "battery.25" : "battery.75")
                    .font(.caption2.monospaced())
                    .foregroundStyle(battery < 20 ? .red : .green)
                Spacer(minLength: 4)
                Button {
                    Haptics.impact(.light)
                    viewModel.scan()
                } label: {
                    Image(systemName: "dot.radiowaves.left.and.right")
                }
                .accessibilityLabel("扫描 SmartCar_S3")
                .disabled(viewModel.status == .scanning)

                Button {
                    Haptics.impact(.light)
                    if isConnected { viewModel.disconnect() } else { viewModel.connect() }
                } label: {
                    Image(systemName: isConnected ? "link.badge.minus" : "link.badge.plus")
                }
                .accessibilityLabel(isConnected ? "断开" : "连接")
                .disabled(viewModel.status == .connecting)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

struct EStopButton: View {
    let action: () -> Void

    var body: some View {
        Button(role: .destructive, action: action) {
            Label("E-STOP", systemImage: "hand.raised.fill")
                .font(.headline.weight(.black))
                .frame(maxWidth: .infinity)
                .padding(.vertical, 10)
        }
        .buttonStyle(.borderedProminent)
        .tint(.red)
        .accessibilityHint("清除所有轮子目标并发送零速度")
    }
}

func degreeText(_ radians: Float, precision: Int = 1) -> String {
    String(format: "%.*f°", precision, radians * 180 / Float.pi)
}

func speedText(_ speed: Float) -> String {
    String(format: "%.0f", speed)
}
