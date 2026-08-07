import AppKit
import SwiftUI
import Foundation
import UniformTypeIdentifiers

struct DeviceLogView: View {
    let title: String
    @ObservedObject var store: DeviceLogStore
    let returnToOverview: () -> Void

    @State private var minimumLevel: SmartCarLogLevel = .info
    @State private var isScrollPaused = false

    private var visibleRecords: [SmartCarLogRecord] {
        store.records.filter { $0.level >= minimumLevel }
    }

    var body: some View {
        VStack(spacing: 0) {
            HStack(spacing: 10) {
                Button(action: returnToOverview) {
                    Image(systemName: "chevron.left")
                }
                .help("Developer overview")

                Text(title)
                    .font(.headline.monospaced())

                Spacer()

                Picker("Minimum log level", selection: $minimumLevel) {
                    Text("INFO").tag(SmartCarLogLevel.info)
                    Text("WARN").tag(SmartCarLogLevel.warn)
                    Text("ERROR").tag(SmartCarLogLevel.error)
                }
                .pickerStyle(.segmented)
                .frame(width: 230)

                Button {
                    isScrollPaused.toggle()
                } label: {
                    Image(systemName: isScrollPaused ? "play.fill" : "pause.fill")
                }
                .help(isScrollPaused ? "Resume automatic scrolling" : "Pause automatic scrolling")

                Button(role: .destructive, action: store.clear) {
                    Image(systemName: "trash")
                }
                .help("Clear \(title) logs")

                Button {
                    let text = store.records.map(copyLine).joined(separator: "\n")
                    NSPasteboard.general.clearContents()
                    NSPasteboard.general.setString(text, forType: .string)
                } label: {
                    Label("Copy All Logs", systemImage: "doc.on.doc")
                }
                .help("Copy all logs")

                Button {
                    exportRecords(store.records)
                } label: {
                    Label("Export TXT", systemImage: "square.and.arrow.down")
                }
                .help("Export all logs as TXT")
            }
            .padding(.horizontal, 24)
            .padding(.vertical, 14)

            Divider()

            ScrollViewReader { proxy in
                ScrollView {
                    LazyVStack(alignment: .leading, spacing: 0) {
                        if visibleRecords.isEmpty {
                            ContentUnavailableView(
                                "No logs",
                                systemImage: "text.alignleft",
                                description: Text("Waiting for \(store.source.displayName) log records."))
                                .frame(maxWidth: .infinity, minHeight: 280)
                        } else {
                            ForEach(visibleRecords) { record in
                                DeviceLogRow(record: record)
                                Divider()
                            }
                        }
                        Color.clear.frame(height: 1).id("log-bottom")
                    }
                    .padding(.horizontal, 24)
                    .padding(.vertical, 12)
                }
                .onAppear {
                    scrollToBottom(proxy, animated: false)
                }
                .onChange(of: store.records.count) { _, _ in
                    guard !isScrollPaused else { return }
                    scrollToBottom(proxy, animated: true)
                }
                .onChange(of: minimumLevel) { _, _ in
                    guard !isScrollPaused else { return }
                    scrollToBottom(proxy, animated: false)
                }
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }

    private func scrollToBottom(_ proxy: ScrollViewProxy, animated: Bool) {
        if animated {
            withAnimation(.easeOut(duration: 0.15)) {
                proxy.scrollTo("log-bottom", anchor: .bottom)
            }
        } else {
            proxy.scrollTo("log-bottom", anchor: .bottom)
        }
    }

    private func copyLine(_ record: SmartCarLogRecord) -> String {
        "\(deviceLogTimestamp(record.timestampMilliseconds)) [\(record.source.displayName)][\(record.level.displayName)] \(record.message)"
    }

    private func exportRecords(_ records: [SmartCarLogRecord]) {
        let panel = NSSavePanel()
        panel.nameFieldStringValue = "smartcar-logs.txt"
        panel.allowedContentTypes = [.plainText]
        panel.canCreateDirectories = true
        guard panel.runModal() == .OK, let url = panel.url else { return }
        try? records.map(copyLine).joined(separator: "\n")
            .write(to: url, atomically: true, encoding: .utf8)
    }
}

private struct DeviceLogRow: View {
    let record: SmartCarLogRecord

    var body: some View {
        HStack(alignment: .top, spacing: 12) {
            Text(deviceLogTimestamp(record.timestampMilliseconds))
                .font(.caption.monospaced())
                .foregroundStyle(.secondary)
                .frame(width: 94, alignment: .leading)

            Text("[\(record.source.displayName)]")
                .font(.caption.monospaced().weight(.bold))
                .frame(width: 42, alignment: .leading)

            Text(record.level.displayName)
                .font(.caption.monospaced().weight(.bold))
                .foregroundStyle(levelColor)
                .frame(width: 46, alignment: .leading)

            Text(record.message)
                .font(.body.monospaced())
                .textSelection(.enabled)
                .frame(maxWidth: .infinity, alignment: .leading)

            Button {
                NSPasteboard.general.clearContents()
                NSPasteboard.general.setString(
                    "\(deviceLogTimestamp(record.timestampMilliseconds)) [\(record.source.displayName)][\(record.level.displayName)] \(record.message)",
                    forType: .string)
            } label: {
                Image(systemName: "doc.on.doc")
            }
            .buttonStyle(.borderless)
            .help("Copy Single Log")
        }
        .padding(.vertical, 8)
    }

    private var levelColor: Color {
        switch record.level {
        case .debug: return .secondary
        case .info: return .blue
        case .warn: return .orange
        case .error: return .red
        }
    }
}

private func deviceLogTimestamp(_ millisecondsValue: UInt32) -> String {
    let totalMilliseconds = UInt64(millisecondsValue)
    let milliseconds = totalMilliseconds % 1_000
    let totalSeconds = totalMilliseconds / 1_000
    let seconds = totalSeconds % 60
    let minutes = (totalSeconds / 60) % 60
    let hours = totalSeconds / 3_600
    return String(format: "%02llu:%02llu:%02llu.%03llu",
                  hours, minutes, seconds, milliseconds)
}
