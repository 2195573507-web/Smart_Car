import SwiftUI

struct LogConsoleView: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @State private var source: SmartCarLogSource = .stm32
    @State private var minimumLevel: SmartCarLogLevel = .info
    @State private var search = ""
    @State private var isPaused = false

    private var filteredRecords: [SmartCarLogRecord] {
        activeStore.records.filter { record in
            guard record.level >= minimumLevel else { return false }
            guard !search.isEmpty else { return true }
            return record.message.range(of: search, options: [.caseInsensitive, .regularExpression]) != nil
        }
    }

    private var exportText: String {
        filteredRecords.map { record in
            "\(timestamp(record.timestampMilliseconds)) [\(record.source.displayName)][\(record.level.displayName)] \(record.message)"
        }.joined(separator: "\n")
    }

    var body: some View {
        ScrollViewReader { proxy in
            ScrollView {
                VStack(spacing: 16) {
                    Surface {
                        VStack(alignment: .leading, spacing: 10) {
                            Picker("Source", selection: $source) {
                                ForEach(SmartCarLogSource.allCases, id: \.self) { source in
                                    Text(source.displayName).tag(source)
                                }
                            }
                            .pickerStyle(.segmented)

                            HStack(spacing: 8) {
                                Image(systemName: "magnifyingglass")
                                    .foregroundStyle(.secondary)
                                TextField("搜索日志文本或正则片段", text: $search)
                                    .textFieldStyle(.roundedBorder)
                            }

                            Picker("Minimum level", selection: $minimumLevel) {
                                Text("INFO").tag(SmartCarLogLevel.info)
                                Text("WARN").tag(SmartCarLogLevel.warn)
                                Text("ERROR").tag(SmartCarLogLevel.error)
                            }
                            .pickerStyle(.segmented)

                            HStack(spacing: 10) {
                                Button {
                                    isPaused.toggle()
                                    Haptics.impact(.light)
                                } label: {
                                    Label(isPaused ? "继续" : "暂停", systemImage: isPaused ? "play.fill" : "pause.fill")
                                }
                                .buttonStyle(.bordered)
                                Spacer(minLength: 4)
                                Button {
#if canImport(UIKit)
                                    UIPasteboard.general.string = exportText
#endif
                                    Haptics.impact(.light)
                                } label: {
                                    Image(systemName: "doc.on.doc")
                                }
                                .buttonStyle(.bordered)
                                .disabled(filteredRecords.isEmpty)
                                .accessibilityLabel("复制筛选日志")
                                ShareLink(item: exportText) {
                                    Image(systemName: "square.and.arrow.up")
                                }
                                .buttonStyle(.bordered)
                                .disabled(filteredRecords.isEmpty)
                                .accessibilityLabel("分享筛选日志")
                                Button(role: .destructive) {
                                    activeStore.clear()
                                } label: {
                                    Image(systemName: "trash")
                                }
                                .accessibilityLabel("清空日志")
                                .buttonStyle(.bordered)
                            }
                        }
                    }
                    .frame(maxWidth: .infinity)

                    Divider()

                    if source == .stm32 {
                        DeviceLogList(store: viewModel.stmLogStore, minimumLevel: minimumLevel, search: search)
                    } else {
                        DeviceLogList(store: viewModel.s3LogStore, minimumLevel: minimumLevel, search: search)
                    }
                    Color.clear
                        .frame(height: 1)
                        .id("log-bottom")
                }
                .frame(maxWidth: .infinity)
                .padding(.horizontal, 14)
                .padding(.top, 14)
                .padding(.bottom, 80)
            }
            .scrollIndicators(.hidden)
            .onAppear { proxy.scrollTo("log-bottom", anchor: .bottom) }
            .onChange(of: activeStore.records.count) { _, _ in
                guard !isPaused else { return }
                withAnimation(.easeOut(duration: 0.15)) {
                    proxy.scrollTo("log-bottom", anchor: .bottom)
                }
            }
        }
        .onChange(of: source) { _, _ in Haptics.impact(.light) }
        .onChange(of: minimumLevel) { _, _ in Haptics.impact(.light) }
        .navigationTitle("日志终端")
    }

    private var activeStore: DeviceLogStore {
        source == .stm32 ? viewModel.stmLogStore : viewModel.s3LogStore
    }
}

private struct DeviceLogList: View {
    @ObservedObject var store: DeviceLogStore
    let minimumLevel: SmartCarLogLevel
    let search: String

    private var records: [SmartCarLogRecord] {
        store.records.filter { record in
            guard record.level >= minimumLevel else { return false }
            guard !search.isEmpty else { return true }
            return record.message.range(of: search, options: [.caseInsensitive, .regularExpression]) != nil
        }
    }

    var body: some View {
        LazyVStack(alignment: .leading, spacing: 0) {
            if records.isEmpty {
                ContentUnavailableView("暂无日志", systemImage: "text.alignleft", description: Text("等待 \(store.source.displayName) 日志流"))
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 80)
            } else {
                ForEach(records) { record in
                    LogRow(record: record)
                    Divider()
                }
            }
        }
    }
}

private struct LogRow: View {
    let record: SmartCarLogRecord

    private var line: String {
        "\(timestamp(record.timestampMilliseconds)) [\(record.source.displayName)][\(record.level.displayName)] \(record.message)"
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 7) {
                Text(timestamp(record.timestampMilliseconds))
                    .font(.caption2.monospaced())
                    .foregroundStyle(.secondary)
                Text(record.level.displayName)
                    .font(.caption2.monospaced().weight(.bold))
                    .foregroundStyle(levelTint)
                Spacer()
                Button {
#if canImport(UIKit)
                    UIPasteboard.general.string = line
#endif
                    Haptics.impact(.light)
                } label: {
                    Image(systemName: "doc.on.doc")
                }
                .buttonStyle(.borderless)
                .accessibilityLabel("复制日志")
            }
            Text(record.message)
                .font(.caption.monospaced())
                .textSelection(.enabled)
                .fixedSize(horizontal: false, vertical: true)
        }
        .padding(.vertical, 9)
        .contextMenu {
            ShareLink(item: line) {
                Label("分享日志", systemImage: "square.and.arrow.up")
            }
        }
    }

    private var levelTint: Color {
        switch record.level {
        case .debug: return .secondary
        case .info: return .blue
        case .warn: return .orange
        case .error: return .red
        }
    }
}

private func timestamp(_ milliseconds: UInt32) -> String {
    let total = UInt64(milliseconds)
    let ms = total % 1000
    let seconds = total / 1000
    return String(format: "%02llu:%02llu:%02llu.%03llu", seconds / 3600, (seconds / 60) % 60, seconds % 60, ms)
}
