import SwiftUI

struct ContentView: View {
    @Bindable var session: LoggerSession

    var body: some View {
        VStack(spacing: 0) {
            connectionToolbar
            Divider()
            logViewer
            Divider()
            statusBar
        }
        .toolbar { toolbarActions }
        .onAppear {
            session.refreshPorts()
            session.startPortScanning()
        }
        .onDisappear {
            session.disconnect()
            session.stopPortScanning()
        }
    }

    private var connectionToolbar: some View {
        HStack(spacing: 10) {
            Image(systemName: "cable.connector")
                .foregroundStyle(.secondary)
            Picker("串口", selection: $session.selectedPortID) {
                if session.ports.isEmpty {
                    Text("未发现串口设备").tag("")
                } else {
                    ForEach(session.ports) { port in
                        Text(port.label).tag(port.id)
                    }
                }
            }
            .frame(minWidth: 270)
            Text("115200 8N1")
                .font(.caption.monospaced())
                .foregroundStyle(.secondary)
            Button { session.refreshPorts() } label: {
                Image(systemName: "arrow.clockwise")
            }
            .help("刷新串口设备")

            connectionButton
            Picker("显示级别", selection: $session.displayLevel) {
                ForEach(LogLevel.allCases, id: \.self) { level in
                    Text(level.rawValue).tag(level)
                }
            }
            .pickerStyle(.menu)
            Spacer()
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 10)
        .background(.bar)
    }

    @ViewBuilder private var connectionButton: some View {
        switch session.state {
        case .connected:
            Button("断开", systemImage: "stop.circle") { session.disconnect() }
                .buttonStyle(.bordered)
        case .connecting:
            ProgressView().controlSize(.small)
        default:
            Button("连接", systemImage: "link") { session.connect() }
                .buttonStyle(.borderedProminent)
                .disabled(session.selectedPort == nil)
        }
    }

    private var logViewer: some View {
        ScrollViewReader { proxy in
            ScrollView {
                LazyVStack(alignment: .leading, spacing: 2) {
                    if session.visibleLogEntries.isEmpty {
                        Text("等待 STM32 USART1 日志...")
                            .foregroundStyle(.secondary)
                    } else {
                        ForEach(session.visibleLogEntries) { entry in
                            logRow(entry)
                        }
                    }
                }
                .font(.system(.body, design: .monospaced))
                .textSelection(.enabled)
                .frame(maxWidth: .infinity, alignment: .topLeading)
                .padding(14)
            }
            .background(Color(nsColor: .textBackgroundColor))
            .onChange(of: session.visibleLogEntries.last?.id) {
                withAnimation(.easeOut(duration: 0.12)) {
                    if let id = session.visibleLogEntries.last?.id {
                        proxy.scrollTo(id, anchor: .bottom)
                    }
                }
            }
        }
    }

    private func logRow(_ entry: LogEntry) -> some View {
        HStack(alignment: .firstTextBaseline, spacing: 8) {
            Text(entry.level.rawValue)
                .foregroundStyle(logLevelColor(entry.level))
                .frame(width: 42, alignment: .leading)
            Text(entry.message)
                .foregroundStyle(.primary)
                .fixedSize(horizontal: false, vertical: true)
        }
        .id(entry.id)
    }

    private var statusBar: some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack(spacing: 10) {
                Circle()
                    .fill(statusColor)
                    .frame(width: 8, height: 8)
                Text(session.state.label)
                Divider().frame(height: 14)
                Text("\(session.receivedBytes.formatted()) bytes")
                    .monospacedDigit()
                Text("buffer \(session.loggerStatistics.storedLineCount)/\(session.loggerStatistics.capacity) (\(session.loggerStatistics.utilization, format: .percent.precision(.fractionLength(0))))")
                    .monospacedDigit()
                Text("dropped \(session.loggerStatistics.droppedLineCount)")
                    .monospacedDigit()
                if let startedAt = session.startedAt {
                    Text("开始于 \(startedAt.formatted(date: .omitted, time: .shortened))")
                        .foregroundStyle(.secondary)
                }
                Spacer()
                Text("仅查看")
                    .foregroundStyle(.secondary)
            }
            HStack(spacing: 10) {
                Text("RX \(session.lastReadByteCount) bytes")
                    .monospacedDigit()
                Text("fd \(session.fileDescriptor.map { String($0) } ?? "-")")
                    .monospacedDigit()
                Text(session.readSourceActive ? "source active" : "source inactive")
                if let lastReadAt = session.lastReadAt {
                    Text("最近读取 \(lastReadAt.formatted(date: .omitted, time: .standard))")
                        .foregroundStyle(.secondary)
                }
                if !session.lastReadHex.isEmpty {
                    Text("HEX \(session.lastReadHex)")
                        .font(.caption2.monospaced())
                        .lineLimit(1)
                        .truncationMode(.tail)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .textSelection(.enabled)
                }
            }
        }
        .font(.caption)
        .padding(.horizontal, 14)
        .frame(minHeight: 46)
        .background(.bar)
    }

    @ToolbarContentBuilder private var toolbarActions: some ToolbarContent {
        ToolbarItemGroup(placement: .primaryAction) {
            Button("Copy All Logs", systemImage: "doc.on.doc") {
                session.copyAllLogs()
            }
            .help("Copy All Logs")
            .disabled(session.loggerStatistics.storedLineCount == 0)
            Button { session.clearLog() } label: {
                Image(systemName: "trash")
            }
            .help("清空日志")
            .disabled(session.loggerStatistics.storedLineCount == 0)
        }
    }

    private func logLevelColor(_ level: LogLevel) -> Color {
        switch level {
        case .off: .secondary
        case .debug: .secondary
        case .trace: .secondary
        case .info: .primary
        case .warn: .orange
        case .error: .red
        }
    }

    private var statusColor: Color {
        switch session.state {
        case .connected: .green
        case .connecting: .orange
        case .failed: .red
        case .disconnected: .secondary
        }
    }

}
