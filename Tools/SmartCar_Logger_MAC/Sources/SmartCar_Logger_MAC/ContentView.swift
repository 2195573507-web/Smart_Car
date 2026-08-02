import SwiftUI
import UniformTypeIdentifiers

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
        .fileExporter(
            isPresented: $session.saveRequest.isPresented,
            document: LogDocument(text: session.logText),
            contentType: .plainText,
            defaultFilename: defaultFilename
        ) { result in
            if case .success(let url) = result {
                try? session.save(to: url)
            }
        }
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
                Text(session.logText.isEmpty ? "等待 STM32 USART1 日志..." : session.logText)
                    .font(.system(.body, design: .monospaced))
                    .foregroundStyle(session.logText.isEmpty ? .secondary : .primary)
                    .textSelection(.enabled)
                    .frame(maxWidth: .infinity, alignment: .topLeading)
                    .padding(14)
                    .id("log")
            }
            .background(Color(nsColor: .textBackgroundColor))
            .onChange(of: session.logText) {
                withAnimation(.easeOut(duration: 0.12)) {
                    proxy.scrollTo("log", anchor: .bottom)
                }
            }
        }
    }

    private var statusBar: some View {
        HStack(spacing: 10) {
            Circle()
                .fill(statusColor)
                .frame(width: 8, height: 8)
            Text(session.state.label)
            Divider().frame(height: 14)
            Text("\(session.receivedBytes.formatted()) bytes")
                .monospacedDigit()
            if let startedAt = session.startedAt {
                Text("开始于 \(startedAt.formatted(date: .omitted, time: .shortened))")
                    .foregroundStyle(.secondary)
            }
            Spacer()
            Text("仅查看")
                .foregroundStyle(.secondary)
        }
        .font(.caption)
        .padding(.horizontal, 14)
        .frame(height: 30)
        .background(.bar)
    }

    @ToolbarContentBuilder private var toolbarActions: some ToolbarContent {
        ToolbarItemGroup(placement: .primaryAction) {
            Button { session.clearLog() } label: {
                Image(systemName: "trash")
            }
            .help("清空日志")
            .disabled(session.logText.isEmpty)
            Button { session.requestSave() } label: {
                Image(systemName: "square.and.arrow.down")
            }
            .help("保存日志")
            .disabled(session.logText.isEmpty)
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

    private var defaultFilename: String {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.dateFormat = "yyyyMMdd-HHmmss"
        return "smartcar-usart1-\(formatter.string(from: Date()))"
    }
}

struct LogDocument: FileDocument {
    static var readableContentTypes: [UTType] { [.plainText] }
    var text: String

    init(text: String = "") { self.text = text }

    init(configuration: ReadConfiguration) throws {
        text = String(decoding: configuration.file.regularFileContents ?? Data(), as: UTF8.self)
    }

    func fileWrapper(configuration: WriteConfiguration) throws -> FileWrapper {
        FileWrapper(regularFileWithContents: Data(text.utf8))
    }
}
