import SwiftUI
import AppKit

@main
struct SmartCarLoggerApp: App {
    @State private var session = LoggerSession()
    @NSApplicationDelegateAdaptor(AppDelegate.self) private var appDelegate

    var body: some Scene {
        WindowGroup("SmartCar Logger", id: "logger") {
            ContentView(session: session)
                .frame(minWidth: 820, minHeight: 520)
        }
        .commands {
            CommandGroup(after: .newItem) {
                Button("刷新串口") { session.refreshPorts() }
                    .keyboardShortcut("r", modifiers: [.command, .option])
                Button("保存日志...") { session.requestSave() }
                    .keyboardShortcut("s", modifiers: [.command, .shift])
                    .disabled(session.logText.isEmpty)
            }
        }
    }
}

final class AppDelegate: NSObject, NSApplicationDelegate {
    func applicationDidFinishLaunching(_ notification: Notification) {
        NSApp.setActivationPolicy(.regular)
        NSApp.activate(ignoringOtherApps: true)
    }
}
