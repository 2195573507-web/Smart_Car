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
                Button("Copy All Logs") { session.copyAllLogs() }
                    .keyboardShortcut("c", modifiers: [.command, .shift])
                    .disabled(session.loggerStatistics.storedLineCount == 0)
                Button("刷新串口") { session.refreshPorts() }
                    .keyboardShortcut("r", modifiers: [.command, .option])
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
