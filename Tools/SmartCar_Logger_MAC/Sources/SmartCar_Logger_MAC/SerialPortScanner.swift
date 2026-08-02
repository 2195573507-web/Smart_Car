import Foundation

enum SerialPortScanner {
    static func scan() -> [SerialPortDescriptor] {
        let directory = URL(fileURLWithPath: "/dev", isDirectory: true)
        let names = (try? FileManager.default.contentsOfDirectory(
            at: directory,
            includingPropertiesForKeys: nil,
            options: [.skipsHiddenFiles]
        ))?.map(\.lastPathComponent) ?? []

        let calloutNames = names.filter { $0.hasPrefix("cu.") }
        let candidates = calloutNames.isEmpty
            ? names.filter { $0.hasPrefix("tty.") }
            : calloutNames

        return candidates
            .sorted { lhs, rhs in
                let leftHint = Self.isCH340(lhs)
                let rightHint = Self.isCH340(rhs)
                return leftHint != rightHint ? leftHint && !rightHint : lhs < rhs
            }
            .map { name in
                SerialPortDescriptor(
                    path: "/dev/\(name)",
                    name: name,
                    isCH340: Self.isCH340(name)
                )
            }
    }

    private static func isCH340(_ name: String) -> Bool {
        let value = name.lowercased()
        return value.contains("usbserial") || value.contains("wch") || value.contains("ch340") || value.contains("ch341")
    }
}
