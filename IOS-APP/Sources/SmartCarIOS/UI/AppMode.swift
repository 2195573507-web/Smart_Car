import Foundation

enum AppMode: CaseIterable, Identifiable {
    case control
    case developer

    var id: Self { self }
}
