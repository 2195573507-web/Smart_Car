import SwiftUI

enum AppMode: CaseIterable, Identifiable {
    case control
    case developer

    var id: Self { self }

    var titleKey: String {
        switch self {
        case .control: return "mode.control"
        case .developer: return "mode.developer"
        }
    }
}

struct ContentView: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @EnvironmentObject private var languageManager: LanguageManager
    @Environment(\.locale) private var locale

    var body: some View {
        VStack(spacing: 0) {
            HStack(spacing: 18) {
                VStack(alignment: .leading, spacing: 3) {
                    Text(AppStrings.text("app.title", locale: locale))
                        .font(.system(.title2, design: .rounded, weight: .bold))
                    Text(AppStrings.text("app.protocol_version", locale: locale))
                        .font(.caption.monospaced())
                        .foregroundStyle(.secondary)
                }
                Spacer()
                StatusBadge(label: AppStrings.text("label.ble", locale: locale), value: AppPresentationStrings.connectionStatus(viewModel.status, locale: locale), active: viewModel.status == .connected)
                StatusBadge(label: "SmartCar_S3", value: AppPresentationStrings.smartCarStatus(viewModel.vehicleStatus.smartCarS3Status, locale: locale), active: viewModel.vehicleStatus.smartCarS3Status == "ONLINE")
                ConnectionActions(viewModel: viewModel)
                AngleUnitMenu(selection: $viewModel.angleUnit)
                LanguageMenu()
            }
            .padding(.horizontal, 24)
            .padding(.vertical, 18)

            Divider()

            Picker(AppStrings.text("mode.picker", locale: locale), selection: $viewModel.mode) {
                ForEach(AppMode.allCases) { mode in Text(AppStrings.text(mode.titleKey, locale: locale)).tag(mode) }
            }
            .pickerStyle(.segmented)
            .padding(.horizontal, 24)
            .padding(.vertical, 14)

            if viewModel.mode == .developer {
                VStack(alignment: .leading, spacing: 8) {
                    Text("Developer Tools:")
                        .font(.caption.weight(.semibold))
                        .foregroundStyle(.secondary)

                    HStack(spacing: 8) {
                        LoggerEntryButton(
                            title: "LOGGER-STM",
                            icon: "cpu",
                            isSelected: viewModel.developerPage == .loggerSTM
                        ) {
                            viewModel.developerPage = .loggerSTM
                        }
                        LoggerEntryButton(
                            title: "LOGGER-S3",
                            icon: "dot.radiowaves.left.and.right",
                            isSelected: viewModel.developerPage == .loggerS3
                        ) {
                            viewModel.developerPage = .loggerS3
                        }
                        Spacer()
                    }
                }
                .padding(.horizontal, 24)
                .padding(.bottom, 12)
            }

            Group {
                switch viewModel.mode {
                case .control: ControlModeView(viewModel: viewModel, telemetryStore: viewModel.telemetryStore, angleUnit: viewModel.angleUnit)
                case .developer: DeveloperModeView(viewModel: viewModel, telemetryStore: viewModel.telemetryStore, angleUnit: viewModel.angleUnit)
                }
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
        }
        .frame(minWidth: 920, minHeight: 680)
    }
}

private struct AngleUnitMenu: View {
    @Binding var selection: AngleUnit
    @Environment(\.locale) private var locale

    var body: some View {
        Menu {
            ForEach(AngleUnit.allCases) { unit in
                Button {
                    selection = unit
                } label: {
                    if selection == unit {
                        Label(AppStrings.text(unit.titleKey, locale: locale), systemImage: "checkmark")
                    } else {
                        Text(AppStrings.text(unit.titleKey, locale: locale))
                    }
                }
            }
        } label: {
            Label(AppStrings.text("angle_unit.label", locale: locale), systemImage: "circle.lefthalf.filled")
        }
        .help(AppStrings.text("angle_unit.label", locale: locale))
    }
}

private struct LoggerEntryButton: View {
    let title: String
    let icon: String
    let isSelected: Bool
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            Label(title, systemImage: icon)
        }
        .buttonStyle(.bordered)
        .tint(isSelected ? .accentColor : .secondary)
    }
}

private struct ConnectionActions: View {
    @ObservedObject var viewModel: SmartCarViewModel
    @Environment(\.locale) private var locale

    var body: some View {
        HStack(spacing: 6) {
            Button(action: viewModel.scan) {
                Image(systemName: "dot.radiowaves.left.and.right")
            }
            .help(AppStrings.text("action.scan.help", locale: locale))
            .disabled(viewModel.status == .scanning)

            if viewModel.status == .connected {
                Button(action: viewModel.disconnect) {
                    Image(systemName: "link.badge.minus")
                }
                .help(AppStrings.text("action.disconnect.help", locale: locale))
            } else {
                Button(action: viewModel.connect) {
                    Image(systemName: "link.badge.plus")
                }
                .help(AppStrings.text("action.connect.help", locale: locale))
                .disabled(viewModel.status == .connecting)
            }
        }
        .buttonStyle(.bordered)
    }
}

private struct LanguageMenu: View {
    @EnvironmentObject private var languageManager: LanguageManager
    @Environment(\.locale) private var locale

    var body: some View {
        Menu {
            ForEach(AppLanguage.allCases) { language in
                Button {
                    languageManager.setLanguage(language)
                } label: {
                    if languageManager.language == language {
                        Label(AppStrings.text(language.titleKey, locale: locale), systemImage: "checkmark")
                    } else {
                        Text(AppStrings.text(language.titleKey, locale: locale))
                    }
                }
            }
        } label: {
            Label(AppStrings.text("language.menu", locale: locale), systemImage: "globe")
        }
        .menuStyle(.borderlessButton)
    }
}

private struct StatusBadge: View {
    let label: String
    let value: String
    let active: Bool

    var body: some View {
        VStack(alignment: .trailing, spacing: 2) {
            Text(label).font(.caption2.weight(.bold)).foregroundStyle(.secondary)
            Text(value).font(.caption.monospaced().weight(.semibold)).foregroundStyle(active ? .green : .secondary)
        }
    }
}
