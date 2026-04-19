import SwiftUI

struct ConsolePanel: View {
    @ObservedObject var model: AppModel
    @State private var filterText = ""

    var body: some View {
        VStack(spacing: 0) {
            HStack(alignment: .top) {
                VStack(alignment: .leading, spacing: 6) {
                    Text("诊断面板")
                        .font(.title3.weight(.semibold))
                    Text("这里集中放运行日志和当前会话诊断信息，不占主工作区。")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }

                Spacer()

                HStack(spacing: 10) {
                    Button("清空日志", action: model.clearLogs)
                        .buttonStyle(.bordered)
                }
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 12)

            Divider()

            VStack(spacing: 14) {
                HStack(spacing: 12) {
                    diagnosticPill("连接", model.runtimeState?.connected == true ? "在线" : "等待")
                    diagnosticPill("跟踪", model.runtimeState?.trackingState ?? "NO_IMAGES_YET")
                    diagnosticPill("视图", model.runtimeState?.followRobot == true ? "跟随" : (model.runtimeState?.autoFit == true ? "总览" : "手动"))
                    Spacer()
                }

                TextField("筛选日志关键字", text: $filterText)
                    .textFieldStyle(.roundedBorder)

                ScrollView {
                    Text(filteredLogs)
                        .font(.system(.body, design: .monospaced))
                        .textSelection(.enabled)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .padding(16)
                }
                .background(Color(nsColor: .textBackgroundColor))
                .clipShape(RoundedRectangle(cornerRadius: 14, style: .continuous))
            }
            .padding(16)
        }
        .frame(minWidth: 860, minHeight: 560)
        .background(
            WindowPlacementReader { window in
                configure(window: window)
            }
        )
    }

    private var filteredLogs: String {
        let source = model.logs.isEmpty ? "日志会显示在这里。" : model.logs
        let keyword = filterText.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !keyword.isEmpty else { return source }
        let lines = source.split(separator: "\n", omittingEmptySubsequences: false)
        let matched = lines.filter { $0.localizedCaseInsensitiveContains(keyword) }
        return matched.isEmpty ? "没有匹配到包含“\(keyword)”的日志。" : matched.joined(separator: "\n")
    }

    @ViewBuilder
    private func diagnosticPill(_ label: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(label)
                .font(.caption2)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.caption.weight(.medium))
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
        .background(
            RoundedRectangle(cornerRadius: 12, style: .continuous)
                .fill(Color(nsColor: .controlBackgroundColor))
        )
    }

    private func configure(window: NSWindow) {
        window.tabbingMode = .disallowed
        guard let screen = window.screen ?? NSScreen.main else { return }
        let visible = screen.visibleFrame
        let frame = window.frame
        let padding: CGFloat = 24
        let targetOrigin = NSPoint(
            x: visible.maxX - frame.width - padding,
            y: visible.maxY - frame.height - 56
        )
        let currentOrigin = window.frame.origin
        if abs(currentOrigin.x - targetOrigin.x) > 8 || abs(currentOrigin.y - targetOrigin.y) > 8 {
            window.setFrameOrigin(targetOrigin)
        }
    }
}

private struct WindowPlacementReader: NSViewRepresentable {
    let onResolve: (NSWindow) -> Void

    func makeNSView(context: Context) -> WindowPlacementView {
        let view = WindowPlacementView()
        view.onResolve = onResolve
        return view
    }

    func updateNSView(_ nsView: WindowPlacementView, context: Context) {
        nsView.onResolve = onResolve
    }
}

private final class WindowPlacementView: NSView {
    var onResolve: ((NSWindow) -> Void)?

    override func viewDidMoveToWindow() {
        super.viewDidMoveToWindow()
        if let window {
            DispatchQueue.main.async { [weak self, weak window] in
                guard let self, let window else { return }
                self.onResolve?(window)
            }
        }
    }
}
