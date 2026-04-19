import SwiftUI

struct ContentView: View {
    @ObservedObject var model: AppModel
    @SceneStorage("orbnavdesk.inspectorVisible") private var inspectorVisible = true
    @Environment(\.openWindow) private var openWindow

    var body: some View {
        HSplitView {
            SidebarPanel(model: model)
                .frame(minWidth: 260, idealWidth: 280, maxWidth: 320)

            WorkspaceView(model: model)
                .frame(minWidth: 860, minHeight: 620)

            if inspectorVisible {
                InspectorView(model: model)
                .frame(minWidth: 270, idealWidth: 290, maxWidth: 330)
            }
        }
        .frame(minWidth: 1500, minHeight: 900)
        .background(Color(nsColor: .windowBackgroundColor))
        .toolbar {
            ToolbarItemGroup(placement: .navigation) {
                Picker("视图", selection: Binding(
                    get: { model.viewMode == "overview" ? "overview" : "follow" },
                    set: { newValue in
                        model.viewMode = newValue
                        model.sendControl()
                    }
                )) {
                    Text("跟随").tag("follow")
                    Text("总览").tag("overview")
                }
                .pickerStyle(.segmented)
                .frame(width: 180)
            }

            ToolbarItemGroup {
                Button {
                    model.toggleStartStop()
                } label: {
                    Label(model.backendRunning ? "停止后端" : "启动后端",
                          systemImage: model.backendRunning ? "stop.circle.fill" : "play.circle.fill")
                }
                .help("启动或停止 RGB-D 接收后端")

                Button("保存截图") {
                    model.saveSnapshot()
                }
                .help("让后端保存一张当前工作区截图")

                Button("打开 latest") {
                    model.openLatestSnapshot()
                }
                .help("打开最新导出的工作区截图")

                Button("清除目标") {
                    model.clearGoal()
                }
                .help("清除当前目标点和路径")
            }

            ToolbarItemGroup(placement: .primaryAction) {
                Button {
                    openWindow(id: "diagnostics")
                } label: {
                    Image(systemName: "doc.text.magnifyingglass")
                }
                .help("打开独立诊断窗口。它最适合放日志，因为不会遮挡主工作区。")

                Toggle(isOn: $inspectorVisible) {
                    Label("Inspector", systemImage: inspectorVisible ? "sidebar.right" : "sidebar.right")
                }
                .toggleStyle(.button)

                Button {
                    model.refreshNow()
                } label: {
                    Label("刷新", systemImage: "arrow.clockwise")
                }
                .help("立即刷新 state 与 latest 图")
            }
        }
        .onAppear {
            model.start()
        }
    }
}
