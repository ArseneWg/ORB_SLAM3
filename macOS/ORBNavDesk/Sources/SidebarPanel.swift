import SwiftUI

struct SidebarPanel: View {
    @ObservedObject var model: AppModel

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                appHeader

                PanelCard(title: "资源", subtitle: "这里保留不常用但和当前工作区相关的文件入口") {
                    VStack(alignment: .leading, spacing: 12) {
                        DetailRow(label: "当前 latest", value: latestFileName)
                        DetailRow(label: "工作目录", value: model.workspaceRoot.lastPathComponent)

                        HStack(spacing: 10) {
                            Button("查看截图文件") { model.revealLatestSnapshotInFinder() }
                            Button("打开工作目录") { model.openWorkspaceRootInFinder() }
                        }
                        .buttonStyle(.bordered)

                        Button("显示 guidance") { model.revealGuidanceFileInFinder() }
                            .buttonStyle(.bordered)
                    }
                }

                PanelCard(title: "导航模式", subtitle: "这里控制适合手持测试还是车载固定安装") {
                    VStack(alignment: .leading, spacing: 14) {
                        Picker("地图视图", selection: $model.viewMode) {
                            Text("跟随").tag("follow")
                            Text("总览").tag("overview")
                        }
                        .pickerStyle(.segmented)
                        .onChange(of: model.viewMode) { _, _ in
                            model.sendControl()
                        }

                        Toggle("车载固定高度", isOn: $model.fixedHeightMode)
                            .toggleStyle(.switch)
                            .onChange(of: model.fixedHeightMode) { _, _ in
                                model.sendControl()
                            }

                        VStack(alignment: .leading, spacing: 8) {
                            HStack {
                                Text("相机高度")
                                Spacer()
                                Text("\(model.fixedCameraHeightMeters, format: .number.precision(.fractionLength(2))) m")
                                    .monospacedDigit()
                                    .foregroundStyle(.secondary)
                            }
                            Slider(value: $model.fixedCameraHeightMeters, in: 0.10...1.20, step: 0.02)
                                .onChange(of: model.fixedCameraHeightMeters) { _, _ in
                                    model.sendControl()
                                }
                        }

                        Toggle("纯定位模式", isOn: $model.localizationOnly)
                            .toggleStyle(.switch)
                            .onChange(of: model.localizationOnly) { _, _ in
                                model.sendControl()
                            }

                        Divider()

                        VStack(alignment: .leading, spacing: 10) {
                            Text("深度来源")
                                .font(.caption.weight(.semibold))
                                .foregroundStyle(.secondary)

                            Picker("深度来源", selection: $model.depthSourceMode) {
                                ForEach(DepthSourceModeOption.allCases) { option in
                                    Text(option.title).tag(option)
                                }
                            }
                            .pickerStyle(.radioGroup)
                            .labelsHidden()
                            .onChange(of: model.depthSourceMode) { _, _ in
                                model.sendControl()
                            }

                            Text(model.depthSourceMode.note)
                                .font(.caption)
                                .foregroundStyle(.secondary)
                                .fixedSize(horizontal: false, vertical: true)

                            Text("说明：当前模型模式仍会借助 iPhone 深度做尺度对齐，所以它适合做替代能力评估，不等同于完全无真深度硬件。")
                                .font(.caption)
                                .foregroundStyle(.secondary)
                                .fixedSize(horizontal: false, vertical: true)
                        }
                    }
                }

                PanelCard(title: "预览与诊断", subtitle: "默认保持轻量运行，需要时再打开辅助预览和误差对比") {
                    VStack(alignment: .leading, spacing: 14) {
                        Toggle("显示 RGB 预览", isOn: $model.enableRgbPreview)
                            .toggleStyle(.switch)
                            .onChange(of: model.enableRgbPreview) { _, _ in
                                model.sendControl()
                            }

                        Toggle("显示深度预览", isOn: $model.enableDepthPreview)
                            .toggleStyle(.switch)
                            .onChange(of: model.enableDepthPreview) { _, _ in
                                model.sendControl()
                            }

                        Toggle("启用 Small V2 深度对比", isOn: $model.enableDepthComparison)
                            .toggleStyle(.switch)
                            .onChange(of: model.enableDepthComparison) { _, isEnabled in
                                if !isEnabled {
                                    model.enableDepthDiffPreview = false
                                }
                                model.sendControl()
                            }

                        Toggle("显示误差热图", isOn: $model.enableDepthDiffPreview)
                            .toggleStyle(.switch)
                            .disabled(!model.enableDepthComparison)
                            .onChange(of: model.enableDepthDiffPreview) { _, _ in
                                model.sendControl()
                            }

                        Text("说明：RGB/深度预览关闭后，App 不再主动读取这些面板；Small V2 对比关闭后，后端也会跳过这条旁路计算。")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }

                PanelCard(title: "地图层", subtitle: "这些设置会影响 2D 导航栅格和引导行为") {
                    VStack(alignment: .leading, spacing: 14) {
                        Toggle("显示膨胀避障层", isOn: $model.showInflation)
                            .toggleStyle(.switch)
                            .onChange(of: model.showInflation) { _, _ in
                                model.sendControl()
                            }

                        Stepper("膨胀半径 \(model.inflationRadiusCells) 格", value: $model.inflationRadiusCells, in: 1...12)
                            .onChange(of: model.inflationRadiusCells) { _, _ in
                                model.sendControl()
                            }

                        VStack(alignment: .leading, spacing: 8) {
                            HStack {
                                Text("前视距离")
                                Spacer()
                                Text("\(model.lookaheadMeters, format: .number.precision(.fractionLength(2))) m")
                                    .monospacedDigit()
                                    .foregroundStyle(.secondary)
                            }
                            Slider(value: $model.lookaheadMeters, in: 0.20...3.00, step: 0.05)
                                .onChange(of: model.lookaheadMeters) { _, _ in
                                    model.sendControl()
                                }
                        }
                    }
                }

                PanelCard(title: "目标与调试", subtitle: "目标点通过点击主图中的地图区域设置") {
                    VStack(alignment: .leading, spacing: 12) {
                        Button("清除目标点") { model.clearGoal() }
                            .buttonStyle(.bordered)

                        Text("如果你在主图里单击的是 RGB 或 Depth 区域，App 不会下发目标点。")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .fixedSize(horizontal: false, vertical: true)

                        Text("右侧 inspector 会显示 guidance 文件和尺度对照，适合核对后端当前状态。")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }
            }
            .padding(20)
        }
        .background(Color(nsColor: .underPageBackgroundColor))
    }

    private var appHeader: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("ORB Nav Desk")
                .font(.system(size: 28, weight: .bold, design: .rounded))
            Text("面向 iPhone RGB-D 与 ORB-SLAM3 的原生 macOS 导航工作台。")
                .font(.callout)
                .foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)

            HStack(spacing: 10) {
                StatusPill(text: model.connectionSummary,
                           color: model.runtimeState?.connected == true ? .green : (model.backendRunning ? .orange : .secondary))
                StatusPill(text: model.operationModeSummary,
                           color: model.fixedHeightMode ? .blue : .secondary)
            }
        }
    }

    private var latestFileName: String {
        model.latestSnapshotURL?.lastPathComponent ?? "--"
    }
}
