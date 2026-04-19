import SwiftUI

struct InspectorView: View {
    @ObservedObject var model: AppModel
    @SceneStorage("orbnavdesk.inspector.statusExpanded") private var statusExpanded = true
    @SceneStorage("orbnavdesk.inspector.guidanceExpanded") private var guidanceExpanded = true
    @SceneStorage("orbnavdesk.inspector.hoverExpanded") private var hoverExpanded = false
    @SceneStorage("orbnavdesk.inspector.mapDisplayExpanded") private var mapDisplayExpanded = false
    @SceneStorage("orbnavdesk.inspector.scaleExpanded") private var scaleExpanded = false
    @SceneStorage("orbnavdesk.inspector.legendExpanded") private var legendExpanded = false
    @SceneStorage("orbnavdesk.inspector.filesExpanded") private var filesExpanded = false

    var body: some View {
        ScrollView {
            LazyVStack(alignment: .leading, spacing: 14) {
                CollapsiblePanelCard(
                    title: "状态详情",
                    subtitle: "这里用于核对当前会话和地图工作面是否一致",
                    isExpanded: $statusExpanded
                ) {
                    DetailRow(label: "连接", value: model.runtimeState?.connected == true ? "在线" : "等待")
                    DetailRow(label: "跟踪", value: model.runtimeState?.trackingState ?? "未知")
                    DetailRow(label: "地面高度", value: metric(model.runtimeState?.floorY))
                    DetailRow(label: "视图模式", value: viewModeLabel)
                    DetailRow(label: "定位模式", value: model.runtimeState?.localizationOnly == true ? "纯定位" : "建图")
                    if let lastStateLoadedAt = model.lastStateLoadedAt {
                        DetailRow(label: "状态刷新", value: lastStateLoadedAt.formatted(date: .omitted, time: .standard))
                    }
                    if let lastImageLoadedAt = model.lastImageLoadedAt {
                        DetailRow(label: "图像刷新", value: lastImageLoadedAt.formatted(date: .omitted, time: .standard))
                    }
                }

                CollapsiblePanelCard(
                    title: "引导",
                    subtitle: "这些值更接近之后接小车控制层时会消费的输出",
                    isExpanded: $guidanceExpanded
                ) {
                    DetailRow(label: "下一航点", value: metric(model.runtimeState?.waypointDistanceMeters))
                    DetailRow(label: "转向角", value: angle(model.runtimeState?.headingErrorDegrees))
                    DetailRow(label: "目标状态", value: goalStatusText)

                    HStack(spacing: 10) {
                        Button("显示 guidance 文件") { model.revealGuidanceFileInFinder() }
                        Button("清除目标") { model.clearGoal() }
                    }
                    .buttonStyle(.bordered)
                }

                CollapsiblePanelCard(
                    title: "悬停信息",
                    subtitle: "鼠标移到主地图上时，这里会显示坐标和栅格类型",
                    isExpanded: $hoverExpanded
                ) {
                    DetailRow(label: "世界坐标", value: hoverWorldText)
                    DetailRow(label: "栅格坐标", value: hoverCellText)
                    DetailRow(label: "栅格类型", value: hoverKindText)
                }

                CollapsiblePanelCard(
                    title: "地图显示",
                    subtitle: "这些开关只影响 App 本地绘制，不会改动后端建图",
                    isExpanded: $mapDisplayExpanded
                ) {
                    Toggle("显示网格", isOn: $model.mapShowGrid)
                    Toggle("显示可通行", isOn: $model.mapShowFreeCells)
                    Toggle("显示障碍物", isOn: $model.mapShowOccupiedCells)
                    Toggle("显示避障缓冲", isOn: $model.mapShowInflationOverlay)
                    Toggle("显示运动轨迹", isOn: $model.mapShowTrajectory)
                    Toggle("显示规划路径", isOn: $model.mapShowPlannedPath)
                    Toggle("显示机器人与目标", isOn: $model.mapShowRobot)
                    Toggle("显示坐标轴", isOn: $model.mapShowAxes)
                }

                CollapsiblePanelCard(
                    title: "尺度与里程",
                    subtitle: "用于核对 ORB 与 iPhone 参考轨迹的比例关系",
                    isExpanded: $scaleExpanded
                ) {
                    DetailRow(label: "ORB 里程", value: metric(model.runtimeState?.orbDistanceMeters))
                    DetailRow(label: "参考里程", value: metric(model.runtimeState?.phoneDistanceMeters))
                    DetailRow(label: "尺度比", value: scaleRatio(model.runtimeState?.scaleRatio))
                    Text("尺度比只用于旁路核对，不反向参与主建图。")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                        .fixedSize(horizontal: false, vertical: true)
                }

                CollapsiblePanelCard(
                    title: "地图图例",
                    subtitle: "图例从主地图挪到了这里，避免遮挡可视区域",
                    isExpanded: $legendExpanded
                ) {
                    VStack(alignment: .leading, spacing: 10) {
                        LegendRow(color: Color.white.opacity(0.95), label: "可通行")
                        LegendRow(color: Color(red: 0.87, green: 0.19, blue: 0.16), label: "障碍物")
                        LegendRow(color: Color(red: 0.34, green: 0.67, blue: 0.96), label: "避障缓冲")
                        LegendRow(color: .green, label: "运动轨迹")
                        LegendRow(color: .cyan, label: "规划路径")
                        LegendRow(color: .yellow, label: "目标 / 朝向")

                        Divider()

                        Text("主图只保留轻量提示和比例尺。完整图例、路径状态和尺度说明都集中在右侧面板。")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }

                CollapsiblePanelCard(
                    title: "文件与输出",
                    subtitle: "便于快速跳到截图和中间产物",
                    isExpanded: $filesExpanded
                ) {
                    DetailRow(label: "latest 图", value: model.latestSnapshotURL?.path ?? "--")
                    DetailRow(label: "guidance", value: model.guidanceURL?.path ?? "--")
                    DetailRow(label: "地图数据", value: model.mapDataURL?.path ?? "--")

                    HStack(spacing: 10) {
                        Button("打开 latest") { model.openLatestSnapshot() }
                        Button("在访达中显示") { model.revealLatestSnapshotInFinder() }
                    }
                    .buttonStyle(.bordered)
                }
            }
            .padding(20)
        }
        .background(Color(nsColor: .underPageBackgroundColor))
    }

    private var viewModeLabel: String {
        if model.runtimeState?.followRobot == true { return "跟随" }
        if model.runtimeState?.autoFit == true { return "总览" }
        return "手动"
    }

    private var goalStatusText: String {
        guard let state = model.runtimeState else { return "--" }
        if state.hasWaypoint { return "已生成下一航点" }
        if state.hasGoal && state.pathValid { return "目标已设置，路径有效" }
        if state.hasGoal { return "目标已设置，等待路径" }
        return "未设置目标"
    }

    private var hoverWorldText: String {
        guard let info = model.mapHoverInfo else { return "--" }
        return String(format: "x %.2f, z %.2f", info.worldX, info.worldZ)
    }

    private var hoverCellText: String {
        guard let info = model.mapHoverInfo else { return "--" }
        return "\(info.cellX), \(info.cellZ)"
    }

    private var hoverKindText: String {
        model.mapHoverInfo?.cellKind.rawValue ?? "--"
    }

    private func metric(_ value: Double?) -> String {
        guard let value else { return "--" }
        return String(format: "%.2f m", value)
    }

    private func angle(_ value: Double?) -> String {
        guard let value else { return "--" }
        return String(format: "%.1f°", value)
    }

    private func scaleRatio(_ value: Double?) -> String {
        guard let value, value > 0 else { return "--" }
        return String(format: "%.3fx", value)
    }
}
