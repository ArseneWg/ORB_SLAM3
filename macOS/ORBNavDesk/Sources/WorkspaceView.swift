import SwiftUI

struct WorkspaceView: View {
    @ObservedObject var model: AppModel

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            header

            HStack(alignment: .top, spacing: 18) {
                MapCanvasView(
                    mapData: model.mapData,
                    state: model.runtimeState,
                    showGrid: model.mapShowGrid,
                    showFreeCells: model.mapShowFreeCells,
                    showOccupiedCells: model.mapShowOccupiedCells,
                    showInflationOverlay: model.mapShowInflationOverlay,
                    showTrajectory: model.mapShowTrajectory,
                    showPlannedPath: model.mapShowPlannedPath,
                    showRobot: model.mapShowRobot,
                    showAxes: model.mapShowAxes,
                    onTapWorld: { worldX, worldZ in
                        model.setGoal(worldX: worldX, worldZ: worldZ)
                    },
                    onHoverInfo: { info in
                        model.updateMapHoverInfo(info)
                    }
                )
                .frame(maxWidth: .infinity, minHeight: 690, maxHeight: .infinity)

                if model.hasVisiblePreviewPanels {
                    VStack(spacing: 18) {
                        if model.shouldShowRgbPreview {
                            ImageCanvasView(
                                title: "RGB 预览",
                                subtitle: "用于观察当前相机看到的真实场景。",
                                image: model.latestRgbImage,
                                accent: .mint,
                                placeholderTitle: "等待 RGB 画面",
                                placeholderDescription: "iPhone 开始推流后，RGB 预览会显示在这里。"
                            )
                            .frame(minWidth: 340, idealWidth: 340, maxWidth: 340, minHeight: 300, maxHeight: .infinity)
                        }

                        if model.shouldShowDepthPreview {
                            ImageCanvasView(
                                title: "深度预览",
                                subtitle: "用于快速检查深度有效区域和量程变化。",
                                image: model.latestDepthImage,
                                accent: .orange,
                                placeholderTitle: "等待深度画面",
                                placeholderDescription: "深度图会随 iPhone RGB-D 数据自动刷新。"
                            )
                            .frame(minWidth: 340, idealWidth: 340, maxWidth: 340, minHeight: 300, maxHeight: .infinity)
                        }

                        if model.shouldShowDepthDiffPreview {
                            ImageCanvasView(
                                title: "Small V2 误差",
                                subtitle: "将模型深度按传感器尺度对齐后，与 iPhone 深度做逐像素绝对误差热图。",
                                image: model.latestDepthDiffImage,
                                accent: .pink,
                                placeholderTitle: "等待深度对比",
                                placeholderDescription: "Small V2 推理与误差热图会在这里更新。"
                            )
                            .frame(minWidth: 340, idealWidth: 340, maxWidth: 340, minHeight: 220, maxHeight: .infinity)
                        }
                    }
                }
            }

            footer
        }
        .padding(20)
        .background(Color(nsColor: .windowBackgroundColor))
    }

    private var header: some View {
        VStack(alignment: .leading, spacing: 14) {
            HStack(spacing: 14) {
                MetricTile(label: "连接", value: model.connectionSummary, accent: model.runtimeState?.connected == true ? .green : .primary)
                MetricTile(label: "跟踪", value: model.trackingSummary, accent: trackingColor)
                MetricTile(label: "模式", value: model.operationModeSummary, accent: model.fixedHeightMode ? .blue : .primary)
                MetricTile(label: "深度源", value: model.depthSourceSummary, accent: .purple)
                MetricTile(label: "尺度比", value: model.scaleRatioSummary, accent: .orange)
                MetricTile(label: "下一航点", value: model.waypointSummary, accent: .mint)
            }

            HStack(spacing: 10) {
                StatusPill(text: model.backendRunning ? "后端运行中" : "后端未运行",
                           color: model.backendRunning ? .green : .secondary)
                if let runtimeState = model.runtimeState {
                    StatusPill(text: runtimeState.localizationOnly ? "纯定位" : "建图中",
                               color: runtimeState.localizationOnly ? .orange : .blue)
                    if let status = runtimeState.depthSourceStatus, !status.isEmpty {
                        StatusPill(text: status, color: .purple)
                    }
                }
                if let lastStateLoadedAt = model.lastStateLoadedAt {
                    StatusPill(text: "状态 \(lastStateLoadedAt.formatted(date: .omitted, time: .standard))",
                               color: .secondary)
                }
                Spacer()
            }
        }
    }

    private var footer: some View {
        HStack(alignment: .center, spacing: 16) {
            VStack(alignment: .leading, spacing: 4) {
                Text("工作区说明")
                    .font(.caption.weight(.semibold))
                Text("地图现在由 App 原生绘制；右侧预览与误差对比默认关闭，需要时可在左侧打开。")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }

            Spacer()

            if let lastImageLoadedAt = model.lastImageLoadedAt {
                Text("latest 更新 \(lastImageLoadedAt.formatted(date: .omitted, time: .standard))")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }

    private var trackingColor: Color {
        switch model.runtimeState?.trackingState {
        case "OK", "OK_KLT":
            return .green
        case "RECENTLY_LOST":
            return .orange
        case "LOST":
            return .red
        case "PAUSED":
            return .blue
        default:
            return .primary
        }
    }
}
