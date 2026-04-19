import SwiftUI

struct ImageCanvasView: View {
    let title: String
    let subtitle: String
    let image: NSImage?
    var accent: Color = .accentColor
    var placeholderTitle: String = "等待图像"
    var placeholderDescription: String = "当新数据到达时，这里会自动刷新。"

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            VStack(alignment: .leading, spacing: 6) {
                HStack(spacing: 8) {
                    Circle()
                        .fill(accent)
                        .frame(width: 8, height: 8)
                    Text(title)
                }
                .font(.headline)

                Text(subtitle)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }

            ZStack {
                RoundedRectangle(cornerRadius: 16, style: .continuous)
                    .fill(Color(nsColor: .windowBackgroundColor))

                if let image {
                    Image(nsImage: image)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .padding(14)
                } else {
                    ContentUnavailableView {
                        Label(placeholderTitle, systemImage: "photo")
                    } description: {
                        Text(placeholderDescription)
                    }
                }
            }
        }
        .padding(18)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
        .background(
            RoundedRectangle(cornerRadius: 18, style: .continuous)
                .fill(Color(nsColor: .controlBackgroundColor))
        )
        .overlay(
            RoundedRectangle(cornerRadius: 18, style: .continuous)
                .stroke(Color.primary.opacity(0.05), lineWidth: 1)
        )
    }
}

struct MapCanvasView: View {
    let mapData: NavigationMapData?
    let state: NavigationRuntimeState?
    let showGrid: Bool
    let showFreeCells: Bool
    let showOccupiedCells: Bool
    let showInflationOverlay: Bool
    let showTrajectory: Bool
    let showPlannedPath: Bool
    let showRobot: Bool
    let showAxes: Bool
    let onTapWorld: (Double, Double) -> Void
    let onHoverInfo: (MapHoverInfo?) -> Void

    @State private var viewport: MapViewport?
    @State private var hasUserAdjustedView = false
    @State private var dragStartViewport: MapViewport?
    @State private var zoomStartViewport: MapViewport?
    @State private var hoverInfo: MapHoverInfo?

    private let drawingInset: CGFloat = 18

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            VStack(alignment: .leading, spacing: 4) {
                Text("导航地图")
                    .font(.headline.weight(.semibold))
                Text(mapSubtitle)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }

            GeometryReader { geometry in
                ZStack {
                    RoundedRectangle(cornerRadius: 18, style: .continuous)
                        .fill(Color(nsColor: .windowBackgroundColor))

                    if let mapData {
                        let activeViewport = resolvedViewport(for: mapData)
                        Canvas { context, size in
                            drawMap(in: &context, size: size, mapData: mapData, viewport: activeViewport)
                        }
                        .padding(drawingInset)
                        .clipped()
                    } else {
                        ContentUnavailableView {
                            Label("等待地图", systemImage: "map")
                        } description: {
                            Text("当 ORB-SLAM3 开始收到 RGB-D 数据后，地图会显示在这里。")
                        }
                    }
                }
                .overlay(alignment: .topTrailing) {
                    if let mapData {
                        VStack(alignment: .trailing, spacing: 10) {
                            MapScaleTag(lengthMeters: 1.0, metersPerPixel: resolvedViewport(for: mapData).metersPerPixel)
                        }
                        .padding(18)
                    }
                }
                .overlay(alignment: .topLeading) {
                    if let mapData {
                        MapViewportControls(
                            zoomLabel: zoomLabel(for: mapData),
                            onReset: { resetViewport(using: mapData) },
                            onZoomIn: { stepZoom(using: mapData, factor: 0.82) },
                            onZoomOut: { stepZoom(using: mapData, factor: 1.22) }
                        )
                        .padding(18)
                    }
                }
                .overlay(alignment: .bottomLeading) {
                    compactHint
                        .padding(18)
                }
                .overlay(alignment: .bottomTrailing) {
                    if let hoverInfo {
                        hoverTag(hoverInfo)
                            .padding(18)
                    }
                }
                .contentShape(Rectangle())
                .onTapGesture(count: 2) {
                    if let mapData {
                        resetViewport(using: mapData)
                    }
                }
                .onContinuousHover(coordinateSpace: .local) { phase in
                    guard let mapData else { return }
                    let drawable = drawingRect(in: geometry.size)
                    switch phase {
                    case .active(let point):
                        guard drawable.contains(point) else {
                            setHoverInfo(nil)
                            return
                        }
                        let local = CGPoint(x: point.x - drawable.minX, y: point.y - drawable.minY)
                        let world = viewToWorld(point: local, size: drawable.size, viewport: resolvedViewport(for: mapData))
                        setHoverInfo(makeHoverInfo(worldX: world.x, worldZ: world.z, mapData: mapData))
                    case .ended:
                        setHoverInfo(nil)
                    }
                }
                .simultaneousGesture(
                    SpatialTapGesture()
                        .onEnded { value in
                            guard let mapData else { return }
                            let drawable = drawingRect(in: geometry.size)
                            let local = CGPoint(x: value.location.x - drawable.minX, y: value.location.y - drawable.minY)
                            guard drawable.contains(value.location) else { return }
                            let world = viewToWorld(point: local, size: drawable.size, viewport: resolvedViewport(for: mapData))
                            onTapWorld(world.x, world.z)
                        }
                )
                .simultaneousGesture(
                    DragGesture(minimumDistance: 4)
                        .onChanged { value in
                            guard let mapData else { return }
                            if dragStartViewport == nil {
                                dragStartViewport = resolvedViewport(for: mapData)
                            }
                            guard let start = dragStartViewport else { return }
                            hasUserAdjustedView = true
                            viewport = start.panned(dx: value.translation.width, dy: value.translation.height)
                        }
                        .onEnded { _ in
                            dragStartViewport = nil
                        }
                )
                .simultaneousGesture(
                    MagnificationGesture()
                        .onChanged { value in
                            guard let mapData else { return }
                            if zoomStartViewport == nil {
                                zoomStartViewport = resolvedViewport(for: mapData)
                            }
                            guard let start = zoomStartViewport else { return }
                            hasUserAdjustedView = true
                            viewport = start.zoomed(by: value)
                        }
                        .onEnded { _ in
                            zoomStartViewport = nil
                        }
                )
            }
            .onAppear {
                if let mapData {
                    syncViewportFromBackend(mapData: mapData, force: true)
                }
            }
            .onChange(of: mapData?.timestamp) { _, _ in
                if let mapData {
                    syncViewportFromBackend(mapData: mapData, force: false)
                }
            }
            .onChange(of: state?.followRobot) { _, _ in
                if let mapData {
                    resetViewport(using: mapData)
                }
            }
            .onChange(of: state?.autoFit) { _, _ in
                if let mapData {
                    resetViewport(using: mapData)
                }
            }
        }
        .padding(18)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
        .background(
            RoundedRectangle(cornerRadius: 18, style: .continuous)
                .fill(Color(nsColor: .controlBackgroundColor))
        )
        .overlay(
            RoundedRectangle(cornerRadius: 18, style: .continuous)
                .stroke(Color.primary.opacity(0.05), lineWidth: 1)
        )
    }

    private var mapSubtitle: String {
        let tracking = state?.trackingState ?? "NO_IMAGES_YET"
        let connection = state?.connected == true ? "流在线" : "等待推流"
        return "\(tracking) · \(connection) · 拖拽平移、缩放，单击地图设置目标点。"
    }

    private var trackingColor: Color {
        switch state?.trackingState {
        case "OK", "OK_KLT":
            return .green
        case "RECENTLY_LOST":
            return .orange
        case "LOST":
            return .red
        case "PAUSED":
            return .blue
        default:
            return .secondary
        }
    }

    private var compactHint: some View {
        HStack(spacing: 10) {
            Image(systemName: "hand.draw")
                .foregroundStyle(.secondary)
            Text("拖拽平移，双击重置，单击设目标")
                .font(.caption.weight(.medium))
                .foregroundStyle(.secondary)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 10)
        .background(.ultraThinMaterial, in: Capsule())
    }

    private func hoverTag(_ info: MapHoverInfo) -> some View {
        VStack(alignment: .trailing, spacing: 4) {
            Text(String(format: "x %.2f  z %.2f", info.worldX, info.worldZ))
                .font(.caption.monospacedDigit())
            Text("格 \(info.cellX), \(info.cellZ) · \(info.cellKind.rawValue)")
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }

    private func drawMap(in context: inout GraphicsContext, size: CGSize, mapData: NavigationMapData, viewport: MapViewport) {
        let rect = CGRect(origin: .zero, size: size)
        context.fill(Path(rect), with: .color(Color(red: 0.07, green: 0.07, blue: 0.07)))

        if showGrid {
            drawGrid(in: &context, size: size, viewport: viewport)
        }
        drawCells(in: &context, size: size, mapData: mapData, viewport: viewport)
        if showTrajectory {
            drawTrajectory(in: &context, size: size, mapData: mapData, viewport: viewport)
        }
        if showPlannedPath {
            drawPath(in: &context, size: size, mapData: mapData, viewport: viewport)
        }
        if showRobot {
            drawGoal(in: &context, size: size, mapData: mapData, viewport: viewport)
            drawRobot(in: &context, size: size, mapData: mapData, viewport: viewport)
        }
        if showAxes {
            drawAxes(in: &context, size: size)
        }
        if let hoverInfo {
            drawHoverHighlight(in: &context, size: size, mapData: mapData, viewport: viewport, hoverInfo: hoverInfo)
        }
    }

    private func drawGrid(in context: inout GraphicsContext, size: CGSize, viewport: MapViewport) {
        let stepMeters = 0.5
        let visibleWidthMeters = Double(size.width) * viewport.metersPerPixel
        let visibleHeightMeters = Double(size.height) * viewport.metersPerPixel
        let leftWorld = viewport.centerX - visibleWidthMeters / 2.0
        let rightWorld = viewport.centerX + visibleWidthMeters / 2.0
        let bottomWorld = viewport.centerZ - visibleHeightMeters / 2.0
        let topWorld = viewport.centerZ + visibleHeightMeters / 2.0

        var path = Path()
        let firstVertical = Int(floor(leftWorld / stepMeters))
        let lastVertical = Int(ceil(rightWorld / stepMeters))
        for index in firstVertical...lastVertical {
            let xWorld = Double(index) * stepMeters
            let pointTop = worldToView(x: xWorld, z: topWorld, size: size, viewport: viewport)
            let pointBottom = worldToView(x: xWorld, z: bottomWorld, size: size, viewport: viewport)
            path.move(to: pointTop)
            path.addLine(to: pointBottom)
        }

        let firstHorizontal = Int(floor(bottomWorld / stepMeters))
        let lastHorizontal = Int(ceil(topWorld / stepMeters))
        for index in firstHorizontal...lastHorizontal {
            let zWorld = Double(index) * stepMeters
            let pointLeft = worldToView(x: leftWorld, z: zWorld, size: size, viewport: viewport)
            let pointRight = worldToView(x: rightWorld, z: zWorld, size: size, viewport: viewport)
            path.move(to: pointLeft)
            path.addLine(to: pointRight)
        }

        context.stroke(path, with: .color(Color.white.opacity(0.08)), lineWidth: 1)

        var axisPath = Path()
        let verticalAxisTop = worldToView(x: 0, z: topWorld, size: size, viewport: viewport)
        let verticalAxisBottom = worldToView(x: 0, z: bottomWorld, size: size, viewport: viewport)
        axisPath.move(to: verticalAxisTop)
        axisPath.addLine(to: verticalAxisBottom)

        let horizontalAxisLeft = worldToView(x: leftWorld, z: 0, size: size, viewport: viewport)
        let horizontalAxisRight = worldToView(x: rightWorld, z: 0, size: size, viewport: viewport)
        axisPath.move(to: horizontalAxisLeft)
        axisPath.addLine(to: horizontalAxisRight)
        context.stroke(axisPath, with: .color(Color.orange.opacity(0.18)), lineWidth: 1)

        if size.width > 140 && size.height > 80 {
            let label = Text("0.5 m / 格")
                .font(.system(size: 11, weight: .medium, design: .rounded))
            context.draw(label, at: CGPoint(x: size.width - 54, y: size.height - 16), anchor: .center)
        }
    }

    private func drawCells(in context: inout GraphicsContext, size: CGSize, mapData: NavigationMapData, viewport: MapViewport) {
        let cellSize = max(1.0, mapData.resolution / viewport.metersPerPixel)
        if showFreeCells {
            var freePath = Path()
            for cell in mapData.freeCells {
                freePath.addRect(cellRect(cell, size: size, mapData: mapData, viewport: viewport, pixelSize: cellSize))
            }
            context.fill(freePath, with: .color(Color.white.opacity(0.9)))
        }

        if mapData.showInflation && showInflationOverlay {
            var inflationPath = Path()
            for cell in mapData.inflatedCells {
                inflationPath.addRect(cellRect(cell, size: size, mapData: mapData, viewport: viewport, pixelSize: cellSize))
            }
            context.fill(inflationPath, with: .color(Color(red: 0.34, green: 0.67, blue: 0.96, opacity: 0.75)))
        }

        if showOccupiedCells {
            var occupiedPath = Path()
            for cell in mapData.occupiedCells {
                occupiedPath.addRect(cellRect(cell, size: size, mapData: mapData, viewport: viewport, pixelSize: cellSize))
            }
            context.fill(occupiedPath, with: .color(Color(red: 0.87, green: 0.19, blue: 0.16)))
        }
    }

    private func drawTrajectory(in context: inout GraphicsContext, size: CGSize, mapData: NavigationMapData, viewport: MapViewport) {
        guard mapData.trajectory.count >= 2 else { return }
        var path = Path()
        for (index, point) in mapData.trajectory.enumerated() {
            let mapped = worldToView(x: point.x, z: point.z, size: size, viewport: viewport)
            if index == 0 {
                path.move(to: mapped)
            } else {
                path.addLine(to: mapped)
            }
        }
        context.stroke(path, with: .color(Color.green), lineWidth: 2)
    }

    private func drawPath(in context: inout GraphicsContext, size: CGSize, mapData: NavigationMapData, viewport: MapViewport) {
        guard mapData.path.count >= 2 else { return }
        var path = Path()
        for (index, point) in mapData.path.enumerated() {
            let mapped = worldToView(x: point.x, z: point.z, size: size, viewport: viewport)
            if index == 0 {
                path.move(to: mapped)
            } else {
                path.addLine(to: mapped)
            }
        }
        context.stroke(path, with: .color(Color.cyan), lineWidth: 2.5)
    }

    private func drawGoal(in context: inout GraphicsContext, size: CGSize, mapData: NavigationMapData, viewport: MapViewport) {
        guard let goal = mapData.goal else { return }
        let point = worldToView(x: goal.x, z: goal.z, size: size, viewport: viewport)
        let outer = CGRect(x: point.x - 7, y: point.y - 7, width: 14, height: 14)
        let inner = CGRect(x: point.x - 2, y: point.y - 2, width: 4, height: 4)
        context.stroke(Path(ellipseIn: outer), with: .color(.yellow), lineWidth: 2)
        context.fill(Path(ellipseIn: inner), with: .color(.yellow))
    }

    private func drawRobot(in context: inout GraphicsContext, size: CGSize, mapData: NavigationMapData, viewport: MapViewport) {
        guard let robot = mapData.robot else { return }
        let center = worldToView(x: robot.x, z: robot.z, size: size, viewport: viewport)
        let radius: CGFloat = 6
        context.fill(Path(ellipseIn: CGRect(x: center.x - radius, y: center.y - radius, width: radius * 2, height: radius * 2)),
                     with: .color(.orange))

        let tipWorldX = robot.x + robot.fx * 0.28
        let tipWorldZ = robot.z + robot.fz * 0.28
        let tip = worldToView(x: tipWorldX, z: tipWorldZ, size: size, viewport: viewport)
        var heading = Path()
        heading.move(to: center)
        heading.addLine(to: tip)
        context.stroke(heading, with: .color(.yellow), lineWidth: 2.5)
    }

    private func drawAxes(in context: inout GraphicsContext, size: CGSize) {
        let origin = CGPoint(x: 30, y: size.height - 30)
        var axis = Path()
        axis.move(to: origin)
        axis.addLine(to: CGPoint(x: origin.x + 44, y: origin.y))
        axis.move(to: origin)
        axis.addLine(to: CGPoint(x: origin.x, y: origin.y - 44))
        context.stroke(axis, with: .color(Color.white.opacity(0.75)), lineWidth: 2)

        let xLabel = Text("X+")
            .font(.system(size: 11, weight: .semibold, design: .rounded))
        let zLabel = Text("Z+")
            .font(.system(size: 11, weight: .semibold, design: .rounded))
        context.draw(xLabel, at: CGPoint(x: origin.x + 55, y: origin.y), anchor: .leading)
        context.draw(zLabel, at: CGPoint(x: origin.x, y: origin.y - 55), anchor: .bottom)
    }

    private func drawHoverHighlight(in context: inout GraphicsContext,
                                    size: CGSize,
                                    mapData: NavigationMapData,
                                    viewport: MapViewport,
                                    hoverInfo: MapHoverInfo) {
        let cell = GridCellRecord(x: hoverInfo.cellX, z: hoverInfo.cellZ)
        let side = max(1.0, mapData.resolution / viewport.metersPerPixel)
        let rect = cellRect(cell, size: size, mapData: mapData, viewport: viewport, pixelSize: side)
        context.stroke(Path(rect), with: .color(.yellow.opacity(0.95)), lineWidth: 2)
    }

    private func worldToView(x: Double, z: Double, size: CGSize, viewport: MapViewport) -> CGPoint {
        let px = size.width / 2 + CGFloat((x - viewport.centerX) / viewport.metersPerPixel)
        let py = size.height / 2 - CGFloat((z - viewport.centerZ) / viewport.metersPerPixel)
        return CGPoint(x: px, y: py)
    }

    private func cellRect(_ cell: GridCellRecord, size: CGSize, mapData: NavigationMapData, viewport: MapViewport, pixelSize: Double) -> CGRect {
        let centerWorldX = (Double(cell.x) + 0.5) * mapData.resolution
        let centerWorldZ = (Double(cell.z) + 0.5) * mapData.resolution
        let center = worldToView(x: centerWorldX, z: centerWorldZ, size: size, viewport: viewport)
        let side = CGFloat(pixelSize)
        return CGRect(x: center.x - side / 2, y: center.y - side / 2, width: side, height: side)
    }

    private func viewToWorld(point: CGPoint, size: CGSize, viewport: MapViewport) -> (x: Double, z: Double) {
        let worldX = viewport.centerX + (Double(point.x) - Double(size.width / 2.0)) * viewport.metersPerPixel
        let worldZ = viewport.centerZ - (Double(point.y) - Double(size.height / 2.0)) * viewport.metersPerPixel
        return (worldX, worldZ)
    }

    private func drawingRect(in containerSize: CGSize) -> CGRect {
        CGRect(x: drawingInset, y: drawingInset, width: max(1, containerSize.width - drawingInset * 2), height: max(1, containerSize.height - drawingInset * 2))
    }

    private func resolvedViewport(for mapData: NavigationMapData) -> MapViewport {
        viewport ?? MapViewport(centerX: mapData.viewCenterX, centerZ: mapData.viewCenterZ, metersPerPixel: mapData.metersPerPixel)
    }

    private func syncViewportFromBackend(mapData: NavigationMapData, force: Bool) {
        guard force || !hasUserAdjustedView || viewport == nil else { return }
        viewport = MapViewport(centerX: mapData.viewCenterX, centerZ: mapData.viewCenterZ, metersPerPixel: mapData.metersPerPixel)
    }

    private func resetViewport(using mapData: NavigationMapData) {
        hasUserAdjustedView = false
        viewport = MapViewport(centerX: mapData.viewCenterX, centerZ: mapData.viewCenterZ, metersPerPixel: mapData.metersPerPixel)
        dragStartViewport = nil
        zoomStartViewport = nil
    }

    private func stepZoom(using mapData: NavigationMapData, factor: Double) {
        hasUserAdjustedView = true
        let current = resolvedViewport(for: mapData)
        viewport = current.zoomed(by: factor)
    }

    private func zoomLabel(for mapData: NavigationMapData) -> String {
        let current = resolvedViewport(for: mapData)
        let ratio = mapData.metersPerPixel / current.metersPerPixel
        return String(format: "%.0f%%", ratio * 100.0)
    }

    private func setHoverInfo(_ info: MapHoverInfo?) {
        hoverInfo = info
        onHoverInfo(info)
    }

    private func makeHoverInfo(worldX: Double, worldZ: Double, mapData: NavigationMapData) -> MapHoverInfo {
        let cellX = Int(floor(worldX / mapData.resolution))
        let cellZ = Int(floor(worldZ / mapData.resolution))
        let cell = GridCellRecord(x: cellX, z: cellZ)
        let kind: MapCellKind
        if mapData.occupiedCells.contains(cell) {
            kind = .occupied
        } else if mapData.inflatedCells.contains(cell) {
            kind = .inflated
        } else if mapData.freeCells.contains(cell) {
            kind = .free
        } else {
            kind = .unknown
        }
        return MapHoverInfo(worldX: worldX, worldZ: worldZ, cellX: cellX, cellZ: cellZ, cellKind: kind)
    }
}

struct LegendRow: View {
    let color: Color
    let label: String

    var body: some View {
        HStack(spacing: 8) {
            RoundedRectangle(cornerRadius: 3, style: .continuous)
                .fill(color)
                .frame(width: 18, height: 10)
            Text(label)
                .font(.caption)
                .foregroundStyle(.secondary)
            Spacer(minLength: 0)
        }
    }
}

private struct MapScaleTag: View {
    let lengthMeters: Double
    let metersPerPixel: Double

    private var barWidth: CGFloat {
        CGFloat(lengthMeters / metersPerPixel)
    }

    var body: some View {
        VStack(alignment: .trailing, spacing: 6) {
            Text("\(lengthMeters.formatted(.number.precision(.fractionLength(0...1)))) m 标尺")
                .font(.caption.weight(.semibold))
            Rectangle()
                .fill(Color.white.opacity(0.95))
                .frame(width: max(36, min(140, barWidth)), height: 3)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }
}

private struct MapViewportControls: View {
    let zoomLabel: String
    let onReset: () -> Void
    let onZoomIn: () -> Void
    let onZoomOut: () -> Void

    var body: some View {
        HStack(spacing: 8) {
            Button(action: onZoomOut) {
                Image(systemName: "minus")
            }
            .buttonStyle(.borderless)

            Text(zoomLabel)
                .font(.caption.monospacedDigit())
                .frame(minWidth: 48)

            Button(action: onZoomIn) {
                Image(systemName: "plus")
            }
            .buttonStyle(.borderless)

            Divider()
                .frame(height: 16)

            Button("重置", action: onReset)
                .buttonStyle(.borderless)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 10)
        .background(.ultraThinMaterial, in: Capsule())
    }
}

private struct MapViewport {
    var centerX: Double
    var centerZ: Double
    var metersPerPixel: Double

    func panned(dx: CGFloat, dy: CGFloat) -> MapViewport {
        MapViewport(
            centerX: centerX - Double(dx) * metersPerPixel,
            centerZ: centerZ + Double(dy) * metersPerPixel,
            metersPerPixel: metersPerPixel
        )
    }

    func zoomed(by factor: Double) -> MapViewport {
        let clamped = min(max(metersPerPixel * factor, 0.0035), 0.18)
        return MapViewport(centerX: centerX, centerZ: centerZ, metersPerPixel: clamped)
    }
}
