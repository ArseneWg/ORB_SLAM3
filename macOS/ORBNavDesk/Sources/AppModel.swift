import AppKit
import Foundation

enum DepthComparisonArtifact {
    case modelDepth
    case depthDiff
}

private struct LoadedImageData: Sendable {
    let modified: Date
    let data: Data
}

@MainActor
final class AppModel: ObservableObject {
    nonisolated private static let backendScriptRelativePath = "Examples/RGB-D/run_iphone_rgbd_orbslam3.sh"
    nonisolated private static let workspaceMarkers = [
        backendScriptRelativePath,
        "Vocabulary/ORBvoc.txt",
    ]

    @Published var runtimeState: NavigationRuntimeState?
    @Published var latestImage: NSImage?
    @Published var latestRgbImage: NSImage?
    @Published var latestDepthImage: NSImage?
    @Published var latestDepthDiffImage: NSImage?
    @Published var mapData: NavigationMapData?
    @Published var logs: String = ""
    @Published var backendRunning = false
    @Published var backendIssue: String?

    @Published var fixedHeightMode = false
    @Published var fixedCameraHeightMeters = 0.30
    @Published var showInflation = true
    @Published var inflationRadiusCells = 3
    @Published var lookaheadMeters = 0.60
    @Published var localizationOnly = false
    @Published var depthSourceMode: DepthSourceModeOption = .sensor
    @Published var enableRgbPreview = false
    @Published var enableDepthPreview = false
    @Published var enableDepthComparison = false
    @Published var enableDepthDiffPreview = false
    @Published var viewMode: String = "follow"
    @Published var mapShowGrid = true
    @Published var mapShowFreeCells = true
    @Published var mapShowOccupiedCells = true
    @Published var mapShowInflationOverlay = true
    @Published var mapShowTrajectory = true
    @Published var mapShowPlannedPath = true
    @Published var mapShowRobot = true
    @Published var mapShowAxes = true
    @Published var mapHoverInfo: MapHoverInfo?

    @Published var lastStateLoadedAt: Date?
    @Published var lastImageLoadedAt: Date?

    let workspaceRoot: URL
    let controlPath = URL(fileURLWithPath: "/tmp/iphone_rgbd_nav_control.json")
    let statePath = URL(fileURLWithPath: "/tmp/iphone_rgbd_nav_state.json")

    private var process: Process?
    private var stdoutPipe: Pipe?
    private var stderrPipe: Pipe?
    private var pollTimer: Timer?
    private var controlRevision = 0
    private var lastImageMTime: Date?
    private var lastRgbImageMTime: Date?
    private var lastDepthImageMTime: Date?
    private var lastDepthDiffImageMTime: Date?
    private var lastMapDataMTime: Date?
    private var lastStateMTime: Date?
    private var panelLoadTask: Task<Void, Never>?
    private var mapLoadTask: Task<Void, Never>?
    private var lastSettingsControlRevision = -1
    private let stateFreshnessWindow: TimeInterval = 3.0
    private let backendAvailabilityCheckInterval: TimeInterval = 1.0
    private var lastBackendAvailabilityCheckAt = Date.distantPast

    init() {
        workspaceRoot = Self.resolveWorkspaceRoot()
            ?? URL(fileURLWithPath: FileManager.default.currentDirectoryPath, isDirectory: true)
    }

    var latestSnapshotURL: URL? {
        guard let path = runtimeState?.latestImagePath, !path.isEmpty else { return nil }
        return URL(fileURLWithPath: path)
    }

    var guidanceURL: URL? {
        guard let path = runtimeState?.guidancePath, !path.isEmpty else { return nil }
        return URL(fileURLWithPath: path)
    }

    var latestRgbURL: URL? {
        guard let path = runtimeState?.latestRgbPath, !path.isEmpty else { return nil }
        return URL(fileURLWithPath: path)
    }

    var latestDepthURL: URL? {
        guard let path = runtimeState?.latestDepthPath, !path.isEmpty else { return nil }
        return URL(fileURLWithPath: path)
    }

    var latestModelDepthURL: URL? {
        guard let path = runtimeState?.latestModelDepthPath, !path.isEmpty else { return nil }
        return URL(fileURLWithPath: path)
    }

    var latestDepthDiffURL: URL? {
        guard let path = runtimeState?.latestDepthDiffPath, !path.isEmpty else { return nil }
        return URL(fileURLWithPath: path)
    }

    var mapDataURL: URL? {
        guard let path = runtimeState?.mapDataPath, !path.isEmpty else { return nil }
        return URL(fileURLWithPath: path)
    }

    private var backendScriptURL: URL? {
        let candidate = workspaceRoot.appendingPathComponent(Self.backendScriptRelativePath)
        guard FileManager.default.fileExists(atPath: candidate.path) else { return nil }
        return candidate
    }

    var connectionSummary: String {
        if let backendIssue, !backendIssue.isEmpty {
            return backendIssue
        }
        if backendRunning {
            return runtimeState?.connected == true ? "iPhone 已连接" : "等待 iPhone 推流"
        }
        return "后端未运行"
    }

    var trackingSummary: String {
        runtimeState?.trackingState ?? "NO_IMAGES_YET"
    }

    var operationModeSummary: String {
        fixedHeightMode ? "车载固定高度" : "手持自适应"
    }

    var depthSourceSummary: String {
        switch runtimeState?.depthSourceMode.flatMap(DepthSourceModeOption.init(rawValue:)) ?? depthSourceMode {
        case .sensor:
            return "传感器"
        case .modelMap:
            return "地图模型"
        case .modelFull:
            return "全链模型"
        }
    }

    var activeDepthSourceSummary: String {
        let slam: String
        switch runtimeState?.activeSlamDepthSource {
        case "model":
            slam = "SLAM 模型"
        case "none":
            slam = "SLAM 未更新"
        default:
            slam = "SLAM 传感器"
        }
        let map: String
        switch runtimeState?.activeMapDepthSource {
        case "model":
            map = "地图 模型"
        case "none":
            map = "地图 未更新"
        default:
            map = "地图 传感器"
        }
        return "\(slam) / \(map)"
    }

    var scaleRatioSummary: String {
        guard let ratio = runtimeState?.scaleRatio, ratio > 0 else { return "--" }
        return String(format: "%.3fx", ratio)
    }

    var waypointSummary: String {
        guard let state = runtimeState else { return "未设置" }
        if state.hasWaypoint {
            return String(format: "%.2f m", state.waypointDistanceMeters)
        }
        if state.hasGoal {
            return state.pathValid ? "路径就绪" : "等待路径"
        }
        return "未设置"
    }

    var shouldShowRgbPreview: Bool {
        enableRgbPreview
    }

    var shouldShowDepthPreview: Bool {
        enableDepthPreview
    }

    var shouldShowDepthDiffPreview: Bool {
        enableDepthComparison && enableDepthDiffPreview
    }

    var hasVisiblePreviewPanels: Bool {
        shouldShowRgbPreview || shouldShowDepthPreview || shouldShowDepthDiffPreview
    }

    func start() {
        if backendRunning { return }
        backendIssue = nil

        guard let backendScriptURL else {
            backendIssue = "未找到工作目录"
            appendLog("无法启动后端：未找到 ORB_SLAM3 工作目录。请从仓库目录启动 ORB Nav Desk，或设置 ORB_SLAM3_WORKSPACE_ROOT。\n")
            return
        }

        if isPortInUse(9000) {
            if canAdoptExistingBackend() {
                backendRunning = true
                backendIssue = nil
                appendLog("检测到已有 rgbd_iphone_stream 在运行，已接管现有后端。\n")
                startPolling()
                refreshNow()
                return
            }
            backendIssue = "端口 9000 被占用"
            appendLog("无法启动后端：端口 9000 已被其他 rgbd_iphone_stream 实例占用。请先停止已有后端。\n")
            refreshNow()
            return
        }

        let process = Process()
        process.currentDirectoryURL = workspaceRoot
        process.executableURL = backendScriptURL
        process.arguments = ["9000", "iphone_rgbd_orbslam3"]

        var environment = ProcessInfo.processInfo.environment
        environment["ORB_NAV_APP_MODE"] = "1"
        environment["ORB_NAV_CONTROL_PATH"] = controlPath.path
        environment["ORB_NAV_STATE_PATH"] = statePath.path
        process.environment = environment

        let stdout = Pipe()
        let stderr = Pipe()
        process.standardOutput = stdout
        process.standardError = stderr

        stdout.fileHandleForReading.readabilityHandler = { [weak self] handle in
            let data = handle.availableData
            guard !data.isEmpty, let text = String(data: data, encoding: .utf8) else { return }
            Task { @MainActor in
                self?.appendLog(text)
            }
        }
        stderr.fileHandleForReading.readabilityHandler = { [weak self] handle in
            let data = handle.availableData
            guard !data.isEmpty, let text = String(data: data, encoding: .utf8) else { return }
            Task { @MainActor in
                self?.appendLog(text)
            }
        }

        process.terminationHandler = { [weak self] terminatedProcess in
            Task { @MainActor in
                let status = terminatedProcess.terminationStatus
                self?.cleanupProcessState()
                if status != 0 {
                    self?.backendIssue = "后端启动失败"
                    self?.appendLog("Backend exited with status \(status).\n")
                } else {
                    self?.appendLog("Backend exited.\n")
                }
            }
        }

        do {
            try process.run()
            self.process = process
            self.stdoutPipe = stdout
            self.stderrPipe = stderr
            self.backendRunning = true
            self.backendIssue = nil
            appendLog("Started backend in app mode.\n")
            startPolling()
            sendControl()
            refreshNow()
        } catch {
            backendIssue = "后端启动失败"
            appendLog("Failed to start backend: \(error.localizedDescription)\n")
        }
    }

    func stop() {
        guard backendRunning else { return }
        sendOneShot { envelope in
            envelope.quit = true
        }
        process?.terminate()
        cleanupProcessState()
    }

    func toggleStartStop() {
        backendRunning ? stop() : start()
    }

    func clearGoal() {
        sendOneShot { envelope in
            envelope.clearGoal = true
        }
    }

    func saveSnapshot() {
        sendOneShot { envelope in
            envelope.saveSnapshot = true
        }
    }

    func setGoal(worldX: Double, worldZ: Double) {
        sendOneShot { envelope in
            envelope.setGoal = true
            envelope.goalWorldX = worldX
            envelope.goalWorldZ = worldZ
        }
    }

    func openLatestSnapshot() {
        guard let url = latestSnapshotURL else { return }
        NSWorkspace.shared.open(url)
    }

    func revealLatestSnapshotInFinder() {
        guard let url = latestSnapshotURL else { return }
        NSWorkspace.shared.activateFileViewerSelecting([url])
    }

    func revealGuidanceFileInFinder() {
        guard let url = guidanceURL else { return }
        NSWorkspace.shared.activateFileViewerSelecting([url])
    }

    func openDepthComparisonArtifact(_ kind: DepthComparisonArtifact) {
        let url: URL?
        switch kind {
        case .modelDepth:
            url = latestModelDepthURL
        case .depthDiff:
            url = latestDepthDiffURL
        }
        guard let url else { return }
        NSWorkspace.shared.open(url)
    }

    func revealDepthComparisonArtifact(_ kind: DepthComparisonArtifact) {
        let url: URL?
        switch kind {
        case .modelDepth:
            url = latestModelDepthURL
        case .depthDiff:
            url = latestDepthDiffURL
        }
        guard let url else { return }
        NSWorkspace.shared.activateFileViewerSelecting([url])
    }

    func openWorkspaceRootInFinder() {
        NSWorkspace.shared.open(workspaceRoot)
    }

    func refreshNow() {
        refreshBackendAvailabilityIfNeeded(force: true)
        pollFiles()
    }

    func bootstrap() {
        startPolling()
        refreshNow()
    }

    func clearLogs() {
        logs = ""
    }

    func updateMapHoverInfo(_ info: MapHoverInfo?) {
        mapHoverInfo = info
    }

    func syncSettingsFromRuntime() {
        guard let state = runtimeState else { return }
        let appliedRevision = state.appliedControlRevision ?? -1
        let runtimeCaughtUp = appliedRevision >= lastSettingsControlRevision

        if runtimeCaughtUp {
            fixedHeightMode = state.fixedHeightMode
            fixedCameraHeightMeters = state.fixedCameraHeightMeters
            showInflation = state.showInflation
            inflationRadiusCells = state.inflationRadiusCells
            lookaheadMeters = state.lookaheadMeters
            localizationOnly = state.localizationOnly
            if let mode = state.depthSourceMode.flatMap(DepthSourceModeOption.init(rawValue:)) {
                depthSourceMode = mode
            }
            enableRgbPreview = state.enableRgbPreview
            enableDepthPreview = state.enableDepthPreview
            enableDepthComparison = state.enableDepthComparison
            enableDepthDiffPreview = state.enableDepthDiffPreview
        }
        viewMode = state.followRobot ? "follow" : (state.autoFit ? "overview" : "manual")
    }

    func sendControl() {
        controlRevision += 1
        lastSettingsControlRevision = controlRevision
        let payload = ControlEnvelope(
            revision: controlRevision,
            viewMode: viewMode,
            clearGoal: false,
            saveSnapshot: false,
            quit: false,
            fixedHeightMode: fixedHeightMode,
            fixedCameraHeightMeters: fixedCameraHeightMeters,
            inflationRadiusCells: inflationRadiusCells,
            showInflation: showInflation,
            lookaheadMeters: lookaheadMeters,
            localizationOnly: localizationOnly,
            depthSourceMode: depthSourceMode.rawValue,
            enableRgbPreview: enableRgbPreview,
            enableDepthPreview: enableDepthPreview,
            enableDepthComparison: enableDepthComparison,
            enableDepthDiffPreview: enableDepthDiffPreview,
            setGoal: false,
            goalWorldX: nil,
            goalWorldZ: nil
        )
        writeControl(payload)
    }

    private func sendOneShot(_ mutate: (inout ControlEnvelope) -> Void) {
        controlRevision += 1
        var payload = ControlEnvelope(
            revision: controlRevision,
            viewMode: viewMode,
            clearGoal: false,
            saveSnapshot: false,
            quit: false,
            fixedHeightMode: fixedHeightMode,
            fixedCameraHeightMeters: fixedCameraHeightMeters,
            inflationRadiusCells: inflationRadiusCells,
            showInflation: showInflation,
            lookaheadMeters: lookaheadMeters,
            localizationOnly: localizationOnly,
            depthSourceMode: depthSourceMode.rawValue,
            enableRgbPreview: enableRgbPreview,
            enableDepthPreview: enableDepthPreview,
            enableDepthComparison: enableDepthComparison,
            enableDepthDiffPreview: enableDepthDiffPreview,
            setGoal: false,
            goalWorldX: nil,
            goalWorldZ: nil
        )
        mutate(&payload)
        writeControl(payload)
    }

    private func writeControl(_ payload: ControlEnvelope) {
        do {
            let data = try JSONEncoder().encode(payload)
            try data.write(to: controlPath, options: .atomic)
        } catch {
            appendLog("Failed to write control file: \(error.localizedDescription)\n")
        }
    }

    private func startPolling() {
        if pollTimer != nil { return }
        pollTimer = Timer.scheduledTimer(withTimeInterval: 0.18, repeats: true) { [weak self] _ in
            Task { @MainActor in
                self?.pollFiles()
            }
        }
    }

    private func stopPolling() {
        pollTimer?.invalidate()
        pollTimer = nil
        panelLoadTask?.cancel()
        panelLoadTask = nil
        mapLoadTask?.cancel()
        mapLoadTask = nil
    }

    private func pollFiles() {
        refreshBackendAvailabilityIfNeeded()
        loadStateIfNeeded()
        loadImageIfNeeded()
        loadPanelImagesIfNeeded()
        loadMapDataIfNeeded()
    }

    private func loadStateIfNeeded() {
        guard let attributes = try? FileManager.default.attributesOfItem(atPath: statePath.path),
              let modified = attributes[.modificationDate] as? Date else { return }
        if let lastStateMTime, modified <= lastStateMTime { return }
        lastStateMTime = modified

        do {
            let data = try Data(contentsOf: statePath)
            let state = try JSONDecoder().decode(NavigationRuntimeState.self, from: data)
            runtimeState = state
            lastStateLoadedAt = modified
            refreshBackendAvailability(using: modified)
            syncSettingsFromRuntime()
        } catch {
            appendLog("Failed to decode runtime state: \(error.localizedDescription)\n")
        }
    }

    private func loadImageIfNeeded() {
        guard let imagePath = runtimeState?.latestImagePath else { return }
        let imageURL = URL(fileURLWithPath: imagePath)
        guard let attributes = try? FileManager.default.attributesOfItem(atPath: imageURL.path),
              let modified = attributes[.modificationDate] as? Date else { return }
        if let lastImageMTime, modified <= lastImageMTime { return }
        lastImageMTime = modified
        latestImage = nil
        lastImageLoadedAt = modified
    }

    private func loadPanelImagesIfNeeded() {
        if !shouldShowRgbPreview {
            latestRgbImage = nil
            lastRgbImageMTime = nil
        }
        if !shouldShowDepthPreview {
            latestDepthImage = nil
            lastDepthImageMTime = nil
        }
        if !shouldShowDepthDiffPreview {
            latestDepthDiffImage = nil
            lastDepthDiffImageMTime = nil
        }

        guard panelLoadTask == nil else { return }

        let rgbURL = shouldShowRgbPreview ? latestRgbURL : nil
        let depthURL = shouldShowDepthPreview ? latestDepthURL : nil
        let diffURL = shouldShowDepthDiffPreview ? latestDepthDiffURL : nil
        let lastRgb = lastRgbImageMTime
        let lastDepth = lastDepthImageMTime
        let lastDiff = lastDepthDiffImageMTime

        panelLoadTask = Task.detached(priority: .utility) { [rgbURL, depthURL, diffURL, lastRgb, lastDepth, lastDiff] in
            let rgbResult = Self.loadImageDataIfNeeded(at: rgbURL, previousModified: lastRgb)
            let depthResult = Self.loadImageDataIfNeeded(at: depthURL, previousModified: lastDepth)
            let diffResult = Self.loadImageDataIfNeeded(at: diffURL, previousModified: lastDiff)

            await MainActor.run {
                if let rgbResult {
                    self.lastRgbImageMTime = rgbResult.modified
                    if self.shouldShowRgbPreview {
                        self.latestRgbImage = NSImage(data: rgbResult.data)
                    }
                }
                if let depthResult {
                    self.lastDepthImageMTime = depthResult.modified
                    if self.shouldShowDepthPreview {
                        self.latestDepthImage = NSImage(data: depthResult.data)
                    }
                }
                if let diffResult {
                    self.lastDepthDiffImageMTime = diffResult.modified
                    if self.shouldShowDepthDiffPreview {
                        self.latestDepthDiffImage = NSImage(data: diffResult.data)
                    }
                }
                self.panelLoadTask = nil
            }
        }
    }

    private func loadMapDataIfNeeded() {
        guard mapLoadTask == nil, let url = mapDataURL else { return }
        let lastModified = lastMapDataMTime

        mapLoadTask = Task.detached(priority: .utility) { [url, lastModified] in
            do {
                guard let attributes = try? FileManager.default.attributesOfItem(atPath: url.path),
                      let modified = attributes[.modificationDate] as? Date else {
                    await MainActor.run { self.mapLoadTask = nil }
                    return
                }
                if let lastModified, modified <= lastModified {
                    await MainActor.run { self.mapLoadTask = nil }
                    return
                }

                let data = try Data(contentsOf: url)
                let decoded = try JSONDecoder().decode(NavigationMapData.self, from: data)
                await MainActor.run {
                    self.lastMapDataMTime = modified
                    self.mapData = decoded
                    self.mapLoadTask = nil
                }
            } catch {
                await MainActor.run {
                    self.appendLog("Failed to decode map data: \(error.localizedDescription)\n")
                    self.mapLoadTask = nil
                }
            }
        }
    }

    nonisolated private static func loadImageDataIfNeeded(at url: URL?, previousModified: Date?) -> LoadedImageData? {
        guard let url,
              let attributes = try? FileManager.default.attributesOfItem(atPath: url.path),
              let modified = attributes[.modificationDate] as? Date else { return nil }
        if let previousModified, modified <= previousModified { return nil }
        guard let data = try? Data(contentsOf: url) else { return nil }
        return LoadedImageData(modified: modified, data: data)
    }

    private func cleanupProcessState() {
        stdoutPipe?.fileHandleForReading.readabilityHandler = nil
        stderrPipe?.fileHandleForReading.readabilityHandler = nil
        stdoutPipe = nil
        stderrPipe = nil
        process = nil
        backendRunning = false
        stopPolling()
    }

    private func canAdoptExistingBackend() -> Bool {
        guard isPortInUse(9000) else { return false }
        return isStateFresh()
    }

    private func isStateFresh() -> Bool {
        guard let attributes = try? FileManager.default.attributesOfItem(atPath: statePath.path),
              let modified = attributes[.modificationDate] as? Date else { return false }
        return Date().timeIntervalSince(modified) <= stateFreshnessWindow
    }

    private func refreshBackendAvailabilityIfNeeded(force: Bool = false) {
        if process?.isRunning == true {
            backendRunning = true
            if backendIssue == "后端启动失败" {
                backendIssue = nil
            }
            return
        }

        let now = Date()
        if !force, now.timeIntervalSince(lastBackendAvailabilityCheckAt) < backendAvailabilityCheckInterval {
            return
        }

        lastBackendAvailabilityCheckAt = now
        refreshBackendAvailability()
    }

    private func refreshBackendAvailability(using stateModified: Date? = nil) {
        let ownsRunningProcess = process?.isRunning == true
        let stateLooksFresh: Bool
        if let stateModified {
            stateLooksFresh = Date().timeIntervalSince(stateModified) <= stateFreshnessWindow
        } else {
            stateLooksFresh = isStateFresh()
        }
        let externalBackendAvailable = isPortInUse(9000) && stateLooksFresh
        backendRunning = ownsRunningProcess || externalBackendAvailable
        if backendRunning, backendIssue == "后端启动失败" {
            backendIssue = nil
        }
        if backendRunning {
            startPolling()
        }
    }

    nonisolated private func isPortInUse(_ port: Int) -> Bool {
        let process = Process()
        process.executableURL = URL(fileURLWithPath: "/usr/sbin/lsof")
        process.arguments = ["-nP", "-iTCP:\(port)", "-sTCP:LISTEN"]
        let pipe = Pipe()
        process.standardOutput = pipe
        process.standardError = Pipe()
        do {
            try process.run()
            process.waitUntilExit()
            let data = pipe.fileHandleForReading.readDataToEndOfFile()
            guard process.terminationStatus == 0 else { return false }
            let output = String(decoding: data, as: UTF8.self)
            return output.contains("rgbd_iphone_stream") || output.contains("run_iphone_rgbd_orbslam3.sh")
        } catch {
            return false
        }
    }

    private func appendLog(_ text: String) {
        logs.append(text)
        trimLogs()
    }

    private func trimLogs() {
        if logs.count > 18000 {
            logs = String(logs.suffix(18000))
        }
    }

    nonisolated private static func resolveWorkspaceRoot() -> URL? {
        let fileManager = FileManager.default
        var candidates: [URL] = []

        if let envPath = ProcessInfo.processInfo.environment["ORB_SLAM3_WORKSPACE_ROOT"],
           !envPath.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            candidates.append(URL(fileURLWithPath: envPath, isDirectory: true))
        }

        candidates.append(URL(fileURLWithPath: fileManager.currentDirectoryPath, isDirectory: true))
        candidates.append(URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .deletingLastPathComponent())
        candidates.append(Bundle.main.bundleURL)

        for candidate in candidates {
            if let root = findWorkspaceRoot(startingAt: candidate) {
                return root
            }
        }
        return nil
    }

    nonisolated private static func findWorkspaceRoot(startingAt startURL: URL) -> URL? {
        var candidate = startURL.standardizedFileURL

        while true {
            if isWorkspaceRoot(candidate) {
                return candidate
            }

            let parent = candidate.deletingLastPathComponent()
            if parent.path == candidate.path {
                return nil
            }
            candidate = parent
        }
    }

    nonisolated private static func isWorkspaceRoot(_ url: URL) -> Bool {
        let fileManager = FileManager.default
        return workspaceMarkers.allSatisfy { marker in
            fileManager.fileExists(atPath: url.appendingPathComponent(marker).path)
        }
    }
}
