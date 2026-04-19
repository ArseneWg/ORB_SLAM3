import AppKit
import Foundation

@MainActor
final class AppModel: ObservableObject {
    @Published var runtimeState: NavigationRuntimeState?
    @Published var latestImage: NSImage?
    @Published var latestRgbImage: NSImage?
    @Published var latestDepthImage: NSImage?
    @Published var mapData: NavigationMapData?
    @Published var logs: String = ""
    @Published var backendRunning = false

    @Published var fixedHeightMode = false
    @Published var fixedCameraHeightMeters = 0.30
    @Published var showInflation = true
    @Published var inflationRadiusCells = 3
    @Published var lookaheadMeters = 0.60
    @Published var localizationOnly = false
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

    let workspaceRoot = URL(fileURLWithPath: "/Users/xy/work/ORB_SLAM3", isDirectory: true)
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
    private var lastMapDataMTime: Date?
    private var lastStateMTime: Date?

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

    var mapDataURL: URL? {
        guard let path = runtimeState?.mapDataPath, !path.isEmpty else { return nil }
        return URL(fileURLWithPath: path)
    }

    var connectionSummary: String {
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

    func start() {
        if backendRunning { return }

        let process = Process()
        process.currentDirectoryURL = workspaceRoot
        process.executableURL = URL(fileURLWithPath: "/Users/xy/work/ORB_SLAM3/Examples/RGB-D/run_iphone_rgbd_orbslam3.sh")
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

        process.terminationHandler = { [weak self] _ in
            Task { @MainActor in
                self?.cleanupProcessState()
                self?.appendLog("Backend exited.\n")
            }
        }

        do {
            try process.run()
            self.process = process
            self.stdoutPipe = stdout
            self.stderrPipe = stderr
            self.backendRunning = true
            appendLog("Started backend in app mode.\n")
            startPolling()
            sendControl()
            refreshNow()
        } catch {
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

    func openWorkspaceRootInFinder() {
        NSWorkspace.shared.open(workspaceRoot)
    }

    func refreshNow() {
        pollFiles()
    }

    func clearLogs() {
        logs = ""
    }

    func updateMapHoverInfo(_ info: MapHoverInfo?) {
        mapHoverInfo = info
    }

    func syncSettingsFromRuntime() {
        guard let state = runtimeState else { return }
        fixedHeightMode = state.fixedHeightMode
        fixedCameraHeightMeters = state.fixedCameraHeightMeters
        showInflation = state.showInflation
        inflationRadiusCells = state.inflationRadiusCells
        lookaheadMeters = state.lookaheadMeters
        localizationOnly = state.localizationOnly
        viewMode = state.followRobot ? "follow" : (state.autoFit ? "overview" : "manual")
    }

    func sendControl() {
        controlRevision += 1
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
        pollTimer?.invalidate()
        pollTimer = Timer.scheduledTimer(withTimeInterval: 0.35, repeats: true) { [weak self] _ in
            Task { @MainActor in
                self?.pollFiles()
            }
        }
    }

    private func stopPolling() {
        pollTimer?.invalidate()
        pollTimer = nil
    }

    private func pollFiles() {
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
        latestImage = NSImage(contentsOf: imageURL)
        lastImageLoadedAt = modified
    }

    private func loadPanelImagesIfNeeded() {
        loadImage(at: latestRgbURL, lastModified: &lastRgbImageMTime, assign: { latestRgbImage = $0 })
        loadImage(at: latestDepthURL, lastModified: &lastDepthImageMTime, assign: { latestDepthImage = $0 })
    }

    private func loadMapDataIfNeeded() {
        guard let url = mapDataURL,
              let attributes = try? FileManager.default.attributesOfItem(atPath: url.path),
              let modified = attributes[.modificationDate] as? Date else { return }
        if let lastMapDataMTime, modified <= lastMapDataMTime { return }
        lastMapDataMTime = modified

        do {
            let data = try Data(contentsOf: url)
            mapData = try JSONDecoder().decode(NavigationMapData.self, from: data)
        } catch {
            appendLog("Failed to decode map data: \(error.localizedDescription)\n")
        }
    }

    private func loadImage(at url: URL?, lastModified: inout Date?, assign: (NSImage?) -> Void) {
        guard let url,
              let attributes = try? FileManager.default.attributesOfItem(atPath: url.path),
              let modified = attributes[.modificationDate] as? Date else { return }
        if let lastModified, modified <= lastModified { return }
        lastModified = modified
        assign(NSImage(contentsOf: url))
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

    private func appendLog(_ text: String) {
        logs.append(text)
        trimLogs()
    }

    private func trimLogs() {
        if logs.count > 18000 {
            logs = String(logs.suffix(18000))
        }
    }
}
