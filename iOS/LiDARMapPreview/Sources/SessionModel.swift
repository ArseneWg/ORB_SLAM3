import ARKit
import RealityKit
import SwiftUI
import simd

@MainActor
final class SessionModel: NSObject, ObservableObject {
    @Published var trackingText = "starting"
    @Published var mappingText = "notAvailable"
    @Published var depthText = "unavailable"
    @Published var meshAnchorCount = 0
    @Published var positionText = "x 0.00  y 0.00  z 0.00"
    @Published var streamHost = {
        let saved = UserDefaults.standard.string(forKey: "LiDARMapPreview.streamHost")?
            .trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        return saved.isEmpty ? "192.168.0.106" : saved
    }()
    @Published var streamPort = {
        let saved = UserDefaults.standard.string(forKey: "LiDARMapPreview.streamPort")?
            .trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        return saved.isEmpty ? "9000" : saved
    }()
    @Published var streamStatus = "stopped"
    @Published var streamFrameCount = "0"
    @Published var isStreaming = false

    private weak var arView: ARView?
    private var hasStarted = false
    private var knownMeshAnchors = Set<UUID>()
    nonisolated private let streamClient = StreamClient()

    override init() {
        super.init()
        UIDevice.current.beginGeneratingDeviceOrientationNotifications()

        streamClient.onStatusChanged = { [weak self] status in
            self?.streamStatus = status
        }

        streamClient.onFrameCountChanged = { [weak self] count in
            self?.streamFrameCount = "\(count)"
        }
    }

    deinit {
        UIDevice.current.endGeneratingDeviceOrientationNotifications()
    }

    func attach(to arView: ARView) {
        if self.arView === arView {
            return
        }

        self.arView = arView
        arView.session.delegate = self

        if !hasStarted {
            startSession(reset: true)
        }

        startStreamingIfConfigured()
    }

    func restartSession() {
        startSession(reset: true)
    }

    func clearCounters() {
        knownMeshAnchors.removeAll()
        meshAnchorCount = 0
    }

    func toggleStreaming() {
        if isStreaming {
            streamClient.stop()
            isStreaming = false
            return
        }

        startStreamingIfConfigured(resetCounter: true)
    }

    private func startStreamingIfConfigured(resetCounter: Bool = false) {
        guard let port = UInt16(streamPort) else {
            streamStatus = "invalid port"
            return
        }

        let trimmedHost = streamHost.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmedHost.isEmpty else {
            streamStatus = "enter Mac IP"
            return
        }

        UserDefaults.standard.set(trimmedHost, forKey: "LiDARMapPreview.streamHost")
        UserDefaults.standard.set(streamPort, forKey: "LiDARMapPreview.streamPort")

        streamClient.start(host: trimmedHost, port: port)
        if resetCounter {
            streamFrameCount = "0"
        }
        isStreaming = true
    }

    private func startSession(reset: Bool) {
        guard let arView else { return }

        let configuration = ARWorldTrackingConfiguration()
        configuration.environmentTexturing = .automatic

        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            configuration.frameSemantics.insert(.smoothedSceneDepth)
        } else if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics.insert(.sceneDepth)
        }

        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) {
            configuration.sceneReconstruction = .meshWithClassification
        } else if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            configuration.sceneReconstruction = .mesh
        }

        let options: ARSession.RunOptions = reset ? [.resetTracking, .removeExistingAnchors] : []
        arView.session.run(configuration, options: options)
        hasStarted = true
        if reset {
            clearCounters()
        }
    }
}

extension SessionModel: ARSessionDelegate {
    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        streamClient.process(frame: frame)

        Task { @MainActor in
            self.trackingText = Self.describe(frame.camera.trackingState)
            self.mappingText = Self.describe(frame.worldMappingStatus)
            self.depthText = Self.describeDepth(frame)
            self.positionText = Self.describePosition(frame.camera.transform)
        }
    }

    nonisolated func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        Task { @MainActor in
            for anchor in anchors.compactMap({ $0 as? ARMeshAnchor }) {
                self.knownMeshAnchors.insert(anchor.identifier)
            }
            self.meshAnchorCount = self.knownMeshAnchors.count
        }
    }

    nonisolated func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        Task { @MainActor in
            for anchor in anchors.compactMap({ $0 as? ARMeshAnchor }) {
                self.knownMeshAnchors.insert(anchor.identifier)
            }
            self.meshAnchorCount = self.knownMeshAnchors.count
        }
    }

    nonisolated func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        Task { @MainActor in
            for anchor in anchors.compactMap({ $0 as? ARMeshAnchor }) {
                self.knownMeshAnchors.remove(anchor.identifier)
            }
            self.meshAnchorCount = self.knownMeshAnchors.count
        }
    }

    nonisolated func session(_ session: ARSession, didFailWithError error: Error) {
        Task { @MainActor in
            self.trackingText = "failed: \(error.localizedDescription)"
        }
    }

    nonisolated func sessionWasInterrupted(_ session: ARSession) {
        Task { @MainActor in
            self.trackingText = "interrupted"
        }
    }

    nonisolated func sessionInterruptionEnded(_ session: ARSession) {
        Task { @MainActor in
            self.trackingText = "restarting"
            self.startSession(reset: true)
        }
    }
}

private extension SessionModel {
    static func describe(_ state: ARCamera.TrackingState) -> String {
        switch state {
        case .normal:
            return "normal"
        case .notAvailable:
            return "notAvailable"
        case .limited(let reason):
            return "limited (\(describe(reason)))"
        }
    }

    static func describe(_ reason: ARCamera.TrackingState.Reason) -> String {
        switch reason {
        case .initializing:
            return "initializing"
        case .excessiveMotion:
            return "excessiveMotion"
        case .insufficientFeatures:
            return "insufficientFeatures"
        case .relocalizing:
            return "relocalizing"
        @unknown default:
            return "unknown"
        }
    }

    static func describe(_ status: ARFrame.WorldMappingStatus) -> String {
        switch status {
        case .notAvailable:
            return "notAvailable"
        case .limited:
            return "limited"
        case .extending:
            return "extending"
        case .mapped:
            return "mapped"
        @unknown default:
            return "unknown"
        }
    }

    static func describeDepth(_ frame: ARFrame) -> String {
        if frame.smoothedSceneDepth != nil {
            return "smoothedSceneDepth"
        }
        if frame.sceneDepth != nil {
            return "sceneDepth"
        }
        return "off (Mac Small V2)"
    }

    static func describePosition(_ transform: simd_float4x4) -> String {
        let translation = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        return String(
            format: "x %.2f  y %.2f  z %.2f",
            translation.x,
            translation.y,
            translation.z
        )
    }
}
