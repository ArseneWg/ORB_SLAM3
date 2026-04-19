import ARKit
import RealityKit
import SwiftUI

struct ARViewContainer: UIViewRepresentable {
    @EnvironmentObject private var sessionModel: SessionModel

    func makeCoordinator() -> Coordinator {
        Coordinator(sessionModel: sessionModel)
    }

    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero, cameraMode: .ar, automaticallyConfigureSession: false)
        arView.debugOptions.insert(.showSceneUnderstanding)
        arView.debugOptions.insert(.showStatistics)

        let coachingOverlay = ARCoachingOverlayView()
        coachingOverlay.session = arView.session
        coachingOverlay.goal = .tracking
        coachingOverlay.activatesAutomatically = true
        coachingOverlay.translatesAutoresizingMaskIntoConstraints = false
        arView.addSubview(coachingOverlay)

        NSLayoutConstraint.activate([
            coachingOverlay.topAnchor.constraint(equalTo: arView.topAnchor),
            coachingOverlay.leadingAnchor.constraint(equalTo: arView.leadingAnchor),
            coachingOverlay.trailingAnchor.constraint(equalTo: arView.trailingAnchor),
            coachingOverlay.bottomAnchor.constraint(equalTo: arView.bottomAnchor),
        ])

        context.coordinator.attach(to: arView)
        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {
        context.coordinator.attach(to: uiView)
    }

    @MainActor
    final class Coordinator {
        private let sessionModel: SessionModel

        init(sessionModel: SessionModel) {
            self.sessionModel = sessionModel
        }

        func attach(to arView: ARView) {
            sessionModel.attach(to: arView)
        }
    }
}
