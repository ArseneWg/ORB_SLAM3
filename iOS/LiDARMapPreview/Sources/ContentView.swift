import SwiftUI

struct ContentView: View {
    @EnvironmentObject private var sessionModel: SessionModel

    var body: some View {
        ZStack(alignment: .topLeading) {
            ARViewContainer()
                .ignoresSafeArea()

            VStack(alignment: .leading, spacing: 10) {
                Text("ORB-SLAM3 Monocular Stream")
                    .font(.headline)

                StatusRow(title: "Tracking", value: sessionModel.trackingText)
                StatusRow(title: "Mapping", value: sessionModel.mappingText)
                StatusRow(title: "Depth", value: sessionModel.depthText)
                StatusRow(title: "Camera", value: sessionModel.positionText)

                Text("Use the rear camera, move slowly in a wide arc, and keep walls, corners, and furniture in view.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
                    .fixedSize(horizontal: false, vertical: true)

                Divider()

                Text("Mac Stream")
                    .font(.subheadline.weight(.semibold))

                TextField("Mac IP", text: $sessionModel.streamHost)
                    .textFieldStyle(.roundedBorder)
                    .textInputAutocapitalization(.never)
                    .autocorrectionDisabled()

                HStack(spacing: 8) {
                    TextField("Port", text: $sessionModel.streamPort)
                        .textFieldStyle(.roundedBorder)
                        .keyboardType(.numberPad)

                    Button(sessionModel.isStreaming ? "Stop Stream" : "Start Stream") {
                        sessionModel.toggleStreaming()
                    }
                    .buttonStyle(.borderedProminent)
                }

                StatusRow(title: "Stream", value: sessionModel.streamStatus)
                StatusRow(title: "Sent", value: sessionModel.streamFrameCount)

                HStack(spacing: 12) {
                    Button("Restart Session") {
                        sessionModel.restartSession()
                    }
                    .buttonStyle(.borderedProminent)

                    Button("Recenter Stats") {
                        sessionModel.clearCounters()
                    }
                    .buttonStyle(.bordered)
                }
            }
            .padding(14)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 18))
            .padding()
        }
    }
}

private struct StatusRow: View {
    let title: String
    let value: String

    var body: some View {
        HStack(alignment: .firstTextBaseline, spacing: 8) {
            Text(title + ":")
                .font(.subheadline.weight(.semibold))
                .frame(width: 74, alignment: .leading)
            Text(value)
                .font(.subheadline.monospaced())
                .foregroundStyle(.secondary)
        }
    }
}
