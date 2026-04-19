import SwiftUI

@main
struct LiDARMapPreviewApp: App {
    @StateObject private var sessionModel = SessionModel()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(sessionModel)
        }
    }
}
