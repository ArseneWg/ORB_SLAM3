import SwiftUI

@main
struct ORBNavDeskApp: App {
    @StateObject private var model = AppModel()

    var body: some Scene {
        WindowGroup("ORB Nav Desk") {
            ContentView(model: model)
        }
        .defaultSize(width: 1600, height: 980)
        .windowResizability(.contentMinSize)

        Window("诊断面板", id: "diagnostics") {
            ConsolePanel(model: model)
        }
        .defaultSize(width: 920, height: 620)
        .windowResizability(.contentMinSize)
        .commands {
            SidebarCommands()
        }
    }
}
