import CoreGraphics
import Foundation

struct NavigationRuntimeState: Codable {
    let timestamp: String
    let connected: Bool
    let trackingState: String
    let hasPose: Bool
    let floorY: Double
    let localizationOnly: Bool
    let fixedHeightMode: Bool
    let fixedCameraHeightMeters: Double
    let inflationRadiusCells: Int
    let showInflation: Bool
    let lookaheadMeters: Double
    let orbDistanceMeters: Double
    let phoneDistanceMeters: Double
    let scaleRatio: Double
    let hasGoal: Bool
    let pathValid: Bool
    let hasWaypoint: Bool
    let waypointDistanceMeters: Double
    let headingErrorDegrees: Double
    let compositeWidth: Int
    let compositeHeight: Int
    let mapRectX: Int
    let mapRectY: Int
    let mapRectWidth: Int
    let mapRectHeight: Int
    let viewCenterX: Double
    let viewCenterZ: Double
    let metersPerPixel: Double
    let followRobot: Bool
    let autoFit: Bool
    let latestImagePath: String
    let latestRgbPath: String
    let latestDepthPath: String
    let latestMapPath: String
    let mapDataPath: String
    let guidancePath: String

    var compositeSize: CGSize {
        CGSize(width: compositeWidth, height: compositeHeight)
    }

    var mapRect: CGRect {
        CGRect(x: mapRectX, y: mapRectY, width: mapRectWidth, height: mapRectHeight)
    }
}

struct ControlEnvelope: Codable {
    var revision: Int
    var viewMode: String?
    var clearGoal: Bool?
    var saveSnapshot: Bool?
    var quit: Bool?
    var fixedHeightMode: Bool?
    var fixedCameraHeightMeters: Double?
    var inflationRadiusCells: Int?
    var showInflation: Bool?
    var lookaheadMeters: Double?
    var localizationOnly: Bool?
    var setGoal: Bool?
    var goalWorldX: Double?
    var goalWorldZ: Double?
}
