import CoreGraphics
import Foundation

enum DepthSourceModeOption: String, CaseIterable, Identifiable {
    case sensor = "sensor"
    case modelMap = "model_map"
    case modelFull = "model_full"

    var id: String { rawValue }

    var title: String {
        switch self {
        case .sensor:
            return "传感器深度"
        case .modelMap:
            return "模型仅地图"
        case .modelFull:
            return "模型主链+地图"
        }
    }

    var note: String {
        switch self {
        case .sensor:
            return "SLAM 与导航图都使用 iPhone 传感器深度。"
        case .modelMap:
            return "SLAM 保持传感器深度，2D 地图改用经传感器对齐的模型深度。"
        case .modelFull:
            return "SLAM 与 2D 地图都改用经传感器对齐的模型深度。"
        }
    }
}

struct NavigationRuntimeState: Codable, Sendable {
    let timestamp: String
    let appliedControlRevision: Int?
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
    let depthSourceMode: String?
    let activeSlamDepthSource: String?
    let activeMapDepthSource: String?
    let depthSourceStatus: String?
    let enableRgbPreview: Bool
    let enableDepthPreview: Bool
    let enableDepthComparison: Bool
    let enableDepthDiffPreview: Bool
    let orbDistanceMeters: Double
    let phoneDistanceMeters: Double
    let scaleRatio: Double
    let depthModelEnabled: Bool
    let depthComparisonReady: Bool
    let depthComparisonStatus: String
    let depthComparisonValidSensorPixels: Int
    let depthComparisonValidOverlapPixels: Int
    let depthComparisonOverlapRatio: Double
    let depthComparisonAlignmentScale: Double
    let depthComparisonAlignmentOffset: Double
    let depthComparisonInferenceMs: Double
    let depthComparisonMaeMeters: Double
    let depthComparisonRmseMeters: Double
    let depthComparisonAbsRel: Double
    let depthComparisonBiasMeters: Double
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
    let latestModelDepthPath: String
    let latestDepthDiffPath: String
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

struct ControlEnvelope: Codable, Sendable {
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
    var depthSourceMode: String?
    var enableRgbPreview: Bool?
    var enableDepthPreview: Bool?
    var enableDepthComparison: Bool?
    var enableDepthDiffPreview: Bool?
    var setGoal: Bool?
    var goalWorldX: Double?
    var goalWorldZ: Double?
}
