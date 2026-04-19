import Foundation

struct GridCellRecord: Codable, Hashable, Sendable {
    let x: Int
    let z: Int
}

struct WorldPointRecord: Codable, Hashable, Sendable {
    let x: Double
    let z: Double
}

struct RobotPoseRecord: Codable, Sendable {
    let x: Double
    let z: Double
    let fx: Double
    let fz: Double
}

struct GoalRecord: Codable, Sendable {
    let x: Double
    let z: Double
}

enum MapCellKind: String, Sendable {
    case free = "可通行"
    case occupied = "障碍物"
    case inflated = "避障缓冲"
    case unknown = "未知"
}

struct MapHoverInfo {
    let worldX: Double
    let worldZ: Double
    let cellX: Int
    let cellZ: Int
    let cellKind: MapCellKind
}

struct NavigationMapData: Codable, Sendable {
    let timestamp: String
    let resolution: Double
    let metersPerPixel: Double
    let viewCenterX: Double
    let viewCenterZ: Double
    let showInflation: Bool
    let inflationRadiusCells: Int
    let hasPose: Bool
    let robot: RobotPoseRecord?
    let goal: GoalRecord?
    let freeCells: [GridCellRecord]
    let occupiedCells: [GridCellRecord]
    let inflatedCells: [GridCellRecord]
    let trajectory: [WorldPointRecord]
    let path: [WorldPointRecord]

    var visibleWidthMetersEstimate: Double {
        metersPerPixel * 760.0
    }
}
