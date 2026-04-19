/**
* Navigation-oriented iPhone RGB-D receiver for ORB-SLAM3.
*/

#include <arpa/inet.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ifaddrs.h>
#include <limits.h>
#include <net/if.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <ctime>
#include <unordered_map>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/freetype.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>

#ifdef __APPLE__
#include "../Monocular/coreml_depth_estimator.h"
#endif

using namespace std;
using boost::property_tree::ptree;

namespace {

constexpr char kMagic[] = {'L', 'D', 'R', '1'};
constexpr char kWindowName[] = "ORB-SLAM3 iPhone RGB-D Navigation";
constexpr int kDefaultFps = 10;
constexpr float kGridResolutionMeters = 0.05f;
constexpr float kMinDepthMeters = 0.15f;
constexpr float kMaxDepthMeters = 4.5f;
constexpr int kDepthStride = 6;
constexpr float kDepthDiffVisualMaxMeters = 1.00f;
constexpr float kFloorClearanceMeters = 0.05f;
constexpr float kObstacleHeightMeters = 1.10f;
constexpr float kAssumedCameraHeightMeters = 0.30f;
constexpr bool kLoadAtlasOnColdStart = false;

atomic<bool> gKeepRunning{true};

struct PacketHeader {
    uint64_t frameIndex = 0;
    double timestamp = 0.0;
    string displayOrientation = "unknown";
    int rgbWidth = 0;
    int rgbHeight = 0;
    int depthWidth = 0;
    int depthHeight = 0;
    int rgbSize = 0;
    int depthSize = 0;
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    array<float, 16> pose {};
    bool hasPose = false;
};

struct StreamFrame {
    PacketHeader header;
    cv::Mat rgb;
    cv::Mat depthMm;
};

struct GridKey {
    int x = 0;
    int z = 0;

    bool operator==(const GridKey& other) const {
        return x == other.x && z == other.z;
    }

    bool operator!=(const GridKey& other) const {
        return !(*this == other);
    }
};

struct GridKeyHash {
    size_t operator()(const GridKey& key) const {
        size_t seed = 0;
        seed ^= std::hash<int>()(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct GridCell {
    float logOdds = 0.0f;
};

struct GridBounds {
    bool valid = false;
    int minX = 0;
    int maxX = 0;
    int minZ = 0;
    int maxZ = 0;
};

struct OccupancyViewState {
    float centerX = 0.0f;
    float centerZ = 0.0f;
    float metersPerPixel = 0.03f;
    bool autoFit = true;
    bool followRobot = true;
    float followMetersPerPixel = 0.02f;
    bool dragging = false;
    cv::Point lastMouse;
    cv::Rect panelRect = cv::Rect(0, 0, 920, 860);
};

struct PlannerState {
    bool hasGoal = false;
    GridKey goalCell;
    vector<GridKey> pathCells;
    bool pathValid = false;
    bool dirty = false;
    GridKey lastStartCell;
    uint64_t lastGridVersion = 0;
};

struct MouseContext {
    OccupancyViewState* view = nullptr;
    PlannerState* planner = nullptr;
    float gridResolution = kGridResolutionMeters;
};

struct LiveStatus {
    string trackingState = "NO_IMAGES_YET";
    bool hasPose = false;
    Sophus::SE3f currentTwc;
    float floorY = kAssumedCameraHeightMeters;
    bool localizationOnly = false;
};

struct TrajectoryState {
    vector<cv::Point2f> samples;
};

struct ScaleDiagnostics {
    bool hasOrbPose = false;
    bool hasPhonePose = false;
    Eigen::Vector3f previousOrbPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f previousPhonePosition = Eigen::Vector3f::Zero();
    float orbDistanceMeters = 0.0f;
    float phoneDistanceMeters = 0.0f;
    uint64_t orbSegments = 0;
    uint64_t phoneSegments = 0;
};

struct NavigationSettings {
    bool fixedHeightMode = false;
    float fixedCameraHeightMeters = kAssumedCameraHeightMeters;
    int inflationRadiusCells = 3;
    float lookaheadMeters = 0.60f;
    bool showInflation = true;
};

struct GuidanceState {
    bool hasWaypoint = false;
    bool hasGoal = false;
    bool pathValid = false;
    cv::Point2f waypointWorld;
    float waypointDistanceMeters = 0.0f;
    float headingErrorDegrees = 0.0f;
    float robotYawDegrees = 0.0f;
    float targetYawDegrees = 0.0f;
};

struct DepthComparisonDiagnostics {
    bool modelEnabled = false;
    bool modelReady = false;
    bool hasComparison = false;
    string status = "DISABLED";
    int validSensorPixels = 0;
    int validOverlapPixels = 0;
    float overlapRatio = 0.0f;
    float alignmentScale = 0.0f;
    float alignmentOffset = 0.0f;
    float inferenceMs = 0.0f;
    float maeMeters = 0.0f;
    float rmseMeters = 0.0f;
    float absRel = 0.0f;
    float biasMeters = 0.0f;
};

struct BackendControl {
    int revision = -1;
    string viewMode;
    bool clearGoal = false;
    bool saveSnapshot = false;
    bool quit = false;
    bool hasFixedHeightMode = false;
    bool fixedHeightMode = false;
    bool hasFixedCameraHeightMeters = false;
    float fixedCameraHeightMeters = kAssumedCameraHeightMeters;
    bool hasInflationRadius = false;
    int inflationRadiusCells = 3;
    bool hasShowInflation = false;
    bool showInflation = true;
    bool hasLookaheadMeters = false;
    float lookaheadMeters = 0.60f;
    bool hasLocalizationOnly = false;
    bool localizationOnly = false;
    bool setGoal = false;
    float goalWorldX = 0.0f;
    float goalWorldZ = 0.0f;
};

class OccupancyGrid2D;

cv::Point WorldToPixel(const OccupancyViewState& view, const cv::Rect& panelRect, float worldX, float worldZ);
cv::Point2f PixelToWorld(const OccupancyViewState& view, const cv::Rect& panelRect, const cv::Point& pixel);
bool IsBlockedForPlanning(const OccupancyGrid2D& grid, const GridKey& cell, int inflationRadius);
string CurrentTimeString();
string BuildLatestSnapshotPath();
string BuildLatestRgbPath();
string BuildLatestDepthPath();
string BuildLatestModelDepthPath();
string BuildLatestDepthDiffPath();
string BuildLatestMapPath();
string BuildLatestMapDataPath();
void WriteMapRenderData(const string& path,
                        const OccupancyGrid2D& grid,
                        const OccupancyViewState& viewState,
                        const LiveStatus& status,
                        const PlannerState& planner,
                        const TrajectoryState& trajectory,
                        const NavigationSettings& settings);
void ResetFollowView(OccupancyViewState& view);
void SetOverviewMode(OccupancyViewState& view);

Sophus::SE3f PoseFromHeader(const PacketHeader& header) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for(int row = 0; row < 4; ++row) {
        for(int col = 0; col < 4; ++col)
            transform(row, col) = header.pose[static_cast<size_t>(row * 4 + col)];
    }
    return Sophus::SE3f(transform.block<3, 3>(0, 0), transform.block<3, 1>(0, 3));
}

float ClampFloat(float value, float minValue, float maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

string GetEnvOrDefault(const char* name, const string& fallback) {
    const char* value = std::getenv(name);
    return (value && *value) ? string(value) : fallback;
}

bool GetEnvFlag(const char* name) {
    const char* value = std::getenv(name);
    if(!value)
        return false;
    const string text(value);
    return text == "1" || text == "true" || text == "TRUE" || text == "yes" || text == "YES";
}

string CurrentWorkingDirectory() {
    array<char, PATH_MAX> buffer {};
    if(::getcwd(buffer.data(), buffer.size()) == nullptr)
        return ".";
    return string(buffer.data());
}

string FindUiFontPath() {
    const vector<string> candidates = {
        "/System/Library/Fonts/Hiragino Sans GB.ttc",
        "/System/Library/Fonts/STHeiti Medium.ttc",
        "/System/Library/Fonts/Supplemental/Songti.ttc"
    };
    for(const string& candidate : candidates) {
        ifstream font(candidate);
        if(font.good())
            return candidate;
    }
    return "";
}

cv::Ptr<cv::freetype::FreeType2> UiFont() {
    static cv::Ptr<cv::freetype::FreeType2> font = []() -> cv::Ptr<cv::freetype::FreeType2> {
        const string fontPath = FindUiFontPath();
        if(fontPath.empty())
            return {};
        try {
            cv::Ptr<cv::freetype::FreeType2> renderer = cv::freetype::createFreeType2();
            renderer->loadFontData(fontPath, 0);
            return renderer;
        } catch(const cv::Exception&) {
            return {};
        }
    }();
    return font;
}

class OccupancyGrid2D {
public:
    explicit OccupancyGrid2D(float resolutionMeters)
        : mResolution(resolutionMeters) {
    }

    float resolution() const {
        return mResolution;
    }

    uint64_t version() const {
        return mVersion;
    }

    GridKey ToCell(float worldX, float worldZ) const {
        return GridKey{
            static_cast<int>(std::floor(worldX / mResolution)),
            static_cast<int>(std::floor(worldZ / mResolution))
        };
    }

    cv::Point2f CellCenterWorld(const GridKey& cell) const {
        return cv::Point2f(
            (static_cast<float>(cell.x) + 0.5f) * mResolution,
            (static_cast<float>(cell.z) + 0.5f) * mResolution);
    }

    float LogOdds(const GridKey& cell) const {
        const auto found = mCells.find(cell);
        if(found == mCells.end())
            return 0.0f;
        return found->second.logOdds;
    }

    bool IsOccupied(const GridKey& cell) const {
        return LogOdds(cell) > 1.2f;
    }

    bool IsKnownFree(const GridKey& cell) const {
        return LogOdds(cell) < -0.65f;
    }

    GridBounds Bounds() const {
        GridBounds bounds;
        for(const auto& entry : mCells) {
            if(std::fabs(entry.second.logOdds) < 0.15f)
                continue;

            if(!bounds.valid) {
                bounds.valid = true;
                bounds.minX = bounds.maxX = entry.first.x;
                bounds.minZ = bounds.maxZ = entry.first.z;
            } else {
                bounds.minX = std::min(bounds.minX, entry.first.x);
                bounds.maxX = std::max(bounds.maxX, entry.first.x);
                bounds.minZ = std::min(bounds.minZ, entry.first.z);
                bounds.maxZ = std::max(bounds.maxZ, entry.first.z);
            }
        }
        return bounds;
    }

    vector<pair<GridKey, float>> Cells() const {
        vector<pair<GridKey, float>> cells;
        cells.reserve(mCells.size());
        for(const auto& entry : mCells)
            cells.emplace_back(entry.first, entry.second.logOdds);
        return cells;
    }

    void IntegratePoints(const Eigen::Vector3f& cameraWorld,
                         const vector<Eigen::Vector3f>& worldPoints,
                         float floorY) {
        if(worldPoints.empty())
            return;

        const GridKey startCell = ToCell(cameraWorld.x(), cameraWorld.z());
        bool anyUpdate = false;
        for(const Eigen::Vector3f& point : worldPoints) {
            const float planarDistance = std::hypot(point.x() - cameraWorld.x(), point.z() - cameraWorld.z());
            if(planarDistance < 0.05f || planarDistance > kMaxDepthMeters)
                continue;

            const GridKey endCell = ToCell(point.x(), point.z());
            const bool isObstacle = point.y() < (floorY - kFloorClearanceMeters) &&
                                    point.y() > (floorY - kObstacleHeightMeters);
            anyUpdate |= Raycast(startCell, endCell, isObstacle);
        }

        if(anyUpdate)
            ++mVersion;
    }

private:
    bool UpdateCell(const GridKey& cell, float delta) {
        GridCell& gridCell = mCells[cell];
        const float previous = gridCell.logOdds;
        gridCell.logOdds = ClampFloat(gridCell.logOdds + delta, -4.0f, 4.0f);
        return std::fabs(gridCell.logOdds - previous) > 1e-4f;
    }

    bool Raycast(const GridKey& start, const GridKey& end, bool markOccupied) {
        bool changed = false;
        int x = start.x;
        int z = start.z;
        const int dx = std::abs(end.x - start.x);
        const int dz = std::abs(end.z - start.z);
        const int stepX = (start.x < end.x) ? 1 : -1;
        const int stepZ = (start.z < end.z) ? 1 : -1;

        int error = dx - dz;
        while(x != end.x || z != end.z) {
            changed |= UpdateCell(GridKey{x, z}, -0.20f);
            const int twiceError = error * 2;
            if(twiceError > -dz) {
                error -= dz;
                x += stepX;
            }
            if(twiceError < dx) {
                error += dx;
                z += stepZ;
            }
        }

        if(markOccupied)
            changed |= UpdateCell(end, 0.85f);
        else
            changed |= UpdateCell(end, -0.10f);
        return changed;
    }

private:
    float mResolution;
    uint64_t mVersion = 0;
    unordered_map<GridKey, GridCell, GridKeyHash> mCells;
};

class FloorEstimator {
public:
    void ObserveFrame(const vector<float>& pointYs,
                      float cameraY,
                      const NavigationSettings& settings) {
        float candidate = cameraY + settings.fixedCameraHeightMeters;
        if(!settings.fixedHeightMode && !pointYs.empty()) {
            vector<float> sorted = pointYs;
            sort(sorted.begin(), sorted.end());
            const size_t index = std::min(sorted.size() - 1, static_cast<size_t>(sorted.size() * 9 / 10));
            candidate = sorted[index];
            const float impliedHeight = candidate - cameraY;
            if(!std::isfinite(candidate) || impliedHeight < 0.08f || impliedHeight > 0.95f)
                candidate = cameraY + settings.fixedCameraHeightMeters;
        } else if(!settings.fixedHeightMode) {
            candidate = cameraY + settings.fixedCameraHeightMeters;
        }

        if(!mHasEstimate) {
            mEstimate = candidate;
            mHasEstimate = true;
        } else if(settings.fixedHeightMode) {
            mEstimate = 0.75f * mEstimate + 0.25f * candidate;
        } else {
            mEstimate = 0.92f * mEstimate + 0.08f * candidate;
        }
    }

    float FloorY(float cameraY) const {
        return mHasEstimate ? mEstimate : (cameraY + kAssumedCameraHeightMeters);
    }

private:
    bool mHasEstimate = false;
    float mEstimate = kAssumedCameraHeightMeters;
};

class NavigationFrameEstimator {
public:
    void Observe(const Sophus::SE3f& orbTwc, const PacketHeader& phoneHeader) {
        if(!phoneHeader.hasPose)
            return;

        const Sophus::SE3f phoneTwc = PoseFromHeader(phoneHeader);
        const Eigen::Matrix3f cameraConvention = (Eigen::Matrix3f() <<
            1.0f,  0.0f,  0.0f,
            0.0f, -1.0f,  0.0f,
            0.0f,  0.0f, -1.0f).finished();

        Eigen::Matrix3f candidateRotation = phoneTwc.rotationMatrix() * cameraConvention * orbTwc.rotationMatrix().transpose();
        Eigen::Quaternionf candidate(candidateRotation);
        if(!std::isfinite(candidate.w()) || !std::isfinite(candidate.x()) ||
           !std::isfinite(candidate.y()) || !std::isfinite(candidate.z()))
            return;
        candidate.normalize();

        if(!mHasAlignment) {
            mOrbToNav = candidate;
            mHasAlignment = true;
            return;
        }

        mOrbToNav = mOrbToNav.slerp(0.08f, candidate).normalized();
    }

    bool HasAlignment() const {
        return mHasAlignment;
    }

    Eigen::Vector3f TransformPoint(const Eigen::Vector3f& pointOrb) const {
        return mHasAlignment ? (mOrbToNav * pointOrb) : pointOrb;
    }

    vector<Eigen::Vector3f> TransformPoints(const vector<Eigen::Vector3f>& pointsOrb) const {
        if(!mHasAlignment)
            return pointsOrb;

        vector<Eigen::Vector3f> pointsNav;
        pointsNav.reserve(pointsOrb.size());
        for(const Eigen::Vector3f& point : pointsOrb)
            pointsNav.push_back(mOrbToNav * point);
        return pointsNav;
    }

    Sophus::SE3f TransformPose(const Sophus::SE3f& orbTwc) const {
        if(!mHasAlignment)
            return orbTwc;
        return Sophus::SE3f(mOrbToNav.toRotationMatrix() * orbTwc.rotationMatrix(),
                            mOrbToNav * orbTwc.translation());
    }

private:
    bool mHasAlignment = false;
    Eigen::Quaternionf mOrbToNav = Eigen::Quaternionf::Identity();
};

void UpdateScaleDiagnostics(ScaleDiagnostics& diagnostics,
                            const Sophus::SE3f& orbTwc,
                            const PacketHeader& phoneHeader) {
    const Eigen::Vector3f orbPosition = orbTwc.translation();
    if(diagnostics.hasOrbPose) {
        const float delta = (orbPosition - diagnostics.previousOrbPosition).norm();
        if(std::isfinite(delta) && delta > 0.01f && delta < 1.0f) {
            diagnostics.orbDistanceMeters += delta;
            ++diagnostics.orbSegments;
        }
    }
    diagnostics.previousOrbPosition = orbPosition;
    diagnostics.hasOrbPose = true;

    if(!phoneHeader.hasPose)
        return;

    const Sophus::SE3f phoneTwc = PoseFromHeader(phoneHeader);
    const Eigen::Vector3f phonePosition = phoneTwc.translation();
    if(diagnostics.hasPhonePose) {
        const float delta = (phonePosition - diagnostics.previousPhonePosition).norm();
        if(std::isfinite(delta) && delta > 0.01f && delta < 1.0f) {
            diagnostics.phoneDistanceMeters += delta;
            ++diagnostics.phoneSegments;
        }
    }
    diagnostics.previousPhonePosition = phonePosition;
    diagnostics.hasPhonePose = true;
}

float NormalizeAngleDegrees(float angleDegrees) {
    while(angleDegrees > 180.0f)
        angleDegrees -= 360.0f;
    while(angleDegrees < -180.0f)
        angleDegrees += 360.0f;
    return angleDegrees;
}

GuidanceState ComputeGuidanceState(const OccupancyGrid2D& grid,
                                   const LiveStatus& status,
                                   const PlannerState& planner,
                                   const NavigationSettings& settings) {
    GuidanceState guidance;
    guidance.hasGoal = planner.hasGoal;
    guidance.pathValid = planner.pathValid;
    if(!status.hasPose || !planner.pathValid || planner.pathCells.empty())
        return guidance;

    vector<cv::Point2f> worldPath;
    worldPath.reserve(planner.pathCells.size());
    for(const GridKey& cell : planner.pathCells)
        worldPath.push_back(grid.CellCenterWorld(cell));

    const cv::Point2f robotWorld(status.currentTwc.translation().x(), status.currentTwc.translation().z());
    cv::Point2f waypoint = worldPath.back();
    float accumulated = 0.0f;
    for(size_t index = 1; index < worldPath.size(); ++index) {
        accumulated += std::hypot(worldPath[index].x - worldPath[index - 1].x,
                                  worldPath[index].y - worldPath[index - 1].y);
        if(accumulated >= settings.lookaheadMeters) {
            waypoint = worldPath[index];
            break;
        }
    }

    const float dx = waypoint.x - robotWorld.x;
    const float dz = waypoint.y - robotWorld.y;
    const float distance = std::hypot(dx, dz);
    if(distance < 1e-4f)
        return guidance;

    const Eigen::Vector3f forward = status.currentTwc.rotationMatrix() * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    const float robotYaw = std::atan2(forward.z(), forward.x()) * 180.0f / static_cast<float>(M_PI);
    const float targetYaw = std::atan2(dz, dx) * 180.0f / static_cast<float>(M_PI);

    guidance.hasWaypoint = true;
    guidance.waypointWorld = waypoint;
    guidance.waypointDistanceMeters = distance;
    guidance.robotYawDegrees = robotYaw;
    guidance.targetYawDegrees = targetYaw;
    guidance.headingErrorDegrees = NormalizeAngleDegrees(targetYaw - robotYaw);
    return guidance;
}

void UpdateGuidanceFile(const GuidanceState& guidance,
                        const LiveStatus& status,
                        const NavigationSettings& settings,
                        const string& path = "/tmp/iphone_rgbd_nav_guidance.json") {
    static auto lastWriteTime = chrono::steady_clock::now() - chrono::seconds(10);
    const auto now = chrono::steady_clock::now();
    if(now - lastWriteTime < chrono::milliseconds(400))
        return;
    lastWriteTime = now;

    ofstream out(path);
    out << "{\n";
    out << "  \"has_pose\": " << (status.hasPose ? "true" : "false") << ",\n";
    out << "  \"tracking_state\": \"" << status.trackingState << "\",\n";
    out << "  \"fixed_height_mode\": " << (settings.fixedHeightMode ? "true" : "false") << ",\n";
    out << "  \"fixed_camera_height_m\": " << fixed << setprecision(3) << settings.fixedCameraHeightMeters << ",\n";
    out << "  \"has_waypoint\": " << (guidance.hasWaypoint ? "true" : "false") << ",\n";
    out << "  \"waypoint_distance_m\": " << fixed << setprecision(3) << guidance.waypointDistanceMeters << ",\n";
    out << "  \"heading_error_deg\": " << fixed << setprecision(3) << guidance.headingErrorDegrees << "\n";
    out << "}\n";
}

bool ReadBackendControl(const string& path, BackendControl& control) {
    ifstream input(path);
    if(!input.good())
        return false;

    try {
        ptree tree;
        boost::property_tree::read_json(input, tree);
        control.revision = tree.get<int>("revision", control.revision);
        control.viewMode = tree.get<string>("viewMode", "");
        control.clearGoal = tree.get<bool>("clearGoal", false);
        control.saveSnapshot = tree.get<bool>("saveSnapshot", false);
        control.quit = tree.get<bool>("quit", false);

        if(const auto value = tree.get_optional<bool>("fixedHeightMode")) {
            control.hasFixedHeightMode = true;
            control.fixedHeightMode = *value;
        }
        if(const auto value = tree.get_optional<float>("fixedCameraHeightMeters")) {
            control.hasFixedCameraHeightMeters = true;
            control.fixedCameraHeightMeters = *value;
        }
        if(const auto value = tree.get_optional<int>("inflationRadiusCells")) {
            control.hasInflationRadius = true;
            control.inflationRadiusCells = *value;
        }
        if(const auto value = tree.get_optional<bool>("showInflation")) {
            control.hasShowInflation = true;
            control.showInflation = *value;
        }
        if(const auto value = tree.get_optional<float>("lookaheadMeters")) {
            control.hasLookaheadMeters = true;
            control.lookaheadMeters = *value;
        }
        if(const auto value = tree.get_optional<bool>("localizationOnly")) {
            control.hasLocalizationOnly = true;
            control.localizationOnly = *value;
        }
        if(const auto value = tree.get_optional<bool>("setGoal")) {
            control.setGoal = *value;
        }
        control.goalWorldX = tree.get<float>("goalWorldX", 0.0f);
        control.goalWorldZ = tree.get<float>("goalWorldZ", 0.0f);
    } catch(const std::exception&) {
        return false;
    }
    return true;
}

void ApplyBackendControl(const BackendControl& control,
                         int& lastRevision,
                         OccupancyViewState& viewState,
                         PlannerState& planner,
                         NavigationSettings& navigationSettings,
                         OccupancyGrid2D& grid,
                         bool& localizationOnly,
                         ORB_SLAM3::System* slam,
                         bool& requestSnapshot,
                         bool& requestQuit) {
    if(control.viewMode == "follow")
        ResetFollowView(viewState);
    else if(control.viewMode == "overview")
        SetOverviewMode(viewState);

    if(control.hasFixedHeightMode)
        navigationSettings.fixedHeightMode = control.fixedHeightMode;
    if(control.hasFixedCameraHeightMeters)
        navigationSettings.fixedCameraHeightMeters = ClampFloat(control.fixedCameraHeightMeters, 0.10f, 1.20f);
    if(control.hasInflationRadius)
        navigationSettings.inflationRadiusCells = std::max(1, std::min(control.inflationRadiusCells, 12));
    if(control.hasShowInflation)
        navigationSettings.showInflation = control.showInflation;
    if(control.hasLookaheadMeters)
        navigationSettings.lookaheadMeters = ClampFloat(control.lookaheadMeters, 0.20f, 3.00f);

    if(control.hasLocalizationOnly && control.localizationOnly != localizationOnly) {
        localizationOnly = control.localizationOnly;
        if(slam) {
            if(localizationOnly)
                slam->ActivateLocalizationMode();
            else
                slam->DeactivateLocalizationMode();
        }
    }

    if(control.setGoal) {
        planner.goalCell = grid.ToCell(control.goalWorldX, control.goalWorldZ);
        planner.hasGoal = true;
        planner.dirty = true;
    }

    if(control.revision == lastRevision)
        return;
    lastRevision = control.revision;

    if(control.clearGoal) {
        planner.hasGoal = false;
        planner.pathCells.clear();
        planner.pathValid = false;
        planner.dirty = false;
    }
    if(control.saveSnapshot)
        requestSnapshot = true;
    if(control.quit)
        requestQuit = true;
}

void WriteBackendState(const string& path,
                       const LiveStatus& status,
                       const PlannerState& planner,
                       const GuidanceState& guidance,
                       const NavigationSettings& navigationSettings,
                       const ScaleDiagnostics& diagnostics,
                       const DepthComparisonDiagnostics& depthComparison,
                       const OccupancyViewState& viewState,
                       const cv::Rect& mapRect,
                       const cv::Size& compositeSize,
                       bool connected) {
    ofstream out(path);
    if(!out.good())
        return;

    out << "{\n";
    out << "  \"timestamp\": \"" << CurrentTimeString() << "\",\n";
    out << "  \"connected\": " << (connected ? "true" : "false") << ",\n";
    out << "  \"trackingState\": \"" << status.trackingState << "\",\n";
    out << "  \"hasPose\": " << (status.hasPose ? "true" : "false") << ",\n";
    out << "  \"floorY\": " << fixed << setprecision(3) << status.floorY << ",\n";
    out << "  \"localizationOnly\": " << (status.localizationOnly ? "true" : "false") << ",\n";
    out << "  \"fixedHeightMode\": " << (navigationSettings.fixedHeightMode ? "true" : "false") << ",\n";
    out << "  \"fixedCameraHeightMeters\": " << fixed << setprecision(3) << navigationSettings.fixedCameraHeightMeters << ",\n";
    out << "  \"inflationRadiusCells\": " << navigationSettings.inflationRadiusCells << ",\n";
    out << "  \"showInflation\": " << (navigationSettings.showInflation ? "true" : "false") << ",\n";
    out << "  \"lookaheadMeters\": " << fixed << setprecision(3) << navigationSettings.lookaheadMeters << ",\n";
    out << "  \"orbDistanceMeters\": " << fixed << setprecision(3) << diagnostics.orbDistanceMeters << ",\n";
    out << "  \"phoneDistanceMeters\": " << fixed << setprecision(3) << diagnostics.phoneDistanceMeters << ",\n";
    out << "  \"scaleRatio\": " << fixed << setprecision(4)
        << ((diagnostics.phoneDistanceMeters > 1e-4f) ? (diagnostics.orbDistanceMeters / diagnostics.phoneDistanceMeters) : 0.0f) << ",\n";
    out << "  \"depthModelEnabled\": " << (depthComparison.modelEnabled ? "true" : "false") << ",\n";
    out << "  \"depthComparisonReady\": " << (depthComparison.hasComparison ? "true" : "false") << ",\n";
    out << "  \"depthComparisonStatus\": \"" << depthComparison.status << "\",\n";
    out << "  \"depthComparisonValidSensorPixels\": " << depthComparison.validSensorPixels << ",\n";
    out << "  \"depthComparisonValidOverlapPixels\": " << depthComparison.validOverlapPixels << ",\n";
    out << "  \"depthComparisonOverlapRatio\": " << fixed << setprecision(4) << depthComparison.overlapRatio << ",\n";
    out << "  \"depthComparisonAlignmentScale\": " << fixed << setprecision(4) << depthComparison.alignmentScale << ",\n";
    out << "  \"depthComparisonAlignmentOffset\": " << fixed << setprecision(4) << depthComparison.alignmentOffset << ",\n";
    out << "  \"depthComparisonInferenceMs\": " << fixed << setprecision(3) << depthComparison.inferenceMs << ",\n";
    out << "  \"depthComparisonMaeMeters\": " << fixed << setprecision(4) << depthComparison.maeMeters << ",\n";
    out << "  \"depthComparisonRmseMeters\": " << fixed << setprecision(4) << depthComparison.rmseMeters << ",\n";
    out << "  \"depthComparisonAbsRel\": " << fixed << setprecision(4) << depthComparison.absRel << ",\n";
    out << "  \"depthComparisonBiasMeters\": " << fixed << setprecision(4) << depthComparison.biasMeters << ",\n";
    out << "  \"hasGoal\": " << (planner.hasGoal ? "true" : "false") << ",\n";
    out << "  \"pathValid\": " << (planner.pathValid ? "true" : "false") << ",\n";
    out << "  \"hasWaypoint\": " << (guidance.hasWaypoint ? "true" : "false") << ",\n";
    out << "  \"waypointDistanceMeters\": " << fixed << setprecision(3) << guidance.waypointDistanceMeters << ",\n";
    out << "  \"headingErrorDegrees\": " << fixed << setprecision(3) << guidance.headingErrorDegrees << ",\n";
    out << "  \"compositeWidth\": " << compositeSize.width << ",\n";
    out << "  \"compositeHeight\": " << compositeSize.height << ",\n";
    out << "  \"mapRectX\": " << mapRect.x << ",\n";
    out << "  \"mapRectY\": " << mapRect.y << ",\n";
    out << "  \"mapRectWidth\": " << mapRect.width << ",\n";
    out << "  \"mapRectHeight\": " << mapRect.height << ",\n";
    out << "  \"viewCenterX\": " << fixed << setprecision(4) << viewState.centerX << ",\n";
    out << "  \"viewCenterZ\": " << fixed << setprecision(4) << viewState.centerZ << ",\n";
    out << "  \"metersPerPixel\": " << fixed << setprecision(6) << viewState.metersPerPixel << ",\n";
    out << "  \"followRobot\": " << (viewState.followRobot ? "true" : "false") << ",\n";
    out << "  \"autoFit\": " << (viewState.autoFit ? "true" : "false") << ",\n";
    out << "  \"latestImagePath\": \"" << BuildLatestSnapshotPath() << "\",\n";
    out << "  \"latestRgbPath\": \"" << BuildLatestRgbPath() << "\",\n";
    out << "  \"latestDepthPath\": \"" << BuildLatestDepthPath() << "\",\n";
    out << "  \"latestModelDepthPath\": \"" << BuildLatestModelDepthPath() << "\",\n";
    out << "  \"latestDepthDiffPath\": \"" << BuildLatestDepthDiffPath() << "\",\n";
    out << "  \"latestMapPath\": \"" << BuildLatestMapPath() << "\",\n";
    out << "  \"mapDataPath\": \"" << BuildLatestMapDataPath() << "\",\n";
    out << "  \"guidancePath\": \"/tmp/iphone_rgbd_nav_guidance.json\"\n";
    out << "}\n";
}

void ExitLoopHandler(int) {
    gKeepRunning = false;
}

bool ReadExact(int fd, void* buffer, size_t size) {
    size_t readBytes = 0;
    auto* out = static_cast<uint8_t*>(buffer);
    while(readBytes < size && gKeepRunning) {
        const ssize_t chunk = recv(fd, out + readBytes, size - readBytes, 0);
        if(chunk == 0)
            return false;
        if(chunk < 0) {
            if(errno == EINTR)
                continue;
            if(errno == EAGAIN || errno == EWOULDBLOCK)
                return false;
            return false;
        }
        readBytes += static_cast<size_t>(chunk);
    }
    return readBytes == size;
}

bool WaitForSocketReadable(int fd, int timeoutMs) {
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(fd, &readSet);

    timeval timeout {};
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;

    const int result = select(fd + 1, &readSet, nullptr, nullptr, &timeout);
    if(result < 0) {
        if(errno == EINTR)
            return false;
        throw runtime_error("socket select failed");
    }
    return result > 0 && FD_ISSET(fd, &readSet);
}

void SetSocketTimeout(int fd, int timeoutMs) {
    timeval timeout {};
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

vector<string> CandidateIps() {
    vector<string> ips;
    ifaddrs* interfaces = nullptr;
    if(getifaddrs(&interfaces) != 0)
        return ips;

    for(ifaddrs* entry = interfaces; entry != nullptr; entry = entry->ifa_next) {
        if(!entry->ifa_addr || entry->ifa_addr->sa_family != AF_INET)
            continue;
        if((entry->ifa_flags & IFF_UP) == 0 || (entry->ifa_flags & IFF_LOOPBACK) != 0)
            continue;

        char addressBuffer[INET_ADDRSTRLEN] = {0};
        auto* address = reinterpret_cast<sockaddr_in*>(entry->ifa_addr);
        if(!inet_ntop(AF_INET, &address->sin_addr, addressBuffer, sizeof(addressBuffer)))
            continue;
        ips.emplace_back(addressBuffer);
    }

    freeifaddrs(interfaces);
    sort(ips.begin(), ips.end());
    ips.erase(unique(ips.begin(), ips.end()), ips.end());
    return ips;
}

void PrintCandidateAddresses(int port) {
    const vector<string> ips = CandidateIps();
    if(ips.empty()) {
        cout << "Listening on port " << port << endl;
        return;
    }

    cout << "Enter one of these Mac IPs on the phone:" << endl;
    for(const string& ip : ips)
        cout << "  " << ip << ":" << port << endl;
}

PacketHeader ParseHeader(const string& jsonText) {
    stringstream stream(jsonText);
    ptree tree;
    boost::property_tree::read_json(stream, tree);

    PacketHeader header;
    header.frameIndex = tree.get<uint64_t>("frameIndex");
    header.timestamp = tree.get<double>("timestamp");
    header.displayOrientation = tree.get<string>("displayOrientation", "unknown");
    header.rgbWidth = tree.get<int>("rgbWidth");
    header.rgbHeight = tree.get<int>("rgbHeight");
    header.depthWidth = tree.get<int>("depthWidth", 0);
    header.depthHeight = tree.get<int>("depthHeight", 0);
    header.rgbSize = tree.get<int>("rgbSize");
    header.depthSize = tree.get<int>("depthSize", 0);
    header.fx = tree.get<float>("fx");
    header.fy = tree.get<float>("fy");
    header.cx = tree.get<float>("cx");
    header.cy = tree.get<float>("cy");
    if(const auto poseChild = tree.get_child_optional("pose")) {
        size_t index = 0;
        for(const auto& item : *poseChild) {
            if(index >= header.pose.size())
                break;
            header.pose[index++] = item.second.get_value<float>();
        }
        header.hasPose = (index == header.pose.size());
    }
    return header;
}

bool ReceiveFrame(int clientFd, StreamFrame& frame) {
    char magic[4] = {};
    if(!ReadExact(clientFd, magic, sizeof(magic)))
        return false;
    if(memcmp(magic, kMagic, sizeof(kMagic)) != 0)
        throw runtime_error("invalid packet magic");

    uint32_t headerSizeBE = 0;
    if(!ReadExact(clientFd, &headerSizeBE, sizeof(headerSizeBE)))
        return false;
    const uint32_t headerSize = ntohl(headerSizeBE);
    string headerJson(headerSize, '\0');
    if(!ReadExact(clientFd, &headerJson[0], headerJson.size()))
        return false;
    frame.header = ParseHeader(headerJson);

    vector<uint8_t> rgbBytes(static_cast<size_t>(frame.header.rgbSize));
    if(!ReadExact(clientFd, rgbBytes.data(), rgbBytes.size()))
        return false;

    vector<uint8_t> depthBytes(static_cast<size_t>(frame.header.depthSize));
    if(frame.header.depthSize > 0) {
        if(!ReadExact(clientFd, depthBytes.data(), depthBytes.size()))
            return false;
    }

    frame.rgb = cv::imdecode(rgbBytes, cv::IMREAD_COLOR);
    if(frame.rgb.empty())
        throw runtime_error("failed to decode JPEG frame");

    if(frame.header.depthWidth <= 0 || frame.header.depthHeight <= 0 || depthBytes.empty()) {
        frame.depthMm.release();
        return true;
    }

    cv::Mat rawDepth(frame.header.depthHeight, frame.header.depthWidth, CV_16UC1, depthBytes.data());
    frame.depthMm = rawDepth.clone();
    return true;
}

int CreateListenSocket(int port) {
    const int serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if(serverFd < 0)
        throw runtime_error("failed to create socket");

    int reuse = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(port));

    if(::bind(serverFd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        close(serverFd);
        throw runtime_error("failed to bind socket");
    }
    if(listen(serverFd, 1) != 0) {
        close(serverFd);
        throw runtime_error("failed to listen on socket");
    }
    return serverFd;
}

cv::Mat RotateForPreview(const cv::Mat& image, const string& orientation) {
    if(orientation == "portrait") {
        cv::Mat rotated;
        cv::rotate(image, rotated, cv::ROTATE_90_CLOCKWISE);
        return rotated;
    }
    if(orientation == "portraitUpsideDown") {
        cv::Mat rotated;
        cv::rotate(image, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
        return rotated;
    }
    if(orientation == "landscapeLeft") {
        cv::Mat rotated;
        cv::rotate(image, rotated, cv::ROTATE_180);
        return rotated;
    }
    return image;
}

cv::Mat ResizeDepthToRgb(const StreamFrame& frame, const cv::Size& size) {
    if(frame.depthMm.size() == size)
        return frame.depthMm;
    cv::Mat resizedDepth;
    cv::resize(frame.depthMm, resizedDepth, size, 0.0, 0.0, cv::INTER_NEAREST);
    return resizedDepth;
}

cv::Mat ColorizeDepthMm(const cv::Mat& depthMm) {
    cv::Mat depthMeters;
    depthMm.convertTo(depthMeters, CV_32F, 0.001);
    cv::Mat validMask = depthMm > 0;
    double minDepth = 0.0;
    double maxDepth = 0.0;
    cv::minMaxLoc(depthMeters, &minDepth, &maxDepth, nullptr, nullptr, validMask);
    if(maxDepth <= minDepth + numeric_limits<double>::epsilon())
        return cv::Mat(depthMm.size(), CV_8UC3, cv::Scalar(20, 20, 20));

    cv::Mat normalized;
    depthMeters.convertTo(normalized, CV_32F, 255.0 / (maxDepth - minDepth), -minDepth * 255.0 / (maxDepth - minDepth));
    normalized.setTo(0.0f, ~validMask);
    cv::Mat normalized8u;
    normalized.convertTo(normalized8u, CV_8U);
    normalized8u = 255 - normalized8u;
    cv::Mat colored;
    cv::applyColorMap(normalized8u, colored, cv::COLORMAP_TURBO);
    colored.setTo(cv::Scalar(0, 0, 0), ~validMask);
    return colored;
}

cv::Mat ColorizeDepthFloat(const cv::Mat& depth32f, const cv::Mat& validMask = cv::Mat()) {
    cv::Mat sanitized = depth32f.clone();
    cv::patchNaNs(sanitized, 0.0f);

    cv::Mat effectiveMask;
    if(validMask.empty())
        effectiveMask = sanitized > 0.0f;
    else
        cv::bitwise_and(validMask, sanitized > 0.0f, effectiveMask);

    double minDepth = 0.0;
    double maxDepth = 0.0;
    cv::minMaxLoc(sanitized, &minDepth, &maxDepth, nullptr, nullptr, effectiveMask);
    if(maxDepth <= minDepth + numeric_limits<double>::epsilon())
        return cv::Mat(depth32f.size(), CV_8UC3, cv::Scalar(20, 20, 20));

    cv::Mat normalized;
    sanitized.convertTo(normalized, CV_32F, 255.0 / (maxDepth - minDepth), -minDepth * 255.0 / (maxDepth - minDepth));
    normalized.setTo(0.0f, ~effectiveMask);

    cv::Mat normalized8u;
    normalized.convertTo(normalized8u, CV_8U);
    normalized8u = 255 - normalized8u;

    cv::Mat colored;
    cv::applyColorMap(normalized8u, colored, cv::COLORMAP_TURBO);
    colored.setTo(cv::Scalar(0, 0, 0), ~effectiveMask);
    return colored;
}

cv::Mat ColorizeDepthError(const cv::Mat& absErrorMeters, const cv::Mat& validMask) {
    cv::Mat sanitized = absErrorMeters.clone();
    cv::patchNaNs(sanitized, 0.0f);
    cv::threshold(sanitized, sanitized, kDepthDiffVisualMaxMeters, kDepthDiffVisualMaxMeters, cv::THRESH_TRUNC);

    cv::Mat normalized;
    sanitized.convertTo(normalized, CV_32F, 255.0 / kDepthDiffVisualMaxMeters);
    normalized.setTo(0.0f, ~validMask);

    cv::Mat normalized8u;
    normalized.convertTo(normalized8u, CV_8U);

    cv::Mat colored;
    cv::applyColorMap(normalized8u, colored, cv::COLORMAP_TURBO);
    colored.setTo(cv::Scalar(0, 0, 0), ~validMask);
    return colored;
}

float MedianInPlace(vector<float>& values) {
    if(values.empty())
        return 0.0f;

    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    float median = values[middle];
    if(values.size() % 2 == 0) {
        std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
        median = 0.5f * (median + values[middle - 1]);
    }
    return median;
}

bool SolveAffineLeastSquares(const vector<float>& x,
                             const vector<float>& y,
                             const vector<uint8_t>* inlierMask,
                             double& scale,
                             double& offset) {
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    size_t count = 0;
    for(size_t index = 0; index < x.size(); ++index) {
        if(inlierMask && (*inlierMask)[index] == 0)
            continue;
        const double xv = static_cast<double>(x[index]);
        const double yv = static_cast<double>(y[index]);
        sumX += xv;
        sumY += yv;
        sumXX += xv * xv;
        sumXY += xv * yv;
        ++count;
    }

    if(count < 64)
        return false;

    const double denom = static_cast<double>(count) * sumXX - sumX * sumX;
    if(std::abs(denom) < 1e-9)
        return false;

    scale = (static_cast<double>(count) * sumXY - sumX * sumY) / denom;
    offset = (sumY - scale * sumX) / static_cast<double>(count);
    return std::isfinite(scale) && std::isfinite(offset);
}

DepthComparisonDiagnostics CompareDepthMaps(const cv::Mat& sensorDepthMm,
                                           const cv::Mat& modelDepth32f,
                                           cv::Mat& alignedModelDepthMeters,
                                           cv::Mat& absErrorMeters,
                                           cv::Mat& validMask) {
    DepthComparisonDiagnostics diagnostics;
    if(sensorDepthMm.empty() || modelDepth32f.empty())
        return diagnostics;

    cv::Mat sensorDepthMeters;
    sensorDepthMm.convertTo(sensorDepthMeters, CV_32F, 0.001);

    cv::Mat modelDepth = modelDepth32f.clone();
    cv::patchNaNs(modelDepth, 0.0f);

    cv::Mat sensorValidMask = (sensorDepthMm > 0);
    cv::bitwise_and(sensorValidMask, sensorDepthMeters >= kMinDepthMeters, sensorValidMask);
    cv::bitwise_and(sensorValidMask, sensorDepthMeters <= kMaxDepthMeters, sensorValidMask);
    diagnostics.validSensorPixels = cv::countNonZero(sensorValidMask);

    cv::Mat modelValidMask = modelDepth > 1e-5f;
    cv::bitwise_and(sensorValidMask, modelValidMask, validMask);
    diagnostics.validOverlapPixels = cv::countNonZero(validMask);
    if(diagnostics.validSensorPixels > 0)
        diagnostics.overlapRatio = static_cast<float>(diagnostics.validOverlapPixels) /
                                   static_cast<float>(diagnostics.validSensorPixels);

    if(diagnostics.validOverlapPixels < 64)
        return diagnostics;

    vector<float> modelSamples;
    vector<float> inverseDepthTargets;
    modelSamples.reserve(static_cast<size_t>(diagnostics.validOverlapPixels));
    inverseDepthTargets.reserve(static_cast<size_t>(diagnostics.validOverlapPixels));
    for(int y = 0; y < sensorDepthMeters.rows; ++y) {
        const float* sensorPtr = sensorDepthMeters.ptr<float>(y);
        const float* modelPtr = modelDepth.ptr<float>(y);
        const uchar* maskPtr = validMask.ptr<uchar>(y);
        for(int x = 0; x < sensorDepthMeters.cols; ++x) {
            if(maskPtr[x] == 0)
                continue;
            modelSamples.push_back(modelPtr[x]);
            inverseDepthTargets.push_back(1.0f / std::max(sensorPtr[x], 1e-6f));
        }
    }

    double scale = 0.0;
    double offset = 0.0;
    if(!SolveAffineLeastSquares(modelSamples, inverseDepthTargets, nullptr, scale, offset))
        return diagnostics;

    vector<float> residuals;
    residuals.reserve(modelSamples.size());
    for(size_t index = 0; index < modelSamples.size(); ++index) {
        const double predictedInverseDepth = scale * static_cast<double>(modelSamples[index]) + offset;
        residuals.push_back(static_cast<float>(predictedInverseDepth - static_cast<double>(inverseDepthTargets[index])));
    }

    vector<float> residualCopy = residuals;
    const float residualMedian = MedianInPlace(residualCopy);
    for(float& residual : residuals)
        residual = std::fabs(residual - residualMedian);

    const float residualMad = MedianInPlace(residuals);
    if(std::isfinite(residualMad) && residualMad > 1e-6f) {
        const float threshold = std::max(3.0f * residualMad, 0.02f);
        vector<uint8_t> inlierMask(modelSamples.size(), 0);
        size_t inlierCount = 0;
        for(size_t index = 0; index < modelSamples.size(); ++index) {
            const double predictedInverseDepth = scale * static_cast<double>(modelSamples[index]) + offset;
            const double centeredResidual = std::fabs(predictedInverseDepth -
                                                      static_cast<double>(inverseDepthTargets[index]) -
                                                      static_cast<double>(residualMedian));
            if(centeredResidual <= static_cast<double>(threshold)) {
                inlierMask[index] = 1;
                ++inlierCount;
            }
        }

        double refinedScale = 0.0;
        double refinedOffset = 0.0;
        if(inlierCount >= 64 && SolveAffineLeastSquares(modelSamples, inverseDepthTargets, &inlierMask, refinedScale, refinedOffset)) {
            scale = refinedScale;
            offset = refinedOffset;
        }
    }

    diagnostics.alignmentScale = static_cast<float>(scale);
    diagnostics.alignmentOffset = static_cast<float>(offset);
    if(!std::isfinite(diagnostics.alignmentScale) || !std::isfinite(diagnostics.alignmentOffset))
        return diagnostics;

    alignedModelDepthMeters = cv::Mat(modelDepth.size(), CV_32F, cv::Scalar(0.0f));
    absErrorMeters = cv::Mat(sensorDepthMeters.size(), CV_32F, cv::Scalar(0.0f));

    double absErrorSum = 0.0;
    double squaredErrorSum = 0.0;
    double absRelSum = 0.0;
    double biasSum = 0.0;

    for(int y = 0; y < sensorDepthMeters.rows; ++y) {
        const float* sensorPtr = sensorDepthMeters.ptr<float>(y);
        const float* modelPtr = modelDepth.ptr<float>(y);
        float* alignedPtr = alignedModelDepthMeters.ptr<float>(y);
        float* errorPtr = absErrorMeters.ptr<float>(y);
        const uchar* maskPtr = validMask.ptr<uchar>(y);
        for(int x = 0; x < sensorDepthMeters.cols; ++x) {
            if(maskPtr[x] == 0)
                continue;

            const double predictedInverseDepth = static_cast<double>(diagnostics.alignmentScale) * static_cast<double>(modelPtr[x]) +
                                                 static_cast<double>(diagnostics.alignmentOffset);
            const double predictedDepth = 1.0 / std::max(1e-6, predictedInverseDepth);
            alignedPtr[x] = static_cast<float>(predictedDepth);
            const double signedError = static_cast<double>(alignedPtr[x]) - static_cast<double>(sensorPtr[x]);
            const double absoluteError = std::fabs(signedError);
            errorPtr[x] = static_cast<float>(absoluteError);
            absErrorSum += absoluteError;
            squaredErrorSum += signedError * signedError;
            absRelSum += absoluteError / std::max(1e-6, static_cast<double>(sensorPtr[x]));
            biasSum += signedError;
        }
    }

    const double validCount = static_cast<double>(diagnostics.validOverlapPixels);
    diagnostics.maeMeters = static_cast<float>(absErrorSum / validCount);
    diagnostics.rmseMeters = static_cast<float>(std::sqrt(squaredErrorSum / validCount));
    diagnostics.absRel = static_cast<float>(absRelSum / validCount);
    diagnostics.biasMeters = static_cast<float>(biasSum / validCount);
    diagnostics.hasComparison = true;
    return diagnostics;
}

cv::Size MeasureOverlayText(const string& text, int fontHeight, int thickness, int* baseline = nullptr) {
    if(const auto font = UiFont())
        return font->getTextSize(text, fontHeight, thickness, baseline);

    const double fallbackScale = std::max(0.45, static_cast<double>(fontHeight) / 32.0);
    return cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fallbackScale, std::max(1, thickness), baseline);
}

void DrawOverlayText(cv::Mat& image,
                     const string& text,
                     const cv::Point& topLeft,
                     const cv::Scalar& color,
                     int fontHeight = 24,
                     int thickness = 1) {
    int baseline = 0;
    const cv::Size textSize = MeasureOverlayText(text, fontHeight, thickness, &baseline);

    if(const auto font = UiFont()) {
        font->putText(image, text, cv::Point(topLeft.x, topLeft.y + textSize.height),
                      fontHeight, color, thickness, cv::LINE_AA, true);
        return;
    }

    const double fallbackScale = std::max(0.45, static_cast<double>(fontHeight) / 32.0);
    cv::putText(image, text, cv::Point(topLeft.x, topLeft.y + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX, fallbackScale, color, std::max(1, thickness), cv::LINE_AA);
}

void DrawLabel(cv::Mat& image, const string& text, const cv::Point& origin = cv::Point(10, 10)) {
    const int fontHeight = 24;
    const int thickness = 1;
    int baseline = 0;
    const cv::Size textSize = MeasureOverlayText(text, fontHeight, thickness, &baseline);
    const cv::Rect box(origin.x, origin.y, textSize.width + 18, textSize.height + baseline + 12);
    cv::rectangle(image, box, cv::Scalar(18, 18, 18), cv::FILLED);
    DrawOverlayText(image, text, cv::Point(origin.x + 9, origin.y + 6), cv::Scalar(255, 255, 255), fontHeight, thickness);
}

void DrawCompactBadge(cv::Mat& image,
                      const string& text,
                      const cv::Point& origin,
                      const cv::Scalar& fill = cv::Scalar(20, 20, 20),
                      const cv::Scalar& stroke = cv::Scalar(80, 80, 80),
                      const cv::Scalar& color = cv::Scalar(245, 245, 245)) {
    const int fontHeight = 20;
    const int thickness = 1;
    int baseline = 0;
    const cv::Size textSize = MeasureOverlayText(text, fontHeight, thickness, &baseline);
    const cv::Rect box(origin.x, origin.y, textSize.width + 20, textSize.height + baseline + 12);
    cv::rectangle(image, box, fill, cv::FILLED);
    cv::rectangle(image, box, stroke, 1);
    DrawOverlayText(image, text, cv::Point(origin.x + 10, origin.y + 6), color, fontHeight, thickness);
}

cv::Rect ResetButtonRectForPanel(const cv::Rect& panelRect) {
    return cv::Rect(panelRect.x + panelRect.width - 168, panelRect.y + 16, 148, 42);
}

cv::Rect OverviewButtonRectForPanel(const cv::Rect& panelRect) {
    return cv::Rect(panelRect.x + panelRect.width - 168, panelRect.y + 66, 148, 42);
}

void DrawResetButton(cv::Mat& panel, const cv::Rect& panelRect) {
    const cv::Rect button = ResetButtonRectForPanel(panelRect);
    cv::rectangle(panel, button, cv::Scalar(45, 90, 45), cv::FILLED);
    cv::rectangle(panel, button, cv::Scalar(110, 220, 110), 2);
    DrawOverlayText(panel, "恢复视图", cv::Point(button.x + 18, button.y + 8), cv::Scalar(255, 255, 255), 24, 1);
}

void DrawOverviewButton(cv::Mat& panel, const cv::Rect& panelRect) {
    const cv::Rect button = OverviewButtonRectForPanel(panelRect);
    cv::rectangle(panel, button, cv::Scalar(70, 70, 95), cv::FILLED);
    cv::rectangle(panel, button, cv::Scalar(150, 180, 255), 2);
    DrawOverlayText(panel, "全局总览", cv::Point(button.x + 18, button.y + 8), cv::Scalar(255, 255, 255), 24, 1);
}

void DrawLegendBox(cv::Mat& panel) {
    const cv::Rect box(panel.cols - 270, 128, 248, 262);
    cv::rectangle(panel, box, cv::Scalar(20, 20, 20), cv::FILLED);
    cv::rectangle(panel, box, cv::Scalar(90, 90, 90), 1);

    DrawOverlayText(panel, "图例说明", cv::Point(box.x + 14, box.y + 10), cv::Scalar(255, 255, 255), 24, 1);

    const int left = box.x + 18;
    int y = box.y + 44;
    const int rowGap = 26;

    cv::rectangle(panel, cv::Rect(left, y + 4, 18, 18), cv::Scalar(220, 220, 220), cv::FILLED);
    DrawOverlayText(panel, "白色：可通行", cv::Point(left + 28, y), cv::Scalar(235, 235, 235), 18, 1);
    y += rowGap;

    cv::rectangle(panel, cv::Rect(left, y + 4, 18, 18), cv::Scalar(30, 35, 190), cv::FILLED);
    DrawOverlayText(panel, "红色：障碍物", cv::Point(left + 28, y), cv::Scalar(235, 235, 235), 18, 1);
    y += rowGap;

    cv::rectangle(panel, cv::Rect(left, y + 4, 18, 18), cv::Scalar(70, 170, 245), cv::FILLED);
    DrawOverlayText(panel, "浅蓝：避障膨胀区", cv::Point(left + 28, y), cv::Scalar(235, 235, 235), 18, 1);
    y += rowGap;

    cv::circle(panel, cv::Point(left + 9, y + 14), 7, cv::Scalar(0, 120, 255), cv::FILLED, cv::LINE_AA);
    cv::arrowedLine(panel, cv::Point(left + 20, y + 14), cv::Point(left + 42, y + 14), cv::Scalar(0, 240, 255), 2, cv::LINE_AA, 0, 0.28);
    DrawOverlayText(panel, "橙点箭头：当前位置", cv::Point(left + 52, y), cv::Scalar(235, 235, 235), 18, 1);
    y += rowGap;

    cv::circle(panel, cv::Point(left + 9, y + 14), 7, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::circle(panel, cv::Point(left + 9, y + 14), 2, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
    DrawOverlayText(panel, "黄色圆点：目标点", cv::Point(left + 28, y), cv::Scalar(235, 235, 235), 18, 1);
    y += rowGap;

    cv::line(panel, cv::Point(left, y + 14), cv::Point(left + 28, y + 14), cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
    DrawOverlayText(panel, "青色线：规划路径", cv::Point(left + 36, y), cv::Scalar(235, 235, 235), 18, 1);
    y += rowGap;

    cv::line(panel, cv::Point(left, y + 14), cv::Point(left + 28, y + 14), cv::Scalar(80, 255, 140), 2, cv::LINE_AA);
    DrawOverlayText(panel, "绿色线：运动轨迹", cv::Point(left + 36, y), cv::Scalar(235, 235, 235), 18, 1);
    y += rowGap;

    DrawOverlayText(panel, "上方 Z+，右侧 X+", cv::Point(left, y + 2), cv::Scalar(220, 220, 220), 17, 1);
    DrawOverlayText(panel, "不是正北，只是地图坐标", cv::Point(left, y + 24), cv::Scalar(220, 220, 220), 17, 1);
}

void DrawAxisHint(cv::Mat& panel, const cv::Rect& panelRect) {
    const cv::Point origin(panelRect.x + 44, panelRect.y + panelRect.height - 78);
    cv::arrowedLine(panel, origin, cv::Point(origin.x + 54, origin.y), cv::Scalar(255, 210, 80), 2, cv::LINE_AA, 0, 0.24);
    cv::arrowedLine(panel, origin, cv::Point(origin.x, origin.y - 54), cv::Scalar(120, 255, 120), 2, cv::LINE_AA, 0, 0.24);
    DrawOverlayText(panel, "X+", cv::Point(origin.x + 58, origin.y - 10), cv::Scalar(255, 210, 80), 20, 1);
    DrawOverlayText(panel, "Z+", cv::Point(origin.x - 8, origin.y - 78), cv::Scalar(120, 255, 120), 20, 1);
}

void DrawScaleBar(cv::Mat& panel, const OccupancyViewState& view, const cv::Rect& panelRect) {
    const float meters = 1.0f;
    const int pixels = static_cast<int>(std::round(meters / std::max(1e-4f, view.metersPerPixel)));
    if(pixels < 40 || pixels > panelRect.width - 120)
        return;

    const int left = panelRect.x + 28;
    const int bottom = panelRect.y + panelRect.height - 88;
    const cv::Point start(left, bottom);
    const cv::Point end(left + pixels, bottom);
    cv::line(panel, start, end, cv::Scalar(245, 245, 245), 3, cv::LINE_AA);
    cv::line(panel, start + cv::Point(0, -8), start + cv::Point(0, 8), cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
    cv::line(panel, end + cv::Point(0, -8), end + cv::Point(0, 8), cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
    DrawOverlayText(panel, "1.0m", cv::Point(left + pixels / 2 - 18, bottom - 28), cv::Scalar(235, 235, 235), 18, 1);
    DrawOverlayText(panel, "主网格 0.5m / 格", cv::Point(left, bottom + 18), cv::Scalar(220, 220, 220), 18, 1);
}

void DrawGridLines(cv::Mat& panel, const OccupancyViewState& view, const cv::Rect& panelRect) {
    const float majorStepMeters = 0.5f;
    const float pixelsPerMeter = 1.0f / std::max(1e-4f, view.metersPerPixel);
    if(majorStepMeters * pixelsPerMeter < 18.0f)
        return;

    const float leftWorld = view.centerX - 0.5f * panelRect.width * view.metersPerPixel;
    const float rightWorld = view.centerX + 0.5f * panelRect.width * view.metersPerPixel;
    const float bottomWorld = view.centerZ - 0.5f * panelRect.height * view.metersPerPixel;
    const float topWorld = view.centerZ + 0.5f * panelRect.height * view.metersPerPixel;

    const int firstVertical = static_cast<int>(std::floor(leftWorld / majorStepMeters));
    const int lastVertical = static_cast<int>(std::ceil(rightWorld / majorStepMeters));
    for(int index = firstVertical; index <= lastVertical; ++index) {
        const float xWorld = static_cast<float>(index) * majorStepMeters;
        const cv::Point top = WorldToPixel(view, panelRect, xWorld, topWorld);
        const cv::Point bottom = WorldToPixel(view, panelRect, xWorld, bottomWorld);
        const cv::Scalar color = (index == 0) ? cv::Scalar(90, 90, 120) : cv::Scalar(40, 40, 40);
        cv::line(panel, top, bottom, color, 1, cv::LINE_AA);
    }

    const int firstHorizontal = static_cast<int>(std::floor(bottomWorld / majorStepMeters));
    const int lastHorizontal = static_cast<int>(std::ceil(topWorld / majorStepMeters));
    for(int index = firstHorizontal; index <= lastHorizontal; ++index) {
        const float zWorld = static_cast<float>(index) * majorStepMeters;
        const cv::Point left = WorldToPixel(view, panelRect, leftWorld, zWorld);
        const cv::Point right = WorldToPixel(view, panelRect, rightWorld, zWorld);
        const cv::Scalar color = (index == 0) ? cv::Scalar(90, 90, 120) : cv::Scalar(40, 40, 40);
        cv::line(panel, left, right, color, 1, cv::LINE_AA);
    }
}

cv::Mat NormalizePanelForConcat(const cv::Mat& input, int targetHeight) {
    cv::Mat panel;
    if(input.channels() == 1)
        cv::cvtColor(input, panel, cv::COLOR_GRAY2BGR);
    else if(input.channels() == 4)
        cv::cvtColor(input, panel, cv::COLOR_BGRA2BGR);
    else
        panel = input;

    if(panel.depth() != CV_8U) {
        cv::Mat converted;
        panel.convertTo(converted, CV_8U);
        panel = converted;
    }

    if(panel.rows == targetHeight)
        return panel;

    const double scale = static_cast<double>(targetHeight) / static_cast<double>(panel.rows);
    const int targetWidth = max(1, static_cast<int>(std::round(panel.cols * scale)));
    cv::Mat resized;
    cv::resize(panel, resized, cv::Size(targetWidth, targetHeight), 0.0, 0.0, cv::INTER_AREA);
    return resized;
}

cv::Mat FitPanelIntoCanvas(const cv::Mat& panel, const cv::Size& canvasSize, const cv::Scalar& background) {
    cv::Mat canvas(canvasSize, CV_8UC3, background);
    cv::Mat normalized = NormalizePanelForConcat(panel, canvasSize.height - 20);
    if(normalized.cols > canvasSize.width - 20) {
        const double scale = static_cast<double>(canvasSize.width - 20) / static_cast<double>(normalized.cols);
        cv::resize(normalized, normalized,
                   cv::Size(std::max(1, static_cast<int>(std::round(normalized.cols * scale))),
                            std::max(1, static_cast<int>(std::round(normalized.rows * scale)))),
                   0.0, 0.0, cv::INTER_AREA);
    }
    const int x = std::max(10, (canvas.cols - normalized.cols) / 2);
    const int y = std::max(10, (canvas.rows - normalized.rows) / 2);
    normalized.copyTo(canvas(cv::Rect(x, y, normalized.cols, normalized.rows)));
    return canvas;
}

string TrackingStateToString(int state) {
    switch(state) {
        case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
            return "SYSTEM_NOT_READY";
        case ORB_SLAM3::Tracking::NO_IMAGES_YET:
            return "NO_IMAGES_YET";
        case ORB_SLAM3::Tracking::NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case ORB_SLAM3::Tracking::OK:
            return "OK";
        case ORB_SLAM3::Tracking::RECENTLY_LOST:
            return "RECENTLY_LOST";
        case ORB_SLAM3::Tracking::LOST:
            return "LOST";
        case ORB_SLAM3::Tracking::OK_KLT:
            return "OK_KLT";
        default:
            return "UNKNOWN";
    }
}

bool HasAtlasFile(const string& outputPrefix) {
    const string atlasPath = "./" + outputPrefix + "_atlas.osa";
    ifstream atlas(atlasPath);
    return atlas.good();
}

string WriteAutoSettings(const PacketHeader& header, const string& outputPrefix, int port) {
    const float scaleX = static_cast<float>(header.rgbWidth) / static_cast<float>(header.depthWidth);
    const float scaleY = static_cast<float>(header.rgbHeight) / static_cast<float>(header.depthHeight);
    const float fx = header.fx * scaleX;
    const float fy = header.fy * scaleY;
    const float cx = header.cx * scaleX;
    const float cy = header.cy * scaleY;

    ostringstream pathBuilder;
    pathBuilder << "/tmp/orbslam3_iphone_rgbd_" << port << ".yaml";
    const string settingsPath = pathBuilder.str();

    ofstream out(settingsPath);
    out << "%YAML:1.0\n\n";
    out << "File.version: \"1.0\"\n\n";
    out << "Camera.type: \"PinHole\"\n\n";
    out << "Camera1.fx: " << fixed << setprecision(6) << fx << "\n";
    out << "Camera1.fy: " << fixed << setprecision(6) << fy << "\n";
    out << "Camera1.cx: " << fixed << setprecision(6) << cx << "\n";
    out << "Camera1.cy: " << fixed << setprecision(6) << cy << "\n\n";
    out << "Camera1.k1: 0.0\n";
    out << "Camera1.k2: 0.0\n";
    out << "Camera1.p1: 0.0\n";
    out << "Camera1.p2: 0.0\n";
    out << "Camera1.k3: 0.0\n\n";
    out << "Camera.width: " << header.rgbWidth << "\n";
    out << "Camera.height: " << header.rgbHeight << "\n";
    out << "Camera.fps: " << kDefaultFps << "\n";
    out << "Camera.RGB: 0\n\n";
    out << "Stereo.ThDepth: 40.0\n";
    out << "Stereo.b: 0.08\n";
    out << "RGBD.DepthMapFactor: 1000.0\n\n";
    out << "ORBextractor.nFeatures: 1600\n";
    out << "ORBextractor.scaleFactor: 1.2\n";
    out << "ORBextractor.nLevels: 8\n";
    out << "ORBextractor.iniThFAST: 15\n";
    out << "ORBextractor.minThFAST: 7\n\n";
    out << "Viewer.KeyFrameSize: 0.05\n";
    out << "Viewer.KeyFrameLineWidth: 1.0\n";
    out << "Viewer.GraphLineWidth: 0.9\n";
    out << "Viewer.PointSize: 2.0\n";
    out << "Viewer.CameraSize: 0.08\n";
    out << "Viewer.CameraLineWidth: 3.0\n";
    out << "Viewer.ViewpointX: 0.0\n";
    out << "Viewer.ViewpointY: -0.7\n";
    out << "Viewer.ViewpointZ: -2.4\n";
    out << "Viewer.ViewpointF: 500.0\n";
    out << "Viewer.imageViewScale: 1.0\n\n";
    if(kLoadAtlasOnColdStart && HasAtlasFile(outputPrefix))
        out << "System.LoadAtlasFromFile: \"" << outputPrefix << "_atlas\"\n";
    out << "System.SaveAtlasToFile: \"" << outputPrefix << "_atlas\"\n";
    out.close();
    return settingsPath;
}

vector<Eigen::Vector3f> SampleWorldPoints(const cv::Mat& depthMm,
                                          const Sophus::SE3f& Twc,
                                          float fx,
                                          float fy,
                                          float cx,
                                          float cy,
                                          vector<float>& ySamples) {
    vector<Eigen::Vector3f> worldPoints;
    worldPoints.reserve((depthMm.rows / kDepthStride) * (depthMm.cols / kDepthStride));
    ySamples.clear();
    ySamples.reserve(worldPoints.capacity());

    for(int v = 6; v < depthMm.rows - 6; v += kDepthStride) {
        for(int u = 6; u < depthMm.cols - 6; u += kDepthStride) {
            const uint16_t depthRaw = depthMm.at<uint16_t>(v, u);
            if(depthRaw == 0)
                continue;

            const float depthMeters = static_cast<float>(depthRaw) * 0.001f;
            if(depthMeters < kMinDepthMeters || depthMeters > kMaxDepthMeters)
                continue;

            const float x = (static_cast<float>(u) - cx) * depthMeters / fx;
            const float y = (static_cast<float>(v) - cy) * depthMeters / fy;
            const Eigen::Vector3f pointWorld = Twc * Eigen::Vector3f(x, y, depthMeters);
            worldPoints.push_back(pointWorld);
            ySamples.push_back(pointWorld.y());
        }
    }
    return worldPoints;
}

bool ShouldAutoFit(const OccupancyViewState& view) {
    return view.autoFit;
}

void ResetFollowView(OccupancyViewState& view) {
    view.followRobot = true;
    view.autoFit = false;
    view.metersPerPixel = view.followMetersPerPixel;
}

void SetOverviewMode(OccupancyViewState& view) {
    view.followRobot = false;
    view.autoFit = true;
}

cv::Point WorldToPixel(const OccupancyViewState& view, const cv::Rect& panelRect, float worldX, float worldZ) {
    const float pixelsPerMeter = 1.0f / std::max(1e-4f, view.metersPerPixel);
    const int px = panelRect.x + panelRect.width / 2 + static_cast<int>(std::round((worldX - view.centerX) * pixelsPerMeter));
    const int py = panelRect.y + panelRect.height / 2 - static_cast<int>(std::round((worldZ - view.centerZ) * pixelsPerMeter));
    return cv::Point(px, py);
}

cv::Point2f PixelToWorld(const OccupancyViewState& view, const cv::Rect& panelRect, const cv::Point& pixel) {
    const float worldX = view.centerX + static_cast<float>(pixel.x - (panelRect.x + panelRect.width / 2)) * view.metersPerPixel;
    const float worldZ = view.centerZ - static_cast<float>(pixel.y - (panelRect.y + panelRect.height / 2)) * view.metersPerPixel;
    return cv::Point2f(worldX, worldZ);
}

void AutoFitView(OccupancyViewState& view,
                 const OccupancyGrid2D& grid,
                 const LiveStatus& status,
                 const PlannerState& planner) {
    if(view.followRobot && status.hasPose) {
        view.centerX = status.currentTwc.translation().x();
        view.centerZ = status.currentTwc.translation().z();
        if(planner.hasGoal) {
            const cv::Point2f goalWorld = grid.CellCenterWorld(planner.goalCell);
            const float spanX = std::max(2.4f, std::fabs(goalWorld.x - view.centerX) * 2.4f + 1.0f);
            const float spanZ = std::max(2.4f, std::fabs(goalWorld.y - view.centerZ) * 2.4f + 1.0f);
            const float metersPerPixelX = spanX / static_cast<float>(std::max(1, view.panelRect.width - 140));
            const float metersPerPixelZ = spanZ / static_cast<float>(std::max(1, view.panelRect.height - 140));
            view.metersPerPixel = ClampFloat(std::max(metersPerPixelX, metersPerPixelZ), 0.01f, 0.08f);
        } else {
            view.metersPerPixel = view.followMetersPerPixel;
        }
        return;
    }

    if(!ShouldAutoFit(view))
        return;

    GridBounds bounds = grid.Bounds();
    float minX = 0.0f;
    float maxX = 0.0f;
    float minZ = 0.0f;
    float maxZ = 0.0f;
    bool valid = false;

    if(bounds.valid) {
        const cv::Point2f minPoint = grid.CellCenterWorld(GridKey{bounds.minX, bounds.minZ});
        const cv::Point2f maxPoint = grid.CellCenterWorld(GridKey{bounds.maxX, bounds.maxZ});
        minX = minPoint.x;
        maxX = maxPoint.x;
        minZ = minPoint.y;
        maxZ = maxPoint.y;
        valid = true;
    }

    if(status.hasPose) {
        const Eigen::Vector3f pose = status.currentTwc.translation();
        if(!valid) {
            minX = maxX = pose.x();
            minZ = maxZ = pose.z();
            valid = true;
        } else {
            minX = std::min(minX, pose.x());
            maxX = std::max(maxX, pose.x());
            minZ = std::min(minZ, pose.z());
            maxZ = std::max(maxZ, pose.z());
        }
    }

    if(planner.hasGoal) {
        const cv::Point2f goalPoint = grid.CellCenterWorld(planner.goalCell);
        if(!valid) {
            minX = maxX = goalPoint.x;
            minZ = maxZ = goalPoint.y;
            valid = true;
        } else {
            minX = std::min(minX, goalPoint.x);
            maxX = std::max(maxX, goalPoint.x);
            minZ = std::min(minZ, goalPoint.y);
            maxZ = std::max(maxZ, goalPoint.y);
        }
    }

    if(!valid)
        return;

    view.centerX = 0.5f * (minX + maxX);
    view.centerZ = 0.5f * (minZ + maxZ);
    const float spanX = std::max(3.0f, maxX - minX + 1.0f);
    const float spanZ = std::max(3.0f, maxZ - minZ + 1.0f);
    const float metersPerPixelX = spanX / static_cast<float>(std::max(1, view.panelRect.width - 80));
    const float metersPerPixelZ = spanZ / static_cast<float>(std::max(1, view.panelRect.height - 80));
    view.metersPerPixel = std::max(0.01f, std::max(metersPerPixelX, metersPerPixelZ));
}

bool IsBlockedForPlanning(const OccupancyGrid2D& grid, const GridKey& cell, int inflationRadius) {
    for(int dz = -inflationRadius; dz <= inflationRadius; ++dz) {
        for(int dx = -inflationRadius; dx <= inflationRadius; ++dx) {
            if(grid.IsOccupied(GridKey{cell.x + dx, cell.z + dz}))
                return true;
        }
    }
    return false;
}

vector<GridKey> PlanPathAStar(const OccupancyGrid2D& grid, const GridKey& start, const GridKey& goal) {
    struct OpenNode {
        float priority;
        GridKey cell;
    };
    struct OpenCompare {
        bool operator()(const OpenNode& a, const OpenNode& b) const {
            return a.priority > b.priority;
        }
    };

    if(start == goal)
        return vector<GridKey>{start};

    GridBounds bounds = grid.Bounds();
    if(!bounds.valid) {
        bounds.valid = true;
        bounds.minX = std::min(start.x, goal.x) - 48;
        bounds.maxX = std::max(start.x, goal.x) + 48;
        bounds.minZ = std::min(start.z, goal.z) - 48;
        bounds.maxZ = std::max(start.z, goal.z) + 48;
    } else {
        bounds.minX = std::min(bounds.minX, std::min(start.x, goal.x)) - 28;
        bounds.maxX = std::max(bounds.maxX, std::max(start.x, goal.x)) + 28;
        bounds.minZ = std::min(bounds.minZ, std::min(start.z, goal.z)) - 28;
        bounds.maxZ = std::max(bounds.maxZ, std::max(start.z, goal.z)) + 28;
    }

    priority_queue<OpenNode, vector<OpenNode>, OpenCompare> openSet;
    unordered_map<GridKey, float, GridKeyHash> gScore;
    unordered_map<GridKey, GridKey, GridKeyHash> cameFrom;

    auto heuristic = [&](const GridKey& a) {
        return std::hypot(static_cast<float>(a.x - goal.x), static_cast<float>(a.z - goal.z));
    };

    gScore[start] = 0.0f;
    openSet.push(OpenNode{heuristic(start), start});

    const array<GridKey, 8> directions = {
        GridKey{1, 0}, GridKey{-1, 0}, GridKey{0, 1}, GridKey{0, -1},
        GridKey{1, 1}, GridKey{1, -1}, GridKey{-1, 1}, GridKey{-1, -1}
    };

    while(!openSet.empty()) {
        const GridKey current = openSet.top().cell;
        openSet.pop();

        if(current == goal) {
            vector<GridKey> path;
            path.push_back(current);
            GridKey cursor = current;
            while(cameFrom.find(cursor) != cameFrom.end()) {
                cursor = cameFrom[cursor];
                path.push_back(cursor);
            }
            reverse(path.begin(), path.end());
            return path;
        }

        for(const GridKey& direction : directions) {
            const GridKey next{current.x + direction.x, current.z + direction.z};
            if(next.x < bounds.minX || next.x > bounds.maxX || next.z < bounds.minZ || next.z > bounds.maxZ)
                continue;
            if(IsBlockedForPlanning(grid, next, 2))
                continue;

            const bool diagonal = direction.x != 0 && direction.z != 0;
            const float stepCost = diagonal ? 1.4142f : 1.0f;
            const float freeBias = grid.IsKnownFree(next) ? 1.0f : 2.2f;
            const float tentative = gScore[current] + stepCost * freeBias;

            auto found = gScore.find(next);
            if(found == gScore.end() || tentative < found->second) {
                cameFrom[next] = current;
                gScore[next] = tentative;
                openSet.push(OpenNode{tentative + heuristic(next), next});
            }
        }
    }

    return {};
}

void AppendTrajectorySample(TrajectoryState& trajectory, const LiveStatus& status) {
    if(!status.hasPose)
        return;

    const cv::Point2f current(status.currentTwc.translation().x(), status.currentTwc.translation().z());
    if(!trajectory.samples.empty()) {
        const cv::Point2f previous = trajectory.samples.back();
        const float distance = std::hypot(current.x - previous.x, current.y - previous.y);
        if(distance < 0.04f)
            return;
    }

    trajectory.samples.push_back(current);
    if(trajectory.samples.size() > 4000)
        trajectory.samples.erase(trajectory.samples.begin(), trajectory.samples.begin() + 1000);
}

void MaybeUpdatePlan(const OccupancyGrid2D& grid,
                     const LiveStatus& status,
                     PlannerState& planner) {
    if(!planner.hasGoal || !status.hasPose) {
        planner.pathCells.clear();
        planner.pathValid = false;
        return;
    }

    const GridKey startCell = grid.ToCell(status.currentTwc.translation().x(), status.currentTwc.translation().z());
    if(!planner.dirty &&
       planner.pathValid &&
       planner.lastStartCell == startCell &&
       grid.version() < planner.lastGridVersion + 20) {
        return;
    }

    planner.pathCells = PlanPathAStar(grid, startCell, planner.goalCell);
    planner.pathValid = !planner.pathCells.empty();
    planner.lastStartCell = startCell;
    planner.lastGridVersion = grid.version();
    planner.dirty = false;
}

void DrawOccupancyPanel(cv::Mat& panel,
                        OccupancyViewState& view,
                        const OccupancyGrid2D& grid,
                        const LiveStatus& status,
                        const PlannerState& planner,
                        const TrajectoryState& trajectory,
                        const NavigationSettings& settings,
                        const GuidanceState& guidance,
                        const ScaleDiagnostics& diagnostics,
                        const string& bannerText,
                        bool compactChrome = false) {
    const cv::Rect previousPanelRect = view.panelRect;
    view.panelRect = cv::Rect(0, 0, panel.cols, panel.rows);
    panel.setTo(cv::Scalar(18, 18, 18));
    AutoFitView(view, grid, status, planner);
    DrawGridLines(panel, view, view.panelRect);

    const auto cells = grid.Cells();
    const float pixelsPerMeter = 1.0f / std::max(1e-4f, view.metersPerPixel);
    const int cellPixelSize = std::max(1, static_cast<int>(std::round(grid.resolution() * pixelsPerMeter)));

    for(const auto& entry : cells) {
        const GridKey& cell = entry.first;
        const float logOdds = entry.second;
        if(std::fabs(logOdds) < 0.15f)
            continue;

        const cv::Point2f centerWorld = grid.CellCenterWorld(cell);
        const cv::Point centerPixel = WorldToPixel(view, view.panelRect, centerWorld.x, centerWorld.y);
        if(!view.panelRect.contains(centerPixel))
            continue;

        cv::Scalar color(70, 70, 70);
        if(logOdds > 1.2f)
            color = cv::Scalar(30, 35, 190);
        else if(logOdds < -0.65f)
            color = cv::Scalar(220, 220, 220);

        const cv::Rect cellRect(centerPixel.x - cellPixelSize / 2,
                                centerPixel.y - cellPixelSize / 2,
                                cellPixelSize,
                                cellPixelSize);
        cv::rectangle(panel, cellRect & view.panelRect, color, cv::FILLED);
    }

    if(settings.showInflation) {
        const GridBounds bounds = grid.Bounds();
        if(bounds.valid) {
            for(int z = bounds.minZ - settings.inflationRadiusCells; z <= bounds.maxZ + settings.inflationRadiusCells; ++z) {
                for(int x = bounds.minX - settings.inflationRadiusCells; x <= bounds.maxX + settings.inflationRadiusCells; ++x) {
                    const GridKey cell{x, z};
                    if(grid.IsOccupied(cell))
                        continue;
                    if(!IsBlockedForPlanning(grid, cell, settings.inflationRadiusCells))
                        continue;

                    const cv::Point2f centerWorld = grid.CellCenterWorld(cell);
                    const cv::Point centerPixel = WorldToPixel(view, view.panelRect, centerWorld.x, centerWorld.y);
                    if(!view.panelRect.contains(centerPixel))
                        continue;

                    const cv::Rect cellRect(centerPixel.x - cellPixelSize / 2,
                                            centerPixel.y - cellPixelSize / 2,
                                            cellPixelSize,
                                            cellPixelSize);
                    cv::rectangle(panel, cellRect & view.panelRect, cv::Scalar(70, 170, 245), cv::FILLED);
                }
            }

            for(const auto& entry : cells) {
                const GridKey& cell = entry.first;
                const float logOdds = entry.second;
                if(std::fabs(logOdds) < 0.15f)
                    continue;

                const cv::Point2f centerWorld = grid.CellCenterWorld(cell);
                const cv::Point centerPixel = WorldToPixel(view, view.panelRect, centerWorld.x, centerWorld.y);
                if(!view.panelRect.contains(centerPixel))
                    continue;

                cv::Scalar color(70, 70, 70);
                if(logOdds > 1.2f)
                    color = cv::Scalar(30, 35, 190);
                else if(logOdds < -0.65f)
                    color = cv::Scalar(220, 220, 220);

                const cv::Rect cellRect(centerPixel.x - cellPixelSize / 2,
                                        centerPixel.y - cellPixelSize / 2,
                                        cellPixelSize,
                                        cellPixelSize);
                cv::rectangle(panel, cellRect & view.panelRect, color, cv::FILLED);
            }
        }
    }

    if(trajectory.samples.size() >= 2) {
        vector<cv::Point> trajectoryPixels;
        trajectoryPixels.reserve(trajectory.samples.size());
        for(const cv::Point2f& sample : trajectory.samples)
            trajectoryPixels.push_back(WorldToPixel(view, view.panelRect, sample.x, sample.y));
        for(size_t index = 1; index < trajectoryPixels.size(); ++index)
            cv::line(panel, trajectoryPixels[index - 1], trajectoryPixels[index], cv::Scalar(80, 255, 140), 2, cv::LINE_AA);
    }

    if(planner.pathValid && planner.pathCells.size() >= 2) {
        vector<cv::Point> pathPixels;
        pathPixels.reserve(planner.pathCells.size());
        for(const GridKey& cell : planner.pathCells) {
            const cv::Point2f worldPoint = grid.CellCenterWorld(cell);
            pathPixels.push_back(WorldToPixel(view, view.panelRect, worldPoint.x, worldPoint.y));
        }
        for(size_t index = 1; index < pathPixels.size(); ++index) {
            cv::line(panel, pathPixels[index - 1], pathPixels[index], cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
        }
    }

    if(planner.hasGoal) {
        const cv::Point2f goalWorld = grid.CellCenterWorld(planner.goalCell);
        const cv::Point goalPixel = WorldToPixel(view, view.panelRect, goalWorld.x, goalWorld.y);
        cv::circle(panel, goalPixel, 7, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        cv::circle(panel, goalPixel, 2, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
    }

    if(status.hasPose) {
        const Eigen::Vector3f position = status.currentTwc.translation();
        const cv::Point robotPixel = WorldToPixel(view, view.panelRect, position.x(), position.z());
        cv::circle(panel, robotPixel, 7, cv::Scalar(0, 120, 255), cv::FILLED, cv::LINE_AA);

        const Eigen::Vector3f forward = status.currentTwc.rotationMatrix() * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        const cv::Point arrowTip = WorldToPixel(view, view.panelRect,
                                                position.x() + forward.x() * 0.28f,
                                                position.z() + forward.z() * 0.28f);
        cv::arrowedLine(panel, robotPixel, arrowTip, cv::Scalar(0, 240, 255), 2, cv::LINE_AA, 0, 0.25);
    }

    if(compactChrome) {
        DrawCompactBadge(panel, "导航地图", cv::Point(18, 18));
        if(status.hasPose)
            DrawCompactBadge(panel, status.trackingState, cv::Point(18, 58), cv::Scalar(26, 44, 26), cv::Scalar(70, 120, 70));
        else
            DrawCompactBadge(panel, status.trackingState, cv::Point(18, 58), cv::Scalar(45, 32, 24), cv::Scalar(130, 95, 60));
        DrawAxisHint(panel, view.panelRect);
        DrawScaleBar(panel, view, view.panelRect);
        view.panelRect = previousPanelRect;
        return;
    }

    DrawLabel(panel, "2D 导航地图");
    DrawLabel(panel, "跟踪状态: " + status.trackingState, cv::Point(10, 54));
    {
        ostringstream info;
        info << fixed << setprecision(2)
             << "地面高度 " << status.floorY
             << "  分辨率 " << grid.resolution() << "m"
             << "  " << (status.localizationOnly ? "纯定位" : "建图");
        DrawLabel(panel, info.str(), cv::Point(10, 98));
    }
    if(!bannerText.empty())
        DrawLabel(panel, bannerText, cv::Point(10, 142));
    if(view.followRobot)
        DrawLabel(panel, "视角：跟随小车", cv::Point(10, 186));
    else if(view.autoFit)
        DrawLabel(panel, "视角：全局总览", cv::Point(10, 186));
    else
        DrawLabel(panel, "视角：手动查看", cv::Point(10, 186));

    {
        ostringstream orbDistanceText;
        orbDistanceText << fixed << setprecision(2) << "ORB 里程 " << diagnostics.orbDistanceMeters << "m";
        DrawLabel(panel, orbDistanceText.str(), cv::Point(10, 230));
    }
    {
        ostringstream phoneDistanceText;
        phoneDistanceText << fixed << setprecision(2) << "iPhone 参考里程 " << diagnostics.phoneDistanceMeters << "m";
        DrawLabel(panel, phoneDistanceText.str(), cv::Point(10, 274));
    }
    if(diagnostics.orbDistanceMeters > 0.15f && diagnostics.phoneDistanceMeters > 0.15f) {
        const float ratio = diagnostics.orbDistanceMeters / diagnostics.phoneDistanceMeters;
        ostringstream ratioText;
        ratioText << fixed << setprecision(3) << "尺度对比 ORB/iPhone = " << ratio << "x";
        DrawLabel(panel, ratioText.str(), cv::Point(10, 318));
    } else {
        DrawLabel(panel, "尺度对比：先多走一段再判断", cv::Point(10, 318));
    }
    DrawLabel(panel, "说明：仅作尺度参考，不参与建图", cv::Point(10, 362));
    DrawLabel(panel,
              settings.fixedHeightMode ? ("导航模式：车载固定高度  " + [&]() {
                    ostringstream stream;
                    stream << fixed << setprecision(2) << settings.fixedCameraHeightMeters << "m";
                    return stream.str();
                }()) : "导航模式：手持自适应高度",
              cv::Point(10, 406));
    {
        ostringstream inflationText;
        inflationText << "避障膨胀半径 " << settings.inflationRadiusCells << " 格";
        DrawLabel(panel, inflationText.str(), cv::Point(10, 450));
    }
    if(guidance.hasWaypoint) {
        ostringstream guidanceText;
        guidanceText << fixed << setprecision(2)
                     << "下一航点 " << guidance.waypointDistanceMeters << "m"
                     << "  转向 " << guidance.headingErrorDegrees << "°";
        DrawLabel(panel, guidanceText.str(), cv::Point(10, 494));
    } else if(guidance.hasGoal && !guidance.pathValid) {
        DrawLabel(panel, "下一航点：目标已设置，等待路径", cv::Point(10, 494));
    } else {
        DrawLabel(panel, "下一航点：未设置目标点", cv::Point(10, 494));
    }
    DrawLabel(panel, "引导文件：/tmp/iphone_rgbd_nav_guidance.json", cv::Point(10, 538));

    DrawResetButton(panel, view.panelRect);
    DrawOverviewButton(panel, view.panelRect);
    DrawLegendBox(panel);
    DrawAxisHint(panel, view.panelRect);
    DrawScaleBar(panel, view, view.panelRect);
    DrawOverlayText(panel, "操作：左键拖动平移  |  滚轮缩放  |  右键设置目标点",
                    cv::Point(18, panel.rows - 42), cv::Scalar(230, 230, 230), 20, 1);
    DrawOverlayText(panel, "按钮/快捷键：F 跟随小车  |  G 全局总览  |  X 清除目标  |  L 切换纯定位/建图",
                    cv::Point(18, panel.rows - 38), cv::Scalar(230, 230, 230), 18, 1);
    DrawOverlayText(panel, "H 切换手持/车载  |  [ / ] 调整固定高度  |  S 保存截图",
                    cv::Point(18, panel.rows - 16), cv::Scalar(230, 230, 230), 20, 1);
    view.panelRect = previousPanelRect;
}

void HandleOccupancyMouse(int event, int x, int y, int flags, void* userdata) {
    auto* context = static_cast<MouseContext*>(userdata);
    if(!context || !context->view || !context->planner)
        return;

    OccupancyViewState& view = *context->view;
    PlannerState& planner = *context->planner;
    const cv::Point point(x, y);
    const bool inside = view.panelRect.contains(point);
    const bool onResetButton = ResetButtonRectForPanel(view.panelRect).contains(point);
    const bool onOverviewButton = OverviewButtonRectForPanel(view.panelRect).contains(point);

    switch(event) {
        case cv::EVENT_LBUTTONDOWN:
            if(onResetButton) {
                ResetFollowView(view);
                view.dragging = false;
                return;
            }
            if(onOverviewButton) {
                SetOverviewMode(view);
                view.dragging = false;
                return;
            }
            if(inside) {
                view.dragging = true;
                view.lastMouse = point;
                view.autoFit = false;
                view.followRobot = false;
            }
            break;

        case cv::EVENT_MOUSEMOVE:
            if(view.dragging) {
                const cv::Point delta = point - view.lastMouse;
                view.centerX -= static_cast<float>(delta.x) * view.metersPerPixel;
                view.centerZ += static_cast<float>(delta.y) * view.metersPerPixel;
                view.lastMouse = point;
            }
            break;

        case cv::EVENT_LBUTTONUP:
            view.dragging = false;
            break;

        case cv::EVENT_MOUSEWHEEL:
            if(inside) {
                const int delta = cv::getMouseWheelDelta(flags);
                if(delta != 0) {
                    const float zoomFactor = std::exp(-static_cast<float>(delta) / 120.0f * 0.14f);
                    view.metersPerPixel = ClampFloat(view.metersPerPixel * zoomFactor, 0.008f, 0.25f);
                    view.autoFit = false;
                    view.followRobot = false;
                }
            }
            break;

        case cv::EVENT_RBUTTONDOWN:
            if(inside) {
                const cv::Point2f worldPoint = PixelToWorld(view, view.panelRect, point);
                planner.goalCell = GridKey{
                    static_cast<int>(std::floor(worldPoint.x / context->gridResolution)),
                    static_cast<int>(std::floor(worldPoint.y / context->gridResolution))
                };
                planner.hasGoal = true;
                planner.dirty = true;
            }
            break;

        case cv::EVENT_LBUTTONDBLCLK:
            if(inside)
                view.autoFit = true;
            break;

        default:
            break;
    }
}

cv::Mat BuildComposite(const cv::Mat& rgbPanel,
                       const cv::Mat& depthPanel,
                       cv::Mat mapPanel,
                       OccupancyViewState& view,
                       bool compactLayout = false) {
    const int targetPanelHeight = mapPanel.rows;
    cv::Mat rgb = NormalizePanelForConcat(rgbPanel, targetPanelHeight);
    cv::Mat depth = NormalizePanelForConcat(depthPanel, targetPanelHeight);
    mapPanel = NormalizePanelForConcat(mapPanel, targetPanelHeight);

    if(compactLayout) {
        const int padding = 14;
        const int gap = 10;
        const int totalWidth = rgb.cols + depth.cols + mapPanel.cols + padding * 2 + gap * 2;
        const int totalHeight = targetPanelHeight + padding * 2;
        cv::Mat composite(totalHeight, totalWidth, CV_8UC3, cv::Scalar(12, 12, 12));

        const int y = padding;
        int x = padding;
        rgb.copyTo(composite(cv::Rect(x, y, rgb.cols, rgb.rows)));
        x += rgb.cols + gap;
        depth.copyTo(composite(cv::Rect(x, y, depth.cols, depth.rows)));
        x += depth.cols + gap;
        mapPanel.copyTo(composite(cv::Rect(x, y, mapPanel.cols, mapPanel.rows)));

        view.panelRect = cv::Rect(x, y, mapPanel.cols, mapPanel.rows);
        return composite;
    }

    cv::Mat composite;
    cv::hconcat(vector<cv::Mat>{rgb, depth, mapPanel}, composite);
    view.panelRect = cv::Rect(rgb.cols + depth.cols, 0, mapPanel.cols, mapPanel.rows);
    return composite;
}

cv::Mat BuildWaitingComposite(const cv::Mat& lastRgb,
                              const cv::Mat& lastDepth,
                              OccupancyViewState& view,
                              const OccupancyGrid2D& grid,
                              const LiveStatus& status,
                              const PlannerState& planner,
                              const TrajectoryState& trajectory,
                              const NavigationSettings& settings,
                              const GuidanceState& guidance,
                              const ScaleDiagnostics& diagnostics,
                              const string& message,
                              bool compactChrome = false) {
    cv::Mat rgbPanel = lastRgb.empty() ? cv::Mat(860, 320, CV_8UC3, cv::Scalar(18, 18, 18)) : lastRgb;
    cv::Mat depthPanel = lastDepth.empty() ? cv::Mat(860, 320, CV_8UC3, cv::Scalar(20, 20, 20)) : lastDepth;
    cv::Mat mapPanel(860, 940, CV_8UC3);
    DrawOccupancyPanel(mapPanel, view, grid, status, planner, trajectory, settings, guidance, diagnostics, message, compactChrome);

    if(compactChrome) {
        DrawCompactBadge(rgbPanel, "RGB 预览", cv::Point(18, 18));
        DrawCompactBadge(depthPanel, "深度预览", cv::Point(18, 18));
        if(lastRgb.empty())
            DrawOverlayText(rgbPanel, "等待 iPhone 视频流", cv::Point(24, rgbPanel.rows / 2 - 18),
                            cv::Scalar(230, 230, 230), 24, 1);
        if(lastDepth.empty())
            DrawOverlayText(depthPanel, "深度画面会在这里更新", cv::Point(24, depthPanel.rows / 2 - 18),
                            cv::Scalar(220, 220, 220), 22, 1);
    } else {
        DrawLabel(rgbPanel, "iPhone RGB");
        DrawLabel(depthPanel, "Depth");
        if(lastRgb.empty())
            DrawOverlayText(rgbPanel, "请在 iPhone 上点 Start Stream 开始推流。", cv::Point(18, rgbPanel.rows / 2 - 18),
                            cv::Scalar(230, 230, 230), 26, 1);
        if(lastDepth.empty())
            DrawOverlayText(depthPanel, "深度预览会显示在这里。", cv::Point(18, depthPanel.rows / 2 - 18),
                            cv::Scalar(220, 220, 220), 26, 1);
    }

    cv::Mat composite = BuildComposite(rgbPanel, depthPanel, mapPanel, view, compactChrome);
    if(!compactChrome) {
        DrawOverlayText(composite, "按 Q / Esc 退出  |  在 iPhone 上点 Start Stream 恢复实时建图  |  F 跟随小车  |  G 全局总览",
                        cv::Point(18, composite.rows - 28), cv::Scalar(235, 235, 235), 22, 1);
        DrawOverlayText(composite, "截图时间 " + CurrentTimeString(),
                        cv::Point(std::max(18, composite.cols / 2 - 120), 12), cv::Scalar(235, 235, 235), 18, 1);
    }
    return composite;
}

string BuildSnapshotPath() {
    const auto timestamp = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    return CurrentWorkingDirectory() + "/iphone_rgbd_nav_snapshot_" + to_string(timestamp) + ".png";
}

string CurrentTimeString() {
    const auto now = chrono::system_clock::now();
    const std::time_t nowTime = chrono::system_clock::to_time_t(now);
    std::tm localTime {};
    localtime_r(&nowTime, &localTime);
    ostringstream stream;
    stream << put_time(&localTime, "%Y-%m-%d %H:%M:%S");
    return stream.str();
}

string BuildLatestSnapshotPath() {
    return CurrentWorkingDirectory() + "/iphone_rgbd_nav_latest.png";
}

string BuildLatestRgbPath() {
    return CurrentWorkingDirectory() + "/iphone_rgbd_nav_latest_rgb.png";
}

string BuildLatestDepthPath() {
    return CurrentWorkingDirectory() + "/iphone_rgbd_nav_latest_depth.png";
}

string BuildLatestModelDepthPath() {
    return CurrentWorkingDirectory() + "/iphone_rgbd_nav_latest_model_depth.png";
}

string BuildLatestDepthDiffPath() {
    return CurrentWorkingDirectory() + "/iphone_rgbd_nav_latest_depth_diff.png";
}

string BuildLatestMapPath() {
    return CurrentWorkingDirectory() + "/iphone_rgbd_nav_latest_map.png";
}

string BuildLatestMapDataPath() {
    return CurrentWorkingDirectory() + "/iphone_rgbd_nav_map.json";
}

cv::Mat BuildAppPreviewPlaceholder(const cv::Size& size, const string& title, const string& message) {
    cv::Mat image(size, CV_8UC3, cv::Scalar(18, 18, 18));
    DrawCompactBadge(image, title, cv::Point(18, 18));
    int baseline = 0;
    const cv::Size messageSize = MeasureOverlayText(message, 24, 1, &baseline);
    const cv::Point origin(std::max(18, (image.cols - messageSize.width) / 2),
                           std::max(48, (image.rows - messageSize.height) / 2));
    DrawOverlayText(image, message, origin, cv::Scalar(220, 220, 220), 24, 1);
    return image;
}

string JsonEscape(const string& input) {
    string output;
    output.reserve(input.size() + 8);
    for(const char ch : input) {
        switch(ch) {
            case '\\':
                output += "\\\\";
                break;
            case '"':
                output += "\\\"";
                break;
            case '\n':
                output += "\\n";
                break;
            case '\r':
                output += "\\r";
                break;
            case '\t':
                output += "\\t";
                break;
            default:
                output += ch;
                break;
        }
    }
    return output;
}

void WriteCellJsonArray(ostream& out, const string& key, const vector<GridKey>& cells, bool trailingComma) {
    out << "  \"" << key << "\": [";
    for(size_t index = 0; index < cells.size(); ++index) {
        if(index > 0)
            out << ",";
        out << "{\"x\":" << cells[index].x << ",\"z\":" << cells[index].z << "}";
    }
    out << "]";
    if(trailingComma)
        out << ",";
    out << "\n";
}

void WriteWorldPointJsonArray(ostream& out, const string& key, const vector<cv::Point2f>& points, bool trailingComma) {
    out << "  \"" << key << "\": [";
    for(size_t index = 0; index < points.size(); ++index) {
        if(index > 0)
            out << ",";
        out << "{\"x\":" << fixed << setprecision(4) << points[index].x
            << ",\"z\":" << fixed << setprecision(4) << points[index].y << "}";
    }
    out << "]";
    if(trailingComma)
        out << ",";
    out << "\n";
}

void WriteMapRenderData(const string& path,
                        const OccupancyGrid2D& grid,
                        const OccupancyViewState& viewState,
                        const LiveStatus& status,
                        const PlannerState& planner,
                        const TrajectoryState& trajectory,
                        const NavigationSettings& settings) {
    static auto lastWriteTime = chrono::steady_clock::now() - chrono::seconds(10);
    const auto now = chrono::steady_clock::now();
    if(now - lastWriteTime < chrono::milliseconds(250))
        return;
    lastWriteTime = now;

    vector<GridKey> freeCells;
    vector<GridKey> occupiedCells;
    const auto cells = grid.Cells();
    freeCells.reserve(cells.size());
    occupiedCells.reserve(cells.size());
    for(const auto& entry : cells) {
        const GridKey& cell = entry.first;
        const float logOdds = entry.second;
        if(std::fabs(logOdds) < 0.15f)
            continue;
        if(logOdds > 1.2f)
            occupiedCells.push_back(cell);
        else if(logOdds < -0.65f)
            freeCells.push_back(cell);
    }

    vector<GridKey> inflatedCells;
    if(settings.showInflation) {
        const GridBounds bounds = grid.Bounds();
        if(bounds.valid) {
            for(int z = bounds.minZ - settings.inflationRadiusCells; z <= bounds.maxZ + settings.inflationRadiusCells; ++z) {
                for(int x = bounds.minX - settings.inflationRadiusCells; x <= bounds.maxX + settings.inflationRadiusCells; ++x) {
                    const GridKey cell{x, z};
                    if(grid.IsOccupied(cell))
                        continue;
                    if(IsBlockedForPlanning(grid, cell, settings.inflationRadiusCells))
                        inflatedCells.push_back(cell);
                }
            }
        }
    }

    vector<cv::Point2f> pathPoints;
    pathPoints.reserve(planner.pathCells.size());
    for(const GridKey& cell : planner.pathCells)
        pathPoints.push_back(grid.CellCenterWorld(cell));

    ofstream out(path);
    if(!out.good())
        return;

    out << "{\n";
    out << "  \"timestamp\": \"" << JsonEscape(CurrentTimeString()) << "\",\n";
    out << "  \"resolution\": " << fixed << setprecision(4) << grid.resolution() << ",\n";
    out << "  \"metersPerPixel\": " << fixed << setprecision(6) << viewState.metersPerPixel << ",\n";
    out << "  \"viewCenterX\": " << fixed << setprecision(4) << viewState.centerX << ",\n";
    out << "  \"viewCenterZ\": " << fixed << setprecision(4) << viewState.centerZ << ",\n";
    out << "  \"showInflation\": " << (settings.showInflation ? "true" : "false") << ",\n";
    out << "  \"inflationRadiusCells\": " << settings.inflationRadiusCells << ",\n";
    out << "  \"hasPose\": " << (status.hasPose ? "true" : "false") << ",\n";
    if(status.hasPose) {
        const Eigen::Vector3f position = status.currentTwc.translation();
        const Eigen::Vector3f forward = status.currentTwc.rotationMatrix() * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        out << "  \"robot\": {\"x\": " << fixed << setprecision(4) << position.x()
            << ", \"z\": " << fixed << setprecision(4) << position.z()
            << ", \"fx\": " << fixed << setprecision(4) << forward.x()
            << ", \"fz\": " << fixed << setprecision(4) << forward.z() << "},\n";
    } else {
        out << "  \"robot\": null,\n";
    }
    if(planner.hasGoal) {
        const cv::Point2f goalWorld = grid.CellCenterWorld(planner.goalCell);
        out << "  \"goal\": {\"x\": " << fixed << setprecision(4) << goalWorld.x
            << ", \"z\": " << fixed << setprecision(4) << goalWorld.y << "},\n";
    } else {
        out << "  \"goal\": null,\n";
    }
    WriteCellJsonArray(out, "freeCells", freeCells, true);
    WriteCellJsonArray(out, "occupiedCells", occupiedCells, true);
    WriteCellJsonArray(out, "inflatedCells", inflatedCells, true);
    WriteWorldPointJsonArray(out, "trajectory", trajectory.samples, true);
    WriteWorldPointJsonArray(out, "path", pathPoints, false);
    out << "}\n";
}

void SaveSnapshot(const cv::Mat& image) {
    const string snapshotPath = BuildSnapshotPath();
    cv::imwrite(snapshotPath, image);
    cout << "Saved navigation snapshot to " << snapshotPath << endl;
    const string latestSnapshotPath = BuildLatestSnapshotPath();
    cv::imwrite(latestSnapshotPath, image);
    cout << "Updated latest navigation snapshot at " << latestSnapshotPath << endl;
}

void UpdateLatestArtifacts(const cv::Mat& image,
                          const cv::Mat& rgbPanel,
                          const cv::Mat& depthPanel,
                          const cv::Mat& mapPanel,
                          const cv::Mat& modelDepthPanel,
                          const cv::Mat& depthDiffPanel) {
    static auto lastWriteTime = chrono::steady_clock::now() - chrono::seconds(10);
    const auto now = chrono::steady_clock::now();
    if(now - lastWriteTime < chrono::seconds(2))
        return;
    lastWriteTime = now;
    cv::imwrite(BuildLatestSnapshotPath(), image);
    cv::imwrite(BuildLatestRgbPath(), rgbPanel);
    cv::imwrite(BuildLatestDepthPath(), depthPanel);
    cv::imwrite(BuildLatestMapPath(), mapPanel);
    if(!modelDepthPanel.empty())
        cv::imwrite(BuildLatestModelDepthPath(), modelDepthPanel);
    if(!depthDiffPanel.empty())
        cv::imwrite(BuildLatestDepthDiffPath(), depthDiffPanel);
}

}  // namespace

int main(int argc, char** argv) {
    if(argc < 2 || argc > 4) {
        cerr << endl
             << "Usage: ./rgbd_iphone_stream path_to_vocabulary [port] [output_prefix]" << endl;
        return 1;
    }

    const string vocabularyPath = argv[1];
    const int port = argc >= 3 ? stoi(argv[2]) : 9000;
    const string outputPrefix = argc >= 4 ? string(argv[3]) : string("iphone_rgbd_orbslam3");
    const bool appMode = GetEnvFlag("ORB_NAV_APP_MODE");
    const string controlPath = GetEnvOrDefault("ORB_NAV_CONTROL_PATH", "/tmp/iphone_rgbd_nav_control.json");
    const string statePath = GetEnvOrDefault("ORB_NAV_STATE_PATH", "/tmp/iphone_rgbd_nav_state.json");
#ifdef __APPLE__
    const string depthModelPath = GetEnvOrDefault("ORB_NAV_DEPTH_MODEL_PATH", "models/DepthAnythingV2SmallF16.mlpackage");
#endif

    struct sigaction sigIntHandler {};
    sigIntHandler.sa_handler = ExitLoopHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    int serverFd = -1;
    try {
        serverFd = CreateListenSocket(port);
    } catch(const exception& error) {
        cerr << "Failed to start server: " << error.what() << endl;
        return 1;
    }

    PrintCandidateAddresses(port);

    OccupancyGrid2D grid(kGridResolutionMeters);
    FloorEstimator floorEstimator;
    NavigationFrameEstimator navFrameEstimator;
    OccupancyViewState viewState;
    PlannerState planner;
    LiveStatus liveStatus;
    TrajectoryState trajectory;
    NavigationSettings navigationSettings;
    GuidanceState guidanceState;
    ScaleDiagnostics scaleDiagnostics;
    DepthComparisonDiagnostics depthComparison;
    MouseContext mouseContext{&viewState, &planner, grid.resolution()};
    int lastControlRevision = -1;

#ifdef __APPLE__
    std::unique_ptr<CoreMLDepthEstimator> depthEstimator;
    depthComparison.modelEnabled = true;
    depthEstimator = std::make_unique<CoreMLDepthEstimator>(depthModelPath);
    depthComparison.modelReady = depthEstimator->IsReady();
    depthComparison.status = depthComparison.modelReady ? "READY" : "MODEL_UNAVAILABLE";
    if(!depthComparison.modelReady)
        cerr << "Depth comparison model unavailable: " << depthEstimator->Error() << endl;
#endif

    if(!appMode) {
        cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(kWindowName, HandleOccupancyMouse, &mouseContext);
    }

    cv::Mat lastRgbPanel;
    cv::Mat lastDepthPanel;
    cv::Mat lastModelDepthPanel;
    cv::Mat lastDepthDiffPanel;
    unique_ptr<ORB_SLAM3::System> slam;
    float imageScale = 1.0f;
    bool localizationOnly = false;

    while(gKeepRunning) {
        cv::Mat waitingRgb = lastRgbPanel.empty() ? cv::Mat(860, 320, CV_8UC3, cv::Scalar(18, 18, 18)) : lastRgbPanel;
        cv::Mat waitingDepth = lastDepthPanel.empty() ? cv::Mat(860, 320, CV_8UC3, cv::Scalar(20, 20, 20)) : lastDepthPanel;
        cv::Mat waitingModelDepthExport = lastModelDepthPanel.empty() ? BuildAppPreviewPlaceholder(cv::Size(640, 480), "Small V2 深度", "模型深度会在这里更新")
                                                                      : lastModelDepthPanel.clone();
        cv::Mat waitingDepthDiffExport = lastDepthDiffPanel.empty() ? BuildAppPreviewPlaceholder(cv::Size(640, 480), "深度误差", "误差热图会在这里更新")
                                                                    : lastDepthDiffPanel.clone();
        cv::Mat waitingRgbExport = lastRgbPanel.empty() ? BuildAppPreviewPlaceholder(cv::Size(640, 480), "RGB 预览", "等待 iPhone 视频流")
                                                        : lastRgbPanel.clone();
        cv::Mat waitingDepthExport = lastDepthPanel.empty() ? BuildAppPreviewPlaceholder(cv::Size(640, 480), "深度预览", "深度画面会在这里更新")
                                                            : lastDepthPanel.clone();
        cv::Mat waitingMap(860, 940, CV_8UC3);
        DrawOccupancyPanel(waitingMap, viewState, grid, liveStatus, planner, trajectory, navigationSettings, guidanceState, scaleDiagnostics,
                           HasAtlasFile(outputPrefix) ? "等待 iPhone RGB-D 流  |  已检测到历史地图" : "等待 iPhone RGB-D 流",
                           appMode);
        if(appMode) {
            DrawCompactBadge(waitingRgb, "RGB 预览", cv::Point(18, 18));
            DrawCompactBadge(waitingDepth, "深度预览", cv::Point(18, 18));
            if(lastRgbPanel.empty())
                DrawOverlayText(waitingRgb, "等待 iPhone 视频流", cv::Point(24, waitingRgb.rows / 2 - 18), cv::Scalar(230, 230, 230), 24, 1);
            if(lastDepthPanel.empty())
                DrawOverlayText(waitingDepth, "深度画面会在这里更新", cv::Point(24, waitingDepth.rows / 2 - 18), cv::Scalar(220, 220, 220), 22, 1);
        } else {
            DrawLabel(waitingRgb, "iPhone RGB");
            DrawLabel(waitingDepth, "Depth");
            if(lastRgbPanel.empty())
                DrawOverlayText(waitingRgb, "请在 iPhone 上点 Start Stream 开始推流。", cv::Point(18, waitingRgb.rows / 2 - 18),
                                cv::Scalar(230, 230, 230), 26, 1);
            if(lastDepthPanel.empty())
                DrawOverlayText(waitingDepth, "深度预览会显示在这里。", cv::Point(18, waitingDepth.rows / 2 - 18),
                                cv::Scalar(220, 220, 220), 26, 1);
        }
        cv::Mat waitingComposite = BuildComposite(waitingRgb, waitingDepth, waitingMap, viewState, appMode);
        if(!appMode) {
            DrawOverlayText(waitingComposite, "按 Q / Esc 退出  |  在 iPhone 上点 Start Stream 恢复实时建图  |  F 跟随小车  |  G 全局总览",
                            cv::Point(18, waitingComposite.rows - 28), cv::Scalar(235, 235, 235), 22, 1);
            DrawOverlayText(waitingComposite, "截图时间 " + CurrentTimeString(),
                            cv::Point(std::max(18, waitingComposite.cols / 2 - 120), 12), cv::Scalar(235, 235, 235), 18, 1);
        }
        UpdateLatestArtifacts(waitingComposite, waitingRgbExport, waitingDepthExport, waitingMap, waitingModelDepthExport, waitingDepthDiffExport);
        WriteMapRenderData(BuildLatestMapDataPath(), grid, viewState, liveStatus, planner, trajectory, navigationSettings);
        WriteBackendState(statePath, liveStatus, planner, guidanceState, navigationSettings, scaleDiagnostics, depthComparison,
                          viewState, viewState.panelRect, waitingComposite.size(), false);

        bool requestSnapshot = false;
        bool requestQuit = false;
        BackendControl control;
        if(ReadBackendControl(controlPath, control))
            ApplyBackendControl(control, lastControlRevision, viewState, planner, navigationSettings, grid, localizationOnly, slam.get(), requestSnapshot, requestQuit);
        if(requestSnapshot)
            SaveSnapshot(waitingComposite);
        if(requestQuit)
            break;

        if(!appMode) {
            cv::imshow(kWindowName, waitingComposite);
            const int waitingKey = cv::waitKey(16);
            if(waitingKey == 27 || waitingKey == 'q' || waitingKey == 'Q')
                break;
            if(waitingKey == 'f' || waitingKey == 'F')
                ResetFollowView(viewState);
            if(waitingKey == 'g' || waitingKey == 'G')
                SetOverviewMode(viewState);
            if(waitingKey == 'x' || waitingKey == 'X') {
                planner.hasGoal = false;
                planner.pathCells.clear();
                planner.pathValid = false;
                planner.dirty = false;
            }
            if(waitingKey == 'h' || waitingKey == 'H')
                navigationSettings.fixedHeightMode = !navigationSettings.fixedHeightMode;
            if(waitingKey == '[' || waitingKey == '{')
                navigationSettings.fixedCameraHeightMeters = ClampFloat(navigationSettings.fixedCameraHeightMeters - 0.02f, 0.10f, 1.20f);
            if(waitingKey == ']' || waitingKey == '}')
                navigationSettings.fixedCameraHeightMeters = ClampFloat(navigationSettings.fixedCameraHeightMeters + 0.02f, 0.10f, 1.20f);
        }

        try {
            if(!WaitForSocketReadable(serverFd, 120))
                continue;
        } catch(const exception& error) {
            cerr << "Socket wait failed: " << error.what() << endl;
            break;
        }

        sockaddr_in clientAddress {};
        socklen_t clientLength = sizeof(clientAddress);
        const int clientFd = accept(serverFd, reinterpret_cast<sockaddr*>(&clientAddress), &clientLength);
        if(clientFd < 0) {
            if(errno == EINTR && !gKeepRunning)
                break;
            cerr << "Failed to accept iPhone connection." << endl;
            continue;
        }

        char clientIp[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &clientAddress.sin_addr, clientIp, sizeof(clientIp));
        cout << "Accepted connection from " << clientIp << ":" << ntohs(clientAddress.sin_port) << endl;
        SetSocketTimeout(clientFd, 1800);

        StreamFrame firstFrame;
        bool haveFirstRgbdFrame = false;
        while(gKeepRunning && !haveFirstRgbdFrame) {
            try {
                if(!ReceiveFrame(clientFd, firstFrame)) {
                    cerr << "Stream closed before the first RGB-D frame arrived." << endl;
                    break;
                }
                if(firstFrame.depthMm.empty()) {
                    cerr << "Waiting for first RGB-D depth payload..." << endl;
                    continue;
                }
                haveFirstRgbdFrame = true;
            } catch(const exception& error) {
                cerr << "Failed to receive first frame: " << error.what() << endl;
                break;
            }
        }
        if(!haveFirstRgbdFrame) {
            close(clientFd);
            continue;
        }

        const string settingsPath = WriteAutoSettings(firstFrame.header, outputPrefix, port);
        cout << "Auto-generated settings: " << settingsPath << endl;
        cout << "RGB frame size: " << firstFrame.header.rgbWidth << "x" << firstFrame.header.rgbHeight
             << ", depth size: " << firstFrame.header.depthWidth << "x" << firstFrame.header.depthHeight
             << ", depth bytes: " << firstFrame.header.depthSize
             << ", phone pose: " << (firstFrame.header.hasPose ? "yes" : "no") << endl;

        if(!slam) {
            slam = make_unique<ORB_SLAM3::System>(vocabularyPath, settingsPath, ORB_SLAM3::System::RGBD, false, 0, outputPrefix);
            imageScale = slam->GetImageScale();
        } else {
            cout << "Reconnected RGB-D stream. Continuing current ORB-SLAM3 session." << endl;
        }

        int frameIndex = 0;
        bool sessionDisconnected = false;
        StreamFrame currentFrame = std::move(firstFrame);

        while(gKeepRunning) {
            cv::Mat slamRgb = currentFrame.rgb;
            cv::Mat slamDepth = ResizeDepthToRgb(currentFrame, currentFrame.rgb.size());
            if(imageScale != 1.0f) {
                const int scaledWidth = static_cast<int>(std::round(slamRgb.cols * imageScale));
                const int scaledHeight = static_cast<int>(std::round(slamRgb.rows * imageScale));
                cv::resize(slamRgb, slamRgb, cv::Size(scaledWidth, scaledHeight), 0.0, 0.0, cv::INTER_LINEAR);
                cv::resize(slamDepth, slamDepth, cv::Size(scaledWidth, scaledHeight), 0.0, 0.0, cv::INTER_NEAREST);
            }

            cv::Mat modelDepthPreview;
            cv::Mat depthDiffPreview;
            cv::Mat alignedModelDepthMeters;
            cv::Mat depthErrorMeters;
            cv::Mat depthCompareMask;
#ifdef __APPLE__
            {
                DepthComparisonDiagnostics frameDepthComparison;
                frameDepthComparison.modelEnabled = true;
                frameDepthComparison.modelReady = depthEstimator && depthEstimator->IsReady();
                frameDepthComparison.status = frameDepthComparison.modelReady ? "READY" : "MODEL_UNAVAILABLE";

                if(frameDepthComparison.modelReady) {
                    cv::Mat modelDepth32f;
                    double inferenceMs = 0.0;
                    if(depthEstimator->Infer(currentFrame.rgb, modelDepth32f, inferenceMs)) {
                        cv::Mat modelDepthAtSensor;
                        cv::resize(modelDepth32f, modelDepthAtSensor, currentFrame.depthMm.size(), 0.0, 0.0, cv::INTER_LINEAR);
                        modelDepthPreview = RotateForPreview(ColorizeDepthFloat(modelDepthAtSensor), currentFrame.header.displayOrientation);

                        frameDepthComparison = CompareDepthMaps(currentFrame.depthMm, modelDepthAtSensor,
                                                                alignedModelDepthMeters, depthErrorMeters, depthCompareMask);
                        frameDepthComparison.modelEnabled = true;
                        frameDepthComparison.modelReady = true;
                        frameDepthComparison.inferenceMs = static_cast<float>(inferenceMs);
                        frameDepthComparison.status = frameDepthComparison.hasComparison ? "READY" : "INSUFFICIENT_OVERLAP";

                        if(frameDepthComparison.hasComparison) {
                            modelDepthPreview = RotateForPreview(
                                ColorizeDepthFloat(alignedModelDepthMeters, depthCompareMask),
                                currentFrame.header.displayOrientation);
                            depthDiffPreview = RotateForPreview(
                                ColorizeDepthError(depthErrorMeters, depthCompareMask),
                                currentFrame.header.displayOrientation);
                        }
                    } else {
                        frameDepthComparison.status = "INFERENCE_FAILED";
                    }
                }
                depthComparison = frameDepthComparison;
            }
#else
            depthComparison = DepthComparisonDiagnostics{};
#endif

            const float scaleX = static_cast<float>(slamRgb.cols) / static_cast<float>(currentFrame.header.rgbWidth);
            const float scaleY = static_cast<float>(slamRgb.rows) / static_cast<float>(currentFrame.header.rgbHeight);
            const float fx = (currentFrame.header.fx * static_cast<float>(currentFrame.header.rgbWidth) / static_cast<float>(currentFrame.header.depthWidth)) * scaleX;
            const float fy = (currentFrame.header.fy * static_cast<float>(currentFrame.header.rgbHeight) / static_cast<float>(currentFrame.header.depthHeight)) * scaleY;
            const float cx = (currentFrame.header.cx * static_cast<float>(currentFrame.header.rgbWidth) / static_cast<float>(currentFrame.header.depthWidth)) * scaleX;
            const float cy = (currentFrame.header.cy * static_cast<float>(currentFrame.header.rgbHeight) / static_cast<float>(currentFrame.header.depthHeight)) * scaleY;

            const Sophus::SE3f Tcw = slam->TrackRGBD(slamRgb, slamDepth, currentFrame.header.timestamp);
            const int trackingState = slam->GetTrackingState();
            liveStatus.trackingState = TrackingStateToString(trackingState);
            liveStatus.localizationOnly = localizationOnly;
            liveStatus.hasPose = (trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT);
            if(liveStatus.hasPose) {
                const Sophus::SE3f orbTwc = Tcw.inverse();
                UpdateScaleDiagnostics(scaleDiagnostics, orbTwc, currentFrame.header);
                navFrameEstimator.Observe(orbTwc, currentFrame.header);
                liveStatus.currentTwc = navFrameEstimator.TransformPose(orbTwc);
            }

            if(liveStatus.hasPose) {
                vector<float> ySamples;
                const Sophus::SE3f orbTwc = Tcw.inverse();
                const vector<Eigen::Vector3f> orbWorldPoints = SampleWorldPoints(slamDepth, orbTwc, fx, fy, cx, cy, ySamples);
                const vector<Eigen::Vector3f> navWorldPoints = navFrameEstimator.TransformPoints(orbWorldPoints);
                vector<float> navYSamples;
                navYSamples.reserve(navWorldPoints.size());
                for(const Eigen::Vector3f& point : navWorldPoints)
                    navYSamples.push_back(point.y());
                floorEstimator.ObserveFrame(navYSamples, liveStatus.currentTwc.translation().y(), navigationSettings);
                liveStatus.floorY = floorEstimator.FloorY(liveStatus.currentTwc.translation().y());
                grid.IntegratePoints(liveStatus.currentTwc.translation(), navWorldPoints, liveStatus.floorY);
                AppendTrajectorySample(trajectory, liveStatus);
            }

            MaybeUpdatePlan(grid, liveStatus, planner);
            guidanceState = ComputeGuidanceState(grid, liveStatus, planner, navigationSettings);
            UpdateGuidanceFile(guidanceState, liveStatus, navigationSettings);

            cv::Mat rgbPreview = RotateForPreview(slamRgb, currentFrame.header.displayOrientation);
            cv::Mat rgbPanel = rgbPreview.clone();
            if(appMode) {
                DrawCompactBadge(rgbPanel, "RGB 预览", cv::Point(18, 18));
            } else {
                DrawLabel(rgbPanel, "iPhone RGB-D 画面");
                if(liveStatus.hasPose) {
                    const Eigen::Vector3f p = liveStatus.currentTwc.translation();
                    ostringstream poseText;
                    poseText << fixed << setprecision(2) << "位姿 x " << p.x() << "  y " << p.y() << "  z " << p.z();
                    DrawLabel(rgbPanel, poseText.str(), cv::Point(10, 54));
                    DrawLabel(rgbPanel, navFrameEstimator.HasAlignment() ? "导航坐标: 已对齐重力" : "导航坐标: 未对齐", cv::Point(10, 98));
                }
            }

            cv::Mat depthPreview = RotateForPreview(ColorizeDepthMm(slamDepth), currentFrame.header.displayOrientation);
            cv::Mat depthPanel = depthPreview.clone();
            if(appMode) {
                DrawCompactBadge(depthPanel, "深度预览", cv::Point(18, 18));
            } else {
                DrawLabel(depthPanel, "深度图");
                {
                    ostringstream depthText;
                    depthText << "状态 " << liveStatus.trackingState;
                    DrawLabel(depthPanel, depthText.str(), cv::Point(10, 54));
                }
                if(depthComparison.hasComparison) {
                    ostringstream compareText;
                    compareText << fixed << setprecision(3)
                                << "Small V2 MAE " << depthComparison.maeMeters << "m"
                                << "  RMSE " << depthComparison.rmseMeters << "m";
                    DrawLabel(depthPanel, compareText.str(), cv::Point(10, 98));
                }
            }

            cv::Mat mapPanel(860, 940, CV_8UC3);
            string mapBanner = planner.hasGoal ? (planner.pathValid ? "已设置目标点  |  路径已规划" : "已设置目标点  |  正在等待路径")
                                               : "在地图上右键设置目标点";
            DrawOccupancyPanel(mapPanel, viewState, grid, liveStatus, planner, trajectory, navigationSettings, guidanceState, scaleDiagnostics, mapBanner, appMode);

            lastRgbPanel = rgbPreview.clone();
            lastDepthPanel = depthPreview.clone();
            lastModelDepthPanel = modelDepthPreview.clone();
            lastDepthDiffPanel = depthDiffPreview.clone();

            cv::Mat composite = BuildComposite(rgbPanel, depthPanel, mapPanel, viewState, appMode);
            if(!appMode) {
                DrawOverlayText(composite, "按 Q / Esc 退出  |  S 保存截图  |  已自动刷新 latest 截图  |  L 切换纯定位/建图  |  F 恢复视图",
                                cv::Point(18, composite.rows - 28), cv::Scalar(235, 235, 235), 24, 1);
                DrawOverlayText(composite, "截图时间 " + CurrentTimeString(),
                                cv::Point(std::max(18, composite.cols / 2 - 120), 12), cv::Scalar(235, 235, 235), 18, 1);
            }
            UpdateLatestArtifacts(composite, rgbPreview, depthPreview, mapPanel, modelDepthPreview, depthDiffPreview);
            WriteMapRenderData(BuildLatestMapDataPath(), grid, viewState, liveStatus, planner, trajectory, navigationSettings);
            WriteBackendState(statePath, liveStatus, planner, guidanceState, navigationSettings, scaleDiagnostics, depthComparison,
                              viewState, viewState.panelRect, composite.size(), true);

            bool requestSnapshot = false;
            bool requestQuit = false;
            BackendControl control;
            if(ReadBackendControl(controlPath, control))
                ApplyBackendControl(control, lastControlRevision, viewState, planner, navigationSettings, grid, localizationOnly, slam.get(), requestSnapshot, requestQuit);
            if(requestSnapshot)
                SaveSnapshot(composite);
            if(requestQuit) {
                gKeepRunning = false;
                break;
            }

            if(!appMode) {
                cv::imshow(kWindowName, composite);
                const int key = cv::waitKey(1);
                if(key == 27 || key == 'q' || key == 'Q') {
                    gKeepRunning = false;
                    break;
                }
                if(key == 'f' || key == 'F')
                    ResetFollowView(viewState);
                if(key == 'g' || key == 'G')
                    SetOverviewMode(viewState);
                if(key == 's' || key == 'S')
                    SaveSnapshot(composite);
                if(key == 'x' || key == 'X') {
                    planner.hasGoal = false;
                    planner.pathCells.clear();
                    planner.pathValid = false;
                    planner.dirty = false;
                }
                if(key == 'h' || key == 'H')
                    navigationSettings.fixedHeightMode = !navigationSettings.fixedHeightMode;
                if(key == '[' || key == '{')
                    navigationSettings.fixedCameraHeightMeters = ClampFloat(navigationSettings.fixedCameraHeightMeters - 0.02f, 0.10f, 1.20f);
                if(key == ']' || key == '}')
                    navigationSettings.fixedCameraHeightMeters = ClampFloat(navigationSettings.fixedCameraHeightMeters + 0.02f, 0.10f, 1.20f);
                if(key == 'l' || key == 'L') {
                    localizationOnly = !localizationOnly;
                    if(localizationOnly)
                        slam->ActivateLocalizationMode();
                    else
                        slam->DeactivateLocalizationMode();
                }
            }

            StreamFrame nextFrame;
            try {
                if(!ReceiveFrame(clientFd, nextFrame)) {
                    cerr << "iPhone stream disconnected." << endl;
                    sessionDisconnected = true;
                    break;
                }
                if(nextFrame.depthMm.empty()) {
                    cerr << "Skipping frame without RGB-D depth payload." << endl;
                    continue;
                }
            } catch(const exception& error) {
                cerr << "Stream error: " << error.what() << endl;
                sessionDisconnected = true;
                break;
            }
            currentFrame = std::move(nextFrame);
            ++frameIndex;
            if(frameIndex % 20 == 0) {
                cout << "[NAV] state=" << liveStatus.trackingState
                     << "  grid_version=" << grid.version()
                     << "  path=" << (planner.pathValid ? planner.pathCells.size() : 0)
                     << "  fixed_h=" << fixed << setprecision(2) << navigationSettings.fixedCameraHeightMeters
                     << (navigationSettings.fixedHeightMode ? "  mode=FIXED_HEIGHT" : "  mode=ADAPTIVE_HEIGHT")
                     << "  orb_m=" << fixed << setprecision(2) << scaleDiagnostics.orbDistanceMeters
                     << "  phone_m=" << fixed << setprecision(2) << scaleDiagnostics.phoneDistanceMeters
                     << (localizationOnly ? "  mode=LOCALIZATION" : "  mode=SLAM")
                     << endl;
            }
        }

        close(clientFd);
        if(gKeepRunning && sessionDisconnected) {
            cout << "Session paused. Waiting for the next iPhone RGB-D stream..." << endl;
            liveStatus.trackingState = "PAUSED";
        }
    }

    if(slam)
        slam->Shutdown();

    if(serverFd >= 0)
        close(serverFd);
    if(!appMode)
        cv::destroyAllWindows();
    return 0;
}
