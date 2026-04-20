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
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <condition_variable>
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
constexpr char kDefaultEventLogPath[] = "/tmp/iphone_rgbd_nav_backend.log";
constexpr uint32_t kMaxPacketHeaderBytes = 64 * 1024;
constexpr int kMaxFrameDimension = 4096;
constexpr size_t kMaxRgbPayloadBytes = 8 * 1024 * 1024;
constexpr size_t kMaxDepthPayloadBytes = 16 * 1024 * 1024;
constexpr float kGridResolutionMeters = 0.05f;
constexpr float kMinDepthMeters = 0.15f;
constexpr float kMaxDepthMeters = 4.5f;
constexpr int kDepthStride = 6;
constexpr float kDepthDiffVisualMaxMeters = 1.00f;
constexpr float kFloorClearanceMeters = 0.05f;
constexpr float kObstacleHeightMeters = 1.10f;
constexpr float kAssumedCameraHeightMeters = 0.30f;
constexpr bool kLoadAtlasOnColdStart = false;
constexpr auto kAppLatestArtifactsInterval = chrono::milliseconds(150);
constexpr auto kDefaultLatestArtifactsInterval = chrono::seconds(2);
constexpr auto kAppMapDataInterval = chrono::milliseconds(180);
constexpr auto kDefaultMapDataInterval = chrono::milliseconds(750);
constexpr int kSensorModeModelEvalStride = 5;
constexpr int kPerformanceSummaryFrames = 20;
constexpr float kScaleRatioWarnMin = 0.75f;
constexpr float kScaleRatioWarnMax = 1.25f;
constexpr float kScaleRatioWarnDistanceMeters = 1.0f;
constexpr double kScaleRatioWindowSeconds = 3.0;
constexpr float kScaleRatioWindowWarnDistanceMeters = 0.35f;
constexpr double kScaleRatioMaxFrameGapSeconds = 0.35;
constexpr float kScaleRatioDiscontinuityMinMeters = 0.05f;
constexpr float kScaleRatioPairJumpFactor = 4.0f;
constexpr int kStreamWarmupTrustedFrames = 12;
constexpr double kStreamWarmupTrustedSeconds = 0.8;
constexpr double kProcessingSummaryWarnMs = 120.0;
constexpr double kTrackSummaryWarnMs = 80.0;
constexpr double kInferenceSummaryWarnMs = 80.0;
constexpr float kDepthOverlapWarnRatio = 0.20f;
constexpr float kDepthMaeWarnMeters = 0.35f;
constexpr size_t kFloorSamplesWarnCount = 48;
constexpr size_t kMapPointsWarnCount = 96;

atomic<bool> gKeepRunning{true};

enum class LogLevel {
    Info,
    Warning,
    Error
};

mutex gEventLogMutex;
string gEventLogPath;
unique_ptr<ofstream> gEventLogFile;

class OccupancyGrid2D;

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
    string arTrackingState = "unknown";
    string arTrackingReason;
    string arWorldMappingStatus = "unknown";
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

struct ScaleWindowSample {
    double timestampSeconds = 0.0;
    float orbDeltaMeters = 0.0f;
    float phoneDeltaMeters = 0.0f;
    Eigen::Vector3f orbStartPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f phoneStartPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f orbEndPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f phoneEndPosition = Eigen::Vector3f::Zero();
};

struct ScaleWindowStats {
    float orbDistanceMeters = 0.0f;
    float phoneDistanceMeters = 0.0f;
    float ratio = 0.0f;
    float orbDisplacementMeters = 0.0f;
    float phoneDisplacementMeters = 0.0f;
    float displacementRatio = 0.0f;
    double durationSeconds = 0.0;
    size_t samples = 0;
};

struct ScaleUpdateEvent {
    bool pairAccepted = false;
    bool windowReset = false;
    string resetReason;
    float orbDeltaMeters = 0.0f;
    float phoneDeltaMeters = 0.0f;
};

struct ScaleDiagnostics {
    bool hasOrbPose = false;
    bool hasPhonePose = false;
    Eigen::Vector3f previousOrbPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f previousPhonePosition = Eigen::Vector3f::Zero();
    bool hasStableAnchor = false;
    Eigen::Vector3f stableAnchorOrbPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f stableAnchorPhonePosition = Eigen::Vector3f::Zero();
    float orbDistanceMeters = 0.0f;
    float phoneDistanceMeters = 0.0f;
    float stableOrbDisplacementMeters = 0.0f;
    float stablePhoneDisplacementMeters = 0.0f;
    uint64_t orbSegments = 0;
    uint64_t phoneSegments = 0;
    deque<ScaleWindowSample> recentSamples;
    float recentOrbDistanceMeters = 0.0f;
    float recentPhoneDistanceMeters = 0.0f;
    bool previousSampleTrusted = false;
    uint64_t previousPhoneFrameIndex = 0;
    double previousTimestampSeconds = 0.0;
};

enum class DepthSourceMode {
    Sensor,
    ModelMap,
    ModelFull
};

struct NavigationSettings {
    bool fixedHeightMode = false;
    float fixedCameraHeightMeters = kAssumedCameraHeightMeters;
    int inflationRadiusCells = 3;
    float lookaheadMeters = 0.60f;
    bool showInflation = true;
    bool enableRgbPreview = false;
    bool enableDepthPreview = false;
    bool enableDepthComparison = false;
    bool enableDepthDiffPreview = false;
    DepthSourceMode depthSourceMode = DepthSourceMode::Sensor;
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

struct PerformanceWindow {
    int frames = 0;
    int captureGapSamples = 0;
    int trackSamples = 0;
    int inferenceSamples = 0;
    int mapSamples = 0;
    int renderSamples = 0;
    int poseFrames = 0;
    double totalCaptureGapMs = 0.0;
    double totalProcessingMs = 0.0;
    double totalTrackMs = 0.0;
    double totalInferenceMs = 0.0;
    double totalMapMs = 0.0;
    double totalRenderMs = 0.0;
    size_t totalFloorSamples = 0;
    size_t totalMapPoints = 0;
    uint64_t startGridVersion = 0;
    bool hasStartGridVersion = false;

    void Reset(uint64_t gridVersion) {
        *this = PerformanceWindow{};
        startGridVersion = gridVersion;
        hasStartGridVersion = true;
    }
};

struct MapQualityStats {
    size_t knownCells = 0;
    size_t occupiedCells = 0;
    size_t freeCells = 0;
    bool hasBounds = false;
    float widthMeters = 0.0f;
    float heightMeters = 0.0f;
};

struct DepthSourceState {
    string requestedMode = "sensor";
    string activeSlamSource = "sensor";
    string activeMapSource = "sensor";
    string status = "传感器深度";
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
    bool hasDepthSourceMode = false;
    DepthSourceMode depthSourceMode = DepthSourceMode::Sensor;
    bool hasEnableRgbPreview = false;
    bool enableRgbPreview = false;
    bool hasEnableDepthPreview = false;
    bool enableDepthPreview = false;
    bool hasEnableDepthComparison = false;
    bool enableDepthComparison = false;
    bool hasEnableDepthDiffPreview = false;
    bool enableDepthDiffPreview = false;
    bool setGoal = false;
    float goalWorldX = 0.0f;
    float goalWorldZ = 0.0f;
};

string CurrentTimeString();
string CurrentLogTimestamp();
string JsonEscape(const string& input);
string BuildLatestSnapshotPath();
string BuildLatestRgbPath();
string BuildLatestDepthPath();
string BuildLatestMapPath();
string BuildLatestModelDepthPath();
string BuildLatestDepthDiffPath();
void InitializeEventLog(const string& path);
void LogInfo(const string& message);
void LogWarning(const string& message);
void LogError(const string& message);
double AverageMs(double total, int count);
bool WriteTextFileAtomically(const string& path, const string& contents);
MapQualityStats ComputeMapQualityStats(const OccupancyGrid2D& grid);
float ScaleRatioForDistances(float orbDistanceMeters, float phoneDistanceMeters);
bool ScaleRatioLooksWrong(float ratio,
                          float orbDistanceMeters,
                          float phoneDistanceMeters,
                          float minDistanceMeters);
ScaleWindowStats CurrentScaleWindowStats(const ScaleDiagnostics& diagnostics);
bool IsPhoneReferenceTrusted(const PacketHeader& header);
bool ResetRecentScaleWindow(ScaleDiagnostics& diagnostics);
bool InvalidateScaleDiagnostics(ScaleDiagnostics& diagnostics, bool clearRecentWindow);
string DescribeArkitTrackingState(const PacketHeader& header);
string DescribeArkitReference(const PacketHeader& header);
void LogPerformanceSummary(const PerformanceWindow& window,
                           const OccupancyGrid2D& grid,
                           const PlannerState& planner,
                           const TrajectoryState& trajectory,
                           const ScaleDiagnostics& diagnostics,
                           const DepthComparisonDiagnostics& depthComparison,
                           const PacketHeader& latestPhoneHeader,
                           bool localizationOnly);
void WriteCellJsonArray(ostream& out, const string& key, const vector<GridKey>& cells, bool trailingComma);
void WriteWorldPointJsonArray(ostream& out, const string& key, const vector<cv::Point2f>& points, bool trailingComma);

struct LatestArtifactsPayload {
    cv::Mat image;
    cv::Mat rgbPanel;
    cv::Mat depthPanel;
    cv::Mat mapPanel;
    cv::Mat modelDepthPanel;
    cv::Mat depthDiffPanel;
    NavigationSettings settings;
};

struct MapRenderPayload {
    string path;
    float resolution = kGridResolutionMeters;
    float metersPerPixel = 0.03f;
    float viewCenterX = 0.0f;
    float viewCenterZ = 0.0f;
    bool showInflation = true;
    int inflationRadiusCells = 3;
    bool hasPose = false;
    Eigen::Vector3f robotPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f robotForward = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    bool hasGoal = false;
    cv::Point2f goalWorld;
    vector<GridKey> freeCells;
    vector<GridKey> occupiedCells;
    vector<GridKey> inflatedCells;
    vector<cv::Point2f> trajectorySamples;
    vector<cv::Point2f> pathPoints;
};

class AsyncRenderWriter {
public:
    AsyncRenderWriter() : mWorker([this] { Run(); }) {}

    ~AsyncRenderWriter() {
        {
            std::lock_guard<std::mutex> lock(mMutex);
            mStop = true;
        }
        mCondition.notify_one();
        if(mWorker.joinable())
            mWorker.join();
    }

    void SubmitLatestArtifacts(LatestArtifactsPayload payload) {
        {
            std::lock_guard<std::mutex> lock(mMutex);
            mLatestArtifacts = std::move(payload);
            mHasLatestArtifacts = true;
        }
        mCondition.notify_one();
    }

    void SubmitMapRender(MapRenderPayload payload) {
        {
            std::lock_guard<std::mutex> lock(mMutex);
            mLatestMapRender = std::move(payload);
            mHasLatestMapRender = true;
        }
        mCondition.notify_one();
    }

private:
    std::mutex mMutex;
    std::condition_variable mCondition;
    LatestArtifactsPayload mLatestArtifacts;
    MapRenderPayload mLatestMapRender;
    bool mHasLatestArtifacts = false;
    bool mHasLatestMapRender = false;
    bool mStop = false;
    std::thread mWorker;

    static void WriteLatestArtifactsNow(const LatestArtifactsPayload& payload) {
        cv::imwrite(BuildLatestSnapshotPath(), payload.image);
        if(payload.settings.enableRgbPreview && !payload.rgbPanel.empty())
            cv::imwrite(BuildLatestRgbPath(), payload.rgbPanel);
        if(payload.settings.enableDepthPreview && !payload.depthPanel.empty())
            cv::imwrite(BuildLatestDepthPath(), payload.depthPanel);
        cv::imwrite(BuildLatestMapPath(), payload.mapPanel);
        if(payload.settings.enableDepthComparison && !payload.modelDepthPanel.empty())
            cv::imwrite(BuildLatestModelDepthPath(), payload.modelDepthPanel);
        if(payload.settings.enableDepthComparison && payload.settings.enableDepthDiffPreview && !payload.depthDiffPanel.empty())
            cv::imwrite(BuildLatestDepthDiffPath(), payload.depthDiffPanel);
    }

    static void WriteMapRenderDataNow(const MapRenderPayload& payload) {
        ostringstream out;

        out << "{\n";
        out << "  \"timestamp\": \"" << JsonEscape(CurrentTimeString()) << "\",\n";
        out << "  \"resolution\": " << fixed << setprecision(4) << payload.resolution << ",\n";
        out << "  \"metersPerPixel\": " << fixed << setprecision(6) << payload.metersPerPixel << ",\n";
        out << "  \"viewCenterX\": " << fixed << setprecision(4) << payload.viewCenterX << ",\n";
        out << "  \"viewCenterZ\": " << fixed << setprecision(4) << payload.viewCenterZ << ",\n";
        out << "  \"showInflation\": " << (payload.showInflation ? "true" : "false") << ",\n";
        out << "  \"inflationRadiusCells\": " << payload.inflationRadiusCells << ",\n";
        out << "  \"hasPose\": " << (payload.hasPose ? "true" : "false") << ",\n";
        if(payload.hasPose) {
            out << "  \"robot\": {\"x\": " << fixed << setprecision(4) << payload.robotPosition.x()
                << ", \"z\": " << fixed << setprecision(4) << payload.robotPosition.z()
                << ", \"fx\": " << fixed << setprecision(4) << payload.robotForward.x()
                << ", \"fz\": " << fixed << setprecision(4) << payload.robotForward.z() << "},\n";
        } else {
            out << "  \"robot\": null,\n";
        }
        if(payload.hasGoal) {
            out << "  \"goal\": {\"x\": " << fixed << setprecision(4) << payload.goalWorld.x
                << ", \"z\": " << fixed << setprecision(4) << payload.goalWorld.y << "},\n";
        } else {
            out << "  \"goal\": null,\n";
        }
        WriteCellJsonArray(out, "freeCells", payload.freeCells, true);
        WriteCellJsonArray(out, "occupiedCells", payload.occupiedCells, true);
        WriteCellJsonArray(out, "inflatedCells", payload.inflatedCells, true);
        WriteWorldPointJsonArray(out, "trajectory", payload.trajectorySamples, true);
        WriteWorldPointJsonArray(out, "path", payload.pathPoints, false);
        out << "}\n";
        WriteTextFileAtomically(payload.path, out.str());
    }

    void Run() {
        std::unique_lock<std::mutex> lock(mMutex);
        while(true) {
            mCondition.wait(lock, [this] {
                return mStop || mHasLatestArtifacts || mHasLatestMapRender;
            });

            const bool hasArtifacts = mHasLatestArtifacts;
            const bool hasMapRender = mHasLatestMapRender;
            auto artifacts = std::move(mLatestArtifacts);
            auto mapRender = std::move(mLatestMapRender);
            mHasLatestArtifacts = false;
            mHasLatestMapRender = false;
            const bool shouldStop = mStop;

            lock.unlock();
            if(hasArtifacts)
                WriteLatestArtifactsNow(artifacts);
            if(hasMapRender)
                WriteMapRenderDataNow(mapRender);
            lock.lock();

            if(shouldStop && !mHasLatestArtifacts && !mHasLatestMapRender)
                break;
        }
    }
};

class OccupancyGrid2D;

cv::Point WorldToPixel(const OccupancyViewState& view, const cv::Rect& panelRect, float worldX, float worldZ);
cv::Point2f PixelToWorld(const OccupancyViewState& view, const cv::Rect& panelRect, const cv::Point& pixel);
bool IsBlockedForPlanning(const OccupancyGrid2D& grid, const GridKey& cell, int inflationRadius);
string CurrentTimeString();
string JsonEscape(const string& input);
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

string DepthSourceModeKey(DepthSourceMode mode) {
    switch(mode) {
        case DepthSourceMode::Sensor:
            return "sensor";
        case DepthSourceMode::ModelMap:
            return "model_map";
        case DepthSourceMode::ModelFull:
            return "model_full";
    }
    return "sensor";
}

DepthSourceMode DepthSourceModeFromKey(const string& key) {
    if(key == "model_map")
        return DepthSourceMode::ModelMap;
    if(key == "model_full")
        return DepthSourceMode::ModelFull;
    return DepthSourceMode::Sensor;
}

string DepthSourceModeLabel(DepthSourceMode mode) {
    switch(mode) {
        case DepthSourceMode::Sensor:
            return "传感器深度";
        case DepthSourceMode::ModelMap:
            return "模型深度仅地图";
        case DepthSourceMode::ModelFull:
            return "模型深度主链+地图";
    }
    return "传感器深度";
}

bool DepthSourceUsesModelForSlam(DepthSourceMode mode) {
    return mode == DepthSourceMode::ModelFull;
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

MapQualityStats ComputeMapQualityStats(const OccupancyGrid2D& grid) {
    MapQualityStats stats;
    const auto cells = grid.Cells();
    stats.knownCells = cells.size();
    for(const auto& entry : cells) {
        const float logOdds = entry.second;
        if(logOdds > 1.2f)
            ++stats.occupiedCells;
        else if(logOdds < -0.65f)
            ++stats.freeCells;
    }

    const GridBounds bounds = grid.Bounds();
    if(bounds.valid) {
        stats.hasBounds = true;
        stats.widthMeters = static_cast<float>(bounds.maxX - bounds.minX + 1) * grid.resolution();
        stats.heightMeters = static_cast<float>(bounds.maxZ - bounds.minZ + 1) * grid.resolution();
    }
    return stats;
}

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

void ResetNavigationExperimentState(OccupancyGrid2D& grid,
                                    PlannerState& planner,
                                    TrajectoryState& trajectory,
                                    FloorEstimator& floorEstimator,
                                    NavigationFrameEstimator& navFrameEstimator,
                                    ScaleDiagnostics& scaleDiagnostics,
                                    GuidanceState& guidanceState,
                                    DepthComparisonDiagnostics& depthComparison,
                                    LiveStatus& liveStatus,
                                    OccupancyViewState& viewState,
                                    bool localizationOnly) {
    const float resolution = grid.resolution();
    grid = OccupancyGrid2D(resolution);
    planner = PlannerState{};
    trajectory = TrajectoryState{};
    floorEstimator = FloorEstimator{};
    navFrameEstimator = NavigationFrameEstimator{};
    scaleDiagnostics = ScaleDiagnostics{};
    guidanceState = GuidanceState{};
    depthComparison = DepthComparisonDiagnostics{};
    liveStatus = LiveStatus{};
    liveStatus.localizationOnly = localizationOnly;
    ResetFollowView(viewState);
}

ScaleUpdateEvent UpdateScaleDiagnostics(ScaleDiagnostics& diagnostics,
                                        const Sophus::SE3f& orbTwc,
                                        const PacketHeader& phoneHeader) {
    ScaleUpdateEvent event;
    const Eigen::Vector3f orbPosition = orbTwc.translation();

    if(!phoneHeader.hasPose) {
        diagnostics.previousOrbPosition = orbPosition;
        diagnostics.hasOrbPose = true;
        event.windowReset = InvalidateScaleDiagnostics(diagnostics, true);
        if(event.windowReset)
            event.resetReason = "phone_pose_missing";
        diagnostics.hasPhonePose = false;
        return event;
    }

    const Sophus::SE3f phoneTwc = PoseFromHeader(phoneHeader);
    const Eigen::Vector3f phonePosition = phoneTwc.translation();
    const bool referenceTrusted = IsPhoneReferenceTrusted(phoneHeader);

    auto storeBaseline = [&] {
        diagnostics.previousOrbPosition = orbPosition;
        diagnostics.previousPhonePosition = phonePosition;
        diagnostics.hasOrbPose = true;
        diagnostics.hasPhonePose = true;
        if(referenceTrusted && !diagnostics.hasStableAnchor) {
            diagnostics.hasStableAnchor = true;
            diagnostics.stableAnchorOrbPosition = orbPosition;
            diagnostics.stableAnchorPhonePosition = phonePosition;
            diagnostics.stableOrbDisplacementMeters = 0.0f;
            diagnostics.stablePhoneDisplacementMeters = 0.0f;
        }
        diagnostics.previousPhoneFrameIndex = phoneHeader.frameIndex;
        diagnostics.previousTimestampSeconds = phoneHeader.timestamp;
        diagnostics.previousSampleTrusted = referenceTrusted;
    };

    if(!referenceTrusted) {
        event.windowReset = InvalidateScaleDiagnostics(diagnostics, true);
        if(event.windowReset)
            event.resetReason = "phone_reference_untrusted";
        storeBaseline();
        return event;
    }

    const bool hasTrustedBaseline =
        diagnostics.hasOrbPose &&
        diagnostics.hasPhonePose &&
        diagnostics.previousSampleTrusted;
    if(!hasTrustedBaseline) {
        storeBaseline();
        return event;
    }

    const bool frameSequenceContinuous =
        diagnostics.previousPhoneFrameIndex != 0 &&
        phoneHeader.frameIndex == diagnostics.previousPhoneFrameIndex + 1;
    const bool timestampContinuous =
        diagnostics.previousTimestampSeconds > 0.0 &&
        phoneHeader.timestamp > diagnostics.previousTimestampSeconds &&
        (phoneHeader.timestamp - diagnostics.previousTimestampSeconds) <= kScaleRatioMaxFrameGapSeconds;
    if(!frameSequenceContinuous || !timestampContinuous) {
        InvalidateScaleDiagnostics(diagnostics, false);
        event.windowReset = ResetRecentScaleWindow(diagnostics);
        if(event.windowReset)
            event.resetReason = frameSequenceContinuous ? "timestamp_gap" : "frame_gap";
        storeBaseline();
        return event;
    }

    const float orbDeltaMeters = (orbPosition - diagnostics.previousOrbPosition).norm();
    const float phoneDeltaMeters = (phonePosition - diagnostics.previousPhonePosition).norm();
    event.orbDeltaMeters = orbDeltaMeters;
    event.phoneDeltaMeters = phoneDeltaMeters;

    const bool orbValid = std::isfinite(orbDeltaMeters) && orbDeltaMeters > 0.01f && orbDeltaMeters < 1.0f;
    const bool phoneValid = std::isfinite(phoneDeltaMeters) && phoneDeltaMeters > 0.01f && phoneDeltaMeters < 1.0f;
    const float pairRatio = ScaleRatioForDistances(orbDeltaMeters, phoneDeltaMeters);
    const bool pairRatioImplausible =
        orbValid &&
        phoneValid &&
        std::max(orbDeltaMeters, phoneDeltaMeters) >= kScaleRatioDiscontinuityMinMeters &&
        (pairRatio > kScaleRatioPairJumpFactor || pairRatio < (1.0f / kScaleRatioPairJumpFactor));
    const bool pairDiscontinuous =
        (!orbValid && phoneValid && phoneDeltaMeters >= kScaleRatioDiscontinuityMinMeters) ||
        (!phoneValid && orbValid && orbDeltaMeters >= kScaleRatioDiscontinuityMinMeters) ||
        pairRatioImplausible;
    if(pairDiscontinuous) {
        InvalidateScaleDiagnostics(diagnostics, false);
        event.windowReset = ResetRecentScaleWindow(diagnostics);
        if(event.windowReset)
            event.resetReason = "paired_motion_discontinuity";
        storeBaseline();
        return event;
    }

    if(orbValid && phoneValid) {
        diagnostics.orbDistanceMeters += orbDeltaMeters;
        diagnostics.phoneDistanceMeters += phoneDeltaMeters;
        diagnostics.stableOrbDisplacementMeters = (orbPosition - diagnostics.stableAnchorOrbPosition).norm();
        diagnostics.stablePhoneDisplacementMeters = (phonePosition - diagnostics.stableAnchorPhonePosition).norm();
        ++diagnostics.orbSegments;
        ++diagnostics.phoneSegments;
        event.pairAccepted = true;

        if(std::isfinite(phoneHeader.timestamp) && phoneHeader.timestamp > 0.0) {
            if(!diagnostics.recentSamples.empty() &&
               phoneHeader.timestamp <= diagnostics.recentSamples.back().timestampSeconds) {
                ResetRecentScaleWindow(diagnostics);
            }

            diagnostics.recentSamples.push_back(ScaleWindowSample{
                phoneHeader.timestamp,
                orbDeltaMeters,
                phoneDeltaMeters,
                diagnostics.previousOrbPosition,
                diagnostics.previousPhonePosition,
                orbPosition,
                phonePosition,
            });
            diagnostics.recentOrbDistanceMeters += orbDeltaMeters;
            diagnostics.recentPhoneDistanceMeters += phoneDeltaMeters;

            while(!diagnostics.recentSamples.empty() &&
                  (phoneHeader.timestamp - diagnostics.recentSamples.front().timestampSeconds) > kScaleRatioWindowSeconds) {
                diagnostics.recentOrbDistanceMeters -= diagnostics.recentSamples.front().orbDeltaMeters;
                diagnostics.recentPhoneDistanceMeters -= diagnostics.recentSamples.front().phoneDeltaMeters;
                diagnostics.recentSamples.pop_front();
            }
        }
    }

    storeBaseline();
    return event;
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

    ostringstream out;
    out << "{\n";
    out << "  \"has_pose\": " << (status.hasPose ? "true" : "false") << ",\n";
    out << "  \"tracking_state\": \"" << status.trackingState << "\",\n";
    out << "  \"fixed_height_mode\": " << (settings.fixedHeightMode ? "true" : "false") << ",\n";
    out << "  \"fixed_camera_height_m\": " << fixed << setprecision(3) << settings.fixedCameraHeightMeters << ",\n";
    out << "  \"has_waypoint\": " << (guidance.hasWaypoint ? "true" : "false") << ",\n";
    out << "  \"waypoint_distance_m\": " << fixed << setprecision(3) << guidance.waypointDistanceMeters << ",\n";
    out << "  \"heading_error_deg\": " << fixed << setprecision(3) << guidance.headingErrorDegrees << "\n";
    out << "}\n";
    WriteTextFileAtomically(path, out.str());
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
        if(const auto value = tree.get_optional<string>("depthSourceMode")) {
            control.hasDepthSourceMode = true;
            control.depthSourceMode = DepthSourceModeFromKey(*value);
        }
        if(const auto value = tree.get_optional<bool>("enableRgbPreview")) {
            control.hasEnableRgbPreview = true;
            control.enableRgbPreview = *value;
        }
        if(const auto value = tree.get_optional<bool>("enableDepthPreview")) {
            control.hasEnableDepthPreview = true;
            control.enableDepthPreview = *value;
        }
        if(const auto value = tree.get_optional<bool>("enableDepthComparison")) {
            control.hasEnableDepthComparison = true;
            control.enableDepthComparison = *value;
        }
        if(const auto value = tree.get_optional<bool>("enableDepthDiffPreview")) {
            control.hasEnableDepthDiffPreview = true;
            control.enableDepthDiffPreview = *value;
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

string DescribeBackendControl(const BackendControl& control,
                              const NavigationSettings& navigationSettings,
                              bool localizationOnly,
                              const PlannerState& planner,
                              bool requestExperimentReset,
                              bool requestSlamRestart,
                              bool requestSnapshot,
                              bool requestQuit) {
    vector<string> parts;
    parts.emplace_back("revision=" + to_string(control.revision));
    if(!control.viewMode.empty())
        parts.emplace_back("view=" + control.viewMode);
    parts.emplace_back("depth_mode=" + DepthSourceModeKey(navigationSettings.depthSourceMode));
    parts.emplace_back(string("tracking_mode=") + (localizationOnly ? "localization" : "slam"));
    parts.emplace_back(string("height_mode=") + (navigationSettings.fixedHeightMode ? "fixed" : "adaptive"));
    {
        ostringstream value;
        value << fixed << setprecision(2) << "fixed_h=" << navigationSettings.fixedCameraHeightMeters;
        parts.push_back(value.str());
    }
    parts.emplace_back("inflation=" + to_string(navigationSettings.inflationRadiusCells));
    {
        ostringstream value;
        value << fixed << setprecision(2) << "lookahead=" << navigationSettings.lookaheadMeters;
        parts.push_back(value.str());
    }
    if(control.setGoal) {
        ostringstream value;
        value << fixed << setprecision(2) << "goal=(" << control.goalWorldX << ", " << control.goalWorldZ << ")";
        parts.push_back(value.str());
    }
    if(control.clearGoal)
        parts.emplace_back("clear_goal");
    if(requestSnapshot)
        parts.emplace_back("save_snapshot");
    if(requestQuit)
        parts.emplace_back("quit");
    if(requestExperimentReset)
        parts.emplace_back("reset_nav_state");
    if(requestSlamRestart)
        parts.emplace_back("restart_slam");
    parts.emplace_back(string("goal_active=") + (planner.hasGoal ? "yes" : "no"));

    ostringstream stream;
    for(size_t index = 0; index < parts.size(); ++index) {
        if(index > 0)
            stream << "  ";
        stream << parts[index];
    }
    return stream.str();
}

void ApplyBackendControl(const BackendControl& control,
                         int& lastRevision,
                         OccupancyViewState& viewState,
                         PlannerState& planner,
                         NavigationSettings& navigationSettings,
                         OccupancyGrid2D& grid,
                         bool& localizationOnly,
                         ORB_SLAM3::System* slam,
                         bool& requestExperimentReset,
                         bool& requestSlamRestart,
                         bool& requestSnapshot,
                         bool& requestQuit) {
    const bool isNewRevision = control.revision != lastRevision;

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
    if(control.hasEnableRgbPreview)
        navigationSettings.enableRgbPreview = control.enableRgbPreview;
    if(control.hasEnableDepthPreview)
        navigationSettings.enableDepthPreview = control.enableDepthPreview;
    if(control.hasEnableDepthComparison)
        navigationSettings.enableDepthComparison = control.enableDepthComparison;
    if(control.hasEnableDepthDiffPreview)
        navigationSettings.enableDepthDiffPreview = control.enableDepthDiffPreview && navigationSettings.enableDepthComparison;
    if(control.hasDepthSourceMode && control.depthSourceMode != navigationSettings.depthSourceMode) {
        requestExperimentReset = true;
        requestSlamRestart = DepthSourceUsesModelForSlam(control.depthSourceMode) !=
                             DepthSourceUsesModelForSlam(navigationSettings.depthSourceMode);
        navigationSettings.depthSourceMode = control.depthSourceMode;
    }

    if(control.hasLocalizationOnly && control.localizationOnly != localizationOnly) {
        localizationOnly = control.localizationOnly;
        if(slam) {
            if(localizationOnly)
                slam->ActivateLocalizationMode();
            else
                slam->DeactivateLocalizationMode();
        }
    }

    if(isNewRevision && control.setGoal) {
        planner.goalCell = grid.ToCell(control.goalWorldX, control.goalWorldZ);
        planner.hasGoal = true;
        planner.dirty = true;
    }

    if(!isNewRevision)
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
                       const DepthSourceState& depthSourceState,
                       const ScaleDiagnostics& diagnostics,
                       const DepthComparisonDiagnostics& depthComparison,
                       int appliedControlRevision,
                       const OccupancyViewState& viewState,
                       const cv::Rect& mapRect,
                       const cv::Size& compositeSize,
                       bool connected) {
    ostringstream out;
    const ScaleWindowStats scaleWindow = CurrentScaleWindowStats(diagnostics);

    out << "{\n";
    out << "  \"timestamp\": \"" << CurrentTimeString() << "\",\n";
    out << "  \"appliedControlRevision\": " << appliedControlRevision << ",\n";
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
    out << "  \"depthSourceMode\": \"" << depthSourceState.requestedMode << "\",\n";
    out << "  \"activeSlamDepthSource\": \"" << depthSourceState.activeSlamSource << "\",\n";
    out << "  \"activeMapDepthSource\": \"" << depthSourceState.activeMapSource << "\",\n";
    out << "  \"depthSourceStatus\": \"" << JsonEscape(depthSourceState.status) << "\",\n";
    out << "  \"enableRgbPreview\": " << (navigationSettings.enableRgbPreview ? "true" : "false") << ",\n";
    out << "  \"enableDepthPreview\": " << (navigationSettings.enableDepthPreview ? "true" : "false") << ",\n";
    out << "  \"enableDepthComparison\": " << (navigationSettings.enableDepthComparison ? "true" : "false") << ",\n";
    out << "  \"enableDepthDiffPreview\": " << (navigationSettings.enableDepthDiffPreview ? "true" : "false") << ",\n";
    out << "  \"orbDistanceMeters\": " << fixed << setprecision(3) << diagnostics.orbDistanceMeters << ",\n";
    out << "  \"phoneDistanceMeters\": " << fixed << setprecision(3) << diagnostics.phoneDistanceMeters << ",\n";
    out << "  \"scaleRatio\": " << fixed << setprecision(4)
        << ScaleRatioForDistances(diagnostics.orbDistanceMeters, diagnostics.phoneDistanceMeters) << ",\n";
    out << "  \"stableOrbDisplacementMeters\": " << fixed << setprecision(3) << diagnostics.stableOrbDisplacementMeters << ",\n";
    out << "  \"stablePhoneDisplacementMeters\": " << fixed << setprecision(3) << diagnostics.stablePhoneDisplacementMeters << ",\n";
    out << "  \"stableScaleRatio\": " << fixed << setprecision(4)
        << ScaleRatioForDistances(diagnostics.stableOrbDisplacementMeters, diagnostics.stablePhoneDisplacementMeters) << ",\n";
    out << "  \"recentOrbDistanceMeters\": " << fixed << setprecision(3) << scaleWindow.orbDistanceMeters << ",\n";
    out << "  \"recentPhoneDistanceMeters\": " << fixed << setprecision(3) << scaleWindow.phoneDistanceMeters << ",\n";
    out << "  \"recentScaleRatio\": " << fixed << setprecision(4) << scaleWindow.ratio << ",\n";
    out << "  \"recentOrbDisplacementMeters\": " << fixed << setprecision(3) << scaleWindow.orbDisplacementMeters << ",\n";
    out << "  \"recentPhoneDisplacementMeters\": " << fixed << setprecision(3) << scaleWindow.phoneDisplacementMeters << ",\n";
    out << "  \"recentDisplacementScaleRatio\": " << fixed << setprecision(4) << scaleWindow.displacementRatio << ",\n";
    out << "  \"recentScaleWindowSeconds\": " << fixed << setprecision(3) << scaleWindow.durationSeconds << ",\n";
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
    WriteTextFileAtomically(path, out.str());
}

chrono::milliseconds EffectiveLatestArtifactsInterval(bool appMode, const NavigationSettings& navigationSettings) {
    if(!appMode)
        return kDefaultLatestArtifactsInterval;

    const bool anyPreviewEnabled = navigationSettings.enableRgbPreview ||
                                   navigationSettings.enableDepthPreview ||
                                   navigationSettings.enableDepthComparison;
    return anyPreviewEnabled ? kAppLatestArtifactsInterval : chrono::milliseconds(450);
}

chrono::milliseconds EffectiveMapDataInterval(bool appMode) {
    return appMode ? kAppMapDataInterval : kDefaultMapDataInterval;
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
        LogInfo("Listening on port " + to_string(port));
        return;
    }

    LogInfo("Enter one of these Mac IPs on the phone:");
    for(const string& ip : ips)
        LogInfo("  " + ip + ":" + to_string(port));
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
    header.arTrackingState = tree.get<string>("arTrackingState", "unknown");
    header.arTrackingReason = tree.get<string>("arTrackingReason", "");
    header.arWorldMappingStatus = tree.get<string>("arWorldMappingStatus", "unknown");
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

void ValidatePacketHeader(const PacketHeader& header) {
    if(header.rgbWidth <= 0 || header.rgbHeight <= 0 ||
       header.rgbWidth > kMaxFrameDimension || header.rgbHeight > kMaxFrameDimension) {
        throw runtime_error("invalid RGB frame dimensions");
    }
    if(header.rgbSize <= 0 || static_cast<size_t>(header.rgbSize) > kMaxRgbPayloadBytes) {
        throw runtime_error("invalid RGB payload size");
    }
    if(header.depthWidth < 0 || header.depthHeight < 0 ||
       header.depthWidth > kMaxFrameDimension || header.depthHeight > kMaxFrameDimension) {
        throw runtime_error("invalid depth frame dimensions");
    }
    if(header.depthSize < 0 || static_cast<size_t>(header.depthSize) > kMaxDepthPayloadBytes) {
        throw runtime_error("invalid depth payload size");
    }
    if((header.depthWidth == 0) != (header.depthHeight == 0)) {
        throw runtime_error("incomplete depth frame dimensions");
    }
    if(header.depthWidth == 0 || header.depthHeight == 0) {
        if(header.depthSize != 0) {
            throw runtime_error("depth payload missing dimensions");
        }
    } else {
        const uint64_t expectedDepthBytes =
            static_cast<uint64_t>(header.depthWidth) * static_cast<uint64_t>(header.depthHeight) * sizeof(uint16_t);
        if(expectedDepthBytes != static_cast<uint64_t>(header.depthSize) ||
           expectedDepthBytes > static_cast<uint64_t>(kMaxDepthPayloadBytes)) {
            throw runtime_error("depth payload size does not match dimensions");
        }
    }
    if(!std::isfinite(header.fx) || !std::isfinite(header.fy) ||
       !std::isfinite(header.cx) || !std::isfinite(header.cy) ||
       header.fx <= 0.0f || header.fy <= 0.0f) {
        throw runtime_error("invalid camera intrinsics");
    }
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
    if(headerSize == 0 || headerSize > kMaxPacketHeaderBytes)
        throw runtime_error("invalid packet header size");
    string headerJson(headerSize, '\0');
    if(!ReadExact(clientFd, &headerJson[0], headerJson.size()))
        return false;
    frame.header = ParseHeader(headerJson);
    ValidatePacketHeader(frame.header);

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
        const uchar* maskPtr = validMask.ptr<uchar>(y);
        float* errorPtr = absErrorMeters.ptr<float>(y);
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

cv::Mat DepthMetersToMillimeters(const cv::Mat& depthMeters) {
    cv::Mat depthMillimeters(depthMeters.size(), CV_16UC1, cv::Scalar(0));
    for(int y = 0; y < depthMeters.rows; ++y) {
        const float* source = depthMeters.ptr<float>(y);
        uint16_t* target = depthMillimeters.ptr<uint16_t>(y);
        for(int x = 0; x < depthMeters.cols; ++x) {
            const float value = source[x];
            if(!std::isfinite(value) || value < kMinDepthMeters || value > kMaxDepthMeters)
                continue;
            target[x] = static_cast<uint16_t>(std::round(value * 1000.0f));
        }
    }
    return depthMillimeters;
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
    if(header.depthWidth <= 0 || header.depthHeight <= 0)
        throw runtime_error("first RGB-D frame is missing depth dimensions");

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

void LogPerformanceSummary(const PerformanceWindow& window,
                           const OccupancyGrid2D& grid,
                           const PlannerState& planner,
                           const TrajectoryState& trajectory,
                           const ScaleDiagnostics& diagnostics,
                           const DepthComparisonDiagnostics& depthComparison,
                           const PacketHeader& latestPhoneHeader,
                           bool localizationOnly) {
    if(window.frames <= 0)
        return;

    const double avgCaptureGapMs = AverageMs(window.totalCaptureGapMs, window.captureGapSamples);
    const double avgProcessingMs = AverageMs(window.totalProcessingMs, window.frames);
    const double avgTrackMs = AverageMs(window.totalTrackMs, window.trackSamples);
    const double avgInferenceMs = AverageMs(window.totalInferenceMs, window.inferenceSamples);
    const double avgMapMs = AverageMs(window.totalMapMs, window.mapSamples);
    const double avgRenderMs = AverageMs(window.totalRenderMs, window.renderSamples);
    const double avgFloorSamples = window.poseFrames > 0
        ? static_cast<double>(window.totalFloorSamples) / static_cast<double>(window.poseFrames)
        : 0.0;
    const double avgMapPoints = window.poseFrames > 0
        ? static_cast<double>(window.totalMapPoints) / static_cast<double>(window.poseFrames)
        : 0.0;
    const double inputFps = avgCaptureGapMs > 1e-3 ? (1000.0 / avgCaptureGapMs) : 0.0;
    const uint64_t gridDelta = window.hasStartGridVersion ? (grid.version() - window.startGridVersion) : 0;

    const MapQualityStats mapStats = ComputeMapQualityStats(grid);
    const float scalePathRatio = ScaleRatioForDistances(diagnostics.orbDistanceMeters, diagnostics.phoneDistanceMeters);
    const float stableScaleRatio =
        ScaleRatioForDistances(diagnostics.stableOrbDisplacementMeters, diagnostics.stablePhoneDisplacementMeters);
    const ScaleWindowStats scaleWindow = CurrentScaleWindowStats(diagnostics);
    ostringstream summary;
    summary << fixed << setprecision(1)
            << "[PERF] frames=" << window.frames
            << "  input_fps=" << inputFps
            << "  process_ms=" << avgProcessingMs
            << "  track_ms=" << avgTrackMs
            << "  infer_ms=" << avgInferenceMs
            << "  map_ms=" << avgMapMs
            << "  render_ms=" << avgRenderMs
            << "  avg_floor_samples=" << avgFloorSamples
            << "  avg_map_points=" << avgMapPoints
            << "  grid_delta=" << gridDelta
            << "  path=" << (planner.pathValid ? planner.pathCells.size() : 0)
            << "  traj=" << trajectory.samples.size();
    summary << fixed << setprecision(3)
            << "  scale_path_total=" << scalePathRatio
            << "  scale_path_recent=" << scaleWindow.ratio
            << "  scale_stable=" << stableScaleRatio
            << "  scale_disp_recent=" << scaleWindow.displacementRatio
            << "  stable_orb_disp_m=" << diagnostics.stableOrbDisplacementMeters
            << "  stable_phone_disp_m=" << diagnostics.stablePhoneDisplacementMeters
            << "  recent_orb_path_m=" << scaleWindow.orbDistanceMeters
            << "  recent_phone_path_m=" << scaleWindow.phoneDistanceMeters
            << "  recent_orb_disp_m=" << scaleWindow.orbDisplacementMeters
            << "  recent_phone_disp_m=" << scaleWindow.phoneDisplacementMeters;
    if(scaleWindow.durationSeconds > 1e-3) {
        summary << fixed << setprecision(1)
                << "  scale_window_s=" << scaleWindow.durationSeconds;
    }
    if(mapStats.hasBounds) {
        summary << fixed << setprecision(1);
        summary << "  map_m=" << mapStats.widthMeters << "x" << mapStats.heightMeters;
    }
    summary << "  cells=" << mapStats.knownCells
            << "  occ=" << mapStats.occupiedCells
            << "  free=" << mapStats.freeCells
            << "  mode=" << (localizationOnly ? "LOCALIZATION" : "SLAM")
            << "  ar_tracking=" << DescribeArkitTrackingState(latestPhoneHeader)
            << "  ar_mapping=" << latestPhoneHeader.arWorldMappingStatus;
    if(depthComparison.modelEnabled) {
        summary << fixed << setprecision(3)
                << "  depth_overlap=" << depthComparison.overlapRatio
                << "  depth_mae=" << depthComparison.maeMeters
                << "  depth_rmse=" << depthComparison.rmseMeters;
    }

    const bool slowProcessing = avgProcessingMs > kProcessingSummaryWarnMs || avgTrackMs > kTrackSummaryWarnMs;
    const bool sparseMapping = window.poseFrames > 0 &&
        (avgFloorSamples < static_cast<double>(kFloorSamplesWarnCount) ||
         avgMapPoints < static_cast<double>(kMapPointsWarnCount) ||
         (!localizationOnly && gridDelta == 0));
    const bool poorDepthQuality = depthComparison.modelEnabled &&
        (depthComparison.status == "INSUFFICIENT_OVERLAP" ||
         (depthComparison.hasComparison &&
          (depthComparison.overlapRatio < kDepthOverlapWarnRatio ||
           depthComparison.maeMeters > kDepthMaeWarnMeters)));
    const bool slowInference = depthComparison.modelEnabled &&
        avgInferenceMs > kInferenceSummaryWarnMs;

    if(slowProcessing || sparseMapping || poorDepthQuality || slowInference)
        LogWarning(summary.str());
    else
        LogInfo(summary.str());
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
        orbDistanceText << fixed << setprecision(2) << "ORB 路径里程 " << diagnostics.orbDistanceMeters << "m";
        DrawLabel(panel, orbDistanceText.str(), cv::Point(10, 230));
    }
    {
        ostringstream phoneDistanceText;
        phoneDistanceText << fixed << setprecision(2) << "iPhone 参考路径 " << diagnostics.phoneDistanceMeters << "m";
        DrawLabel(panel, phoneDistanceText.str(), cv::Point(10, 274));
    }
    if(diagnostics.stableOrbDisplacementMeters > 0.15f && diagnostics.stablePhoneDisplacementMeters > 0.15f) {
        const float ratio = diagnostics.stableOrbDisplacementMeters / diagnostics.stablePhoneDisplacementMeters;
        ostringstream ratioText;
        ratioText << fixed << setprecision(3) << "稳定位移尺度 ORB/iPhone = " << ratio << "x";
        DrawLabel(panel, ratioText.str(), cv::Point(10, 318));
    } else {
        DrawLabel(panel, "尺度对比：先多走一段再判断", cv::Point(10, 318));
    }
    DrawLabel(panel, "说明：路径里程容易被抖动放大，上面这行更可信", cv::Point(10, 362));
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

string CurrentLogTimestamp() {
    const auto now = chrono::system_clock::now();
    const auto nowMs = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    const std::time_t nowTime = chrono::system_clock::to_time_t(now);
    std::tm localTime {};
    localtime_r(&nowTime, &localTime);
    ostringstream stream;
    stream << put_time(&localTime, "%Y-%m-%d %H:%M:%S")
           << '.' << setw(3) << setfill('0') << nowMs.count();
    return stream.str();
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

const char* LogLevelTag(LogLevel level) {
    switch(level) {
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warning:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
    }
    return "INFO";
}

void WriteLogLine(LogLevel level, const string& message) {
    const string line = "[" + CurrentLogTimestamp() + "] [" + LogLevelTag(level) + "] " + message;

    lock_guard<mutex> lock(gEventLogMutex);
    ostream& console = (level == LogLevel::Error) ? cerr : cout;
    console << line << endl;
    if(gEventLogFile && gEventLogFile->good()) {
        (*gEventLogFile) << line << '\n';
        gEventLogFile->flush();
    }
}

void InitializeEventLog(const string& path) {
    lock_guard<mutex> lock(gEventLogMutex);
    gEventLogPath = path;
    gEventLogFile = make_unique<ofstream>(path, ios::app);
}

void LogInfo(const string& message) {
    WriteLogLine(LogLevel::Info, message);
}

void LogWarning(const string& message) {
    WriteLogLine(LogLevel::Warning, message);
}

void LogError(const string& message) {
    WriteLogLine(LogLevel::Error, message);
}

double AverageMs(double total, int count) {
    return count > 0 ? (total / static_cast<double>(count)) : 0.0;
}

bool WriteTextFileAtomically(const string& path, const string& contents) {
    static atomic<uint64_t> tempFileCounter{0};
    ostringstream tempPathBuilder;
    tempPathBuilder << path
                    << ".tmp."
                    << getpid()
                    << '.'
                    << tempFileCounter.fetch_add(1, std::memory_order_relaxed);
    const string tempPath = tempPathBuilder.str();

    {
        ofstream out(tempPath, ios::binary | ios::trunc);
        if(!out.good()) {
            LogError("Failed to open temporary file for atomic write: " + tempPath);
            return false;
        }
        out << contents;
        out.flush();
        if(!out.good()) {
            LogError("Failed to flush temporary file for atomic write: " + tempPath);
            out.close();
            std::remove(tempPath.c_str());
            return false;
        }
    }

    if(std::rename(tempPath.c_str(), path.c_str()) != 0) {
        LogError("Failed to replace file atomically: " + path + " (" + std::strerror(errno) + ")");
        std::remove(tempPath.c_str());
        return false;
    }
    return true;
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

float ScaleRatioForDistances(float orbDistanceMeters, float phoneDistanceMeters) {
    return phoneDistanceMeters > 1e-4f ? (orbDistanceMeters / phoneDistanceMeters) : 0.0f;
}

bool ScaleRatioLooksWrong(float ratio,
                          float orbDistanceMeters,
                          float phoneDistanceMeters,
                          float minDistanceMeters) {
    return orbDistanceMeters >= minDistanceMeters &&
           phoneDistanceMeters >= minDistanceMeters &&
           std::isfinite(ratio) &&
           (ratio < kScaleRatioWarnMin || ratio > kScaleRatioWarnMax);
}

ScaleWindowStats CurrentScaleWindowStats(const ScaleDiagnostics& diagnostics) {
    ScaleWindowStats stats;
    stats.orbDistanceMeters = diagnostics.recentOrbDistanceMeters;
    stats.phoneDistanceMeters = diagnostics.recentPhoneDistanceMeters;
    stats.ratio = ScaleRatioForDistances(stats.orbDistanceMeters, stats.phoneDistanceMeters);
    stats.samples = diagnostics.recentSamples.size();
    if(!diagnostics.recentSamples.empty()) {
        const ScaleWindowSample& firstSample = diagnostics.recentSamples.front();
        const ScaleWindowSample& lastSample = diagnostics.recentSamples.back();
        stats.orbDisplacementMeters = (lastSample.orbEndPosition - firstSample.orbStartPosition).norm();
        stats.phoneDisplacementMeters = (lastSample.phoneEndPosition - firstSample.phoneStartPosition).norm();
        stats.displacementRatio = ScaleRatioForDistances(stats.orbDisplacementMeters, stats.phoneDisplacementMeters);
    }
    if(diagnostics.recentSamples.size() >= 2) {
        stats.durationSeconds =
            diagnostics.recentSamples.back().timestampSeconds - diagnostics.recentSamples.front().timestampSeconds;
    }
    return stats;
}

bool IsPhoneReferenceTrusted(const PacketHeader& header) {
    return header.hasPose &&
           header.arTrackingState == "normal" &&
           header.arWorldMappingStatus != "not_available";
}

bool ResetRecentScaleWindow(ScaleDiagnostics& diagnostics) {
    const bool hadSamples = !diagnostics.recentSamples.empty() ||
                            diagnostics.recentOrbDistanceMeters > 1e-4f ||
                            diagnostics.recentPhoneDistanceMeters > 1e-4f;
    diagnostics.recentSamples.clear();
    diagnostics.recentOrbDistanceMeters = 0.0f;
    diagnostics.recentPhoneDistanceMeters = 0.0f;
    return hadSamples;
}

bool InvalidateScaleDiagnostics(ScaleDiagnostics& diagnostics, bool clearRecentWindow) {
    const bool hadTrustedBaseline = diagnostics.previousSampleTrusted;
    diagnostics.previousSampleTrusted = false;
    diagnostics.previousPhoneFrameIndex = 0;
    diagnostics.previousTimestampSeconds = 0.0;
    diagnostics.hasStableAnchor = false;
    diagnostics.stableAnchorOrbPosition = Eigen::Vector3f::Zero();
    diagnostics.stableAnchorPhonePosition = Eigen::Vector3f::Zero();
    diagnostics.stableOrbDisplacementMeters = 0.0f;
    diagnostics.stablePhoneDisplacementMeters = 0.0f;
    const bool clearedWindow = clearRecentWindow ? ResetRecentScaleWindow(diagnostics) : false;
    return hadTrustedBaseline || clearedWindow;
}

string DescribeArkitTrackingState(const PacketHeader& header) {
    if(header.arTrackingReason.empty())
        return header.arTrackingState;
    return header.arTrackingState + "(" + header.arTrackingReason + ")";
}

string DescribeArkitReference(const PacketHeader& header) {
    ostringstream out;
    out << "tracking=" << DescribeArkitTrackingState(header)
        << "  mapping=" << header.arWorldMappingStatus
        << "  phone_pose=" << (header.hasPose ? "yes" : "no")
        << "  frame=" << header.frameIndex;
    return out.str();
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

MapRenderPayload BuildMapRenderPayload(const string& path,
                                       const OccupancyGrid2D& grid,
                                       const OccupancyViewState& viewState,
                                       const LiveStatus& status,
                                       const PlannerState& planner,
                                       const TrajectoryState& trajectory,
                                       const NavigationSettings& settings) {
    MapRenderPayload payload;
    payload.path = path;
    payload.resolution = grid.resolution();
    payload.metersPerPixel = viewState.metersPerPixel;
    payload.viewCenterX = viewState.centerX;
    payload.viewCenterZ = viewState.centerZ;
    payload.showInflation = settings.showInflation;
    payload.inflationRadiusCells = settings.inflationRadiusCells;
    payload.hasPose = status.hasPose;
    if(status.hasPose) {
        payload.robotPosition = status.currentTwc.translation();
        payload.robotForward = status.currentTwc.rotationMatrix() * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    }
    payload.hasGoal = planner.hasGoal;
    if(planner.hasGoal)
        payload.goalWorld = grid.CellCenterWorld(planner.goalCell);

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
    payload.freeCells = std::move(freeCells);
    payload.occupiedCells = std::move(occupiedCells);
    payload.inflatedCells = std::move(inflatedCells);
    payload.trajectorySamples = trajectory.samples;
    payload.pathPoints = std::move(pathPoints);
    return payload;
}

void SaveSnapshot(const cv::Mat& image) {
    const string snapshotPath = BuildSnapshotPath();
    cv::imwrite(snapshotPath, image);
    LogInfo("Saved navigation snapshot to " + snapshotPath);
    const string latestSnapshotPath = BuildLatestSnapshotPath();
    cv::imwrite(latestSnapshotPath, image);
    LogInfo("Updated latest navigation snapshot at " + latestSnapshotPath);
}

void UpdateLatestArtifacts(AsyncRenderWriter& writer,
                          const cv::Mat& image,
                          const cv::Mat& rgbPanel,
                          const cv::Mat& depthPanel,
                          const cv::Mat& mapPanel,
                          const cv::Mat& modelDepthPanel,
                          const cv::Mat& depthDiffPanel,
                          const NavigationSettings& navigationSettings,
                          chrono::steady_clock::duration minInterval) {
    static auto lastWriteTime = chrono::steady_clock::now() - chrono::seconds(10);
    const auto now = chrono::steady_clock::now();
    if(now - lastWriteTime < minInterval)
        return;
    lastWriteTime = now;
    writer.SubmitLatestArtifacts(LatestArtifactsPayload{
        image.clone(),
        rgbPanel.clone(),
        depthPanel.clone(),
        mapPanel.clone(),
        modelDepthPanel.clone(),
        depthDiffPanel.clone(),
        navigationSettings
    });
}

void QueueMapRenderData(AsyncRenderWriter& writer,
                        const string& path,
                        const OccupancyGrid2D& grid,
                        const OccupancyViewState& viewState,
                        const LiveStatus& status,
                        const PlannerState& planner,
                        const TrajectoryState& trajectory,
                        const NavigationSettings& settings,
                        chrono::steady_clock::duration minInterval) {
    static auto lastWriteTime = chrono::steady_clock::now() - chrono::seconds(10);
    const auto now = chrono::steady_clock::now();
    if(now - lastWriteTime < minInterval)
        return;
    lastWriteTime = now;
    writer.SubmitMapRender(BuildMapRenderPayload(path, grid, viewState, status, planner, trajectory, settings));
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
    const string eventLogPath = GetEnvOrDefault("ORB_NAV_LOG_PATH", kDefaultEventLogPath);
#ifdef __APPLE__
    const string depthModelPath = GetEnvOrDefault("ORB_NAV_DEPTH_MODEL_PATH", "models/DepthAnythingV2SmallF16.mlpackage");
#endif
    InitializeEventLog(eventLogPath);
    LogInfo("===== rgbd_iphone_stream session started =====");
    LogInfo("pid=" + to_string(getpid()) +
            "  app_mode=" + string(appMode ? "true" : "false") +
            "  port=" + to_string(port) +
            "  output_prefix=" + outputPrefix);
    LogInfo("control_path=" + controlPath + "  state_path=" + statePath + "  log_path=" + eventLogPath);

    struct sigaction sigIntHandler {};
    sigIntHandler.sa_handler = ExitLoopHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    int serverFd = -1;
    try {
        serverFd = CreateListenSocket(port);
    } catch(const exception& error) {
        LogError("Failed to start server: " + string(error.what()));
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
    DepthSourceState depthSourceState;
    MouseContext mouseContext{&viewState, &planner, grid.resolution()};
    int lastControlRevision = -1;
    AsyncRenderWriter renderWriter;

#ifdef __APPLE__
    std::unique_ptr<CoreMLDepthEstimator> depthEstimator;
    depthComparison.modelEnabled = navigationSettings.enableDepthComparison;
    depthEstimator = std::make_unique<CoreMLDepthEstimator>(depthModelPath);
    depthComparison.modelReady = depthEstimator->IsReady();
    depthComparison.status = navigationSettings.enableDepthComparison
        ? (depthComparison.modelReady ? "READY" : "MODEL_UNAVAILABLE")
        : "DISABLED";
    if(!depthComparison.modelReady)
        LogWarning("Depth comparison model unavailable: " + depthEstimator->Error());
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
    string loggedTrackingState = liveStatus.trackingState;
    bool loggedHasPose = liveStatus.hasPose;
    string loggedDepthSourceStatus = depthSourceState.status;
    bool loggedPathValid = planner.pathValid;
    bool loggedScaleRatioWarning = false;
    bool loggedScaleWindowWarning = false;
    bool loggedDepthQualityWarning = false;
    string loggedArkitTrackingState;
    string loggedArkitTrackingReason;
    string loggedArkitWorldMappingStatus;
    PerformanceWindow performanceWindow;
    double previousFrameTimestamp = 0.0;

    while(gKeepRunning) {
        depthSourceState.requestedMode = DepthSourceModeKey(navigationSettings.depthSourceMode);
        if(lastRgbPanel.empty() && lastDepthPanel.empty()) {
            depthSourceState.activeSlamSource = "sensor";
            depthSourceState.activeMapSource = "sensor";
            depthSourceState.status = DepthSourceModeLabel(navigationSettings.depthSourceMode);
        }

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
        const auto latestArtifactsInterval = EffectiveLatestArtifactsInterval(appMode, navigationSettings);
        const auto mapDataInterval = EffectiveMapDataInterval(appMode);
        UpdateLatestArtifacts(renderWriter, waitingComposite, waitingRgbExport, waitingDepthExport, waitingMap,
                              waitingModelDepthExport, waitingDepthDiffExport, navigationSettings, latestArtifactsInterval);
        QueueMapRenderData(renderWriter, BuildLatestMapDataPath(), grid, viewState, liveStatus, planner, trajectory, navigationSettings,
                           mapDataInterval);
        WriteBackendState(statePath, liveStatus, planner, guidanceState, navigationSettings, depthSourceState, scaleDiagnostics, depthComparison,
                          lastControlRevision, viewState, viewState.panelRect, waitingComposite.size(), false);

        bool requestSnapshot = false;
        bool requestQuit = false;
        bool requestExperimentReset = false;
        bool requestSlamRestart = false;
        BackendControl control;
        const int previousControlRevision = lastControlRevision;
        if(ReadBackendControl(controlPath, control))
            ApplyBackendControl(control, lastControlRevision, viewState, planner, navigationSettings, grid, localizationOnly, slam.get(),
                                requestExperimentReset, requestSlamRestart, requestSnapshot, requestQuit);
        if(lastControlRevision != previousControlRevision) {
            LogInfo("Applied control: " + DescribeBackendControl(control, navigationSettings, localizationOnly, planner,
                                                                 requestExperimentReset, requestSlamRestart, requestSnapshot, requestQuit));
        }
        if(requestExperimentReset) {
            LogInfo("Resetting navigation experiment state while waiting for stream.");
            ResetNavigationExperimentState(grid, planner, trajectory, floorEstimator, navFrameEstimator,
                                           scaleDiagnostics, guidanceState, depthComparison, liveStatus,
                                           viewState, localizationOnly);
            if(requestSlamRestart && slam) {
                LogInfo("Shutting down ORB-SLAM3 before next stream because control requested a SLAM restart.");
                slam->Shutdown();
                slam.reset();
                imageScale = 1.0f;
                loggedTrackingState = liveStatus.trackingState;
                loggedHasPose = liveStatus.hasPose;
                loggedDepthSourceStatus = depthSourceState.status;
                loggedPathValid = planner.pathValid;
                loggedScaleRatioWarning = false;
                loggedDepthQualityWarning = false;
                performanceWindow.Reset(grid.version());
            }
        }
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
            LogError("Socket wait failed: " + string(error.what()));
            break;
        }

        sockaddr_in clientAddress {};
        socklen_t clientLength = sizeof(clientAddress);
        const int clientFd = accept(serverFd, reinterpret_cast<sockaddr*>(&clientAddress), &clientLength);
        if(clientFd < 0) {
            if(errno == EINTR && !gKeepRunning)
                break;
            LogWarning("Failed to accept iPhone connection.");
            continue;
        }

        char clientIp[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &clientAddress.sin_addr, clientIp, sizeof(clientIp));
        LogInfo("Accepted connection from " + string(clientIp) + ":" + to_string(ntohs(clientAddress.sin_port)));
        SetSocketTimeout(clientFd, 1800);

        StreamFrame firstFrame;
        bool haveFirstRgbdFrame = false;
        uint64_t warmupTrustedFrames = 0;
        double warmupTrustedStartTimestamp = 0.0;
        string lastWarmupReason;
        while(gKeepRunning && !haveFirstRgbdFrame) {
            try {
                if(!ReceiveFrame(clientFd, firstFrame)) {
                    LogWarning("Stream closed before the first RGB-D frame arrived.");
                    break;
                }
                if(firstFrame.depthMm.empty()) {
                    warmupTrustedFrames = 0;
                    warmupTrustedStartTimestamp = 0.0;
                    if(lastWarmupReason != "missing_depth") {
                        LogWarning("Waiting for first RGB-D depth payload...");
                        lastWarmupReason = "missing_depth";
                    }
                    continue;
                }

                if(!firstFrame.header.hasPose) {
                    warmupTrustedFrames = 0;
                    warmupTrustedStartTimestamp = 0.0;
                    if(lastWarmupReason != "phone_pose_missing") {
                        LogInfo("Deferring ORB-SLAM3 start until iPhone pose is available.");
                        lastWarmupReason = "phone_pose_missing";
                    }
                    continue;
                }

                if(!IsPhoneReferenceTrusted(firstFrame.header)) {
                    warmupTrustedFrames = 0;
                    warmupTrustedStartTimestamp = 0.0;
                    const string warmupReason = "tracking=" + DescribeArkitTrackingState(firstFrame.header) +
                        "  mapping=" + firstFrame.header.arWorldMappingStatus;
                    if(lastWarmupReason != warmupReason) {
                        ostringstream warmupLog;
                        warmupLog << "Deferring ORB-SLAM3 start until ARKit stabilizes"
                                  << "  " << warmupReason
                                  << "  frame=" << firstFrame.header.frameIndex;
                        LogInfo(warmupLog.str());
                        lastWarmupReason = warmupReason;
                    }
                    continue;
                }

                if(warmupTrustedFrames == 0) {
                    warmupTrustedStartTimestamp = firstFrame.header.timestamp;
                    LogInfo("ARKit reference looks healthy. Warming up before starting ORB-SLAM3.");
                }
                ++warmupTrustedFrames;

                const double warmupTrustedSeconds =
                    std::max(0.0, firstFrame.header.timestamp - warmupTrustedStartTimestamp);
                if(warmupTrustedFrames < static_cast<uint64_t>(kStreamWarmupTrustedFrames) ||
                   warmupTrustedSeconds < kStreamWarmupTrustedSeconds) {
                    continue;
                }

                {
                    ostringstream warmupComplete;
                    warmupComplete << fixed << setprecision(2)
                                   << "Warm-up complete"
                                   << "  trusted_frames=" << warmupTrustedFrames
                                   << "  trusted_s=" << warmupTrustedSeconds
                                   << "  " << DescribeArkitReference(firstFrame.header);
                    LogInfo(warmupComplete.str());
                }
                haveFirstRgbdFrame = true;
            } catch(const exception& error) {
                LogError("Failed to receive first frame: " + string(error.what()));
                break;
            }
        }
        if(!haveFirstRgbdFrame) {
            close(clientFd);
            continue;
        }

        const string settingsPath = WriteAutoSettings(firstFrame.header, outputPrefix, port);
        {
            ostringstream frameInfo;
            frameInfo << "Auto-generated settings: " << settingsPath
                      << "  rgb=" << firstFrame.header.rgbWidth << "x" << firstFrame.header.rgbHeight
                      << "  depth=" << firstFrame.header.depthWidth << "x" << firstFrame.header.depthHeight
                      << "  depth_bytes=" << firstFrame.header.depthSize
                      << "  " << DescribeArkitReference(firstFrame.header);
            LogInfo(frameInfo.str());
        }

        if(!slam) {
            slam = make_unique<ORB_SLAM3::System>(vocabularyPath, settingsPath, ORB_SLAM3::System::RGBD, false, 0, outputPrefix);
            imageScale = slam->GetImageScale();
            LogInfo("Started a new ORB-SLAM3 RGB-D session.");
        } else {
            LogInfo("Reconnected RGB-D stream. Continuing current ORB-SLAM3 session.");
        }

        int frameIndex = 0;
        bool sessionDisconnected = false;
        StreamFrame currentFrame = std::move(firstFrame);
        previousFrameTimestamp = currentFrame.header.timestamp;
        performanceWindow.Reset(grid.version());
        loggedScaleRatioWarning = false;
        loggedScaleWindowWarning = false;
        loggedDepthQualityWarning = false;
        loggedArkitTrackingState.clear();
        loggedArkitTrackingReason.clear();
        loggedArkitWorldMappingStatus.clear();

        while(gKeepRunning) {
            const auto processingStart = chrono::steady_clock::now();
            const double captureGapMs =
                previousFrameTimestamp > 0.0
                    ? std::max(0.0, (currentFrame.header.timestamp - previousFrameTimestamp) * 1000.0)
                    : 0.0;
            previousFrameTimestamp = currentFrame.header.timestamp;
            double trackMs = 0.0;
            double mapMs = 0.0;
            double renderMs = 0.0;
            size_t floorSampleCount = 0;
            size_t mapPointCount = 0;

            cv::Mat slamRgb = currentFrame.rgb;
            cv::Mat sensorDepthRgb = ResizeDepthToRgb(currentFrame, currentFrame.rgb.size());
            cv::Mat slamDepth = sensorDepthRgb.clone();
            cv::Mat mapDepth = sensorDepthRgb.clone();

            depthSourceState.requestedMode = DepthSourceModeKey(navigationSettings.depthSourceMode);
            depthSourceState.activeSlamSource = "sensor";
            depthSourceState.activeMapSource = "sensor";
            depthSourceState.status = DepthSourceModeLabel(navigationSettings.depthSourceMode);

            cv::Mat modelDepthPreview;
            cv::Mat depthDiffPreview;
            cv::Mat alignedModelDepthMeters;
            cv::Mat depthErrorMeters;
            cv::Mat depthCompareMask;
            cv::Mat modelDepthRgb;
#ifdef __APPLE__
            {
                DepthComparisonDiagnostics frameDepthComparison = depthComparison;
                frameDepthComparison.modelEnabled = navigationSettings.enableDepthComparison;
                frameDepthComparison.modelReady = depthEstimator && depthEstimator->IsReady();
                frameDepthComparison.status = navigationSettings.enableDepthComparison
                    ? (frameDepthComparison.modelReady ? "READY" : "MODEL_UNAVAILABLE")
                    : "DISABLED";

                const bool needModelForPipeline = navigationSettings.depthSourceMode != DepthSourceMode::Sensor;
                const bool needModelForDiagnostics = navigationSettings.enableDepthComparison;
                const bool shouldEvaluateModel = needModelForPipeline ||
                                                (needModelForDiagnostics &&
                                                 (frameIndex == 0 || (frameIndex % kSensorModeModelEvalStride) == 0));

                if(frameDepthComparison.modelReady && shouldEvaluateModel) {
                    cv::Mat modelDepth32f;
                    double inferenceMs = 0.0;
                    if(depthEstimator->Infer(currentFrame.rgb, modelDepth32f, inferenceMs)) {
                        cv::Mat modelDepthAtSensor;
                        cv::resize(modelDepth32f, modelDepthAtSensor, currentFrame.depthMm.size(), 0.0, 0.0, cv::INTER_LINEAR);
                        if(navigationSettings.enableDepthComparison)
                            modelDepthPreview = RotateForPreview(ColorizeDepthFloat(modelDepthAtSensor), currentFrame.header.displayOrientation);

                        frameDepthComparison = CompareDepthMaps(currentFrame.depthMm, modelDepthAtSensor,
                                                                alignedModelDepthMeters, depthErrorMeters, depthCompareMask);
                        frameDepthComparison.modelEnabled = navigationSettings.enableDepthComparison;
                        frameDepthComparison.modelReady = true;
                        frameDepthComparison.inferenceMs = static_cast<float>(inferenceMs);
                        frameDepthComparison.status = navigationSettings.enableDepthComparison
                            ? (frameDepthComparison.hasComparison ? "READY" : "INSUFFICIENT_OVERLAP")
                            : "DISABLED";

                        if(frameDepthComparison.hasComparison) {
                            const cv::Mat modelDepthMmSensor = DepthMetersToMillimeters(alignedModelDepthMeters);
                            cv::resize(modelDepthMmSensor, modelDepthRgb, currentFrame.rgb.size(), 0.0, 0.0, cv::INTER_NEAREST);
                            if(navigationSettings.enableDepthComparison) {
                                modelDepthPreview = RotateForPreview(
                                    ColorizeDepthFloat(alignedModelDepthMeters, depthCompareMask),
                                    currentFrame.header.displayOrientation);
                            }
                            if(navigationSettings.enableDepthComparison && navigationSettings.enableDepthDiffPreview) {
                                depthDiffPreview = RotateForPreview(
                                    ColorizeDepthError(depthErrorMeters, depthCompareMask),
                                    currentFrame.header.displayOrientation);
                            }
                        }
                    } else {
                        frameDepthComparison.status = navigationSettings.enableDepthComparison ? "INFERENCE_FAILED" : "DISABLED";
                    }
                } else if(frameDepthComparison.modelReady && !needModelForPipeline && navigationSettings.enableDepthComparison) {
                    frameDepthComparison.status = frameDepthComparison.hasComparison ? "THROTTLED" : "READY";
                }
                depthComparison = frameDepthComparison;
            }
#else
            depthComparison = DepthComparisonDiagnostics{};
#endif
            const bool depthQualityLooksBad = navigationSettings.enableDepthComparison &&
                (depthComparison.status == "INFERENCE_FAILED" ||
                 depthComparison.status == "INSUFFICIENT_OVERLAP" ||
                 (depthComparison.hasComparison &&
                  (depthComparison.overlapRatio < kDepthOverlapWarnRatio ||
                   depthComparison.maeMeters > kDepthMaeWarnMeters)));
            if(depthQualityLooksBad != loggedDepthQualityWarning) {
                ostringstream depthLog;
                depthLog << fixed << setprecision(3)
                         << "Depth comparison quality "
                         << (depthQualityLooksBad ? "degraded" : "recovered")
                         << "  status=" << depthComparison.status
                         << "  overlap=" << depthComparison.overlapRatio
                         << "  mae=" << depthComparison.maeMeters
                         << "  rmse=" << depthComparison.rmseMeters
                         << "  infer_ms=" << depthComparison.inferenceMs;
                if(depthQualityLooksBad)
                    LogWarning(depthLog.str());
                else
                    LogInfo(depthLog.str());
                loggedDepthQualityWarning = depthQualityLooksBad;
            }

            const bool modelDepthUsable = depthComparison.hasComparison && !modelDepthRgb.empty();
            bool skipSlamUpdate = false;
            bool skipMapIntegration = false;
            switch(navigationSettings.depthSourceMode) {
                case DepthSourceMode::Sensor:
                    depthSourceState.status = "SLAM 与地图都使用 iPhone 传感器深度";
                    break;
                case DepthSourceMode::ModelMap:
                    if(modelDepthUsable) {
                        mapDepth = modelDepthRgb;
                        depthSourceState.activeMapSource = "model";
                        depthSourceState.status = "SLAM 使用传感器深度，地图使用经传感器对齐的模型深度";
                    } else {
                        mapDepth.release();
                        depthSourceState.activeMapSource = "none";
                        depthSourceState.status = "模型深度暂不可用，本帧未更新地图";
                        skipMapIntegration = true;
                    }
                    break;
                case DepthSourceMode::ModelFull:
                    if(modelDepthUsable) {
                        slamDepth = modelDepthRgb;
                        mapDepth = modelDepthRgb;
                        depthSourceState.activeSlamSource = "model";
                        depthSourceState.activeMapSource = "model";
                        depthSourceState.status = "SLAM 与地图都使用经传感器对齐的模型深度";
                    } else {
                        slamDepth.release();
                        mapDepth.release();
                        depthSourceState.activeSlamSource = "none";
                        depthSourceState.activeMapSource = "none";
                        depthSourceState.status = "模型深度暂不可用，本帧未更新 SLAM/地图";
                        skipSlamUpdate = true;
                        skipMapIntegration = true;
                    }
                    break;
            }
            if(depthSourceState.status != loggedDepthSourceStatus) {
                LogInfo("Depth source status -> " + depthSourceState.status +
                        "  active_slam=" + depthSourceState.activeSlamSource +
                        "  active_map=" + depthSourceState.activeMapSource);
                loggedDepthSourceStatus = depthSourceState.status;
            }

            if(currentFrame.header.arTrackingState != loggedArkitTrackingState ||
               currentFrame.header.arTrackingReason != loggedArkitTrackingReason ||
               currentFrame.header.arWorldMappingStatus != loggedArkitWorldMappingStatus) {
                LogInfo("ARKit reference -> " + DescribeArkitReference(currentFrame.header));
                loggedArkitTrackingState = currentFrame.header.arTrackingState;
                loggedArkitTrackingReason = currentFrame.header.arTrackingReason;
                loggedArkitWorldMappingStatus = currentFrame.header.arWorldMappingStatus;
            }

            if(imageScale != 1.0f) {
                const int scaledWidth = static_cast<int>(std::round(slamRgb.cols * imageScale));
                const int scaledHeight = static_cast<int>(std::round(slamRgb.rows * imageScale));
                cv::resize(slamRgb, slamRgb, cv::Size(scaledWidth, scaledHeight), 0.0, 0.0, cv::INTER_LINEAR);
                if(!slamDepth.empty())
                    cv::resize(slamDepth, slamDepth, cv::Size(scaledWidth, scaledHeight), 0.0, 0.0, cv::INTER_NEAREST);
                if(!mapDepth.empty())
                    cv::resize(mapDepth, mapDepth, cv::Size(scaledWidth, scaledHeight), 0.0, 0.0, cv::INTER_NEAREST);
            }

            const float scaleX = static_cast<float>(slamRgb.cols) / static_cast<float>(currentFrame.header.rgbWidth);
            const float scaleY = static_cast<float>(slamRgb.rows) / static_cast<float>(currentFrame.header.rgbHeight);
            const float fx = (currentFrame.header.fx * static_cast<float>(currentFrame.header.rgbWidth) / static_cast<float>(currentFrame.header.depthWidth)) * scaleX;
            const float fy = (currentFrame.header.fy * static_cast<float>(currentFrame.header.rgbHeight) / static_cast<float>(currentFrame.header.depthHeight)) * scaleY;
            const float cx = (currentFrame.header.cx * static_cast<float>(currentFrame.header.rgbWidth) / static_cast<float>(currentFrame.header.depthWidth)) * scaleX;
            const float cy = (currentFrame.header.cy * static_cast<float>(currentFrame.header.rgbHeight) / static_cast<float>(currentFrame.header.depthHeight)) * scaleY;

            bool poseUpdatedThisFrame = false;
            Sophus::SE3f orbTwc;
            ScaleUpdateEvent scaleUpdateEvent;
            if(!skipSlamUpdate) {
                const auto trackStart = chrono::steady_clock::now();
                const Sophus::SE3f Tcw = slam->TrackRGBD(slamRgb, slamDepth, currentFrame.header.timestamp);
                trackMs = chrono::duration<double, std::milli>(chrono::steady_clock::now() - trackStart).count();
                const int trackingState = slam->GetTrackingState();
                liveStatus.trackingState = TrackingStateToString(trackingState);
                liveStatus.localizationOnly = localizationOnly;
                liveStatus.hasPose = (trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT);
                if(liveStatus.hasPose) {
                    orbTwc = Tcw.inverse();
                    scaleUpdateEvent = UpdateScaleDiagnostics(scaleDiagnostics, orbTwc, currentFrame.header);
                    navFrameEstimator.Observe(orbTwc, currentFrame.header);
                    liveStatus.currentTwc = navFrameEstimator.TransformPose(orbTwc);
                    poseUpdatedThisFrame = true;
                } else if(InvalidateScaleDiagnostics(scaleDiagnostics, true)) {
                    scaleUpdateEvent.windowReset = true;
                    scaleUpdateEvent.resetReason = "tracking_unstable";
                }
            }
            if(liveStatus.trackingState != loggedTrackingState || liveStatus.hasPose != loggedHasPose) {
                ostringstream trackingLog;
                trackingLog << "Tracking state -> " << liveStatus.trackingState
                            << "  has_pose=" << (liveStatus.hasPose ? "yes" : "no");
                if(liveStatus.hasPose) {
                    const Eigen::Vector3f position = liveStatus.currentTwc.translation();
                    trackingLog << fixed << setprecision(2)
                                << "  pose=(" << position.x() << ", " << position.y() << ", " << position.z() << ")";
                }
                LogInfo(trackingLog.str());
                loggedTrackingState = liveStatus.trackingState;
                loggedHasPose = liveStatus.hasPose;
            }
            if(scaleUpdateEvent.windowReset && !scaleUpdateEvent.resetReason.empty()) {
                ostringstream scaleResetLog;
                scaleResetLog << fixed << setprecision(3)
                              << "Scale diagnostics reset"
                              << "  reason=" << scaleUpdateEvent.resetReason
                              << "  orb_delta=" << scaleUpdateEvent.orbDeltaMeters
                              << "  phone_delta=" << scaleUpdateEvent.phoneDeltaMeters
                              << "  " << DescribeArkitReference(currentFrame.header);
                LogInfo(scaleResetLog.str());
            }

            if(poseUpdatedThisFrame && !skipMapIntegration) {
                const auto mapStart = chrono::steady_clock::now();
                const cv::Mat& floorDepth = (navigationSettings.depthSourceMode == DepthSourceMode::ModelMap) ? slamDepth : mapDepth;
                vector<float> floorYSamples;
                const vector<Eigen::Vector3f> floorPointsOrb = SampleWorldPoints(floorDepth, orbTwc, fx, fy, cx, cy, floorYSamples);
                const vector<Eigen::Vector3f> floorPointsNav = navFrameEstimator.TransformPoints(floorPointsOrb);
                vector<float> navYSamples;
                navYSamples.reserve(floorPointsNav.size());
                for(const Eigen::Vector3f& point : floorPointsNav)
                    navYSamples.push_back(point.y());
                floorSampleCount = navYSamples.size();
                floorEstimator.ObserveFrame(navYSamples, liveStatus.currentTwc.translation().y(), navigationSettings);
                liveStatus.floorY = floorEstimator.FloorY(liveStatus.currentTwc.translation().y());

                vector<float> mapYSamples;
                const vector<Eigen::Vector3f> orbWorldPoints = SampleWorldPoints(mapDepth, orbTwc, fx, fy, cx, cy, mapYSamples);
                const vector<Eigen::Vector3f> navWorldPoints = navFrameEstimator.TransformPoints(orbWorldPoints);
                mapPointCount = navWorldPoints.size();
                grid.IntegratePoints(liveStatus.currentTwc.translation(), navWorldPoints, liveStatus.floorY);
                mapMs = chrono::duration<double, std::milli>(chrono::steady_clock::now() - mapStart).count();
            }
            if(poseUpdatedThisFrame) {
                AppendTrajectorySample(trajectory, liveStatus);
                const float stableScaleRatio =
                    ScaleRatioForDistances(scaleDiagnostics.stableOrbDisplacementMeters,
                                           scaleDiagnostics.stablePhoneDisplacementMeters);
                const float scalePathRatio =
                    ScaleRatioForDistances(scaleDiagnostics.orbDistanceMeters, scaleDiagnostics.phoneDistanceMeters);
                const bool scaleRatioLooksWrong = ScaleRatioLooksWrong(stableScaleRatio,
                                                                       scaleDiagnostics.stableOrbDisplacementMeters,
                                                                       scaleDiagnostics.stablePhoneDisplacementMeters,
                                                                       kScaleRatioWarnDistanceMeters);
                if(scaleRatioLooksWrong != loggedScaleRatioWarning) {
                    ostringstream scaleLog;
                    scaleLog << fixed << setprecision(3)
                             << "Scale ratio "
                             << (scaleRatioLooksWrong ? "out of range" : "back to normal")
                             << "  stable_ratio=" << stableScaleRatio
                             << "  stable_orb_m=" << scaleDiagnostics.stableOrbDisplacementMeters
                             << "  stable_phone_m=" << scaleDiagnostics.stablePhoneDisplacementMeters
                             << "  path_ratio=" << scalePathRatio
                             << "  path_orb_m=" << scaleDiagnostics.orbDistanceMeters
                             << "  path_phone_m=" << scaleDiagnostics.phoneDistanceMeters
                             << "  " << DescribeArkitReference(currentFrame.header);
                    if(scaleRatioLooksWrong)
                        LogWarning(scaleLog.str());
                    else
                        LogInfo(scaleLog.str());
                    loggedScaleRatioWarning = scaleRatioLooksWrong;
                }

                const ScaleWindowStats scaleWindow = CurrentScaleWindowStats(scaleDiagnostics);
                const bool scaleWindowLooksWrong = ScaleRatioLooksWrong(scaleWindow.displacementRatio,
                                                                        scaleWindow.orbDisplacementMeters,
                                                                        scaleWindow.phoneDisplacementMeters,
                                                                        kScaleRatioWindowWarnDistanceMeters);
                if(scaleWindowLooksWrong != loggedScaleWindowWarning) {
                    ostringstream scaleWindowLog;
                    scaleWindowLog << fixed << setprecision(3)
                                   << "Recent scale ratio "
                                   << (scaleWindowLooksWrong ? "out of range" : "back to normal")
                                   << "  disp_ratio=" << scaleWindow.displacementRatio
                                   << "  recent_orb_disp_m=" << scaleWindow.orbDisplacementMeters
                                   << "  recent_phone_disp_m=" << scaleWindow.phoneDisplacementMeters
                                   << "  path_ratio=" << scaleWindow.ratio
                                   << "  recent_orb_path_m=" << scaleWindow.orbDistanceMeters
                                   << "  recent_phone_path_m=" << scaleWindow.phoneDistanceMeters
                                   << "  window_s=" << scaleWindow.durationSeconds
                                   << "  " << DescribeArkitReference(currentFrame.header);
                    if(scaleWindowLooksWrong)
                        LogWarning(scaleWindowLog.str());
                    else
                        LogInfo(scaleWindowLog.str());
                    loggedScaleWindowWarning = scaleWindowLooksWrong;
                }
            }

            const bool previousPathValid = planner.pathValid;
            MaybeUpdatePlan(grid, liveStatus, planner);
            if(planner.pathValid != previousPathValid || planner.pathValid != loggedPathValid) {
                if(planner.pathValid) {
                    ostringstream pathLog;
                    pathLog << "Planner path ready  cells=" << planner.pathCells.size();
                    LogInfo(pathLog.str());
                } else if(planner.hasGoal) {
                    LogWarning("Planner path unavailable for the current goal.");
                }
                loggedPathValid = planner.pathValid;
            }
            guidanceState = ComputeGuidanceState(grid, liveStatus, planner, navigationSettings);
            UpdateGuidanceFile(guidanceState, liveStatus, navigationSettings);

            const auto renderStart = chrono::steady_clock::now();
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

            cv::Mat depthPreview;
            if(!mapDepth.empty())
                depthPreview = RotateForPreview(ColorizeDepthMm(mapDepth), currentFrame.header.displayOrientation);
            else
                depthPreview = BuildAppPreviewPlaceholder(cv::Size(640, 480), "深度预览", "当前模式下本帧没有可用构图深度");
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
            const auto latestArtifactsInterval = EffectiveLatestArtifactsInterval(appMode, navigationSettings);
            const auto mapDataInterval = EffectiveMapDataInterval(appMode);
            UpdateLatestArtifacts(renderWriter, composite, rgbPreview, depthPreview, mapPanel,
                                  modelDepthPreview, depthDiffPreview, navigationSettings, latestArtifactsInterval);
            QueueMapRenderData(renderWriter, BuildLatestMapDataPath(), grid, viewState, liveStatus, planner, trajectory, navigationSettings,
                               mapDataInterval);
            WriteBackendState(statePath, liveStatus, planner, guidanceState, navigationSettings, depthSourceState, scaleDiagnostics, depthComparison,
                              lastControlRevision, viewState, viewState.panelRect, composite.size(), true);
            renderMs = chrono::duration<double, std::milli>(chrono::steady_clock::now() - renderStart).count();

            bool requestSnapshot = false;
            bool requestQuit = false;
            bool requestExperimentReset = false;
            bool requestSlamRestart = false;
            BackendControl control;
            const int previousControlRevision = lastControlRevision;
            if(ReadBackendControl(controlPath, control))
                ApplyBackendControl(control, lastControlRevision, viewState, planner, navigationSettings, grid, localizationOnly, slam.get(),
                                    requestExperimentReset, requestSlamRestart, requestSnapshot, requestQuit);
            if(lastControlRevision != previousControlRevision) {
                LogInfo("Applied control: " + DescribeBackendControl(control, navigationSettings, localizationOnly, planner,
                                                                     requestExperimentReset, requestSlamRestart, requestSnapshot, requestQuit));
            }
            if(requestExperimentReset) {
                LogInfo("Resetting navigation experiment state during an active stream.");
                ResetNavigationExperimentState(grid, planner, trajectory, floorEstimator, navFrameEstimator,
                                               scaleDiagnostics, guidanceState, depthComparison, liveStatus,
                                               viewState, localizationOnly);
                if(requestSlamRestart && slam) {
                    LogInfo("Restarting ORB-SLAM3 due to control request.");
                    slam->Shutdown();
                    slam = make_unique<ORB_SLAM3::System>(vocabularyPath, settingsPath, ORB_SLAM3::System::RGBD, false, 0, outputPrefix);
                    imageScale = slam->GetImageScale();
                    if(localizationOnly)
                        slam->ActivateLocalizationMode();
                    loggedTrackingState = liveStatus.trackingState;
                    loggedHasPose = liveStatus.hasPose;
                    loggedDepthSourceStatus = depthSourceState.status;
                    loggedPathValid = planner.pathValid;
                    loggedScaleRatioWarning = false;
                    loggedScaleWindowWarning = false;
                    loggedDepthQualityWarning = false;
                    performanceWindow.Reset(grid.version());
                }
            }
            if(requestSnapshot)
                SaveSnapshot(composite);
            if(requestQuit) {
                LogInfo("Quit requested by control.");
                gKeepRunning = false;
                break;
            }

            const double processingMs =
                chrono::duration<double, std::milli>(chrono::steady_clock::now() - processingStart).count();
            if(!performanceWindow.hasStartGridVersion)
                performanceWindow.Reset(grid.version());
            ++performanceWindow.frames;
            performanceWindow.totalProcessingMs += processingMs;
            if(captureGapMs > 1e-3) {
                ++performanceWindow.captureGapSamples;
                performanceWindow.totalCaptureGapMs += captureGapMs;
            }
            if(trackMs > 1e-3) {
                ++performanceWindow.trackSamples;
                performanceWindow.totalTrackMs += trackMs;
            }
            if(depthComparison.inferenceMs > 1e-3f) {
                ++performanceWindow.inferenceSamples;
                performanceWindow.totalInferenceMs += depthComparison.inferenceMs;
            }
            if(mapMs > 1e-3) {
                ++performanceWindow.mapSamples;
                performanceWindow.totalMapMs += mapMs;
            }
            if(renderMs > 1e-3) {
                ++performanceWindow.renderSamples;
                performanceWindow.totalRenderMs += renderMs;
            }
            if(poseUpdatedThisFrame) {
                ++performanceWindow.poseFrames;
                performanceWindow.totalFloorSamples += floorSampleCount;
                performanceWindow.totalMapPoints += mapPointCount;
            }
            if(performanceWindow.frames >= kPerformanceSummaryFrames) {
                LogPerformanceSummary(performanceWindow, grid, planner, trajectory, scaleDiagnostics, depthComparison,
                                      currentFrame.header, localizationOnly);
                performanceWindow.Reset(grid.version());
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
            while(gKeepRunning) {
                try {
                    if(!ReceiveFrame(clientFd, nextFrame)) {
                        LogWarning("iPhone stream disconnected.");
                        sessionDisconnected = true;
                        break;
                    }
                    if(nextFrame.depthMm.empty()) {
                        LogWarning("Skipping frame without RGB-D depth payload.");
                        continue;
                    }
                    break;
                } catch(const exception& error) {
                    LogError("Stream error: " + string(error.what()));
                    sessionDisconnected = true;
                    break;
                }
            }
            if(sessionDisconnected || !gKeepRunning)
                break;

            currentFrame = std::move(nextFrame);
            ++frameIndex;
        }

        close(clientFd);
        if(gKeepRunning && sessionDisconnected) {
            LogInfo("Session paused. Waiting for the next iPhone RGB-D stream...");
            liveStatus.trackingState = "PAUSED";
            loggedTrackingState = liveStatus.trackingState;
            loggedHasPose = liveStatus.hasPose;
        }
    }

    if(slam) {
        LogInfo("Shutting down ORB-SLAM3.");
        slam->Shutdown();
    }

    if(serverFd >= 0)
        close(serverFd);
    if(!appMode)
        cv::destroyAllWindows();
    LogInfo("===== rgbd_iphone_stream session ended =====");
    return 0;
}
