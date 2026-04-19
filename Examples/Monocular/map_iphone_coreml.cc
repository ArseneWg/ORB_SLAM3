#include "coreml_depth_estimator.h"

#include <arpa/inet.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fcntl.h>
#include <ifaddrs.h>
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
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Eigenvalues>

#include <MapPoint.h>
#include <System.h>

using namespace std;
using boost::property_tree::ptree;

namespace
{
constexpr char kMagic[] = {'L', 'D', 'R', '1'};
constexpr char kWindowName[] = "ORB-SLAM3 iPhone Triptych";
constexpr int kDefaultFps = 10;
constexpr int kMapPanelWidth = 760;
constexpr int kMapPanelHeight = 520;
constexpr int kPointStride = 12;
constexpr float kMinDepth = 0.15f;
constexpr float kMaxDepth = 4.5f;
constexpr float kMapViewYaw = 0.85f;
constexpr float kMapViewPitch = -0.52f;

atomic<bool> gKeepRunning{true};

struct PacketHeader
{
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
};

struct StreamFrame
{
    PacketHeader header;
    cv::Mat rgb;
};

struct VoxelKey
{
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash
{
    size_t operator()(const VoxelKey& key) const
    {
        size_t seed = 0;
        seed ^= std::hash<int>()(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct CloudPoint
{
    Eigen::Vector3f position;
    cv::Vec3b color;
    float confidence = 1.0f;
};

enum class MapProjectionMode
{
    PerspectiveOrbit = 1,
    TopDownOrtho = 2,
    SideOrtho = 3
};

struct DisplayPose
{
    bool valid = false;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
};

struct DisplayAlignment
{
    bool estimated = false;
    Eigen::Matrix3f worldToDisplay = Eigen::Matrix3f::Identity();
    string referenceLabel = "SLAM frame";
};

struct SceneBounds
{
    bool valid = false;
    Eigen::Vector3f minPoint = Eigen::Vector3f::Zero();
    Eigen::Vector3f maxPoint = Eigen::Vector3f::Zero();
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    float radius = 1.0f;
    float planeY = 0.0f;
};

struct MapViewState
{
    MapProjectionMode projection = MapProjectionMode::TopDownOrtho;
    float yaw = kMapViewYaw;
    float pitch = kMapViewPitch;
    float distance = 1.8f;
    float orthoSpan = 2.2f;
    Eigen::Vector3f target = Eigen::Vector3f::Zero();
    cv::Rect panelRect = cv::Rect(0, 0, kMapPanelWidth, kMapPanelHeight);
    bool autoFit = true;
    bool leftDragging = false;
    bool rightDragging = false;
    cv::Point lastMouse;
    bool hasBounds = false;
    SceneBounds bounds;

    void ApplyBounds(const SceneBounds& sceneBounds)
    {
        bounds = sceneBounds;
        hasBounds = sceneBounds.valid;
        if(!autoFit || !sceneBounds.valid)
            return;

        target = sceneBounds.center;
        const Eigen::Vector3f span = sceneBounds.maxPoint - sceneBounds.minPoint;

        switch(projection)
        {
            case MapProjectionMode::PerspectiveOrbit:
                yaw = kMapViewYaw;
                pitch = kMapViewPitch;
                target.y() = std::max(sceneBounds.center.y(), sceneBounds.planeY + sceneBounds.radius * 0.22f);
                distance = std::max(0.95f, sceneBounds.radius * 2.25f);
                break;

            case MapProjectionMode::TopDownOrtho:
                target.y() = sceneBounds.planeY;
                orthoSpan = std::max(1.2f, std::max(span.x(), span.z()) * 1.18f + 0.35f);
                break;

            case MapProjectionMode::SideOrtho:
                orthoSpan = std::max(1.2f, std::max(span.x(), span.y()) * 1.18f + 0.35f);
                break;
        }
    }

    void ResetView()
    {
        autoFit = true;
        if(hasBounds)
            ApplyBounds(bounds);
    }
};

struct ViewCamera
{
    Eigen::Matrix3f Rcw = Eigen::Matrix3f::Identity();
    Eigen::Vector3f tcw = Eigen::Vector3f::Zero();
    Eigen::Vector3f eye = Eigen::Vector3f::Zero();
    Eigen::Vector3f forward = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f right = Eigen::Vector3f::UnitX();
    Eigen::Vector3f up = Eigen::Vector3f::UnitY();
};

void ExitLoopHandler(int)
{
    gKeepRunning = false;
}

bool ReadExact(int fd, void* buffer, size_t size)
{
    size_t readBytes = 0;
    auto* output = static_cast<uint8_t*>(buffer);
    while(readBytes < size && gKeepRunning)
    {
        const ssize_t chunk = recv(fd, output + readBytes, size - readBytes, 0);
        if(chunk == 0)
            return false;
        if(chunk < 0)
        {
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

vector<string> CandidateIps()
{
    vector<string> ips;
    ifaddrs* interfaces = nullptr;
    if(getifaddrs(&interfaces) != 0)
        return ips;

    for(ifaddrs* entry = interfaces; entry != nullptr; entry = entry->ifa_next)
    {
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

void PrintCandidateAddresses(int port)
{
    const vector<string> ips = CandidateIps();
    if(ips.empty())
    {
        cout << "Listening on port " << port << endl;
        return;
    }

    cout << "Enter one of these Mac IPs on the phone:" << endl;
    for(const string& ip : ips)
        cout << "  " << ip << ":" << port << endl;
}

PacketHeader ParseHeader(const string& jsonText)
{
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
    return header;
}

bool ReceiveFrame(int clientFd, StreamFrame& frame)
{
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

    if(frame.header.depthSize > 0)
    {
        vector<uint8_t> ignoredDepth(static_cast<size_t>(frame.header.depthSize));
        if(!ReadExact(clientFd, ignoredDepth.data(), ignoredDepth.size()))
            return false;
    }

    frame.rgb = cv::imdecode(rgbBytes, cv::IMREAD_COLOR);
    if(frame.rgb.empty())
        throw runtime_error("failed to decode JPEG frame");
    return true;
}

int CreateListenSocket(int port)
{
    const int serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if(serverFd < 0)
        throw runtime_error("failed to create socket");

    int reuse = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(port));

    if(::bind(serverFd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
    {
        close(serverFd);
        throw runtime_error("failed to bind socket");
    }
    if(listen(serverFd, 1) != 0)
    {
        close(serverFd);
        throw runtime_error("failed to listen on socket");
    }

    return serverFd;
}

bool WaitForSocketReadable(int fd, int timeoutMs)
{
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(fd, &readSet);

    timeval timeout {};
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;

    const int result = select(fd + 1, &readSet, nullptr, nullptr, &timeout);
    if(result < 0)
    {
        if(errno == EINTR)
            return false;
        throw runtime_error("socket select failed");
    }
    return result > 0 && FD_ISSET(fd, &readSet);
}

void SetSocketTimeout(int fd, int timeoutMs)
{
    timeval timeout {};
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

cv::Mat RotateForPreview(const cv::Mat& image, const string& orientation)
{
    if(orientation == "portrait")
    {
        cv::Mat rotated;
        cv::rotate(image, rotated, cv::ROTATE_90_CLOCKWISE);
        return rotated;
    }
    if(orientation == "portraitUpsideDown")
    {
        cv::Mat rotated;
        cv::rotate(image, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
        return rotated;
    }
    if(orientation == "landscapeLeft")
    {
        cv::Mat rotated;
        cv::rotate(image, rotated, cv::ROTATE_180);
        return rotated;
    }
    return image;
}

string TrackingStateToString(int state)
{
    switch(state)
    {
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

bool IsFinitePositive(float value)
{
    return std::isfinite(value) && value > 0.0f;
}

cv::Mat ColorizeDepth(const cv::Mat& depth32f)
{
    cv::Mat sanitized = depth32f.clone();
    cv::patchNaNs(sanitized, 0.0f);

    cv::Mat validMask = sanitized > 0.0f;
    double minDepth = 0.0;
    double maxDepth = 0.0;
    cv::minMaxLoc(sanitized, &minDepth, &maxDepth, nullptr, nullptr, validMask);

    if(maxDepth <= minDepth + numeric_limits<double>::epsilon())
        return cv::Mat(sanitized.size(), CV_8UC3, cv::Scalar(28, 28, 28));

    cv::Mat normalized;
    sanitized.convertTo(normalized, CV_32F, 255.0 / (maxDepth - minDepth), -minDepth * 255.0 / (maxDepth - minDepth));
    normalized.setTo(0.0f, ~validMask);

    cv::Mat normalized8u;
    normalized.convertTo(normalized8u, CV_8U);
    normalized8u = 255 - normalized8u;

    cv::Mat coloredDepth;
    cv::applyColorMap(normalized8u, coloredDepth, cv::COLORMAP_TURBO);
    coloredDepth.setTo(cv::Scalar(0, 0, 0), ~validMask);
    return coloredDepth;
}

void DrawLabel(cv::Mat& image, const string& text, const cv::Point& origin = cv::Point(10, 10))
{
    const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 0.7;
    const int thickness = 2;
    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    const cv::Rect box(origin.x, origin.y, textSize.width + 18, textSize.height + baseline + 14);
    cv::rectangle(image, box, cv::Scalar(18, 18, 18), cv::FILLED);
    cv::putText(image, text, cv::Point(origin.x + 9, origin.y + textSize.height + 1),
                fontFace, fontScale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
}

class VoxelCloud
{
public:
    explicit VoxelCloud(float voxelSize)
    : mVoxelSize(voxelSize)
    {
    }

    void Insert(const Eigen::Vector3f& position, const cv::Vec3b& color, float weight = 1.0f)
    {
        VoxelCell& cell = mVoxels[Quantize(position)];
        const float safeWeight = std::max(0.05f, weight);
        cell.positionSum += position * safeWeight;
        cell.colorSum += Eigen::Vector3f(color[0], color[1], color[2]) * safeWeight;
        cell.weightSum += safeWeight;
        cell.samples += 1;
    }

    vector<CloudPoint> Points() const
    {
        vector<CloudPoint> points;
        points.reserve(mVoxels.size());
        for(const auto& entry : mVoxels)
        {
            const VoxelCell& cell = entry.second;
            if(cell.weightSum <= numeric_limits<float>::epsilon())
                continue;

            const Eigen::Vector3f averageColor = cell.colorSum / cell.weightSum;
            CloudPoint point;
            point.position = cell.positionSum / cell.weightSum;
            point.color = cv::Vec3b(
                static_cast<uchar>(std::max(0.0f, std::min(255.0f, averageColor.x()))),
                static_cast<uchar>(std::max(0.0f, std::min(255.0f, averageColor.y()))),
                static_cast<uchar>(std::max(0.0f, std::min(255.0f, averageColor.z()))));
            point.confidence = std::max(0.22f, std::min(1.0f, std::log1p(static_cast<float>(cell.samples)) / 2.0f));
            points.push_back(point);
        }
        return points;
    }

private:
    struct VoxelCell
    {
        Eigen::Vector3f positionSum = Eigen::Vector3f::Zero();
        Eigen::Vector3f colorSum = Eigen::Vector3f::Zero();
        float weightSum = 0.0f;
        int samples = 0;
    };

    VoxelKey Quantize(const Eigen::Vector3f& position) const
    {
        return VoxelKey{
            static_cast<int>(std::floor(position.x() / mVoxelSize)),
            static_cast<int>(std::floor(position.y() / mVoxelSize)),
            static_cast<int>(std::floor(position.z() / mVoxelSize))
        };
    }

private:
    float mVoxelSize;
    unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> mVoxels;
};

float ComputeDepthScale(const Sophus::SE3f& Tcw,
                        const cv::Mat& depth32f,
                        const vector<ORB_SLAM3::MapPoint*>& trackedMapPoints,
                        const vector<cv::KeyPoint>& trackedKeypoints,
                        int& anchorCount)
{
    vector<float> ratios;
    const size_t count = min(trackedMapPoints.size(), trackedKeypoints.size());
    ratios.reserve(count);

    for(size_t i = 0; i < count; ++i)
    {
        ORB_SLAM3::MapPoint* mapPoint = trackedMapPoints[i];
        if(!mapPoint || mapPoint->isBad())
            continue;

        const cv::Point2f keypoint = trackedKeypoints[i].pt;
        const int u = static_cast<int>(std::round(keypoint.x));
        const int v = static_cast<int>(std::round(keypoint.y));
        if(u < 0 || u >= depth32f.cols || v < 0 || v >= depth32f.rows)
            continue;

        const float predictedDepth = depth32f.at<float>(v, u);
        if(!IsFinitePositive(predictedDepth))
            continue;

        const Eigen::Vector3f pointCamera = Tcw * mapPoint->GetWorldPos();
        if(pointCamera.z() <= 0.0f)
            continue;

        ratios.push_back(pointCamera.z() / predictedDepth);
    }

    anchorCount = static_cast<int>(ratios.size());
    if(ratios.size() < 20)
        return -1.0f;

    const size_t mid = ratios.size() / 2;
    nth_element(ratios.begin(), ratios.begin() + mid, ratios.end());
    return ratios[mid];
}

bool ShouldIntegrateFrame(const Sophus::SE3f& Twc,
                          const Sophus::SE3f& lastIntegratedTwc,
                          bool hasLastIntegratedPose,
                          int frameIndex)
{
    if(!hasLastIntegratedPose)
        return true;
    if(frameIndex % 2 != 0)
        return false;

    const float translation = (Twc.translation() - lastIntegratedTwc.translation()).norm();
    const Sophus::SE3f delta = lastIntegratedTwc.inverse() * Twc;
    const float rotation = delta.so3().log().norm();
    return translation > 0.025f || rotation > 0.05f;
}

void IntegrateFrame(const cv::Mat& frameBgr,
                    const cv::Mat& depth32f,
                    const Sophus::SE3f& Twc,
                    float fx,
                    float fy,
                    float cx,
                    float cy,
                    VoxelCloud& cloud)
{
    for(int v = 8; v < depth32f.rows - 8; v += kPointStride)
    {
        for(int u = 8; u < depth32f.cols - 8; u += kPointStride)
        {
            const float depth = depth32f.at<float>(v, u);
            if(!IsFinitePositive(depth) || depth < kMinDepth || depth > kMaxDepth)
                continue;

            const float x = (static_cast<float>(u) - cx) * depth / fx;
            const float y = (static_cast<float>(v) - cy) * depth / fy;
            const Eigen::Vector3f pointWorld = Twc * Eigen::Vector3f(x, y, depth);
            const float normalizedRadius = std::sqrt(
                std::pow((static_cast<float>(u) - cx) / std::max(1.0f, cx), 2.0f) +
                std::pow((static_cast<float>(v) - cy) / std::max(1.0f, cy), 2.0f));
            const float radialWeight = std::max(0.55f, std::min(1.15f, 1.15f - normalizedRadius * 0.28f));
            const float depthWeight = 1.0f / (1.0f + depth * 0.35f);
            cloud.Insert(pointWorld, frameBgr.at<cv::Vec3b>(v, u), radialWeight * depthWeight);
        }
    }
}

struct ProjectedPoint
{
    float depth;
    cv::Point pixel;
    cv::Vec3b color;
};

bool ProjectPoint(const Eigen::Vector3f& pointWorld,
                  const Eigen::Matrix3f& Rcw,
                  const Eigen::Vector3f& tcw,
                  float fx,
                  float fy,
                  float cx,
                  float cy,
                  cv::Point& pixel,
                  float& depth)
{
    const Eigen::Vector3f pointCamera = Rcw * pointWorld + tcw;
    depth = pointCamera.z();
    if(depth <= 0.05f)
        return false;

    const float invDepth = 1.0f / depth;
    pixel = cv::Point(
        static_cast<int>(std::round(fx * pointCamera.x() * invDepth + cx)),
        static_cast<int>(std::round(-fy * pointCamera.y() * invDepth + cy)));
    return true;
}

float ClampFloat(float value, float minValue, float maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

Eigen::Vector3f SafeNormalize(const Eigen::Vector3f& vector, const Eigen::Vector3f& fallback)
{
    const float norm = vector.norm();
    if(norm <= 1e-5f)
        return fallback;
    return vector / norm;
}

DisplayAlignment EstimateDisplayAlignment(const vector<CloudPoint>& cloudPoints,
                                          const vector<Eigen::Vector3f>& trajectory,
                                          const Sophus::SE3f* currentTwc)
{
    DisplayAlignment alignment;
    if(trajectory.size() < 6)
        return alignment;

    const size_t startIndex = trajectory.size() > 180 ? trajectory.size() - 180 : 0;
    vector<Eigen::Vector3f> samples(trajectory.begin() + static_cast<long>(startIndex), trajectory.end());

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for(const Eigen::Vector3f& sample : samples)
        mean += sample;
    mean /= static_cast<float>(samples.size());

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for(const Eigen::Vector3f& sample : samples)
    {
        const Eigen::Vector3f centered = sample - mean;
        covariance += centered * centered.transpose();
    }
    covariance /= static_cast<float>(samples.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if(solver.info() != Eigen::Success)
        return alignment;

    Eigen::Vector3f up = solver.eigenvectors().col(0);
    Eigen::Vector3f forwardSeed = samples.back() - samples.front();
    forwardSeed -= up * forwardSeed.dot(up);
    if(forwardSeed.squaredNorm() < 1e-4f)
    {
        forwardSeed = solver.eigenvectors().col(2);
        forwardSeed -= up * forwardSeed.dot(up);
    }

    Eigen::Vector3f forward = SafeNormalize(forwardSeed, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f right = SafeNormalize(up.cross(forward), Eigen::Vector3f::UnitX());
    forward = SafeNormalize(right.cross(up), Eigen::Vector3f::UnitZ());

    Eigen::Matrix3f worldToDisplay = Eigen::Matrix3f::Identity();
    worldToDisplay.row(0) = right.transpose();
    worldToDisplay.row(1) = up.transpose();
    worldToDisplay.row(2) = forward.transpose();

    vector<float> ySamples;
    ySamples.reserve(std::min<size_t>(cloudPoints.size(), 2048));
    const size_t sampleStride = std::max<size_t>(1, cloudPoints.size() / 2048);
    for(size_t index = 0; index < cloudPoints.size(); index += sampleStride)
        ySamples.push_back((worldToDisplay * cloudPoints[index].position).y());
    if(ySamples.empty())
    {
        ySamples.reserve(samples.size());
        for(const Eigen::Vector3f& sample : samples)
            ySamples.push_back((worldToDisplay * sample).y());
    }

    if(!ySamples.empty())
    {
        sort(ySamples.begin(), ySamples.end());
        const size_t quantileIndex = std::min(ySamples.size() - 1, ySamples.size() / 5);
        const float lowQuantile = ySamples[quantileIndex];
        const Eigen::Vector3f referencePoint = currentTwc ? currentTwc->translation() : samples.back();
        if((worldToDisplay * referencePoint).y() < lowQuantile)
        {
            worldToDisplay.row(0) *= -1.0f;
            worldToDisplay.row(1) *= -1.0f;
        }
    }

    alignment.estimated = true;
    alignment.worldToDisplay = worldToDisplay;
    alignment.referenceLabel = "Estimated walk plane";
    return alignment;
}

vector<CloudPoint> TransformCloudPoints(const vector<CloudPoint>& cloudPoints, const DisplayAlignment& alignment)
{
    vector<CloudPoint> transformed;
    transformed.reserve(cloudPoints.size());
    for(const CloudPoint& point : cloudPoints)
    {
        CloudPoint transformedPoint = point;
        transformedPoint.position = alignment.worldToDisplay * point.position;
        transformed.push_back(transformedPoint);
    }
    return transformed;
}

vector<Eigen::Vector3f> TransformTrajectoryPoints(const vector<Eigen::Vector3f>& trajectory, const DisplayAlignment& alignment)
{
    vector<Eigen::Vector3f> transformed;
    transformed.reserve(trajectory.size());
    for(const Eigen::Vector3f& point : trajectory)
        transformed.push_back(alignment.worldToDisplay * point);
    return transformed;
}

DisplayPose TransformPoseForDisplay(const Sophus::SE3f* currentTwc, const DisplayAlignment& alignment)
{
    DisplayPose pose;
    if(!currentTwc)
        return pose;

    pose.valid = true;
    pose.position = alignment.worldToDisplay * currentTwc->translation();
    pose.rotation = alignment.worldToDisplay * currentTwc->rotationMatrix();
    return pose;
}

cv::Vec3b HeightTintColor(float normalizedHeight)
{
    const float t = ClampFloat(normalizedHeight, 0.0f, 1.0f);
    const array<cv::Vec3f, 4> palette = {
        cv::Vec3f(180.0f, 90.0f, 30.0f),
        cv::Vec3f(250.0f, 210.0f, 70.0f),
        cv::Vec3f(115.0f, 220.0f, 160.0f),
        cv::Vec3f(70.0f, 90.0f, 255.0f)
    };

    const float scaled = t * static_cast<float>(palette.size() - 1);
    const int lower = static_cast<int>(std::floor(scaled));
    const int upper = std::min(lower + 1, static_cast<int>(palette.size() - 1));
    const float alpha = scaled - static_cast<float>(lower);
    const cv::Vec3f blended = palette[lower] * (1.0f - alpha) + palette[upper] * alpha;
    return cv::Vec3b(
        static_cast<uchar>(ClampFloat(blended[0], 0.0f, 255.0f)),
        static_cast<uchar>(ClampFloat(blended[1], 0.0f, 255.0f)),
        static_cast<uchar>(ClampFloat(blended[2], 0.0f, 255.0f)));
}

cv::Vec3b BlendHeightColor(const cv::Vec3b& baseColor, float normalizedHeight, float confidence)
{
    const cv::Vec3b tint = HeightTintColor(normalizedHeight);
    const float tintWeight = ClampFloat(0.55f + (1.0f - ClampFloat(confidence, 0.0f, 1.0f)) * 0.15f, 0.45f, 0.75f);
    cv::Vec3b blended;
    for(int channel = 0; channel < 3; ++channel)
    {
        const float value = static_cast<float>(baseColor[channel]) * (1.0f - tintWeight) + static_cast<float>(tint[channel]) * tintWeight;
        blended[channel] = static_cast<uchar>(ClampFloat(value, 0.0f, 255.0f));
    }
    return blended;
}

SceneBounds ComputeSceneBounds(const vector<CloudPoint>& cloudPoints,
                               const vector<Eigen::Vector3f>& trajectory,
                               const Eigen::Vector3f* currentPosition)
{
    SceneBounds bounds;
    vector<float> ySamples;
    ySamples.reserve(cloudPoints.size() + trajectory.size() + 1);

    auto extend = [&](const Eigen::Vector3f& point)
    {
        if(!bounds.valid)
        {
            bounds.valid = true;
            bounds.minPoint = point;
            bounds.maxPoint = point;
        }
        else
        {
            bounds.minPoint = bounds.minPoint.cwiseMin(point);
            bounds.maxPoint = bounds.maxPoint.cwiseMax(point);
        }
        ySamples.push_back(point.y());
    };

    for(const CloudPoint& point : cloudPoints)
        extend(point.position);
    for(const Eigen::Vector3f& point : trajectory)
        extend(point);
    if(currentPosition)
        extend(*currentPosition);

    if(!bounds.valid)
        return bounds;

    bounds.center = 0.5f * (bounds.minPoint + bounds.maxPoint);
    bounds.radius = std::max(0.65f, (bounds.maxPoint - bounds.minPoint).norm() * 0.78f + 0.55f);

    sort(ySamples.begin(), ySamples.end());
    const size_t quantileIndex = std::min(ySamples.size() - 1, ySamples.size() / 8);
    bounds.planeY = std::min(ySamples[quantileIndex], bounds.center.y());
    bounds.planeY -= bounds.radius * 0.06f;
    return bounds;
}

ViewCamera BuildOrbitCamera(const MapViewState& viewState)
{
    ViewCamera camera;

    Eigen::Vector3f offset;
    offset.x() = std::cos(viewState.pitch) * std::cos(viewState.yaw);
    offset.y() = std::sin(viewState.pitch);
    offset.z() = std::cos(viewState.pitch) * std::sin(viewState.yaw);

    camera.eye = viewState.target + viewState.distance * offset;
    camera.forward = (viewState.target - camera.eye).normalized();

    Eigen::Vector3f worldUp(0.0f, 1.0f, 0.0f);
    if(std::fabs(camera.forward.dot(worldUp)) > 0.95f)
        worldUp = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

    camera.right = camera.forward.cross(worldUp).normalized();
    camera.up = camera.right.cross(camera.forward).normalized();

    camera.Rcw.row(0) = camera.right.transpose();
    camera.Rcw.row(1) = camera.up.transpose();
    camera.Rcw.row(2) = camera.forward.transpose();
    camera.tcw = -camera.Rcw * camera.eye;
    return camera;
}

bool ProjectWorldPointToPanel(const Eigen::Vector3f& pointWorld,
                              const ViewCamera& camera,
                              float fx,
                              float fy,
                              float cx,
                              float cy,
                              int panelWidth,
                              int panelHeight,
                              cv::Point& pixel,
                              float& depth)
{
    if(!ProjectPoint(pointWorld, camera.Rcw, camera.tcw, fx, fy, cx, cy, pixel, depth))
        return false;
    return pixel.x >= 0 && pixel.x < panelWidth && pixel.y >= 0 && pixel.y < panelHeight;
}

void DrawWorldSegment(cv::Mat& panel,
                      const Eigen::Vector3f& start,
                      const Eigen::Vector3f& end,
                      const ViewCamera& camera,
                      float fx,
                      float fy,
                      float cx,
                      float cy,
                      const cv::Scalar& color,
                      int thickness,
                      int lineType = cv::LINE_AA)
{
    cv::Point startPixel;
    cv::Point endPixel;
    float startDepth = 0.0f;
    float endDepth = 0.0f;
    if(!ProjectWorldPointToPanel(start, camera, fx, fy, cx, cy, panel.cols, panel.rows, startPixel, startDepth))
        return;
    if(!ProjectWorldPointToPanel(end, camera, fx, fy, cx, cy, panel.cols, panel.rows, endPixel, endDepth))
        return;
    cv::line(panel, startPixel, endPixel, color, thickness, lineType);
}

void DrawReferenceGrid(cv::Mat& panel,
                       const SceneBounds& bounds,
                       const ViewCamera& camera,
                       float fx,
                       float fy,
                       float cx,
                       float cy)
{
    const float spanX = std::max(0.8f, bounds.maxPoint.x() - bounds.minPoint.x());
    const float spanZ = std::max(0.8f, bounds.maxPoint.z() - bounds.minPoint.z());
    const float halfExtent = std::max(spanX, spanZ) * 0.7f + 0.35f;
    const float spacing = ClampFloat(halfExtent / 5.0f, 0.16f, 0.5f);
    const float originX = std::floor(bounds.center.x() / spacing) * spacing;
    const float originZ = std::floor(bounds.center.z() / spacing) * spacing;
    const int lineCount = static_cast<int>(std::ceil(halfExtent / spacing));

    for(int lineIndex = -lineCount; lineIndex <= lineCount; ++lineIndex)
    {
        const float offset = lineIndex * spacing;
        const cv::Scalar color = (lineIndex == 0) ? cv::Scalar(90, 90, 90) : cv::Scalar(42, 42, 42);

        DrawWorldSegment(panel,
                         Eigen::Vector3f(originX + offset, bounds.planeY, originZ - halfExtent),
                         Eigen::Vector3f(originX + offset, bounds.planeY, originZ + halfExtent),
                         camera, fx, fy, cx, cy, color, 1);
        DrawWorldSegment(panel,
                         Eigen::Vector3f(originX - halfExtent, bounds.planeY, originZ + offset),
                         Eigen::Vector3f(originX + halfExtent, bounds.planeY, originZ + offset),
                         camera, fx, fy, cx, cy, color, 1);
    }
}

void DrawReferenceAxes(cv::Mat& panel,
                       const SceneBounds& bounds,
                       const ViewCamera& camera,
                       float fx,
                       float fy,
                       float cx,
                       float cy)
{
    const Eigen::Vector3f origin(bounds.center.x(), bounds.planeY, bounds.center.z());
    const float axisLength = std::max(0.18f, bounds.radius * 0.28f);

    const Eigen::Vector3f axisX = origin + Eigen::Vector3f(axisLength, 0.0f, 0.0f);
    const Eigen::Vector3f axisY = origin + Eigen::Vector3f(0.0f, axisLength, 0.0f);
    const Eigen::Vector3f axisZ = origin + Eigen::Vector3f(0.0f, 0.0f, axisLength);

    DrawWorldSegment(panel, origin, axisX, camera, fx, fy, cx, cy, cv::Scalar(70, 70, 255), 2);
    DrawWorldSegment(panel, origin, axisY, camera, fx, fy, cx, cy, cv::Scalar(90, 220, 90), 2);
    DrawWorldSegment(panel, origin, axisZ, camera, fx, fy, cx, cy, cv::Scalar(255, 160, 70), 2);

    cv::Point labelPixel;
    float labelDepth = 0.0f;
    if(ProjectWorldPointToPanel(axisX, camera, fx, fy, cx, cy, panel.cols, panel.rows, labelPixel, labelDepth))
        cv::putText(panel, "X", labelPixel + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(70, 70, 255), 2, cv::LINE_AA);
    if(ProjectWorldPointToPanel(axisY, camera, fx, fy, cx, cy, panel.cols, panel.rows, labelPixel, labelDepth))
        cv::putText(panel, "Y", labelPixel + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(90, 220, 90), 2, cv::LINE_AA);
    if(ProjectWorldPointToPanel(axisZ, camera, fx, fy, cx, cy, panel.cols, panel.rows, labelPixel, labelDepth))
        cv::putText(panel, "Z", labelPixel + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 160, 70), 2, cv::LINE_AA);
}

void DrawCurrentCameraMarker(cv::Mat& panel,
                             const DisplayPose& pose,
                             const SceneBounds& bounds,
                             const ViewCamera& camera,
                             float fx,
                             float fy,
                             float cx,
                             float cy)
{
    const Eigen::Vector3f center = pose.position;
    const Eigen::Matrix3f& Rwc = pose.rotation;
    const float frustumDepth = std::max(0.10f, bounds.radius * 0.16f);
    const float halfWidth = frustumDepth * 0.65f;
    const float halfHeight = frustumDepth * 0.42f;

    const array<Eigen::Vector3f, 4> cornersCamera = {
        Eigen::Vector3f(-halfWidth, -halfHeight, frustumDepth),
        Eigen::Vector3f( halfWidth, -halfHeight, frustumDepth),
        Eigen::Vector3f( halfWidth,  halfHeight, frustumDepth),
        Eigen::Vector3f(-halfWidth,  halfHeight, frustumDepth)
    };

    array<Eigen::Vector3f, 4> cornersWorld;
    for(size_t i = 0; i < cornersCamera.size(); ++i)
        cornersWorld[i] = center + Rwc * cornersCamera[i];

    for(const Eigen::Vector3f& corner : cornersWorld)
        DrawWorldSegment(panel, center, corner, camera, fx, fy, cx, cy, cv::Scalar(40, 210, 255), 2);
    for(size_t i = 0; i < cornersWorld.size(); ++i)
        DrawWorldSegment(panel, cornersWorld[i], cornersWorld[(i + 1) % cornersWorld.size()],
                         camera, fx, fy, cx, cy, cv::Scalar(40, 210, 255), 2);

    const Eigen::Vector3f forwardTip = center + Rwc * Eigen::Vector3f(0.0f, 0.0f, frustumDepth * 1.55f);
    DrawWorldSegment(panel, center, forwardTip, camera, fx, fy, cx, cy, cv::Scalar(0, 100, 255), 3);
}

void ApplyViewPreset(MapViewState& viewState, int preset)
{
    if(!viewState.hasBounds)
        return;

    switch(preset)
    {
        case 1:
            viewState.projection = MapProjectionMode::PerspectiveOrbit;
            break;
        case 2:
            viewState.projection = MapProjectionMode::TopDownOrtho;
            break;
        case 3:
            viewState.projection = MapProjectionMode::SideOrtho;
            break;
        default:
            viewState.ResetView();
            break;
    }

    if(viewState.hasBounds)
    {
        const bool previousAutoFit = viewState.autoFit;
        viewState.autoFit = true;
        viewState.ApplyBounds(viewState.bounds);
        viewState.autoFit = previousAutoFit ? true : false;
        viewState.autoFit = false;
    }
}

void DrawTopDownInset(cv::Mat& panel,
                      const SceneBounds& bounds,
                      const vector<CloudPoint>& cloudPoints,
                      const vector<Eigen::Vector3f>& trajectory,
                      const DisplayPose* currentPose)
{
    const int insetWidth = std::max(180, panel.cols / 4);
    const int insetHeight = std::max(150, panel.rows / 4);
    const cv::Rect inset(panel.cols - insetWidth - 18, 18, insetWidth, insetHeight);

    cv::rectangle(panel, inset, cv::Scalar(12, 12, 12), cv::FILLED);
    cv::rectangle(panel, inset, cv::Scalar(72, 72, 72), 1);
    cv::putText(panel, "Top-down", inset.tl() + cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(245, 245, 245), 2, cv::LINE_AA);

    const float spanX = std::max(0.45f, bounds.maxPoint.x() - bounds.minPoint.x());
    const float spanZ = std::max(0.45f, bounds.maxPoint.z() - bounds.minPoint.z());
    const float span = std::max(spanX, spanZ) * 1.12f;
    const Eigen::Vector2f center(bounds.center.x(), bounds.center.z());

    auto toPixel = [&](const Eigen::Vector3f& point) -> cv::Point
    {
        const float nx = (point.x() - center.x()) / span;
        const float nz = (point.z() - center.y()) / span;
        const int px = inset.x + inset.width / 2 + static_cast<int>(std::round(nx * (inset.width * 0.78f)));
        const int py = inset.y + inset.height / 2 - static_cast<int>(std::round(nz * (inset.height * 0.78f)));
        return cv::Point(px, py);
    };

    for(size_t i = 0; i < cloudPoints.size(); i += 3)
    {
        const cv::Point pixel = toPixel(cloudPoints[i].position);
        if(pixel.x <= inset.x + 2 || pixel.x >= inset.x + inset.width - 2 ||
           pixel.y <= inset.y + 2 || pixel.y >= inset.y + inset.height - 2)
            continue;
        panel.at<cv::Vec3b>(pixel) = cv::Vec3b(210, 210, 210);
    }

    vector<cv::Point> projectedTrajectory;
    projectedTrajectory.reserve(trajectory.size());
    for(const Eigen::Vector3f& point : trajectory)
        projectedTrajectory.push_back(toPixel(point));
    for(size_t i = 1; i < projectedTrajectory.size(); ++i)
        cv::line(panel, projectedTrajectory[i - 1], projectedTrajectory[i], cv::Scalar(0, 220, 255), 1, cv::LINE_AA);

    if(currentPose && currentPose->valid)
    {
        const Eigen::Vector3f currentPosition = currentPose->position;
        const cv::Point centerPixel = toPixel(currentPosition);
        cv::circle(panel, centerPixel, 4, cv::Scalar(0, 90, 255), cv::FILLED, cv::LINE_AA);

        const Eigen::Vector3f forward = currentPose->rotation * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f tipPoint = currentPosition + forward.normalized() * std::max(0.12f, bounds.radius * 0.18f);
        cv::arrowedLine(panel, centerPixel, toPixel(tipPoint), cv::Scalar(0, 180, 255), 1, cv::LINE_AA, 0, 0.24);
    }
}

bool ProjectOrthoPointToPanel(const Eigen::Vector3f& pointWorld,
                              const MapViewState& viewState,
                              int horizontalAxis,
                              int verticalAxis,
                              int depthAxis,
                              int panelWidth,
                              int panelHeight,
                              cv::Point& pixel,
                              float& depth)
{
    const float verticalSpan = std::max(0.65f, viewState.orthoSpan);
    const float horizontalSpan = verticalSpan * static_cast<float>(panelWidth) / static_cast<float>(panelHeight);
    const float halfHorizontalSpan = horizontalSpan * 0.5f;
    const float halfVerticalSpan = verticalSpan * 0.5f;

    const float normalizedX = (pointWorld[horizontalAxis] - viewState.target[horizontalAxis]) / std::max(0.01f, halfHorizontalSpan);
    const float normalizedY = (pointWorld[verticalAxis] - viewState.target[verticalAxis]) / std::max(0.01f, halfVerticalSpan);

    pixel.x = static_cast<int>(std::round(panelWidth * 0.5f + normalizedX * panelWidth * 0.43f));
    pixel.y = static_cast<int>(std::round(panelHeight * 0.54f - normalizedY * panelHeight * 0.43f));
    depth = pointWorld[depthAxis];
    return pixel.x >= 0 && pixel.x < panelWidth && pixel.y >= 0 && pixel.y < panelHeight;
}

void DrawOrthoSegment(cv::Mat& panel,
                      const Eigen::Vector3f& start,
                      const Eigen::Vector3f& end,
                      const MapViewState& viewState,
                      int horizontalAxis,
                      int verticalAxis,
                      int depthAxis,
                      const cv::Scalar& color,
                      int thickness)
{
    cv::Point startPixel;
    cv::Point endPixel;
    float startDepth = 0.0f;
    float endDepth = 0.0f;
    if(!ProjectOrthoPointToPanel(start, viewState, horizontalAxis, verticalAxis, depthAxis, panel.cols, panel.rows, startPixel, startDepth))
        return;
    if(!ProjectOrthoPointToPanel(end, viewState, horizontalAxis, verticalAxis, depthAxis, panel.cols, panel.rows, endPixel, endDepth))
        return;
    cv::line(panel, startPixel, endPixel, color, thickness, cv::LINE_AA);
}

void DrawOrthoReference(cv::Mat& panel,
                        const SceneBounds& bounds,
                        const MapViewState& viewState,
                        int horizontalAxis,
                        int verticalAxis,
                        int depthAxis)
{
    const Eigen::Vector3f span = bounds.maxPoint - bounds.minPoint;
    const float horizontalSpan = (horizontalAxis == 2) ? span.z() : span[horizontalAxis];
    const float verticalSpan = (verticalAxis == 2) ? span.z() : span[verticalAxis];
    const float halfExtentH = std::max(0.8f, horizontalSpan) * 0.75f + 0.25f;
    const float halfExtentV = std::max(0.8f, verticalSpan) * 0.75f + 0.25f;
    const float spacing = ClampFloat(std::max(halfExtentH, halfExtentV) / 6.0f, 0.18f, 0.6f);

    const int lineCountH = static_cast<int>(std::ceil(halfExtentH / spacing));
    const int lineCountV = static_cast<int>(std::ceil(halfExtentV / spacing));

    for(int index = -lineCountH; index <= lineCountH; ++index)
    {
        Eigen::Vector3f start = bounds.center;
        Eigen::Vector3f end = bounds.center;
        start[horizontalAxis] += index * spacing;
        end[horizontalAxis] += index * spacing;
        start[verticalAxis] -= halfExtentV;
        end[verticalAxis] += halfExtentV;
        DrawOrthoSegment(panel, start, end, viewState, horizontalAxis, verticalAxis, depthAxis,
                         index == 0 ? cv::Scalar(92, 92, 92) : cv::Scalar(38, 38, 38), 1);
    }

    for(int index = -lineCountV; index <= lineCountV; ++index)
    {
        Eigen::Vector3f start = bounds.center;
        Eigen::Vector3f end = bounds.center;
        start[verticalAxis] += index * spacing;
        end[verticalAxis] += index * spacing;
        start[horizontalAxis] -= halfExtentH;
        end[horizontalAxis] += halfExtentH;
        DrawOrthoSegment(panel, start, end, viewState, horizontalAxis, verticalAxis, depthAxis,
                         index == 0 ? cv::Scalar(92, 92, 92) : cv::Scalar(38, 38, 38), 1);
    }
}

void DrawOrthoCameraMarker(cv::Mat& panel,
                           const DisplayPose& pose,
                           const SceneBounds& bounds,
                           const MapViewState& viewState,
                           int horizontalAxis,
                           int verticalAxis,
                           int depthAxis)
{
    cv::Point centerPixel;
    float depth = 0.0f;
    if(!ProjectOrthoPointToPanel(pose.position, viewState, horizontalAxis, verticalAxis, depthAxis, panel.cols, panel.rows, centerPixel, depth))
        return;

    cv::circle(panel, centerPixel, 6, cv::Scalar(0, 90, 255), cv::FILLED, cv::LINE_AA);
    const Eigen::Vector3f forward = pose.rotation * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    const Eigen::Vector3f tip = pose.position + forward.normalized() * std::max(0.10f, bounds.radius * 0.16f);
    cv::Point tipPixel;
    if(ProjectOrthoPointToPanel(tip, viewState, horizontalAxis, verticalAxis, depthAxis, panel.cols, panel.rows, tipPixel, depth))
        cv::arrowedLine(panel, centerPixel, tipPixel, cv::Scalar(0, 180, 255), 2, cv::LINE_AA, 0, 0.22);
}

void HandleMapMouse(int event, int x, int y, int flags, void* userdata)
{
    auto* viewState = static_cast<MapViewState*>(userdata);
    if(!viewState)
        return;

    const cv::Point mousePoint(x, y);
    const bool insidePanel = viewState->panelRect.contains(mousePoint);

    switch(event)
    {
        case cv::EVENT_LBUTTONDOWN:
            if(insidePanel)
            {
                viewState->leftDragging = true;
                viewState->rightDragging = false;
                viewState->lastMouse = mousePoint;
                viewState->autoFit = false;
            }
            break;

        case cv::EVENT_RBUTTONDOWN:
            if(insidePanel)
            {
                viewState->rightDragging = true;
                viewState->leftDragging = false;
                viewState->lastMouse = mousePoint;
                viewState->autoFit = false;
            }
            break;

        case cv::EVENT_MOUSEMOVE:
            if(viewState->leftDragging)
            {
                const cv::Point delta = mousePoint - viewState->lastMouse;
                if(viewState->projection == MapProjectionMode::PerspectiveOrbit)
                {
                    viewState->yaw -= static_cast<float>(delta.x) * 0.0065f;
                    viewState->pitch = ClampFloat(viewState->pitch - static_cast<float>(delta.y) * 0.0065f, -1.38f, 1.25f);
                }
                else
                {
                    const float worldPerPixel = std::max(0.0035f, viewState->orthoSpan / std::max(1, viewState->panelRect.height));
                    viewState->target.x() -= static_cast<float>(delta.x) * worldPerPixel;
                    if(viewState->projection == MapProjectionMode::TopDownOrtho)
                        viewState->target.z() += static_cast<float>(delta.y) * worldPerPixel;
                    else
                        viewState->target.y() += static_cast<float>(delta.y) * worldPerPixel;
                }
                viewState->lastMouse = mousePoint;
            }
            else if(viewState->rightDragging)
            {
                const cv::Point delta = mousePoint - viewState->lastMouse;
                if(viewState->projection == MapProjectionMode::PerspectiveOrbit)
                {
                    const ViewCamera camera = BuildOrbitCamera(*viewState);
                    const float panScale = viewState->distance * 0.0016f;
                    viewState->target += (-camera.right * static_cast<float>(delta.x) + camera.up * static_cast<float>(delta.y)) * panScale;
                }
                else
                {
                    const float worldPerPixel = std::max(0.0035f, viewState->orthoSpan / std::max(1, viewState->panelRect.height));
                    viewState->target.x() -= static_cast<float>(delta.x) * worldPerPixel;
                    if(viewState->projection == MapProjectionMode::TopDownOrtho)
                        viewState->target.z() += static_cast<float>(delta.y) * worldPerPixel;
                    else
                        viewState->target.y() += static_cast<float>(delta.y) * worldPerPixel;
                }
                viewState->lastMouse = mousePoint;
            }
            break;

        case cv::EVENT_LBUTTONUP:
            viewState->leftDragging = false;
            break;

        case cv::EVENT_RBUTTONUP:
            viewState->rightDragging = false;
            break;

        case cv::EVENT_LBUTTONDBLCLK:
            if(insidePanel)
                viewState->ResetView();
            break;

        case cv::EVENT_MOUSEWHEEL:
            if(insidePanel)
            {
                const int delta = cv::getMouseWheelDelta(flags);
                if(delta != 0)
                {
                    const float zoomFactor = std::exp(-static_cast<float>(delta) / 120.0f * 0.14f);
                    if(viewState->projection == MapProjectionMode::PerspectiveOrbit)
                        viewState->distance = ClampFloat(viewState->distance * zoomFactor, 0.3f, 28.0f);
                    else
                        viewState->orthoSpan = ClampFloat(viewState->orthoSpan * zoomFactor, 0.5f, 35.0f);
                    viewState->autoFit = false;
                }
            }
            break;

        default:
            break;
    }
}

cv::Mat RenderMapPanel(const vector<CloudPoint>& cloudPoints,
                       const vector<Eigen::Vector3f>& trajectory,
                       const Sophus::SE3f* currentTwc,
                       const string& trackingState,
                       MapViewState& viewState,
                       const cv::Size& panelSize = cv::Size(kMapPanelWidth, kMapPanelHeight),
                       bool showInset = true)
{
    cv::Mat panel(panelSize.height, panelSize.width, CV_8UC3, cv::Scalar(10, 14, 18));
    const DisplayAlignment alignment = EstimateDisplayAlignment(cloudPoints, trajectory, currentTwc);
    const vector<CloudPoint> displayCloud = TransformCloudPoints(cloudPoints, alignment);
    const vector<Eigen::Vector3f> displayTrajectory = TransformTrajectoryPoints(trajectory, alignment);
    const DisplayPose displayPose = TransformPoseForDisplay(currentTwc, alignment);
    const Eigen::Vector3f* currentPosition = displayPose.valid ? &displayPose.position : nullptr;
    const SceneBounds bounds = ComputeSceneBounds(displayCloud, displayTrajectory, currentPosition);
    viewState.ApplyBounds(bounds);

    DrawLabel(panel, "Map");
    DrawLabel(panel, "Tracking: " + trackingState, cv::Point(10, 56));
    DrawLabel(panel, "Ref: " + alignment.referenceLabel, cv::Point(10, 102));

    if(!bounds.valid)
    {
        cv::putText(panel, "Move the phone slowly to initialize and grow the map.",
                    cv::Point(28, panel.rows / 2), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
        cv::putText(panel, "Drag to orbit, right-drag to pan, wheel to zoom, f to refit.",
                    cv::Point(28, panel.rows / 2 + 34), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(190, 190, 190), 1, cv::LINE_AA);
        return panel;
    }

    const float heightSpan = std::max(0.35f, bounds.maxPoint.y() - bounds.planeY);

    if(viewState.projection == MapProjectionMode::PerspectiveOrbit)
    {
        const ViewCamera camera = BuildOrbitCamera(viewState);
        const float fx = static_cast<float>(panel.cols) * 0.72f;
        const float fy = static_cast<float>(panel.cols) * 0.72f;
        const float cx = static_cast<float>(panel.cols) * 0.5f;
        const float cy = static_cast<float>(panel.rows) * 0.54f;

        DrawReferenceGrid(panel, bounds, camera, fx, fy, cx, cy);
        DrawReferenceAxes(panel, bounds, camera, fx, fy, cx, cy);

        vector<ProjectedPoint> projected;
        projected.reserve(displayCloud.size());
        for(const CloudPoint& point : displayCloud)
        {
            ProjectedPoint item;
            if(!ProjectWorldPointToPanel(point.position, camera, fx, fy, cx, cy, panel.cols, panel.rows, item.pixel, item.depth))
                continue;
            item.color = BlendHeightColor(point.color, (point.position.y() - bounds.planeY) / heightSpan, point.confidence);
            projected.push_back(item);
        }

        sort(projected.begin(), projected.end(),
             [](const ProjectedPoint& a, const ProjectedPoint& b) { return a.depth > b.depth; });

        const int pointRadius = panel.cols >= 1100 ? 2 : 1;
        for(const ProjectedPoint& item : projected)
        {
            cv::circle(panel, item.pixel, pointRadius,
                       cv::Scalar(item.color[0], item.color[1], item.color[2]),
                       cv::FILLED, cv::LINE_AA);
        }

        vector<cv::Point> projectedTrajectory;
        projectedTrajectory.reserve(displayTrajectory.size());
        for(const Eigen::Vector3f& point : displayTrajectory)
        {
            cv::Point pixel;
            float depth = 0.0f;
            if(!ProjectWorldPointToPanel(point, camera, fx, fy, cx, cy, panel.cols, panel.rows, pixel, depth))
                continue;
            projectedTrajectory.push_back(pixel);
        }

        for(size_t i = 1; i < projectedTrajectory.size(); ++i)
            cv::line(panel, projectedTrajectory[i - 1], projectedTrajectory[i], cv::Scalar(0, 220, 255), 2, cv::LINE_AA);
        if(!projectedTrajectory.empty())
            cv::circle(panel, projectedTrajectory.back(), 6, cv::Scalar(0, 90, 255), cv::FILLED, cv::LINE_AA);

        if(displayPose.valid)
            DrawCurrentCameraMarker(panel, displayPose, bounds, camera, fx, fy, cx, cy);

        if(showInset)
            DrawTopDownInset(panel, bounds, displayCloud, displayTrajectory, displayPose.valid ? &displayPose : nullptr);

        DrawLabel(panel, "Perspective orbit  |  height tint", cv::Point(10, 148));
        cv::putText(panel, "Drag orbit  |  Right-drag pan  |  Wheel zoom  |  1/2/3 presets  |  f refit",
                    cv::Point(18, panel.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.54,
                    cv::Scalar(205, 205, 205), 1, cv::LINE_AA);
    }
    else
    {
        const bool topDown = viewState.projection == MapProjectionMode::TopDownOrtho;
        const int horizontalAxis = 0;
        const int verticalAxis = topDown ? 2 : 1;
        const int depthAxis = topDown ? 1 : 2;
        const string projectionLabel = topDown ? "Top-down orthographic" : "Side orthographic";

        DrawOrthoReference(panel, bounds, viewState, horizontalAxis, verticalAxis, depthAxis);

        vector<ProjectedPoint> projected;
        projected.reserve(displayCloud.size());
        for(const CloudPoint& point : displayCloud)
        {
            ProjectedPoint item;
            if(!ProjectOrthoPointToPanel(point.position, viewState, horizontalAxis, verticalAxis, depthAxis,
                                         panel.cols, panel.rows, item.pixel, item.depth))
                continue;
            item.color = BlendHeightColor(point.color, (point.position.y() - bounds.planeY) / heightSpan, point.confidence);
            projected.push_back(item);
        }

        sort(projected.begin(), projected.end(),
             [topDown](const ProjectedPoint& a, const ProjectedPoint& b)
             {
                 return topDown ? (a.depth < b.depth) : (a.depth > b.depth);
             });

        const int pointRadius = panel.cols >= 1100 ? 3 : 2;
        for(const ProjectedPoint& item : projected)
        {
            cv::circle(panel, item.pixel, pointRadius,
                       cv::Scalar(item.color[0], item.color[1], item.color[2]),
                       cv::FILLED, cv::LINE_AA);
        }

        vector<cv::Point> projectedTrajectory;
        projectedTrajectory.reserve(displayTrajectory.size());
        for(const Eigen::Vector3f& point : displayTrajectory)
        {
            cv::Point pixel;
            float depth = 0.0f;
            if(!ProjectOrthoPointToPanel(point, viewState, horizontalAxis, verticalAxis, depthAxis,
                                         panel.cols, panel.rows, pixel, depth))
                continue;
            projectedTrajectory.push_back(pixel);
        }

        for(size_t index = 1; index < projectedTrajectory.size(); ++index)
            cv::line(panel, projectedTrajectory[index - 1], projectedTrajectory[index], cv::Scalar(0, 220, 255), 2, cv::LINE_AA);
        if(!projectedTrajectory.empty())
            cv::circle(panel, projectedTrajectory.back(), 6, cv::Scalar(0, 90, 255), cv::FILLED, cv::LINE_AA);

        if(displayPose.valid)
            DrawOrthoCameraMarker(panel, displayPose, bounds, viewState, horizontalAxis, verticalAxis, depthAxis);

        DrawLabel(panel, projectionLabel + "  |  height tint", cv::Point(10, 148));
        cv::putText(panel, "Drag pan  |  Wheel zoom  |  1 orbit  2 top  3 side  |  f refit",
                    cv::Point(18, panel.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.54,
                    cv::Scalar(205, 205, 205), 1, cv::LINE_AA);
    }
    return panel;
}

cv::Mat NormalizePanelForConcat(const cv::Mat& input, int targetHeight)
{
    cv::Mat panel;
    if(input.channels() == 1)
        cv::cvtColor(input, panel, cv::COLOR_GRAY2BGR);
    else if(input.channels() == 4)
        cv::cvtColor(input, panel, cv::COLOR_BGRA2BGR);
    else
        panel = input;

    if(panel.depth() != CV_8U)
    {
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

cv::Mat BuildWaitingComposite(const string& message)
{
    cv::Mat rgbPanel(kMapPanelHeight, 360, CV_8UC3, cv::Scalar(18, 18, 18));
    cv::Mat depthPanel(kMapPanelHeight, 360, CV_8UC3, cv::Scalar(22, 22, 22));
    cv::Mat mapPanel(kMapPanelHeight, kMapPanelWidth, CV_8UC3, cv::Scalar(10, 14, 18));

    DrawLabel(rgbPanel, "iPhone RGB");
    DrawLabel(depthPanel, "Depth V2 Small");
    DrawLabel(mapPanel, "Map");

    cv::putText(rgbPanel, "Open the iPhone app and start streaming.",
                cv::Point(24, rgbPanel.rows / 2 - 8), cv::FONT_HERSHEY_SIMPLEX, 0.72,
                cv::Scalar(230, 230, 230), 2, cv::LINE_AA);
    cv::putText(depthPanel, "Depth will appear after the first frame arrives.",
                cv::Point(18, depthPanel.rows / 2 - 8), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(215, 215, 215), 2, cv::LINE_AA);
    cv::putText(mapPanel, message,
                cv::Point(28, mapPanel.rows / 2 - 12), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(235, 235, 235), 2, cv::LINE_AA);
    cv::putText(mapPanel, "When the phone stops streaming, this Mac app will stay open and wait again.",
                cv::Point(28, mapPanel.rows / 2 + 28), cv::FONT_HERSHEY_SIMPLEX, 0.58,
                cv::Scalar(190, 190, 190), 1, cv::LINE_AA);

    cv::Mat composite;
    cv::hconcat(vector<cv::Mat>{rgbPanel, depthPanel, mapPanel}, composite);
    cv::putText(composite, "q/ESC quit",
                cv::Point(18, composite.rows - 18), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                cv::Scalar(235, 235, 235), 2, cv::LINE_AA);
    return composite;
}

cv::Mat FitPanelIntoCanvas(const cv::Mat& panel, const cv::Size& canvasSize, const cv::Scalar& background)
{
    cv::Mat canvas(canvasSize, CV_8UC3, background);
    cv::Mat normalized = NormalizePanelForConcat(panel, canvasSize.height - 20);
    if(normalized.cols > canvasSize.width - 20)
    {
        const double scale = static_cast<double>(canvasSize.width - 20) / static_cast<double>(normalized.cols);
        cv::resize(normalized, normalized,
                   cv::Size(std::max(1, static_cast<int>(std::round(normalized.cols * scale))), std::max(1, static_cast<int>(std::round(normalized.rows * scale)))),
                   0.0, 0.0, cv::INTER_AREA);
    }

    const int x = std::max(10, (canvas.cols - normalized.cols) / 2);
    const int y = std::max(10, (canvas.rows - normalized.rows) / 2);
    normalized.copyTo(canvas(cv::Rect(x, y, normalized.cols, normalized.rows)));
    return canvas;
}

cv::Mat BuildPausedComposite(const cv::Mat& rgbPanel,
                             const cv::Mat& depthPanel,
                             const vector<CloudPoint>& cloudPoints,
                             const vector<Eigen::Vector3f>& trajectory,
                             const Sophus::SE3f* currentTwc,
                             const string& trackingState,
                             MapViewState& viewState)
{
    const int leftColumnWidth = 320;
    const int padding = 18;
    const int sideColumnWidth = 390;
    const cv::Size mapSize(980, 860);
    const cv::Size miniMapSize(sideColumnWidth, (mapSize.height - padding) / 2);
    const cv::Size thumbSize(leftColumnWidth, 250);

    cv::Mat rgbThumb = FitPanelIntoCanvas(rgbPanel.empty() ? cv::Mat(thumbSize, CV_8UC3, cv::Scalar(18, 18, 18)) : rgbPanel, thumbSize, cv::Scalar(18, 18, 18));
    cv::Mat depthThumb = FitPanelIntoCanvas(depthPanel.empty() ? cv::Mat(thumbSize, CV_8UC3, cv::Scalar(22, 22, 22)) : depthPanel, thumbSize, cv::Scalar(22, 22, 22));
    cv::Mat mapPanel = RenderMapPanel(cloudPoints, trajectory, currentTwc, trackingState, viewState, mapSize, true);

    MapViewState topDownState = viewState;
    MapViewState sideState = viewState;
    MapViewState orbitState = viewState;
    ApplyViewPreset(topDownState, 2);
    ApplyViewPreset(sideState, 3);
    ApplyViewPreset(orbitState, 1);

    cv::Mat topDownPanel = RenderMapPanel(cloudPoints, trajectory, currentTwc, trackingState, topDownState, miniMapSize, false);
    cv::Mat sidePanel = RenderMapPanel(cloudPoints, trajectory, currentTwc, trackingState, sideState, miniMapSize, false);
    cv::Mat orbitPanel = RenderMapPanel(cloudPoints, trajectory, currentTwc, trackingState, orbitState, miniMapSize, false);

    cv::Mat upperMiniPanel;
    cv::Mat lowerMiniPanel;
    switch(viewState.projection)
    {
        case MapProjectionMode::PerspectiveOrbit:
            upperMiniPanel = topDownPanel;
            lowerMiniPanel = sidePanel;
            break;
        case MapProjectionMode::SideOrtho:
            upperMiniPanel = topDownPanel;
            lowerMiniPanel = orbitPanel;
            break;
        case MapProjectionMode::TopDownOrtho:
        default:
            upperMiniPanel = sidePanel;
            lowerMiniPanel = orbitPanel;
            break;
    }

    cv::Mat leftColumn(mapSize.height, leftColumnWidth, CV_8UC3, cv::Scalar(16, 16, 16));
    rgbThumb.copyTo(leftColumn(cv::Rect(0, 0, rgbThumb.cols, rgbThumb.rows)));
    depthThumb.copyTo(leftColumn(cv::Rect(0, rgbThumb.rows + padding, depthThumb.cols, depthThumb.rows)));

    cv::putText(leftColumn, "Paused", cv::Point(18, depthThumb.rows + rgbThumb.rows + padding + 48),
                cv::FONT_HERSHEY_SIMPLEX, 0.95, cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
    cv::putText(leftColumn, "Phone stream stopped. Inspect the map now.", cv::Point(18, depthThumb.rows + rgbThumb.rows + padding + 86),
                cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(leftColumn, "Start Stream on iPhone to auto-resume.", cv::Point(18, depthThumb.rows + rgbThumb.rows + padding + 122),
                cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(190, 190, 190), 1, cv::LINE_AA);
    cv::putText(leftColumn, "1 orbit  2 top-down  3 side", cv::Point(18, depthThumb.rows + rgbThumb.rows + padding + 154),
                cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(190, 190, 190), 1, cv::LINE_AA);
    cv::putText(leftColumn, "r: go back to plain waiting screen", cv::Point(18, depthThumb.rows + rgbThumb.rows + padding + 186),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(175, 175, 175), 1, cv::LINE_AA);

    cv::Mat sideColumn(mapSize.height, sideColumnWidth, CV_8UC3, cv::Scalar(12, 12, 12));
    upperMiniPanel.copyTo(sideColumn(cv::Rect(0, 0, upperMiniPanel.cols, upperMiniPanel.rows)));
    lowerMiniPanel.copyTo(sideColumn(cv::Rect(0, upperMiniPanel.rows + padding, lowerMiniPanel.cols, lowerMiniPanel.rows)));

    cv::Mat composite(mapSize.height, leftColumn.cols + padding + mapPanel.cols + padding + sideColumn.cols, CV_8UC3, cv::Scalar(8, 8, 8));
    leftColumn.copyTo(composite(cv::Rect(0, 0, leftColumn.cols, leftColumn.rows)));
    mapPanel.copyTo(composite(cv::Rect(leftColumn.cols + padding, 0, mapPanel.cols, mapPanel.rows)));
    sideColumn.copyTo(composite(cv::Rect(leftColumn.cols + padding + mapPanel.cols + padding, 0, sideColumn.cols, sideColumn.rows)));
    viewState.panelRect = cv::Rect(leftColumn.cols + padding, 0, mapPanel.cols, mapPanel.rows);

    cv::putText(composite, "Paused inspection  |  q quit  |  r reconnect  |  s save screenshot  |  p export point cloud",
                cv::Point(18, composite.rows - 18), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                cv::Scalar(235, 235, 235), 2, cv::LINE_AA);
    return composite;
}

string BuildSnapshotPath()
{
    const auto timestamp = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    return "iphone_triptych_snapshot_" + to_string(timestamp) + ".png";
}

string BuildPointCloudPath()
{
    const auto timestamp = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    return "iphone_triptych_cloud_" + to_string(timestamp) + ".ply";
}

bool SavePointCloudAsPly(const string& filename, const vector<CloudPoint>& cloudPoints)
{
    ofstream file(filename);
    if(!file.is_open())
        return false;

    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << cloudPoints.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";

    for(const CloudPoint& point : cloudPoints)
    {
        file << point.position.x() << " "
             << point.position.y() << " "
             << point.position.z() << " "
             << static_cast<int>(point.color[2]) << " "
             << static_cast<int>(point.color[1]) << " "
             << static_cast<int>(point.color[0]) << "\n";
    }
    return true;
}

string WriteAutoSettings(const PacketHeader& header, const string& outputPrefix, int port)
{
    const float scaleX = (header.depthWidth > 0) ? static_cast<float>(header.rgbWidth) / static_cast<float>(header.depthWidth) : 1.0f;
    const float scaleY = (header.depthHeight > 0) ? static_cast<float>(header.rgbHeight) / static_cast<float>(header.depthHeight) : 1.0f;
    const float fx = header.fx * scaleX;
    const float fy = header.fy * scaleY;
    const float cx = header.cx * scaleX;
    const float cy = header.cy * scaleY;

    ostringstream pathBuilder;
    pathBuilder << "/tmp/orbslam3_iphone_mono_coreml_" << port << ".yaml";
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
    out << "Viewer.ViewpointZ: -2.2\n";
    out << "Viewer.ViewpointF: 500.0\n";
    out << "Viewer.imageViewScale: 1.0\n\n";
    out.close();

    return settingsPath;
}
}

int main(int argc, char** argv)
{
    const string defaultVocabulary = "Vocabulary/ORBvoc.txt";
    const string defaultModel = "models/DepthAnythingV2SmallF16.mlpackage";

    if(argc > 5)
    {
        cerr << endl
             << "Usage: ./map_iphone_coreml [vocabulary] [model_path] [port] [output_prefix]" << endl;
        return 1;
    }

    const string vocabularyPath = argc >= 2 ? string(argv[1]) : defaultVocabulary;
    const string modelPath = argc >= 3 ? string(argv[2]) : defaultModel;
    const int port = argc >= 4 ? stoi(argv[3]) : 9000;
    const string outputPrefix = argc >= 5 ? string(argv[4]) : string("iphone_mono_coreml_orbslam3");

    struct sigaction sigIntHandler {};
    sigIntHandler.sa_handler = ExitLoopHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    CoreMLDepthEstimator depthEstimator(modelPath);
    if(!depthEstimator.IsReady())
    {
        cerr << depthEstimator.Error() << endl;
        return 1;
    }

    int serverFd = -1;
    try
    {
        serverFd = CreateListenSocket(port);
    }
    catch(const exception& error)
    {
        cerr << "Failed to start stream receiver: " << error.what() << endl;
        return 1;
    }

    PrintCandidateAddresses(port);
    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    MapViewState mapViewState;
    cv::setMouseCallback(kWindowName, HandleMapMouse, &mapViewState);

    while(gKeepRunning)
    {
        cv::imshow(kWindowName, BuildWaitingComposite("Waiting for iPhone RGB stream..."));
        const int waitingKey = cv::waitKey(16);
        if(waitingKey == 27 || waitingKey == 'q' || waitingKey == 'Q')
            break;

        if(!WaitForSocketReadable(serverFd, 120))
            continue;

        int clientFd = -1;
        sockaddr_in clientAddress {};
        socklen_t clientLength = sizeof(clientAddress);
        clientFd = accept(serverFd, reinterpret_cast<sockaddr*>(&clientAddress), &clientLength);
        if(clientFd < 0)
        {
            if(errno == EINTR && !gKeepRunning)
                break;
            cerr << "Failed to accept iPhone connection." << endl;
            continue;
        }

        char clientIp[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &clientAddress.sin_addr, clientIp, sizeof(clientIp));
        cout << "Accepted connection from " << clientIp << ":" << ntohs(clientAddress.sin_port) << endl;
        SetSocketTimeout(clientFd, 1600);

        StreamFrame firstFrame;
        try
        {
            if(!ReceiveFrame(clientFd, firstFrame))
            {
                cerr << "Stream closed before the first frame arrived." << endl;
                close(clientFd);
                continue;
            }
        }
        catch(const exception& error)
        {
            cerr << "Failed to receive first frame: " << error.what() << endl;
            close(clientFd);
            continue;
        }

        const string settingsPath = WriteAutoSettings(firstFrame.header, outputPrefix, port);
        cout << "Auto-generated settings: " << settingsPath << endl;
        cout << "RGB frame size: " << firstFrame.header.rgbWidth << "x" << firstFrame.header.rgbHeight << endl;

        {
            ORB_SLAM3::System slam(vocabularyPath, settingsPath, ORB_SLAM3::System::MONOCULAR, false);
            const float imageScale = slam.GetImageScale();

            const float rgbScaleX = (firstFrame.header.depthWidth > 0) ? static_cast<float>(firstFrame.header.rgbWidth) / static_cast<float>(firstFrame.header.depthWidth) : 1.0f;
            const float rgbScaleY = (firstFrame.header.depthHeight > 0) ? static_cast<float>(firstFrame.header.rgbHeight) / static_cast<float>(firstFrame.header.depthHeight) : 1.0f;
            const float rawFx = firstFrame.header.fx * rgbScaleX;
            const float rawFy = firstFrame.header.fy * rgbScaleY;
            const float rawCx = firstFrame.header.cx * rgbScaleX;
            const float rawCy = firstFrame.header.cy * rgbScaleY;

            const float scaledFx = rawFx * imageScale;
            const float scaledFy = rawFy * imageScale;
            const float scaledCx = rawCx * imageScale;
            const float scaledCy = rawCy * imageScale;

            VoxelCloud cloud(0.035f);
            vector<Eigen::Vector3f> trajectory;
            trajectory.reserve(4096);

            bool hasScale = false;
            float smoothedScale = 1.0f;
            bool hasLastIntegratedPose = false;
            Sophus::SE3f lastIntegratedTwc;
            int frameIndex = 0;
            int previousTrackingState = ORB_SLAM3::Tracking::NO_IMAGES_YET;
            mapViewState = MapViewState{};
            cv::Mat pausedRgbPanel;
            cv::Mat pausedDepthPanel;
            Sophus::SE3f pausedTwc;
            bool hasPausedPose = false;
            bool hasPausedPanels = false;
            bool pauseAfterDisconnect = false;
            string pausedTrackingState = "UNKNOWN";

            StreamFrame currentFrame = std::move(firstFrame);
            while(gKeepRunning)
            {
                cv::Mat slamFrame = currentFrame.rgb;
                if(imageScale != 1.f)
                {
                    const int scaledWidth = static_cast<int>(currentFrame.rgb.cols * imageScale);
                    const int scaledHeight = static_cast<int>(currentFrame.rgb.rows * imageScale);
                    cv::resize(currentFrame.rgb, slamFrame, cv::Size(scaledWidth, scaledHeight));
                }

                double inferenceMs = 0.0;
                cv::Mat depth32f;
                if(!depthEstimator.Infer(slamFrame, depth32f, inferenceMs))
                {
                    cerr << depthEstimator.Error() << endl;
                    break;
                }
                cv::resize(depth32f, depth32f, slamFrame.size(), 0.0, 0.0, cv::INTER_LINEAR);

                const Sophus::SE3f Tcw = slam.TrackMonocular(slamFrame, currentFrame.header.timestamp);
                const int trackingState = slam.GetTrackingState();
                const string trackingStateText = TrackingStateToString(trackingState);

                if(trackingState == ORB_SLAM3::Tracking::NOT_INITIALIZED &&
                   (previousTrackingState == ORB_SLAM3::Tracking::OK ||
                    previousTrackingState == ORB_SLAM3::Tracking::OK_KLT ||
                    previousTrackingState == ORB_SLAM3::Tracking::RECENTLY_LOST ||
                    previousTrackingState == ORB_SLAM3::Tracking::LOST))
                {
                    cloud = VoxelCloud(0.035f);
                    trajectory.clear();
                    hasScale = false;
                    smoothedScale = 1.0f;
                    hasLastIntegratedPose = false;
                }

                int anchorCount = 0;
                if(trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT)
                {
                    const vector<ORB_SLAM3::MapPoint*> trackedMapPoints = slam.GetTrackedMapPoints();
                    const vector<cv::KeyPoint> trackedKeypoints = slam.GetTrackedKeyPointsUn();
                    const float scaleCandidate = ComputeDepthScale(Tcw, depth32f, trackedMapPoints, trackedKeypoints, anchorCount);
                    if(scaleCandidate > 0.0f)
                    {
                        if(!hasScale)
                            smoothedScale = scaleCandidate;
                        else
                            smoothedScale = 0.9f * smoothedScale + 0.1f * scaleCandidate;
                        hasScale = true;
                    }
                }

                cv::Mat scaledDepth = depth32f;
                if(hasScale)
                    scaledDepth = depth32f * smoothedScale;

                bool hasCurrentPose = false;
                Sophus::SE3f currentTwc;
                if(trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT)
                {
                    currentTwc = Tcw.inverse();
                    hasCurrentPose = true;
                }

                if(hasCurrentPose && hasScale)
                {
                    trajectory.push_back(currentTwc.translation());
                    if(ShouldIntegrateFrame(currentTwc, lastIntegratedTwc, hasLastIntegratedPose, frameIndex))
                    {
                        IntegrateFrame(slamFrame, scaledDepth, currentTwc, scaledFx, scaledFy, scaledCx, scaledCy, cloud);
                        lastIntegratedTwc = currentTwc;
                        hasLastIntegratedPose = true;
                    }
                }

                cv::Mat rgbPanel = RotateForPreview(slamFrame, currentFrame.header.displayOrientation);
                rgbPanel = rgbPanel.clone();
                DrawLabel(rgbPanel, "iPhone RGB");
                DrawLabel(rgbPanel, "Tracking: " + trackingStateText, cv::Point(10, 56));
                if(hasScale)
                {
                    ostringstream scaleLabel;
                    scaleLabel << fixed << setprecision(2) << "Depth scale " << smoothedScale;
                    DrawLabel(rgbPanel, scaleLabel.str(), cv::Point(10, 102));
                }

                cv::Mat depthPanel = RotateForPreview(ColorizeDepth(scaledDepth), currentFrame.header.displayOrientation);
                DrawLabel(depthPanel, "Depth V2 Small");
                {
                    ostringstream depthStats;
                    depthStats << fixed << setprecision(1) << inferenceMs << " ms";
                    if(anchorCount > 0)
                        depthStats << "  anchors " << anchorCount;
                    DrawLabel(depthPanel, depthStats.str(), cv::Point(10, 56));
                }

                const vector<CloudPoint> cloudPoints = cloud.Points();
                cv::Mat mapPanel = RenderMapPanel(cloudPoints, trajectory, hasCurrentPose ? &currentTwc : nullptr, trackingStateText, mapViewState);
                {
                    ostringstream mapStats;
                    mapStats << "Cloud " << cloudPoints.size() << " pts";
                    DrawLabel(mapPanel, mapStats.str(), cv::Point(10, 148));
                }

                pausedRgbPanel = rgbPanel.clone();
                pausedDepthPanel = depthPanel.clone();
                pausedTrackingState = trackingStateText;
                hasPausedPanels = true;
                if(hasCurrentPose)
                {
                    pausedTwc = currentTwc;
                    hasPausedPose = true;
                }

                const int targetPanelHeight = mapPanel.rows;
                rgbPanel = NormalizePanelForConcat(rgbPanel, targetPanelHeight);
                depthPanel = NormalizePanelForConcat(depthPanel, targetPanelHeight);
                mapPanel = NormalizePanelForConcat(mapPanel, targetPanelHeight);

                cv::Mat composite;
                cv::hconcat(vector<cv::Mat>{rgbPanel, depthPanel, mapPanel}, composite);
                mapViewState.panelRect = cv::Rect(rgbPanel.cols + depthPanel.cols, 0, mapPanel.cols, mapPanel.rows);
                cv::putText(composite, "q/ESC quit  |  s save screenshot  |  p export point cloud  |  f refit map",
                            cv::Point(18, composite.rows - 18), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                            cv::Scalar(235, 235, 235), 2, cv::LINE_AA);
                cv::imshow(kWindowName, composite);

                const int key = cv::waitKey(1);
                if(key == 27 || key == 'q' || key == 'Q')
                {
                    gKeepRunning = false;
                    break;
                }
                if(key == 'f' || key == 'F')
                    mapViewState.ResetView();
                if(key == '1' || key == '2' || key == '3')
                    ApplyViewPreset(mapViewState, key - '0');
                if(key == 's' || key == 'S')
                {
                    const string snapshotPath = BuildSnapshotPath();
                    cv::imwrite(snapshotPath, composite);
                    cout << "Saved triptych snapshot to " << snapshotPath << endl;
                }
                if(key == 'p' || key == 'P')
                {
                    const string cloudPath = BuildPointCloudPath();
                    if(SavePointCloudAsPly(cloudPath, cloudPoints))
                        cout << "Saved point cloud to " << cloudPath << endl;
                    else
                        cerr << "Failed to save point cloud to " << cloudPath << endl;
                }

                previousTrackingState = trackingState;
                ++frameIndex;

                StreamFrame nextFrame;
                try
                {
                    if(!ReceiveFrame(clientFd, nextFrame))
                    {
                        cerr << "iPhone stream disconnected." << endl;
                        pauseAfterDisconnect = true;
                        break;
                    }
                }
                catch(const exception& error)
                {
                    cerr << "Stream error: " << error.what() << endl;
                    pauseAfterDisconnect = true;
                    break;
                }
                currentFrame = std::move(nextFrame);
            }

            slam.Shutdown();
            slam.SaveKeyFrameTrajectoryTUM(outputPrefix + "_KeyFrameTrajectory.txt");

            if(gKeepRunning && pauseAfterDisconnect && hasPausedPanels)
            {
                ApplyViewPreset(mapViewState, 2);
                bool resumeWhenReconnected = false;
                while(gKeepRunning)
                {
                    cv::Mat pausedComposite = BuildPausedComposite(pausedRgbPanel, pausedDepthPanel, cloud.Points(), trajectory,
                                                                   hasPausedPose ? &pausedTwc : nullptr,
                                                                   pausedTrackingState, mapViewState);
                    cv::imshow(kWindowName, pausedComposite);

                    const int key = cv::waitKey(16);
                    if(key == 27 || key == 'q' || key == 'Q')
                    {
                        gKeepRunning = false;
                        break;
                    }
                    if(key == 'r' || key == 'R')
                        break;
                    if(key == 'f' || key == 'F')
                        mapViewState.ResetView();
                    if(key == '1' || key == '2' || key == '3')
                        ApplyViewPreset(mapViewState, key - '0');
                    if(key == 's' || key == 'S')
                    {
                        const string snapshotPath = BuildSnapshotPath();
                        cv::imwrite(snapshotPath, pausedComposite);
                        cout << "Saved paused snapshot to " << snapshotPath << endl;
                    }
                    if(key == 'p' || key == 'P')
                    {
                        const string cloudPath = BuildPointCloudPath();
                        const vector<CloudPoint> cloudPoints = cloud.Points();
                        if(SavePointCloudAsPly(cloudPath, cloudPoints))
                            cout << "Saved point cloud to " << cloudPath << endl;
                        else
                            cerr << "Failed to save point cloud to " << cloudPath << endl;
                    }

                    try
                    {
                        if(WaitForSocketReadable(serverFd, 1))
                        {
                            resumeWhenReconnected = true;
                            cout << "iPhone stream restarted. Resuming live mapping..." << endl;
                            break;
                        }
                    }
                    catch(const exception& error)
                    {
                        cerr << "Reconnect wait failed: " << error.what() << endl;
                        break;
                    }
                }

                if(resumeWhenReconnected)
                    cout << "Returning to live session setup..." << endl;
            }
        }

        if(clientFd >= 0)
            close(clientFd);

        if(gKeepRunning)
            cout << "Session ended. Waiting for the next iPhone stream..." << endl;
    }

    if(serverFd >= 0)
        close(serverFd);

    cv::destroyAllWindows();
    return 0;
}
