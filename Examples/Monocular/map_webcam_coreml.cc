#include "coreml_depth_estimator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <MapPoint.h>
#include <System.h>

using namespace std;

namespace
{
constexpr int kDefaultWidth = 640;
constexpr int kDefaultHeight = 480;
constexpr int kDefaultFps = 30;
constexpr int kMapPanelWidth = 640;
constexpr int kPointStride = 12;
constexpr float kMinDepth = 0.15f;
constexpr float kMaxDepth = 4.5f;
constexpr float kMapViewYaw = 0.85f;
constexpr float kMapViewPitch = -0.52f;

int ReadIntSetting(const cv::FileStorage& settings, const string& key, int fallback)
{
    const cv::FileNode node = settings[key];
    if(node.empty())
        return fallback;
    return static_cast<int>(node.real());
}

float ReadFloatSetting(const cv::FileStorage& settings, const string& key, float fallback)
{
    const cv::FileNode node = settings[key];
    if(node.empty())
        return fallback;
    return static_cast<float>(node.real());
}

bool OpenCamera(cv::VideoCapture& capture, int cameraIndex, int width, int height, int fps)
{
#ifdef __APPLE__
    if(!capture.open(cameraIndex, cv::CAP_AVFOUNDATION) && !capture.open(cameraIndex))
        return false;
#else
    if(!capture.open(cameraIndex))
        return false;
#endif

    if(width > 0)
        capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
    if(height > 0)
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    if(fps > 0)
        capture.set(cv::CAP_PROP_FPS, fps);
    return capture.isOpened();
}

bool IsFinitePositive(float value)
{
    return std::isfinite(value) && value > 0.0f;
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
};

class VoxelCloud
{
public:
    explicit VoxelCloud(float voxelSize)
    : mVoxelSize(voxelSize)
    {
    }

    void Insert(const Eigen::Vector3f& position, const cv::Vec3b& color)
    {
        mVoxels[Quantize(position)] = CloudPoint{position, color};
    }

    vector<CloudPoint> Points() const
    {
        vector<CloudPoint> points;
        points.reserve(mVoxels.size());
        for(const auto& entry : mVoxels)
            points.push_back(entry.second);
        return points;
    }

private:
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
    unordered_map<VoxelKey, CloudPoint, VoxelKeyHash> mVoxels;
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
            cloud.Insert(pointWorld, frameBgr.at<cv::Vec3b>(v, u));
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

void BuildVirtualCamera(const Eigen::Vector3f& center,
                        float radius,
                        float yaw,
                        float pitch,
                        Eigen::Matrix3f& Rcw,
                        Eigen::Vector3f& tcw)
{
    Eigen::Vector3f offset;
    offset.x() = std::cos(pitch) * std::cos(yaw);
    offset.y() = std::sin(pitch);
    offset.z() = std::cos(pitch) * std::sin(yaw);

    const Eigen::Vector3f eye = center + radius * offset;
    Eigen::Vector3f forward = (center - eye).normalized();
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    if(std::fabs(forward.dot(up)) > 0.95f)
        up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

    const Eigen::Vector3f right = forward.cross(up).normalized();
    const Eigen::Vector3f cameraUp = right.cross(forward).normalized();

    Rcw.row(0) = right.transpose();
    Rcw.row(1) = cameraUp.transpose();
    Rcw.row(2) = forward.transpose();
    tcw = -Rcw * eye;
}

cv::Mat RenderMapPanel(const vector<CloudPoint>& cloudPoints,
                       const vector<Eigen::Vector3f>& trajectory,
                       double /*elapsedSeconds*/,
                       const string& trackingState)
{
    cv::Mat panel(kDefaultHeight, kMapPanelWidth, CV_8UC3, cv::Scalar(10, 14, 18));

    if(cloudPoints.empty())
    {
        DrawLabel(panel, "Map");
        DrawLabel(panel, "Tracking: " + trackingState, cv::Point(10, 56));
        cv::putText(panel, "Move the camera slowly to initialize and grow the map.",
                    cv::Point(24, panel.rows / 2), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
        return panel;
    }

    Eigen::Vector3f minPoint = cloudPoints.front().position;
    Eigen::Vector3f maxPoint = cloudPoints.front().position;
    for(const CloudPoint& point : cloudPoints)
    {
        minPoint = minPoint.cwiseMin(point.position);
        maxPoint = maxPoint.cwiseMax(point.position);
    }
    for(const Eigen::Vector3f& point : trajectory)
    {
        minPoint = minPoint.cwiseMin(point);
        maxPoint = maxPoint.cwiseMax(point);
    }

    const Eigen::Vector3f center = 0.5f * (minPoint + maxPoint);
    const float radius = std::max(0.6f, (maxPoint - minPoint).norm() * 0.8f + 0.6f);
    const float yaw = kMapViewYaw;
    const float pitch = kMapViewPitch;

    Eigen::Matrix3f Rcw;
    Eigen::Vector3f tcw;
    BuildVirtualCamera(center, radius, yaw, pitch, Rcw, tcw);

    const float fx = static_cast<float>(panel.cols) * 0.65f;
    const float fy = static_cast<float>(panel.cols) * 0.65f;
    const float cx = static_cast<float>(panel.cols) * 0.5f;
    const float cy = static_cast<float>(panel.rows) * 0.52f;

    vector<ProjectedPoint> projected;
    projected.reserve(cloudPoints.size());
    for(const CloudPoint& point : cloudPoints)
    {
        ProjectedPoint item;
        if(!ProjectPoint(point.position, Rcw, tcw, fx, fy, cx, cy, item.pixel, item.depth))
            continue;
        if(item.pixel.x < 0 || item.pixel.x >= panel.cols || item.pixel.y < 0 || item.pixel.y >= panel.rows)
            continue;
        item.color = point.color;
        projected.push_back(item);
    }

    sort(projected.begin(), projected.end(),
         [](const ProjectedPoint& a, const ProjectedPoint& b) { return a.depth > b.depth; });

    for(const ProjectedPoint& item : projected)
    {
        cv::circle(panel, item.pixel, 1,
                   cv::Scalar(item.color[0], item.color[1], item.color[2]),
                   cv::FILLED, cv::LINE_AA);
    }

    vector<cv::Point> projectedTrajectory;
    projectedTrajectory.reserve(trajectory.size());
    for(const Eigen::Vector3f& point : trajectory)
    {
        cv::Point pixel;
        float depth = 0.0f;
        if(!ProjectPoint(point, Rcw, tcw, fx, fy, cx, cy, pixel, depth))
            continue;
        if(pixel.x < 0 || pixel.x >= panel.cols || pixel.y < 0 || pixel.y >= panel.rows)
            continue;
        projectedTrajectory.push_back(pixel);
    }

    for(size_t i = 1; i < projectedTrajectory.size(); ++i)
        cv::line(panel, projectedTrajectory[i - 1], projectedTrajectory[i], cv::Scalar(0, 220, 255), 2, cv::LINE_AA);
    if(!projectedTrajectory.empty())
        cv::circle(panel, projectedTrajectory.back(), 5, cv::Scalar(0, 80, 255), cv::FILLED, cv::LINE_AA);

    DrawLabel(panel, "Map");
    DrawLabel(panel, "Tracking: " + trackingState, cv::Point(10, 56));
    return panel;
}

string BuildSnapshotPath()
{
    const auto timestamp = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    return "triptych_snapshot_" + to_string(timestamp) + ".png";
}

string BuildPointCloudPath()
{
    const auto timestamp = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    return "triptych_cloud_" + to_string(timestamp) + ".ply";
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
}

int main(int argc, char** argv)
{
    const string defaultVocabulary = "Vocabulary/ORBvoc.txt";
    const string defaultSettings = "Examples/Monocular/MacBookAir_640x480.yaml";
    const string defaultModel = "models/DepthAnythingV2SmallF16.mlpackage";
    const string defaultTrajectory = "KeyFrameTrajectory.txt";

    if(argc > 6)
    {
        cerr << endl
             << "Usage: ./map_webcam_coreml [vocabulary] [settings] [model_path] [camera_index] [trajectory_file]" << endl;
        return 1;
    }

    const string vocabularyPath = argc >= 2 ? string(argv[1]) : defaultVocabulary;
    const string settingsPath = argc >= 3 ? string(argv[2]) : defaultSettings;
    const string modelPath = argc >= 4 ? string(argv[3]) : defaultModel;
    const int cameraIndex = argc >= 5 ? stoi(argv[4]) : 0;
    const string trajectoryFile = argc >= 6 ? string(argv[5]) : defaultTrajectory;

    cv::FileStorage settings(settingsPath, cv::FileStorage::READ);
    if(!settings.isOpened())
    {
        cerr << "Failed to open settings file at: " << settingsPath << endl;
        return 1;
    }

    const int width = ReadIntSetting(settings, "Camera.newWidth", ReadIntSetting(settings, "Camera.width", kDefaultWidth));
    const int height = ReadIntSetting(settings, "Camera.newHeight", ReadIntSetting(settings, "Camera.height", kDefaultHeight));
    const int fps = ReadIntSetting(settings, "Camera.fps", kDefaultFps);
    const float fx = ReadFloatSetting(settings, "Camera1.fx", 530.0f);
    const float fy = ReadFloatSetting(settings, "Camera1.fy", 530.0f);
    const float cx = ReadFloatSetting(settings, "Camera1.cx", width * 0.5f);
    const float cy = ReadFloatSetting(settings, "Camera1.cy", height * 0.5f);

    CoreMLDepthEstimator depthEstimator(modelPath);
    if(!depthEstimator.IsReady())
    {
        cerr << depthEstimator.Error() << endl;
        return 1;
    }

    cv::VideoCapture capture;
    if(!OpenCamera(capture, cameraIndex, width, height, fps))
    {
        cerr << "Failed to open camera " << cameraIndex << "." << endl;
        cerr << "On macOS, please grant camera access to your terminal app." << endl;
        return 1;
    }

    cout << "Starting RGB + depth + map demo" << endl;
    cout << "Vocabulary: " << vocabularyPath << endl;
    cout << "Settings:   " << settingsPath << endl;
    cout << "Model:      " << modelPath << endl;

    ORB_SLAM3::System slam(vocabularyPath, settingsPath, ORB_SLAM3::System::MONOCULAR, false);
    const float imageScale = slam.GetImageScale();
    const float scaledFx = fx * imageScale;
    const float scaledFy = fy * imageScale;
    const float scaledCx = cx * imageScale;
    const float scaledCy = cy * imageScale;

    VoxelCloud cloud(0.035f);
    vector<Eigen::Vector3f> trajectory;
    trajectory.reserve(4096);

    bool hasScale = false;
    float smoothedScale = 1.0f;
    bool hasLastIntegratedPose = false;
    Sophus::SE3f lastIntegratedTwc;
    int frameIndex = 0;
    int previousTrackingState = ORB_SLAM3::Tracking::NO_IMAGES_YET;
    const auto startTime = chrono::steady_clock::now();

    cv::namedWindow("ORB-SLAM3 Triptych", cv::WINDOW_AUTOSIZE);

    while(true)
    {
        cv::Mat frame;
        if(!capture.read(frame) || frame.empty())
        {
            cerr << "Failed to read a frame from camera " << cameraIndex << "." << endl;
            break;
        }

        cv::Mat slamFrame = frame;
        if(imageScale != 1.f)
        {
            const int scaledWidth = static_cast<int>(frame.cols * imageScale);
            const int scaledHeight = static_cast<int>(frame.rows * imageScale);
            cv::resize(frame, slamFrame, cv::Size(scaledWidth, scaledHeight));
        }

        const double timestamp = chrono::duration_cast<chrono::duration<double> >(
            chrono::steady_clock::now() - startTime).count();

        double inferenceMs = 0.0;
        cv::Mat depth32f;
        if(!depthEstimator.Infer(slamFrame, depth32f, inferenceMs))
        {
            cerr << depthEstimator.Error() << endl;
            break;
        }
        cv::resize(depth32f, depth32f, slamFrame.size(), 0.0, 0.0, cv::INTER_LINEAR);

        const Sophus::SE3f Tcw = slam.TrackMonocular(slamFrame, timestamp);
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

        if((trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT) && hasScale)
        {
            const Sophus::SE3f Twc = Tcw.inverse();
            trajectory.push_back(Twc.translation());
            if(ShouldIntegrateFrame(Twc, lastIntegratedTwc, hasLastIntegratedPose, frameIndex))
            {
                IntegrateFrame(slamFrame, scaledDepth, Twc, scaledFx, scaledFy, scaledCx, scaledCy, cloud);
                lastIntegratedTwc = Twc;
                hasLastIntegratedPose = true;
            }
        }

        cv::Mat rgbPanel = slamFrame.clone();
        DrawLabel(rgbPanel, "RGB");
        DrawLabel(rgbPanel, "Tracking: " + trackingStateText, cv::Point(10, 56));
        if(hasScale)
        {
            ostringstream scaleLabel;
            scaleLabel << fixed << setprecision(2) << "Depth scale " << smoothedScale;
            DrawLabel(rgbPanel, scaleLabel.str(), cv::Point(10, 102));
        }

        cv::Mat depthPanel = ColorizeDepth(scaledDepth);
        DrawLabel(depthPanel, "Depth");
        {
            ostringstream depthStats;
            depthStats << fixed << setprecision(1) << inferenceMs << " ms";
            if(anchorCount > 0)
                depthStats << "  anchors " << anchorCount;
            DrawLabel(depthPanel, depthStats.str(), cv::Point(10, 56));
        }

        const vector<CloudPoint> cloudPoints = cloud.Points();
        cv::Mat mapPanel = RenderMapPanel(
            cloudPoints,
            trajectory,
            chrono::duration_cast<chrono::duration<double> >(chrono::steady_clock::now() - startTime).count(),
            trackingStateText);
        {
            ostringstream mapStats;
            mapStats << "Cloud " << cloudPoints.size() << " pts";
            DrawLabel(mapPanel, mapStats.str(), cv::Point(10, 102));
        }

        cv::Mat composite;
        cv::hconcat(vector<cv::Mat>{rgbPanel, depthPanel, mapPanel}, composite);
        cv::putText(composite, "q/ESC quit  |  s save screenshot  |  p export point cloud",
                    cv::Point(18, composite.rows - 18), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                    cv::Scalar(235, 235, 235), 2, cv::LINE_AA);
        cv::imshow("ORB-SLAM3 Triptych", composite);

        const int key = cv::waitKey(1);
        if(key == 27 || key == 'q' || key == 'Q')
            break;
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
    }

    capture.release();
    cv::destroyAllWindows();
    slam.Shutdown();
    slam.SaveKeyFrameTrajectoryTUM(trajectoryFile);
    return 0;
}
