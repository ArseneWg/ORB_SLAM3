#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>

#import <CoreML/CoreML.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

using namespace std;

namespace
{
constexpr int kModelWidth = 518;
constexpr int kModelHeight = 392;
constexpr int kDefaultWidth = 640;
constexpr int kDefaultHeight = 480;
constexpr int kDefaultFps = 30;

string ExpandUserPath(const string& path)
{
    if(path.empty() || path[0] != '~')
        return path;

    const char* home = getenv("HOME");
    if(!home)
        return path;

    if(path.size() == 1)
        return string(home);

    if(path[1] == '/')
        return string(home) + path.substr(1);

    return path;
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

class DepthAnythingCoreML
{
public:
    explicit DepthAnythingCoreML(const string& modelPath)
    : mInputBuffer(nullptr)
    {
        @autoreleasepool
        {
            const string expandedPath = ExpandUserPath(modelPath);
            NSString* nsPath = [NSString stringWithUTF8String:expandedPath.c_str()];
            NSURL* modelURL = [NSURL fileURLWithPath:nsPath];
            if(![[NSFileManager defaultManager] fileExistsAtPath:nsPath])
            {
                SetError("Model package not found at: " + expandedPath);
                return;
            }

            NSError* error = nil;
            NSString* extension = [[nsPath pathExtension] lowercaseString];
            if([extension isEqualToString:@"mlpackage"])
            {
                mCompiledURL = [MLModel compileModelAtURL:modelURL error:&error];
                if(!mCompiledURL)
                {
                    SetNSError("Failed to compile the Core ML model", error);
                    return;
                }
            }
            else if([extension isEqualToString:@"mlmodelc"])
            {
                mCompiledURL = modelURL;
            }
            else
            {
                SetError("Unsupported model path. Expected a .mlpackage or .mlmodelc directory.");
                return;
            }

            MLModelConfiguration* configuration = [[MLModelConfiguration alloc] init];
            configuration.computeUnits = MLComputeUnitsAll;

            mModel = [MLModel modelWithContentsOfURL:mCompiledURL configuration:configuration error:&error];
            if(!mModel)
            {
                SetNSError("Failed to load the Core ML model", error);
                return;
            }

            CVReturn status = CVPixelBufferCreate(
                kCFAllocatorDefault,
                kModelWidth,
                kModelHeight,
                kCVPixelFormatType_32ARGB,
                nullptr,
                &mInputBuffer);
            if(status != kCVReturnSuccess || !mInputBuffer)
            {
                SetError("Failed to allocate the Core ML input pixel buffer.");
                return;
            }
        }
    }

    ~DepthAnythingCoreML()
    {
        if(mInputBuffer)
            CFRelease(mInputBuffer);
    }

    bool IsReady() const
    {
        return mModel != nil && mInputBuffer != nullptr;
    }

    const string& Error() const
    {
        return mError;
    }

    bool Infer(const cv::Mat& frameBgr, cv::Mat& depth32f, double& inferenceMs)
    {
        if(!IsReady())
            return false;

        if(frameBgr.empty())
        {
            SetError("Cannot run inference on an empty frame.");
            return false;
        }

        cv::Mat resizedFrame;
        cv::resize(frameBgr, resizedFrame, cv::Size(kModelWidth, kModelHeight), 0.0, 0.0, cv::INTER_LINEAR);
        if(!CopyToInputBuffer(resizedFrame))
            return false;

        @autoreleasepool
        {
            NSError* error = nil;
            MLFeatureValue* imageFeature = [MLFeatureValue featureValueWithPixelBuffer:mInputBuffer];
            MLDictionaryFeatureProvider* provider =
                [[MLDictionaryFeatureProvider alloc] initWithDictionary:@{@"image": imageFeature} error:&error];
            if(!provider)
            {
                SetNSError("Failed to create the Core ML input provider", error);
                return false;
            }

            const auto start = chrono::steady_clock::now();
            id<MLFeatureProvider> result = [mModel predictionFromFeatures:provider error:&error];
            const auto end = chrono::steady_clock::now();
            inferenceMs = chrono::duration_cast<chrono::duration<double, milli> >(end - start).count();

            if(!result)
            {
                SetNSError("Core ML inference failed", error);
                return false;
            }

            MLFeatureValue* depthFeature = [result featureValueForName:@"depth"];
            if(!depthFeature || depthFeature.type != MLFeatureTypeImage)
            {
                SetError("The Core ML model did not return an image feature named 'depth'.");
                return false;
            }

            CVPixelBufferRef outputBuffer = depthFeature.imageBufferValue;
            if(!DepthBufferToFloatMat(outputBuffer, depth32f))
                return false;
        }

        return true;
    }

private:
    bool CopyToInputBuffer(const cv::Mat& frameBgr)
    {
        cv::Mat bgra;
        cv::cvtColor(frameBgr, bgra, cv::COLOR_BGR2BGRA);

        cv::Mat argb(bgra.rows, bgra.cols, CV_8UC4);
        const int fromTo[] = {3, 0, 2, 1, 1, 2, 0, 3};
        cv::mixChannels(&bgra, 1, &argb, 1, fromTo, 4);

        CVReturn status = CVPixelBufferLockBaseAddress(mInputBuffer, 0);
        if(status != kCVReturnSuccess)
        {
            SetError("Failed to lock the Core ML input pixel buffer.");
            return false;
        }

        void* baseAddress = CVPixelBufferGetBaseAddress(mInputBuffer);
        const size_t bytesPerRow = CVPixelBufferGetBytesPerRow(mInputBuffer);
        for(int row = 0; row < argb.rows; ++row)
        {
            memcpy(static_cast<unsigned char*>(baseAddress) + row * bytesPerRow,
                   argb.ptr(row),
                   static_cast<size_t>(argb.cols * argb.elemSize()));
        }

        CVPixelBufferUnlockBaseAddress(mInputBuffer, 0);
        return true;
    }

    bool DepthBufferToFloatMat(CVPixelBufferRef depthBuffer, cv::Mat& depth32f)
    {
        if(!depthBuffer)
        {
            SetError("The Core ML model returned an empty depth buffer.");
            return false;
        }

        CVReturn status = CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
        if(status != kCVReturnSuccess)
        {
            SetError("Failed to lock the Core ML output pixel buffer.");
            return false;
        }

        const int width = static_cast<int>(CVPixelBufferGetWidth(depthBuffer));
        const int height = static_cast<int>(CVPixelBufferGetHeight(depthBuffer));
        const size_t bytesPerRow = CVPixelBufferGetBytesPerRow(depthBuffer);
        const OSType pixelFormat = CVPixelBufferGetPixelFormatType(depthBuffer);
        void* baseAddress = CVPixelBufferGetBaseAddress(depthBuffer);

        bool success = true;
        if(pixelFormat == kCVPixelFormatType_OneComponent16Half)
        {
            cv::Mat depth16f(height, width, CV_16FC1, baseAddress, bytesPerRow);
            depth16f.convertTo(depth32f, CV_32F);
        }
        else if(pixelFormat == kCVPixelFormatType_OneComponent32Float)
        {
            depth32f = cv::Mat(height, width, CV_32FC1, baseAddress, bytesPerRow).clone();
        }
        else if(pixelFormat == kCVPixelFormatType_OneComponent8)
        {
            cv::Mat depth8u(height, width, CV_8UC1, baseAddress, bytesPerRow);
            depth8u.convertTo(depth32f, CV_32F, 1.0 / 255.0);
        }
        else
        {
            success = false;
            SetError("Unsupported depth buffer pixel format: " + to_string(pixelFormat));
        }

        CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
        return success;
    }

    void SetError(const string& error)
    {
        mError = error;
    }

    void SetNSError(const string& prefix, NSError* error)
    {
        if(error)
        {
            mError = prefix + ": " + string([[error localizedDescription] UTF8String]);
            return;
        }

        mError = prefix + ".";
    }

private:
    MLModel* mModel = nil;
    NSURL* mCompiledURL = nil;
    CVPixelBufferRef mInputBuffer;
    string mError;
};

cv::Mat ColorizeDepth(const cv::Mat& depth32f)
{
    cv::Mat sanitized = depth32f.clone();
    cv::patchNaNs(sanitized, 0.0f);

    cv::Mat validMask = sanitized > 0.0f;
    double minDepth = 0.0;
    double maxDepth = 0.0;
    cv::minMaxLoc(sanitized, &minDepth, &maxDepth, nullptr, nullptr, validMask);

    if(maxDepth <= minDepth + numeric_limits<double>::epsilon())
    {
        return cv::Mat(sanitized.size(), CV_8UC3, cv::Scalar(32, 32, 32));
    }

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

void DrawLabel(cv::Mat& image, const string& text)
{
    const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 0.8;
    const int thickness = 2;
    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    const cv::Rect box(10, 10, textSize.width + 20, textSize.height + baseline + 16);
    cv::rectangle(image, box, cv::Scalar(20, 20, 20), cv::FILLED);
    cv::putText(image, text, cv::Point(20, box.y + textSize.height + 2), fontFace, fontScale,
                cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
}

cv::Mat ComposeDisplay(const cv::Mat& frameBgr, const cv::Mat& coloredDepth, double inferenceMs, double averageMs)
{
    cv::Mat left = frameBgr.clone();
    cv::Mat right = coloredDepth.clone();

    DrawLabel(left, "Camera");

    ostringstream label;
    label << fixed << setprecision(1) << "Depth  " << inferenceMs << " ms";
    if(averageMs > 0.0)
        label << "  avg " << averageMs << " ms";
    DrawLabel(right, label.str());

    cv::Mat combined;
    cv::hconcat(left, right, combined);

    cv::putText(combined,
                "Press q or ESC to quit, s to save a snapshot",
                cv::Point(16, combined.rows - 18),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(230, 230, 230),
                1,
                cv::LINE_AA);

    return combined;
}

string BuildSnapshotPath()
{
    auto now = chrono::system_clock::now();
    auto timestamp = chrono::duration_cast<chrono::seconds>(now.time_since_epoch()).count();
    return "depth_snapshot_" + to_string(timestamp) + ".png";
}
}

int main(int argc, char** argv)
{
    const string defaultModelPath = "models/DepthAnythingV2SmallF16.mlpackage";

    if(argc > 6)
    {
        cerr << endl
             << "Usage: ./depth_webcam_coreml [model_path] [camera_index] [width] [height] [fps]" << endl;
        return 1;
    }

    const string modelPath = argc >= 2 ? string(argv[1]) : defaultModelPath;
    const int cameraIndex = argc >= 3 ? stoi(argv[2]) : 0;
    const int width = argc >= 4 ? stoi(argv[3]) : kDefaultWidth;
    const int height = argc >= 5 ? stoi(argv[4]) : kDefaultHeight;
    const int fps = argc >= 6 ? stoi(argv[5]) : kDefaultFps;

    DepthAnythingCoreML estimator(modelPath);
    if(!estimator.IsReady())
    {
        cerr << estimator.Error() << endl;
        return 1;
    }

    cv::VideoCapture capture;
    if(!OpenCamera(capture, cameraIndex, width, height, fps))
    {
        cerr << "Failed to open camera " << cameraIndex << "." << endl;
        cerr << "On macOS, please grant camera access to your terminal app." << endl;
        return 1;
    }

    cout << "Depth Anything V2 Small (Core ML) is ready." << endl;
    cout << "Model: " << ExpandUserPath(modelPath) << endl;
    cout << "Camera: " << cameraIndex << "  Requested mode: " << width << "x" << height << " @" << fps << " FPS" << endl;

    cv::namedWindow("Depth Anything V2 - Webcam", cv::WINDOW_AUTOSIZE);

    vector<double> recentTimes;
    recentTimes.reserve(30);

    while(true)
    {
        cv::Mat frame;
        if(!capture.read(frame) || frame.empty())
        {
            cerr << "Failed to read a frame from camera " << cameraIndex << "." << endl;
            break;
        }

        cv::Mat depth32f;
        double inferenceMs = 0.0;
        if(!estimator.Infer(frame, depth32f, inferenceMs))
        {
            cerr << estimator.Error() << endl;
            break;
        }

        cv::resize(depth32f, depth32f, frame.size(), 0.0, 0.0, cv::INTER_LINEAR);
        const cv::Mat coloredDepth = ColorizeDepth(depth32f);

        recentTimes.push_back(inferenceMs);
        if(recentTimes.size() > 30)
            recentTimes.erase(recentTimes.begin());
        const double averageMs = recentTimes.empty()
            ? 0.0
            : accumulate(recentTimes.begin(), recentTimes.end(), 0.0) / recentTimes.size();

        const cv::Mat display = ComposeDisplay(frame, coloredDepth, inferenceMs, averageMs);
        cv::imshow("Depth Anything V2 - Webcam", display);

        const int key = cv::waitKey(1);
        if(key == 27 || key == 'q' || key == 'Q')
            break;
        if(key == 's' || key == 'S')
        {
            const string snapshotPath = BuildSnapshotPath();
            cv::imwrite(snapshotPath, display);
            cout << "Saved snapshot to " << snapshotPath << endl;
        }
    }

    capture.release();
    cv::destroyAllWindows();
    return 0;
}
