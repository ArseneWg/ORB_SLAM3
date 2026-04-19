#include "coreml_depth_estimator.h"

#include <cmath>
#include <cstring>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>

#import <CoreML/CoreML.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

namespace
{
constexpr int kModelWidth = 518;
constexpr int kModelHeight = 392;

std::string ExpandUserPath(const std::string& path)
{
    if(path.empty() || path[0] != '~')
        return path;

    const char* home = getenv("HOME");
    if(!home)
        return path;

    if(path.size() == 1)
        return std::string(home);

    if(path[1] == '/')
        return std::string(home) + path.substr(1);

    return path;
}
}

struct CoreMLDepthEstimator::Impl
{
    MLModel* model = nil;
    NSURL* compiledURL = nil;
    CVPixelBufferRef inputBuffer = nullptr;
    std::string error;

    ~Impl()
    {
        if(inputBuffer)
            CFRelease(inputBuffer);
    }

    void SetError(const std::string& message)
    {
        error = message;
    }

    void SetNSError(const std::string& prefix, NSError* nsError)
    {
        if(nsError)
        {
            error = prefix + ": " + std::string([[nsError localizedDescription] UTF8String]);
            return;
        }
        error = prefix + ".";
    }

    bool CopyToInputBuffer(const cv::Mat& frameBgr)
    {
        cv::Mat bgra;
        cv::cvtColor(frameBgr, bgra, cv::COLOR_BGR2BGRA);

        cv::Mat argb(bgra.rows, bgra.cols, CV_8UC4);
        const int fromTo[] = {3, 0, 2, 1, 1, 2, 0, 3};
        cv::mixChannels(&bgra, 1, &argb, 1, fromTo, 4);

        const CVReturn status = CVPixelBufferLockBaseAddress(inputBuffer, 0);
        if(status != kCVReturnSuccess)
        {
            SetError("Failed to lock the Core ML input pixel buffer.");
            return false;
        }

        void* baseAddress = CVPixelBufferGetBaseAddress(inputBuffer);
        const size_t bytesPerRow = CVPixelBufferGetBytesPerRow(inputBuffer);
        for(int row = 0; row < argb.rows; ++row)
        {
            memcpy(static_cast<unsigned char*>(baseAddress) + row * bytesPerRow,
                   argb.ptr(row),
                   static_cast<size_t>(argb.cols * argb.elemSize()));
        }

        CVPixelBufferUnlockBaseAddress(inputBuffer, 0);
        return true;
    }

    bool DepthBufferToFloatMat(CVPixelBufferRef depthBuffer, cv::Mat& depth32f)
    {
        if(!depthBuffer)
        {
            SetError("The Core ML model returned an empty depth buffer.");
            return false;
        }

        const CVReturn status = CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
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
            SetError("Unsupported depth buffer pixel format: " + std::to_string(pixelFormat));
        }

        CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
        return success;
    }
};

CoreMLDepthEstimator::CoreMLDepthEstimator(const std::string& modelPath)
: mImpl(new Impl())
{
    @autoreleasepool
    {
        const std::string expandedPath = ExpandUserPath(modelPath);
        NSString* nsPath = [NSString stringWithUTF8String:expandedPath.c_str()];
        NSURL* modelURL = [NSURL fileURLWithPath:nsPath];
        if(![[NSFileManager defaultManager] fileExistsAtPath:nsPath])
        {
            mImpl->SetError("Model package not found at: " + expandedPath);
            return;
        }

        NSError* error = nil;
        NSString* extension = [[nsPath pathExtension] lowercaseString];
        if([extension isEqualToString:@"mlpackage"])
        {
            mImpl->compiledURL = [MLModel compileModelAtURL:modelURL error:&error];
            if(!mImpl->compiledURL)
            {
                mImpl->SetNSError("Failed to compile the Core ML model", error);
                return;
            }
        }
        else if([extension isEqualToString:@"mlmodelc"])
        {
            mImpl->compiledURL = modelURL;
        }
        else
        {
            mImpl->SetError("Unsupported model path. Expected a .mlpackage or .mlmodelc directory.");
            return;
        }

        MLModelConfiguration* configuration = [[MLModelConfiguration alloc] init];
        configuration.computeUnits = MLComputeUnitsAll;

        mImpl->model = [MLModel modelWithContentsOfURL:mImpl->compiledURL configuration:configuration error:&error];
        if(!mImpl->model)
        {
            mImpl->SetNSError("Failed to load the Core ML model", error);
            return;
        }

        const CVReturn status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            kModelWidth,
            kModelHeight,
            kCVPixelFormatType_32ARGB,
            nullptr,
            &mImpl->inputBuffer);
        if(status != kCVReturnSuccess || !mImpl->inputBuffer)
        {
            mImpl->SetError("Failed to allocate the Core ML input pixel buffer.");
            return;
        }
    }
}

CoreMLDepthEstimator::~CoreMLDepthEstimator() = default;

bool CoreMLDepthEstimator::IsReady() const
{
    return mImpl && mImpl->model != nil && mImpl->inputBuffer != nullptr;
}

const std::string& CoreMLDepthEstimator::Error() const
{
    return mImpl->error;
}

bool CoreMLDepthEstimator::Infer(const cv::Mat& frameBgr, cv::Mat& depth32f, double& inferenceMs)
{
    if(!IsReady())
        return false;

    cv::Mat resizedFrame;
    cv::resize(frameBgr, resizedFrame, cv::Size(kModelWidth, kModelHeight), 0.0, 0.0, cv::INTER_LINEAR);
    if(!mImpl->CopyToInputBuffer(resizedFrame))
        return false;

    @autoreleasepool
    {
        NSError* error = nil;
        MLFeatureValue* imageFeature = [MLFeatureValue featureValueWithPixelBuffer:mImpl->inputBuffer];
        MLDictionaryFeatureProvider* provider =
            [[MLDictionaryFeatureProvider alloc] initWithDictionary:@{@"image": imageFeature} error:&error];
        if(!provider)
        {
            mImpl->SetNSError("Failed to create the Core ML input provider", error);
            return false;
        }

        const auto start = std::chrono::steady_clock::now();
        id<MLFeatureProvider> result = [mImpl->model predictionFromFeatures:provider error:&error];
        const auto end = std::chrono::steady_clock::now();
        inferenceMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(end - start).count();

        if(!result)
        {
            mImpl->SetNSError("Core ML inference failed", error);
            return false;
        }

        MLFeatureValue* depthFeature = [result featureValueForName:@"depth"];
        if(!depthFeature || depthFeature.type != MLFeatureTypeImage)
        {
            mImpl->SetError("The Core ML model did not return an image feature named 'depth'.");
            return false;
        }

        return mImpl->DepthBufferToFloatMat(depthFeature.imageBufferValue, depth32f);
    }
}
