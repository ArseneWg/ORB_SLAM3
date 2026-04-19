#ifndef ORB_SLAM3_COREML_DEPTH_ESTIMATOR_H
#define ORB_SLAM3_COREML_DEPTH_ESTIMATOR_H

#include <memory>
#include <string>

#include <opencv2/core/core.hpp>

class CoreMLDepthEstimator
{
public:
    explicit CoreMLDepthEstimator(const std::string& modelPath);
    ~CoreMLDepthEstimator();

    bool IsReady() const;
    const std::string& Error() const;
    bool Infer(const cv::Mat& frameBgr, cv::Mat& depth32f, double& inferenceMs);

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

#endif
