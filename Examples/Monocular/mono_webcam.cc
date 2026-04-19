#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <functional>
#include <iostream>
#include <pthread.h>
#include <string>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <System.h>

using namespace std;

namespace
{
atomic<bool> g_keepRunning(true);

void exit_loop_handler(int)
{
    g_keepRunning = false;
}

int ReadIntSetting(const cv::FileStorage& settings, const string& key, int fallback)
{
    cv::FileNode node = settings[key];
    if(node.empty())
        return fallback;

    return static_cast<int>(node.real());
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

void* RunCaptureLoop(void* arg)
{
    auto* fn = static_cast<function<void()>*>(arg);
    (*fn)();
    return nullptr;
}
}

int main(int argc, char **argv)
{
    if(argc < 3 || argc > 5)
    {
        cerr << endl
             << "Usage: ./mono_webcam path_to_vocabulary path_to_settings [camera_index] [trajectory_file]"
             << endl;
        return 1;
    }

    int cameraIndex = 0;
    if(argc >= 4)
    {
        try
        {
            cameraIndex = stoi(argv[3]);
        }
        catch(const exception&)
        {
            cerr << "Invalid camera index: " << argv[3] << endl;
            return 1;
        }
    }

    const string trajectoryFile = argc >= 5 ? string(argv[4]) : "KeyFrameTrajectory.txt";

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    cv::FileStorage settings(argv[2], cv::FileStorage::READ);
    if(!settings.isOpened())
    {
        cerr << "Failed to open settings file at: " << argv[2] << endl;
        return 1;
    }

    const int width = ReadIntSetting(settings, "Camera.newWidth",
                      ReadIntSetting(settings, "Camera.width", 640));
    const int height = ReadIntSetting(settings, "Camera.newHeight",
                       ReadIntSetting(settings, "Camera.height", 480));
    const int fps = ReadIntSetting(settings, "Camera.fps", 30);

    cv::VideoCapture probe;
    if(!OpenCamera(probe, cameraIndex, width, height, fps))
    {
        cerr << "Failed to open camera " << cameraIndex << "." << endl;
#ifdef __APPLE__
        cerr << "On macOS, please grant camera access to Codex or your terminal app." << endl;
#endif
        return 1;
    }
    probe.release();

#ifdef __APPLE__
    const bool useViewerThread = false;
    cout << "macOS detected: running the Pangolin viewer on the main thread." << endl;
#else
    const bool useViewerThread = true;
#endif

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, useViewerThread);
    const float imageScale = SLAM.GetImageScale();

    atomic<bool> captureFailed(false);
    string failureMessage;

    function<void()> captureLoop = [&]()
    {
        cv::VideoCapture capture;
        if(!OpenCamera(capture, cameraIndex, width, height, fps))
        {
            failureMessage = "Failed to reopen camera " + to_string(cameraIndex) + ".";
            captureFailed = true;
            SLAM.Shutdown();
            return;
        }

        const auto startTime = chrono::steady_clock::now();

        while(g_keepRunning && !SLAM.isShutDown())
        {
            cv::Mat frame;
            if(!capture.read(frame) || frame.empty())
            {
                failureMessage = "Failed to read a frame from camera " + to_string(cameraIndex) + ".";
                captureFailed = true;
                break;
            }

            if(SLAM.isShutDown())
                break;

            if(imageScale != 1.f)
            {
                const int scaledWidth = frame.cols * imageScale;
                const int scaledHeight = frame.rows * imageScale;
                cv::resize(frame, frame, cv::Size(scaledWidth, scaledHeight));
            }

            const double timestamp = chrono::duration_cast<chrono::duration<double> >(
                chrono::steady_clock::now() - startTime).count();
            SLAM.TrackMonocular(frame, timestamp);
        }

        capture.release();
        SLAM.Shutdown();
        SLAM.SaveKeyFrameTrajectoryTUM(trajectoryFile);
    };

    pthread_t captureThread;
    pthread_attr_t captureAttr;
    pthread_attr_init(&captureAttr);
    pthread_attr_setstacksize(&captureAttr, 8 * 1024 * 1024);

    const int captureThreadStatus = pthread_create(&captureThread, &captureAttr, &RunCaptureLoop, &captureLoop);
    pthread_attr_destroy(&captureAttr);

    if(captureThreadStatus != 0)
    {
        cerr << "Failed to start capture thread." << endl;
        return 1;
    }

#ifdef __APPLE__
    SLAM.RunViewerOnCurrentThread();
#else
    while(g_keepRunning && !SLAM.isShutDown())
    {
        this_thread::sleep_for(chrono::milliseconds(50));
    }
#endif

    pthread_join(captureThread, nullptr);

    if(captureFailed)
    {
        cerr << failureMessage << endl;
        return 1;
    }

    return 0;
}
