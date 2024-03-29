#include <Vision/IntakeCamera.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#pragma GCC diagnostic pop

#include <fmt/core.h>
#include <map>
#include <vector>
#include <utility>

#define CAMERA_NAME "intake_camera"

std::map<Grabber::GamePieceType, std::pair<cv::Scalar, cv::Scalar>> gamePieceHSVTolerances {
    { Grabber::GamePieceType::CONE, { cv::Scalar(21, 41, 197), cv::Scalar(72, 255, 255) } },
    // { Grabber::GamePieceType::CONE, { cv::Scalar(18, 121, 85), cv::Scalar(46, 255, 255) } },
    { Grabber::GamePieceType::CUBE, { cv::Scalar(109, 107, 120), cv::Scalar(128, 172, 255) } },
    { Grabber::GamePieceType::NONE, { cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255) } }
};

IntakeCamera::IntakeCamera() {
    // visionThread = std::thread([this] { threadMain(); });
}

IntakeCamera::~IntakeCamera() {
    // visionThread.join();
}

void IntakeCamera::setTarget(Grabber::GamePieceType target) {
    std::lock_guard<std::mutex> lk(visionMutex);
    gamePieceTarget = target;
}

void IntakeCamera::threadMain() {
    using namespace std::chrono_literals;

    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(640, 480);

    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    cs::CvSource outputStream = frc::CameraServer::PutVideo("intake_camera", 426, 320);

    cv::Mat frame, frame_threshold;

    while (true) {
        Grabber::GamePieceType target;
        // {
        //     std::lock_guard<std::mutex> lk(visionMutex);
        //     target = gamePieceTarget;
        // }
        // if (target == Grabber::GamePieceType::NONE) {
        //     std::this_thread::sleep_for(100ms);
        //     continue;
        // }
        target = Grabber::GamePieceType::CONE;

        uint64_t frameTime = cvSink.GrabFrame(frame);
        if (!frameTime) {
            std::this_thread::sleep_for(100ms);
            fmt::print("Failed to grab frame from intake_camera\n");
            continue;
        }
        /*

        cv::cvtColor(frame, frame_threshold, cv::COLOR_BGR2HSV);

        cv::inRange(frame_threshold, gamePieceHSVTolerances[target].first, gamePieceHSVTolerances[target].second, frame_threshold);

        cv::medianBlur(frame_threshold, frame_threshold, 3);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point());

        for (std::size_t i = 0; i < contours.size(); i++) {
            cv::Rect rect = cv::boundingRect(contours[i]);
            cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
        }

        */
        outputStream.PutFrame(frame);
        // fmt::print("Sent frame, {}, {}, {}\n", cvSink.GetError(), outputStream.IsEnabled(), outputStream.IsConnected());
    }
}