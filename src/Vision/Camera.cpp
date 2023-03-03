#include <Vision/Camera.h>

#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240

#define CAMERA_FPS 30
#define CAMERA_EXPOSURE 40

Camera::Camera(int id, std::string name)
: camera(frc::CameraServer::StartAutomaticCapture(name, id)), cvSink(name) {

    camera.SetResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
    camera.SetFPS(CAMERA_FPS);
    camera.SetExposureManual(CAMERA_EXPOSURE);

    // Send the video stream to the dashboard.
    outputStream = frc::CameraServer::PutVideo(name, CAMERA_WIDTH, CAMERA_HEIGHT);

    cvSink.SetSource(camera);
    cvSink.SetEnabled(true);
}

Camera::~Camera() = default;

void Camera::process() { }

void Camera::sendFeedback() { }

void Camera::doPersistentConfiguration() { }

void Camera::resetToMode(MatchMode mode) { }
