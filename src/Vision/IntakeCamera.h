#pragma once

#include <Basic/Mechanism.h>
#include <cameraserver/CameraServer.h>
#include <Hardware/HardwareManager.h>
#include <Vision/Camera.h>
#include <GamePiece/Grabber.h>
#include <thread>
#include <mutex>

class IntakeCamera : public Mechanism {
public:
    IntakeCamera();
    ~IntakeCamera();

    void setTarget(Grabber::GamePieceType target);

private:
    void threadMain();

    std::thread visionThread;
    std::mutex visionMutex;

    Grabber::GamePieceType gamePieceTarget;
};