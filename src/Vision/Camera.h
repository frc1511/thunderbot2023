#pragma once

#include <Basic/Mechanism.h>
#include <cameraserver/CameraServer.h>
#include <Hardware/IOMap.h>
#include <string>

class Camera : public Mechanism {
public:
    Camera(int id, std::string name);
    ~Camera();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;

private:
    cs::UsbCamera intakeCamera;
};

