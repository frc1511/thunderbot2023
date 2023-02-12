#pragma once

#include <Hardware/GameController/GameController.h>
#include <frc/GenericHID.h>

class ThunderGenericHIDGameController : public ThunderGameController {
public:
    ThunderGenericHIDGameController(Controller controller);
    virtual ~ThunderGenericHIDGameController();

    bool getButton(int button, ButtonState state = ButtonState::HELD) override;
    double getAxis(int axis) override;
    DPad getDPad() override;

    void setRumble(double left, double right) override;

private:
    frc::GenericHID controller;
};