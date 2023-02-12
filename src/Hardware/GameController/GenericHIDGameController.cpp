#include <Hardware/GameController/GenericHIDGameController.h>

ThunderGenericHIDGameController::ThunderGenericHIDGameController(Controller whichController)
: ThunderGameController(whichController), controller(whichController == Controller::AUX) { }

ThunderGenericHIDGameController::~ThunderGenericHIDGameController() { }

bool ThunderGenericHIDGameController::getButton(int button, ButtonState state) {
    switch (state) {
        case ButtonState::HELD:
            return controller.GetRawButton(button);
        case ButtonState::PRESSED:
            return controller.GetRawButtonPressed(button);
        case ButtonState::RELEASED:
            return controller.GetRawButtonReleased(button);
        default:
            return false;
    }
}

double ThunderGenericHIDGameController::getAxis(int axis) {
    return controller.GetRawAxis(axis);
}

ThunderGameController::DPad ThunderGenericHIDGameController::getDPad() {
    return static_cast<DPad>(controller.GetPOV());
}

void ThunderGenericHIDGameController::setRumble(double left, double right) {
    controller.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, left);
    controller.SetRumble(frc::GenericHID::RumbleType::kRightRumble, right);
}
