#include <Control/Controls.h>
#include <Drive/Drive.h>
#include <GamePiece/GamePiece.h>
#include <cmath>
#include <numbers>

#define AXIS_DEADZONE 0.1

Controls::Controls(Drive* _drive, GamePiece* _gamePiece)
: drive(_drive), gamePiece(_gamePiece) {

}

Controls::~Controls() {

}

void Controls::resetToMode(MatchMode mode) {

}

void Controls::process() {
    doDrive();
    doAux();
    doSwitchPanel();
}

void Controls::processInDisabled() {
    doSwitchPanel();

    using DriveButton = HardwareManager::DriveGameController::Button;

    bool resetOdometry = driveController.GetRawButtonPressed(DriveButton::OPTIONS);
    bool calIMU = driveController.GetRawButtonPressed(DriveButton::SHARE);

    if (resetOdometry) {
        drive->resetOdometry();
    }

    if (calIMU) {
        drive->calibrateIMU();
    }
}

void Controls::doDrive() {
    using DriveButton = HardwareManager::DriveGameController::Button;
    using DriveAxis = HardwareManager::DriveGameController::Axis;

    bool brickDrive = driveController.GetRawButton(DriveButton::CROSS);
    
    bool toggleRotation = driveController.GetRawButtonPressed(DriveButton::TRIANGLE);
    double xVel = driveController.GetRawAxis(DriveAxis::LEFT_X);
    double yVel = driveController.GetRawAxis(DriveAxis::LEFT_Y);
    double angVel = driveController.GetRawAxis(DriveAxis::RIGHT_X);
    bool xySlowMode = driveController.GetRawButton(DriveButton::LEFT_BUMPER);
    bool angSlowMode = driveController.GetRawButton(DriveButton::RIGHT_BUMPER);

    double xAng = driveController.GetRawAxis(DriveAxis::RIGHT_X);
    double yAng = driveController.GetRawAxis(DriveAxis::RIGHT_Y);

    bool resetOdometry = driveController.GetRawButtonPressed(DriveButton::OPTIONS);
    bool calIMU = driveController.GetRawButtonPressed(DriveButton::SHARE);
    if (driveController.GetRawButtonPressed(DriveButton::LEFT_STICK)) {
        driveLockX = !driveLockX;
    }

    bool wasBrickDrive = driveCtrlFlags & Drive::ControlFlag::BRICK;
    
    driveCtrlFlags = Drive::ControlFlag::NONE;

    if (!driveRobotCentric) {
        driveCtrlFlags |= Drive::ControlFlag::FIELD_CENTRIC;
    }

    if (brickDrive) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }
    
    if (driveRecording) {
        driveCtrlFlags |= Drive::ControlFlag::RECORDING;
    }

    if (driveLockX) {
        driveCtrlFlags |= Drive::ControlFlag::LOCK_X;
    }

    if (toggleRotation) {
        if (!driveAbsRotation) {
            driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
            driveAbsRotation = true;
        }
        else {
            driveAbsRotation = false;
        }
    }

    if (resetOdometry) {
        drive->resetOdometry();
        driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
    }

    if (calIMU) {
        drive->calibrateIMU();
        driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
    }

    double finalXVel = 0.0,
           finalYVel = 0.0,
           finalAngVel = 0.0,
           finalXAng = 0.0,
           finalYAng = 0.0;

    // Improves the joystick axis to be smoother and easier to control.
    auto improveAxis = [](double axis) -> double {
        return std::sin(axis * (std::numbers::pi / 2.0));
    };

    if (std::fabs(xVel) > AXIS_DEADZONE) {
        finalXVel = improveAxis(xVel);
    }
    if (std::fabs(yVel) > AXIS_DEADZONE) {
        finalYVel = improveAxis(yVel);
    }
    if (std::fabs(angVel) > AXIS_DEADZONE) {
        finalAngVel = improveAxis(angVel);
    }
    if (std::fabs(xAng) > AXIS_DEADZONE) {
        finalXAng = xAng;
    }
    if (std::fabs(yAng) > AXIS_DEADZONE) {
        finalYAng = yAng;
    }

    // Returns whether the robot should be moving.
    auto isMoving = [&]() -> bool {
        bool r = driveAbsRotation ? finalXAng || finalYAng : finalAngVel;
        return finalXVel || finalYVel || r;
    };

    // Stay in brick drive mode if the robot isn't moving.
    if (wasBrickDrive && !isMoving()) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }

    if (xySlowMode){
        finalXVel *= .5;
        finalYVel *= .5;
    }
    if(angSlowMode){
        finalAngVel *= .5;
    }

    // Control the drivetrain.    
    if (driveAbsRotation) {
        // If changed rotation.
        if (finalYAng || finalXAng) {
            // Calculate the new absolute rotation for the robot.
            driveAbsAngle = units::radian_t(std::atan2(-finalYAng, finalXAng)) - 90_deg;
        }

        drive->manualControlAbsRotation(finalXVel, -finalYVel, driveAbsAngle, driveCtrlFlags);
    }
    else {
        drive->manualControlRelRotation(finalXVel, -finalYVel, -finalAngVel, driveCtrlFlags);
    }
}

bool Controls::getShouldPersistConfig() {
    doSwitchPanel();

    using DriveButton = HardwareManager::DriveGameController::Button;
    using AuxButton = HardwareManager::AuxGameController::Button;

    return settings.isCraterMode
        && driveController.GetRawButton(DriveButton::TRIANGLE) && driveController.GetPOV() == 180
        && auxController.GetRawButton(AuxButton::CROSS) && auxController.GetPOV() == 0;
}

void Controls::doAux() {
    // using AuxButton = HardwareManager::AuxGameController::Button;
    // using AuxAxis = HardwareManager::AuxGameController::Axis;

    // D:
}

void Controls::doSwitchPanel() {
    settings.isCraterMode = switchPanel.GetRawButton(1);
    driveRobotCentric = switchPanel.GetRawButton(2);
    driveRecording = switchPanel.GetRawButton(3);
}

void Controls::sendFeedback() {

}
