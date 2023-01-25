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
    doSwitchPanel();
    doDrive();

    if (manualAux) {
        doAuxManual();
    }
    else {
        doAux();
    }
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

bool Controls::getShouldPersistConfig() {
    doSwitchPanel();

    using DriveButton = HardwareManager::DriveGameController::Button;
    using AuxButton = HardwareManager::AuxGameController::Button;

    return settings.isCraterMode
        && driveController.GetRawButton(DriveButton::TRIANGLE) && driveController.GetPOV() == 180
        && auxController.GetRawButton(AuxButton::CROSS) && auxController.GetPOV() == 0;
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

void Controls::doAux() {
    using AuxButton = HardwareManager::AuxGameController::Button;
    using AuxAxis = HardwareManager::AuxGameController::Axis;

    // Regular Aux controls.

    bool prepareCone = auxController.GetRawButtonPressed(AuxButton::TRIANGLE);
    bool prepareCube = auxController.GetRawButtonPressed(AuxButton::SQUARE);
    bool liftHigh = auxController.GetPOV() == 0;
    bool liftMid = auxController.GetPOV() == 90;
    bool liftLow = auxController.GetPOV() == 180;
    bool score = auxController.GetRawAxis(AuxAxis::LEFT_TRIGGER) > AXIS_DEADZONE;
    bool intake = auxController.GetRawAxis(AuxAxis::RIGHT_TRIGGER) > AXIS_DEADZONE;
    bool outtake = auxController.GetRawButton(AuxButton::RIGHT_BUMPER);
    bool overrideGamePiece = auxController.GetRawButton(AuxButton::SHARE);

    //Sets the grabber to the AGAPE position to fit a cone.
    if(prepareCone) {
        if(gamePiece->getGamePieceType() == Grabber::GamePieceType::NONE) {
            gamePiece->setGrabberPosition(Grabber::Position::AGAPE);
        }
    }
    //Sets the grabber to the OPEN position to fit a cube.
    if(prepareCube) {
        if(gamePiece->getGamePieceType() == Grabber::GamePieceType::NONE) {
            gamePiece->setGrabberPosition(Grabber::Position::OPEN);
        }
    }
    //Moves the lift to the high grid position if it has a game piece, and moves it to the balcony position if it has no game piece.
    if(liftHigh) {
        if(gamePiece->getGamePieceType() == Grabber::GamePieceType::NONE) {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::BALCONY);
        }
        else {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);
        }
    }
    //Moves the lift to the mid grid position.
    if(liftMid) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::MID);
    }
    //Moves the lift to the hybrid grid position if it has a game piece, and moves it to the intaking position if it has no game piece.
    if(liftLow) {
        if(gamePiece->getGamePieceType() == Grabber::GamePieceType::NONE) {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);
        }
        else {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::GROUND);
        }        
    }
    //Initiates the automated scoring process of the grabber.
    if(score) {
        if(gamePiece->getGamePieceType() != Grabber::GamePieceType::NONE) {
            gamePiece->placeGamePiece();
        }
    }
    //Spins the wheels on the grabber inward to intake a gamepiece.
    if(intake) {
        gamePiece->setGrabberAction(Grabber::Action::INTAKE);
    }
    //Spins the wheels on the grabber outward to outtake a gamepiece.
    else if(outtake) {
        gamePiece->setGrabberAction(Grabber::Action::OUTTAKE);
    }
    //If the grabber is neither intaking or outtaking the action is set to idle.
    else {
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
    }
    //Overrides the current game piece count.
    if(overrideGamePiece) {
        gamePiece->overrideHasGamePiece();
    }
}

void Controls::doAuxManual() {
    using AuxButton = HardwareManager::AuxGameController::Button;
    using AuxAxis = HardwareManager::AuxGameController::Axis;

    // Manual Aux controls.

    bool agape = auxController.GetRawButtonPressed(AuxButton::TRIANGLE);
    bool ajar = auxController.GetRawButtonPressed(AuxButton::CIRCLE);
    bool open = auxController.GetRawButtonPressed(AuxButton::SQUARE);
    bool outake = auxController.GetRawButton(AuxButton::RIGHT_BUMPER);
    bool intake = auxController.GetRawAxis(AuxAxis::RIGHT_TRIGGER) > AXIS_DEADZONE;

    double pivotLift = -auxController.GetRawAxis(AuxAxis::RIGHT_Y);
    double extendLift = -auxController.GetRawAxis(AuxAxis::LEFT_Y);

    if (agape) {
        gamePiece->setGrabberPosition(Grabber::Position::AGAPE);
    } else if (ajar) {
        gamePiece->setGrabberPosition(Grabber::Position::AJAR);
    } else if (open) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
    }

    if (intake) {
        gamePiece->setGrabberAction(Grabber::Action::INTAKE);
    } else if (outake) {
        gamePiece->setGrabberAction(Grabber::Action::OUTTAKE);
    } else {
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
    }

    if (pivotLift < AXIS_DEADZONE) {
        pivotLift = 0;
    }
    if (extendLift < AXIS_DEADZONE) {
        extendLift = 0;
    }

    gamePiece->setManualPivotSpeed(pivotLift);
    gamePiece->setManualExtensionSpeed(extendLift);
}

void Controls::doSwitchPanel() {
    settings.isCraterMode = switchPanel.GetRawButton(1);
    driveRobotCentric = switchPanel.GetRawButton(2);
    driveRecording = switchPanel.GetRawButton(3);
    manualAux = switchPanel.GetRawButton(4);
}

void Controls::sendFeedback() {

}
