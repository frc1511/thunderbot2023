#include <Control/Controls.h>
#include <Drive/Drive.h>
#include <GamePiece/GamePiece.h>
#include <Autonomous/Autonomous.h>
#include <cmath>
#include <numbers>
#include <Drive/UltraBrickMode.h>
#include <Illumination/BlinkyBlinky.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define AXIS_DEADZONE 0.1

Controls::Controls(Drive* _drive, GamePiece* _gamePiece, UltraBrickMode* _ultraBrickMode, BlinkyBlinky* _blinkyBlinky
#if WHICH_ROBOT == 2023
, Autonomous* _autonomous
#endif
)
: drive(_drive), gamePiece(_gamePiece), ultraBrickMode(_ultraBrickMode), blinkyBlinky(_blinkyBlinky)
#if WHICH_ROBOT == 2023
, autonomous(_autonomous)
#endif
 { }

Controls::~Controls() { }

void Controls::resetToMode(MatchMode mode) { }

void Controls::process() {
    driveController.process();
    auxController.process();

    driveController.setLightbarColor(255, 0, 0);
    auxController.setLightbarColor(0, 255, 0);

    doSwitchPanel();
    if (!callaDisable) {
        doDrive();
    }

    if (!haileyDisable) {
        if (manualAux) {
            doAuxManual();
        }
        else {
            doAux();
        }
    }
}

void Controls::processInDisabled() {
    doSwitchPanel();

    using DriveButton = HardwareManager::DriveGameController::Button;

    bool resetOdometry = driveController.getButton(DriveButton::OPTIONS, ThunderGameController::ButtonState::PRESSED);
    bool calIMU = driveController.getButton(DriveButton::SHARE, ThunderGameController::ButtonState::PRESSED);

    if (resetOdometry) {
        drive->resetOdometry();
    }

    if (calIMU) {
        drive->calibrateIMU();
    }
}

void Controls::processInAutonomous() {
    doSwitchPanel();
}

bool Controls::getShouldPersistConfig() {
    doSwitchPanel();

    using DriveButton = HardwareManager::DriveGameController::Button;
    using AuxButton = HardwareManager::AuxGameController::Button;

    return settings.isCraterMode
        && driveController.getButton(DriveButton::TRIANGLE) && driveController.getDPad() == ThunderGameController::DPad::DOWN
        && auxController.getButton(AuxButton::CROSS) && auxController.getDPad() == ThunderGameController::DPad::UP;
}

void Controls::doDrive() {
    using DriveButton = HardwareManager::DriveGameController::Button;
    using DriveAxis = HardwareManager::DriveGameController::Axis;

    bool brickDrive = driveController.getButton(DriveButton::CROSS);
    
    bool toggleRotation = driveController.getButton(DriveButton::TRIANGLE, ThunderGameController::ButtonState::PRESSED);
    double xVel = driveController.getLeftXAxis();
    double yVel = driveController.getLeftYAxis();
    double angVel = driveController.getRightXAxis();
    bool xySlowMode = driveController.getLeftBumper();
    bool angSlowMode = driveController.getRightBumper();
    bool toggleUltraBrickMode = driveController.getButton(DriveButton::SQUARE, ThunderGameController::ButtonState::PRESSED);

    double xAng = driveController.getAxis(DriveAxis::RIGHT_X);
    double yAng = driveController.getAxis(DriveAxis::RIGHT_Y);

    bool resetOdometry = driveController.getButton(DriveButton::OPTIONS, ThunderGameController::ButtonState::PRESSED);
    bool calIMU = driveController.getButton(DriveButton::SHARE, ThunderGameController::ButtonState::PRESSED);

    ThunderGameController::DPad dpad = driveController.getDPad();

    if (driveController.getButton(DriveButton::LEFT_STICK, ThunderGameController::ButtonState::PRESSED)) {
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
        // driveCtrlFlags |= Drive::ControlFlag::LOCK_X;
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

    switch (dpad) {
        case ThunderGameController::DPad::DOWN:
            // Align center.
            driveAligning = true;
            drive->alignToGrid(Drive::AlignmentDirection::CENTER);
            break;
        case ThunderGameController::DPad::RIGHT:
            // Align right.
            driveAligning = true;
            drive->alignToGrid(Drive::AlignmentDirection::RIGHT);
            break;
        case ThunderGameController::DPad::LEFT:
            // Align left.
            driveAligning = true;
            drive->alignToGrid(Drive::AlignmentDirection::LEFT);
            break;
        default:
            break;
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
        return finalXVel || finalYVel || r || driveAligning;
    };

    // Stay in brick drive mode if the robot isn't moving.
    if (wasBrickDrive && !isMoving()) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }

    // Don't allow enabling ultra brick mode if the robot is moving.
    if (isMoving() && !doUltraBrickMode && toggleUltraBrickMode) {
        toggleUltraBrickMode = false;
    }

    if (toggleUltraBrickMode) {
        doUltraBrickMode = !doUltraBrickMode;
    }

    ultraBrickMode->setState(doUltraBrickMode);
    // Put it in brick mode if we're in ultra brick mode.
    if (doUltraBrickMode) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }

    // Hi Peter!!!
    if (xySlowMode){
        finalXVel *= .428571439;
        finalYVel *= .428571439;
    }
    if (angSlowMode){
        finalAngVel *= .35;
    }

    auto isManualControl = [&]() -> bool {
        return finalXVel || finalYVel || finalAngVel || finalXAng || finalYAng;
    };

    if (isManualControl()) {
        driveAligning = false;
    }

    if (!driveAligning) {
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
}

void Controls::doAux() {
    using AuxButton = HardwareManager::AuxGameController::Button;
    using AuxAxis = HardwareManager::AuxGameController::Axis;

    // Regular Aux controls.

    bool prepareCone = auxController.getButton(AuxButton::TRIANGLE, ThunderGameController::ButtonState::PRESSED);
    bool prepareCube = auxController.getButton(AuxButton::SQUARE, ThunderGameController::ButtonState::PRESSED);
    bool prepareTippedCone = auxController.getButton(AuxButton::CIRCLE, ThunderGameController::ButtonState::PRESSED);
    bool liftHigh = auxController.getDPad() == ThunderGameController::DPad::UP;
    bool liftMid = auxController.getDPad() == ThunderGameController::DPad::RIGHT;
    bool liftLow = auxController.getDPad() == ThunderGameController::DPad::DOWN;
    bool score = false;
    bool shouldScore = auxController.getAxis(AuxAxis::LEFT_TRIGGER) > AXIS_DEADZONE;
    bool onlyPivot = auxController.getButton(AuxButton::LEFT_BUMPER);

    // Make the non-button controls be toggles.

    if (shouldScore) score = !wasScoring;
    wasScoring = shouldScore;

    bool intake = auxController.getAxis(AuxAxis::RIGHT_TRIGGER) > AXIS_DEADZONE;
    bool outtake = auxController.getButton(AuxButton::RIGHT_BUMPER);
    bool overrideGamePieceNo = auxController.getButton(AuxButton::SHARE, ThunderGameController::ButtonState::PRESSED);
    bool overrideGamePieceYes = auxController.getButton(AuxButton::CROSS, ThunderGameController::ButtonState::PRESSED);
    
    // If we have a GamePiece, we don't want to prepare to grab another one.
    if (gamePiece->getGamePieceType() != Grabber::GamePieceType::NONE ||
        // Also don't prepare for multiple game pieces at once.
        prepareCone + prepareCube + prepareTippedCone > 1) {

        prepareCone = false;
        prepareCube = false;
        prepareTippedCone = false;
    }

    // Sets the grabber to the AGAPE position to fit a cone.
    if (prepareCone || prepareTippedCone) {
        gamePiece->setGrabberPosition(Grabber::Position::AGAPE);
    }
    // Sets the grabber to the OPEN position to fit a cube.
    else if (prepareCube) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
    }

    if (prepareCone || prepareCube) {
        if (gamePiece->getLiftPreset() != GamePiece::LiftPreset::BALCONY && gamePiece->getLiftPreset() != GamePiece::LiftPreset::SLIDE) {
            // Move the lift to the correct intake position.
            gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);
        }

        if (gamePiece->getLiftPreset() != GamePiece::LiftPreset::BALCONY) {
            // Set the wrist to the upright position.
            gamePiece->setWrist(false);
        }
    }
    else if (prepareTippedCone) {
        // Move the lift to the correct intake position.
        gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE_FUNKY);

        // Set the wrist to the upright position.
        gamePiece->setWrist(true);
    }

    if (liftHigh) {
        // If we don't have a GamePiece, go to the balcony position.
        if (gamePiece->getGamePieceType() == Grabber::GamePieceType::NONE) {
            // Toggle between the balcony and balcony pivot positions.
            if (onlyPivot) {
                gamePiece->setLiftPreset(GamePiece::LiftPreset::BALCONY_PIVOT);
            }
            else {
                gamePiece->setLiftPreset(GamePiece::LiftPreset::BALCONY);
            }
        }
        // If we do, go to the high position.
        else {
            // Toggle between the high and high pivot positions.
            if (onlyPivot) {
                gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);
            }
            else {
                gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);
            }
        }
    }
    // Moves the lift to the mid grid position.
    if (liftMid) {
        // If we don't have a GamePiece, go to the slide position.
        if (gamePiece->getGamePieceType() == Grabber::GamePieceType::NONE) {
            // Toggle between the slide and slide pivot positions.
            if (onlyPivot) {
                gamePiece->setLiftPreset(GamePiece::LiftPreset::SLIDE_PIVOT);
            }
            else {
                gamePiece->setLiftPreset(GamePiece::LiftPreset::SLIDE);
            }
        }
        // If we do, go to the mid position.
        else {
            // Toggle between the mid and mid pivot positions.
            if (onlyPivot) {
                gamePiece->setLiftPreset(GamePiece::LiftPreset::MID_PIVOT);
            }
            else {
                gamePiece->setLiftPreset(GamePiece::LiftPreset::MID);
            }
        }
    }
    if (liftLow) {
        // If we don't have a GamePiece, go to the intake position.
        if (gamePiece->getGamePieceType() == Grabber::GamePieceType::NONE) {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);
        }
        // If we do, go to the low scoring position.
        else {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::GROUND);
        }        
    }

    // Initiates the automated scoring process of the grabber.
    if (score) {
        if (gamePiece->getGamePieceType() != Grabber::GamePieceType::NONE) {
            gamePiece->placeGamePiece();
            blinkyBlinky->playScoreAnimation();
        }
    }

    // Spins the wheels on the grabber inward to intake a gamepiece.
    if (intake && gamePiece->getGamePieceType() == Grabber::GamePieceType::NONE) {
        gamePiece->setGrabberAction(Grabber::Action::INTAKE);
    }
    // Spins the wheels on the grabber outward to outtake a gamepiece.
    else if (outtake) {
        gamePiece->setGrabberAction(Grabber::Action::OUTTAKE);
        gamePiece->overrideHasGamePiece(false);
    }
    //If the grabber is neither intaking nor outtaking, the action is set to idle.
    else {
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
    }

    // Overrides the current game piece count.
    if (overrideGamePieceNo) {
        gamePiece->overrideHasGamePiece(false);
        gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);
    }
    if (overrideGamePieceYes) {
        gamePiece->overrideHasGamePiece(true);
    }
}

void Controls::doAuxManual() {
    using AuxButton = HardwareManager::AuxGameController::Button;
    using AuxAxis = HardwareManager::AuxGameController::Axis;

    // Manual Aux controls.

    bool agape = auxController.getButton(AuxButton::TRIANGLE, ThunderGameController::ButtonState::PRESSED);
    bool ajar = auxController.getButton(AuxButton::CIRCLE, ThunderGameController::ButtonState::PRESSED);
    bool open = auxController.getButton(AuxButton::SQUARE, ThunderGameController::ButtonState::PRESSED);
    bool outake = auxController.getButton(AuxButton::RIGHT_BUMPER);
    bool intake = auxController.getAxis(AuxAxis::RIGHT_TRIGGER) > AXIS_DEADZONE;
    bool wristTipped = auxController.getButton(AuxButton::CROSS, ThunderGameController::ButtonState::PRESSED);

    double pivotLift = -auxController.getAxis(AuxAxis::RIGHT_Y);
    double extendLift = -auxController.getAxis(AuxAxis::LEFT_Y);

    if (agape) {
        gamePiece->setGrabberPosition(Grabber::Position::AGAPE);
    } else if (ajar) {
        gamePiece->setGrabberPosition(Grabber::Position::AJAR);
    } else if (open) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
    }

    if (wristTipped){
        manualWrist = !manualWrist;
    }

    gamePiece->setWrist(manualWrist);

    if (intake) {
        gamePiece->setGrabberAction(Grabber::Action::INTAKE);
    } else if (outake) {
        gamePiece->setGrabberAction(Grabber::Action::OUTTAKE);
    } else {
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
    }

    if (std::fabs(pivotLift) < AXIS_DEADZONE) {
        pivotLift = 0;
    }
    if (std::fabs(extendLift) < AXIS_DEADZONE) {
        extendLift = 0;
    }

    gamePiece->setManualPivotSpeed(pivotLift);
    gamePiece->setManualExtensionSpeed(extendLift);
}

void Controls::doSwitchPanel() {
    settings.isCraterMode = switchPanel.GetRawButton(6);
    driveRobotCentric = switchPanel.GetRawButton(2);
    // callaDisable = switchPanel.GetRawButton(3);
    // if (switchPanel.GetRawButtonPressed(4)) {

    // }
    // haileyDisable = switchPanel.GetRawButton(4);
    // driveRecording = switchPanel.GetRawButton(3);
    manualAux = switchPanel.GetRawButton(5);

    // Switch 7 disables the lift.
    settings.liftActive = !switchPanel.GetRawButton(7);

    if (switchPanel.GetRawButton(12)) {
        gamePiece->overrideLiftKindaBroken();
    }

    int ledMode = frc::SmartDashboard::GetNumber("thunderdashboard_led_mode", 0.0);

#if WHICH_ROBOT == 2023
    if (switchPanel.GetRawButton(1)) {
        blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::OFF);
    }
    else if (ledMode == 0) {
        if (settings.isCraterMode) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::CRATER_MODE);
        }
        else if (!drive->isIMUCalibrated()) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::CALIBRATING);
        }
        else if (getCurrentMode() == MatchMode::DISABLED || doUltraBrickMode) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::HOME_DEPOT);

            if (getCurrentMode() == MatchMode::DISABLED && getLastMode() == MatchMode::AUTO) {
                blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::RAINBOW);
            }
        }
        else if (autonomous->isBalancing()) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::BALANCING);
        }
        else if (!switchPanel.GetRawButton(6)) {
            if (gamePiece->getGamePieceType() != Grabber::GamePieceType::NONE) {
                blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::HAS_GAMEPIECE);
            }
            else if (gamePiece->getGrabberPosition() == Grabber::Position::OPEN) {
                blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::CUBE);
            }
            else if (gamePiece->getGrabberPosition() == Grabber::Position::AGAPE) {
                blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::CONE);
            }
            else {
                blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::RAINBOW);
            }
        }
        else {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::ALLIANCE);
        }
    }
    else if (ledMode == 1) {
        blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::ALLIANCE);
    }
    else if (ledMode == 2) {
        blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::CUSTOM);

        double r = frc::SmartDashboard::GetNumber("thunderdashboard_led_custom_r", 0.0);
        double g = frc::SmartDashboard::GetNumber("thunderdashboard_led_custom_g", 0.0);
        double b = frc::SmartDashboard::GetNumber("thunderdashboard_led_custom_b", 0.0);

        blinkyBlinky->setCustomColor(frc::Color(r, g, b));
    }
    else {
        blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::OFF);
    }
#endif
}

void Controls::sendFeedback() {

}
