#include <GamePiece/Grabber.h>
#include <frc/smartdashboard/SmartDashboard.h>

// The max amperage of the intake motors.
#define INTAKE_MAX_AMPERAGE 20_A

#define INTAKE_SPEED 1.0

Grabber::Grabber() 
: leftIntakeMotor(HardwareManager::IOMap::CAN_GRABBER_INTAKE_LEFT),
  rightIntakeMotor(HardwareManager::IOMap::CAN_GRABBER_INTAKE_RIGHT),
  intakeSensor(HardwareManager::IOMap::DIO_GRABBER_INTAKE)
#if WHICH_ROBOT == 2023
  ,grabberPiston1(HardwareManager::IOMap::CAN_PCM, frc::PneumaticsModuleType::CTREPCM,
       HardwareManager::IOMap::PCM_GRABBER_PISTON_1_EXTEND,
       HardwareManager::IOMap::PCM_GRABBER_PISTON_1_RETRACT),
  grabberPiston2(HardwareManager::IOMap::CAN_PCM, frc::PneumaticsModuleType::CTREPCM,
       HardwareManager::IOMap::PCM_GRABBER_PISTON_2_EXTEND,
       HardwareManager::IOMap::PCM_GRABBER_PISTON_2_RETRACT),
   wristPiston(HardwareManager::IOMap::CAN_PCM, frc::PneumaticsModuleType::CTREPCM,
       HardwareManager::IOMap::PCM_GRABBER_WRIST_EXTEND,
       HardwareManager::IOMap::PCM_GRABBER_WRIST_RETRACT)
#endif
   {

    configureMotors();
}

Grabber::~Grabber() {

}

void Grabber::resetToMode(MatchMode mode) {
    currentAction = Action::IDLE;
    leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
    rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);

    placingGamePiece = false;

    if (!(mode == Mechanism::MatchMode::DISABLED && getLastMode() == Mechanism::MatchMode::AUTO)
     && !(mode == Mechanism::MatchMode::TELEOP && getLastMode() == Mechanism::MatchMode::DISABLED)) {
        gamePieceType = GamePieceType::NONE;
        tipped = false;
    }
}

void Grabber::doPersistentConfiguration() {

}

void Grabber::process() {
    // If we are going through the actions of placing a GamePiece.
    if (placingGamePiece) {
        // Cube: Outake for 0.5 seconds, then stop.
        if (gamePieceType == GamePieceType::CUBE) {
            if (placingGamePieceTimer.Get() >= 0.5_s) {
                placingGamePiece = false;
                setAction(Action::IDLE);
                gamePieceType = GamePieceType::NONE;
                if (scoreCallback) {
                    scoreCallback();
                }
            }
            else {
                setAction(Action::OUTTAKE);
            }
        }
        // Cone: Open the grabber completely.
        else if (gamePieceType == GamePieceType::CONE) {
            if (placingGamePieceTimer.Get() >= 0.25_s) {
                placingGamePiece = false;
                setAction(Action::IDLE);
                gamePieceType = GamePieceType::NONE;
                if (scoreCallback) {
                    scoreCallback();
                }
            }
            else {
                setPosition(Position::OPEN);
            }
        }
        // How do we place a GamePiece that doesn't exist?
        else if (gamePieceType == GamePieceType::NONE) {
            placingGamePiece = false;
        }
    }
    else if (autoIntaking) {
        if (autoIntakingTimer.Get() >= 0.5_s) {
            autoIntaking = false;
            setAction(Action::IDLE);   
        } else {
            //continues to run motors during the timer
            setAction(Action::INTAKE);
        }
    }
    else if (finishIntaking) {
        // Keep intaking for a little bit to make sure it's in there.
        if (finishIntakingTimer.Get() >= 0.0_s) {
            finishIntaking = false;
            setAction(Action::IDLE);   

            if (currentPosition == Position::OPEN) {
                gamePieceType = GamePieceType::CUBE;
            }
            else {
                gamePieceType = GamePieceType::CONE;
                setPosition(Position::AJAR);
            }

            // Call the acquire callback.
            if (acquireCallback) {
                acquireCallback(gamePieceType);
            }
        }
        else {
            setAction(Action::INTAKE);
        }
    }

    if (currentAction == Action::INTAKE) {
        leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, INTAKE_SPEED);
        rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, INTAKE_SPEED);
        if ((currentPosition == Position::OPEN && (leftIntakeMotor.getOutputCurrent() >= 20_A || rightIntakeMotor.getOutputCurrent() >= 20_A)) ||
            (currentPosition == Position::AGAPE && (leftIntakeMotor.getOutputCurrent() >= 18_A || rightIntakeMotor.getOutputCurrent() >= 18_A))) {
            intakeCurrentTimer.Start();
            // Sustained current spike for 0.25 seconds.
            if ((currentPosition == Position::OPEN && intakeCurrentTimer.Get() > 0.20_s) ||
                (currentPosition == Position::AGAPE && intakeCurrentTimer.Get() > 0.15_s)) {
                finishIntakingTimer.Reset();
                finishIntakingTimer.Start();
                finishIntaking = true;
            }
        }
        else {
            intakeCurrentTimer.Reset();
            intakeCurrentTimer.Stop();
        }
    } 
    else if (currentAction == Action::OUTTAKE) {
        leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, -INTAKE_SPEED);
        rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, -INTAKE_SPEED);
    } 
    else if (currentAction == Action::IDLE) {
        // Stop the motors.
        leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
        rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
    }

    if (currentPosition == Position::OPEN) {
#if WHICH_ROBOT == 2023
        // Both pistons are extended to intake a cube.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kForward); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kForward);   
#endif
    } 
    else if (currentPosition == Position::AGAPE) {
#if WHICH_ROBOT == 2023
        // Only one piston is extended to intake a cone.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kForward); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kReverse);
#endif
    } 
    else if (currentPosition == Position::AJAR) {
#if WHICH_ROBOT == 2023
        // Both pistons are retracted to hold a cone.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kReverse); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kReverse);     
#endif
    }

    // Set the wrist piston.
    if (tipped == true){
#if WHICH_ROBOT == 2023
        wristPiston.Set(frc::DoubleSolenoid::Value::kForward);
#endif
    }
    else{
#if WHICH_ROBOT == 2023
        wristPiston.Set(frc::DoubleSolenoid::Value::kReverse);
#endif
    }
}

void Grabber::setWristPosition(bool _tipped){
    tipped = _tipped;
}

void Grabber::setAction(Action action) {
    currentAction = action;
}

Grabber::Action Grabber::getAction(){
    return currentAction;
}

void Grabber::setPosition(Position position) {
    currentPosition = position;
}

Grabber::Position Grabber::getPosition() {
    return currentPosition;
}

Grabber::GamePieceType Grabber::getGamePieceType() {
    return gamePieceType;
}

void Grabber::intakeGamePiece() {
    autoIntaking = true;
    autoIntakingTimer.Reset();
    autoIntakingTimer.Start();
}

void Grabber::overrideHasGamePiece(bool hasGamePiece) {
    placingGamePiece = false;
    if (hasGamePiece) {
        if (gamePieceType != GamePieceType::NONE) {
            return;
        }

        switch (currentPosition) {
            case Position::OPEN:
                gamePieceType = GamePieceType::CUBE;
                if (acquireCallback) {
                    acquireCallback(gamePieceType);
                }
                break;
            case Position::AGAPE:
                gamePieceType = GamePieceType::CONE;
                setPosition(Position::AJAR);
                if (acquireCallback) {
                    acquireCallback(gamePieceType);
                }
                break;
            case Position::AJAR:
                break;
        }
    }
    else {
        gamePieceType = GamePieceType::NONE;
    }
}

void Grabber::placeGamePiece() {
    if (getGamePieceType() != Grabber::GamePieceType::NONE) {
        placingGamePiece = true;
        placingGamePieceTimer.Reset();
        placingGamePieceTimer.Start();
    }
}

bool Grabber::isPlacingGamePiece() {
    return placingGamePiece;
}

void Grabber::setGamePiece(Grabber::GamePieceType type) {
    gamePieceType = type;
}

bool Grabber::isFinishedIntaking() {
    return !autoIntaking;
}

void Grabber::onScore(std::function<void()> callback) {
    scoreCallback = callback;
}

void Grabber::onAcquire(std::function<void(GamePieceType)> callback) {
    acquireCallback = callback;
}

void Grabber::configureMotors() {
    leftIntakeMotor.configFactoryDefault();
    leftIntakeMotor.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    leftIntakeMotor.configSmartCurrentLimit(INTAKE_MAX_AMPERAGE);
    leftIntakeMotor.setInverted(false);
    
    rightIntakeMotor.configFactoryDefault();
    rightIntakeMotor.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    rightIntakeMotor.configSmartCurrentLimit(INTAKE_MAX_AMPERAGE);
    rightIntakeMotor.setInverted(true);
}

void Grabber::sendFeedback() {
    std::string grabberAction = "";
    switch (currentAction) {
        case Action::INTAKE:
            grabberAction = "Intake";
            break;
        case Action::OUTTAKE:
            grabberAction = "Outtake";
            break;
        case Action::IDLE:
            grabberAction = "Idle";
            break;
    }

    frc::SmartDashboard::PutString("Grabber_Action", grabberAction.c_str());

    std::string grabberPosition = "";
    switch (currentPosition) {
        case Position::OPEN:
            grabberPosition = "Open (Cube)";
            break;
        case Position::AGAPE:
            grabberPosition = "Agape (Cone intake)";
            break;
        case Position::AJAR:
            grabberPosition = "Ajar (Cone transport)";
            break;
    }

    frc::SmartDashboard::PutString("Grabber_Position", grabberPosition.c_str());

    std::string typeGamePiece = "";
    switch (gamePieceType) {
        case GamePieceType::CONE:
            typeGamePiece = "Cone";
            break;
        case GamePieceType::CUBE:
            typeGamePiece = "Cube";
            break;
        case GamePieceType::NONE:
            typeGamePiece = "None";
            break;
    }

    frc::SmartDashboard::PutString("Grabber_GamePieceType", typeGamePiece.c_str());

    frc::SmartDashboard::PutBoolean("Grabber_PlacingGamePiece", placingGamePiece);
    frc::SmartDashboard::PutNumber("Grabber_PlacingGamePieceTimer_s", placingGamePieceTimer.Get().value());
    
    frc::SmartDashboard::PutBoolean("Grabber_SensorIntake", intakeSensor.Get());

    frc::SmartDashboard::PutNumber("Grabber_TempRightIntake_F", rightIntakeMotor.getTemperature().value());
    frc::SmartDashboard::PutNumber("Grabber_TempLeftIntake_F", leftIntakeMotor.getTemperature().value());
    frc::SmartDashboard::PutNumber("Grabber_CurrentRightIntake_A", rightIntakeMotor.getOutputCurrent().value());
    frc::SmartDashboard::PutNumber("Grabber_CurrentLeftIntake_A", leftIntakeMotor.getOutputCurrent().value());

    // Dashboard feedback.
    frc::SmartDashboard::PutNumber("thunderdashboard_grabber_position", static_cast<int>(currentPosition));
    frc::SmartDashboard::PutNumber("thunderdashboard_gamepiece", static_cast<int>(gamePieceType));
}
