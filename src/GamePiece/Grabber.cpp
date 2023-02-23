#include <GamePiece/Grabber.h>
#include <frc/smartdashboard/SmartDashboard.h>

// The max amperage of the intake motors.
#define INTAKE_MAX_AMPERAGE 20_A

#define INTAKE_SPEED 0.7

Grabber::Grabber() 
: leftIntakeMotor(HardwareManager::IOMap::CAN_GRABBER_INTAKE_LEFT),
  rightIntakeMotor(HardwareManager::IOMap::CAN_GRABBER_INTAKE_RIGHT),
  intakeSensor(HardwareManager::IOMap::DIO_GRABBER_INTAKE),
  grabberPiston1(HardwareManager::IOMap::CAN_PCM, frc::PneumaticsModuleType::CTREPCM,
       HardwareManager::IOMap::PCM_GRABBER_PISTON_1_EXTEND,
       HardwareManager::IOMap::PCM_GRABBER_PISTON_1_RETRACT),
  grabberPiston2(HardwareManager::IOMap::CAN_PCM, frc::PneumaticsModuleType::CTREPCM,
       HardwareManager::IOMap::PCM_GRABBER_PISTON_2_EXTEND,
       HardwareManager::IOMap::PCM_GRABBER_PISTON_2_RETRACT),
   wristPiston(HardwareManager::IOMap::CAN_PCM, frc::PneumaticsModuleType::CTREPCM,
       HardwareManager::IOMap::PCM_GRABBER_WRIST_EXTEND,
       HardwareManager::IOMap::PCM_GRABBER_WRIST_RETRACT)
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
            } else {
                setAction(Action::OUTTAKE);
            }
        }
        // Cone: Open the grabber completely.
        else if (gamePieceType == GamePieceType::CONE) {
            setPosition(Position::OPEN);
            gamePieceType = GamePieceType::NONE;
            placingGamePiece = false;
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
        }
        else {
            setAction(Action::INTAKE);
        }
    }

    if (currentAction == Action::INTAKE) {
        leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, INTAKE_SPEED);
        rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, INTAKE_SPEED);
        if (leftIntakeMotor.getOutputCurrent() >= 10_A || rightIntakeMotor.getOutputCurrent() >= 10_A) {
            intakeCurrentTimer.Start();
            // Sustained current spike for 0.25 seconds.
            if (intakeCurrentTimer.Get() > 0.25_s) {
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
        // Both pistons are extended to intake a cube.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kForward); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kForward);   
    } 
    else if (currentPosition == Position::AGAPE) {
        // Only one piston is extended to intake a cone.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kReverse); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kForward);
    } 
    else if (currentPosition == Position::AJAR) {
        // Both pistons are retracted to hold a cone.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kReverse); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kReverse);     
    }

    // Set the wrist piston.
    if (tipped == true){
        wristPiston.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else{
        wristPiston.Set(frc::DoubleSolenoid::Value::kReverse);
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

Grabber::GamePieceType Grabber::getGamePieceType() {
    return gamePieceType;
}

void Grabber::intakeGamePiece() {
    autoIntaking = true;
    autoIntakingTimer.Reset();
    autoIntakingTimer.Start();
}

void Grabber::overrideHasGamePiece() {
    gamePieceType = GamePieceType::NONE;
    placingGamePiece = false;
}

void Grabber::placeGamePiece() {
    placingGamePiece = true;
    placingGamePieceTimer.Reset();
    placingGamePieceTimer.Start();
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
