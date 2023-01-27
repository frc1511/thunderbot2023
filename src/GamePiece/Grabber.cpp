#include <GamePiece/Grabber.h>

// The max amperage of the drive motors.
#define INTAKE_MAX_AMPERAGE 40_A


// --- PID Values ---

#define INTAKE_P 0.00001
#define INTAKE_I 0
#define INTAKE_D 0
#define INTAKE_I_ZONE 0
#define INTAKE_FF 0.000187



Grabber::Grabber() 
: leftIntakeMotor(HardwareManager::IOMap::CAN_GRABBER_INTAKE_LEFT),
  rightIntakeMotor(HardwareManager::IOMap::CAN_GRABBER_INTAKE_RIGHT),
  intakeSensor(HardwareManager::IOMap::DIO_GRABBER_INTAKE),
   grabberPiston1(frc::PneumaticsModuleType::CTREPCM,
        HardwareManager::IOMap::PCM_GRABBER_PISTON_1_EXTEND,
        HardwareManager::IOMap::PCM_GRABBER_PISTON_1_RETRACT),
   grabberPiston2(frc::PneumaticsModuleType::CTREPCM,
        HardwareManager::IOMap::PCM_GRABBER_PISTON_2_EXTEND,
        HardwareManager::IOMap::PCM_GRABBER_PISTON_2_RETRACT)
   {
  
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
    }
}

void Grabber::doPersistentConfiguration() {

}

void Grabber::process() {
    if (placingGamePiece) {
        if (gamePieceType == GamePieceType::CUBE) {
            if (placingGamePieceTimer.Get() >= 0.5_s) {
                placingGamePiece = false;
                setAction(Action::IDLE);
                gamePieceType = GamePieceType::NONE;
            }
            else {
                setAction(Action::OUTTAKE);
            }
        }
        else if (gamePieceType == GamePieceType::CONE) {
            setPosition(Position::OPEN);
            gamePieceType = GamePieceType::NONE;
            placingGamePiece = false;
        }
        else if (gamePieceType == GamePieceType::NONE) {
            placingGamePiece = false;
        }
    }


    if (currentAction == Action::INTAKE) {
        leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 1);
        rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, -1);
        if (intakeSensor.Get()) {
            if (currentPosition == Position::OPEN) {
                gamePieceType = GamePieceType::CUBE;
            }
            else {
                gamePieceType = GamePieceType::CONE;
            }
                
        }
    } 
    else if (currentAction == Action::OUTTAKE) {
        leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, -1);
        rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 1);
        gamePieceType = GamePieceType::NONE;
    } 
    else if (currentAction == Action::IDLE) {
        leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
        rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
    }


    if (currentPosition == Position::OPEN) {
        //Both pistons are extended and the grabber is able to fit a cube.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kForward); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kForward);   
    } 
    else if (currentPosition == Position::AGAPE) {
        //Only the shorter piston is extended and the grabber is able to fit a cone.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kReverse); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kForward);
    } 
    else if (currentPosition == Position::AJAR) {
        //Both pistons are retracted and the grabber is able to squish and transport the cone after intaking it.
        grabberPiston1.Set(frc::DoubleSolenoid::Value::kReverse); 
        grabberPiston2.Set(frc::DoubleSolenoid::Value::kReverse);     
    }
}

void Grabber::setAction(Action action) {
    currentAction = action;
}

void Grabber::setPosition(Position position) {
    currentPosition = position;
}

Grabber::GamePieceType Grabber::getGamePieceType() {
    return gamePieceType;
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
void Grabber::configureMotors() {
    leftIntakeMotor.configFactoryDefault();
    leftIntakeMotor.setIdleMode(ThunderCANMotorController::IdleMode::COAST);
    leftIntakeMotor.configSmartCurrentLimit(INTAKE_MAX_AMPERAGE);
    leftIntakeMotor.setInverted(false);

    leftIntakeMotor.configP(INTAKE_P);
    leftIntakeMotor.configI(INTAKE_I);
    leftIntakeMotor.configD(INTAKE_D);
    leftIntakeMotor.configIZone(INTAKE_I_ZONE);
    leftIntakeMotor.configFF(INTAKE_FF);
    
    rightIntakeMotor.configFactoryDefault();
    rightIntakeMotor.setIdleMode(ThunderCANMotorController::IdleMode::COAST);
    rightIntakeMotor.configSmartCurrentLimit(INTAKE_MAX_AMPERAGE);
    rightIntakeMotor.setInverted(true);

    rightIntakeMotor.configP(INTAKE_P);
    rightIntakeMotor.configI(INTAKE_I);
    rightIntakeMotor.configD(INTAKE_D);
    rightIntakeMotor.configIZone(INTAKE_I_ZONE);
    rightIntakeMotor.configFF(INTAKE_FF);
}

void Grabber::sendFeedback() {

}
