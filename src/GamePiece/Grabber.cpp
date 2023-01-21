#include <GamePiece/Grabber.h>

Grabber::Grabber() 
: leftIntakeMotor((int)HardwareManager::IOMap::CAN_GRABBER_INTAKE_L),
  rightIntakeMotor((int)HardwareManager::IOMap::CAN_GRABBER_INTAKE_R),
  intakeSensor((int)HardwareManager::IOMap::DIO_GRABBER_INTAKE) {

}

Grabber::~Grabber() {

}

void Grabber::resetToMode(MatchMode mode) {
    currentAction = Action::IDLE;
    leftIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
    rightIntakeMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
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
        //TO DO:Implement extending both pistons.
    } 
    else if (currentPosition == Position::AGAPE) {
        //TO DO:Implement extending only shorter piston.
    } 
    else if (currentPosition == Position::AJAR) {
        //TO DO:Implement closing both pistons.
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
}

void Grabber::placeGamePiece() {
    placingGamePiece = true;
    placingGamePieceTimer.Reset();
    placingGamePieceTimer.Start();
}

void Grabber::sendFeedback() {

}
