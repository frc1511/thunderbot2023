#include <GamePiece/GamePiece.h>
#include <frc/smartdashboard/SmartDashboard.h>


GamePiece::GamePiece(Grabber* _grabber, Lift* _lift) 
: grabber(_grabber), lift(_lift){

}

GamePiece::~GamePiece() {

}

void GamePiece::resetToMode(MatchMode mode) {

}

void GamePiece::doPersistentConfiguration() {

}

void GamePiece::process() {
    if (grabber->getAction() == Grabber::Action::INTAKE) {
        if (getGamePieceType() != Grabber::GamePieceType::NONE) {
            if (liftPreset == LiftPreset::GROUND) {
                setLiftPreset(LiftPreset::TRAVEL);
            }
        }
    }
}

void GamePiece::setLiftPreset(LiftPreset preset) {
    const auto& [y, z] = presetMap.at(preset);
    lift->setEndPosition(y, z);
    liftPreset = preset;
}

void GamePiece::setManualPivotSpeed(double speed) {
    lift->setManualPivotSpeed(speed);
}

void GamePiece::setManualExtensionSpeed(double speed) {
    lift->setManualExtensionSpeed(speed);
}

void GamePiece::setGrabberAction(Grabber::Action action) {
    grabber->setAction(action);
}

void GamePiece::setGrabberPosition(Grabber::Position position) {
    grabber->setPosition(position);
}

Grabber::GamePieceType GamePiece::getGamePieceType() {
    return grabber->getGamePieceType();
}

void GamePiece::overrideHasGamePiece() {
    grabber->overrideHasGamePiece();
}

void GamePiece::placeGamePiece() {
    grabber->placeGamePiece();
}

void GamePiece::sendFeedback() {
    std::string liftPresetString = "";

    switch (liftPreset) {
        case LiftPreset::BALCONY:
            liftPresetString = "Balcony";
            break;
        case LiftPreset::GROUND:
            liftPresetString = "Ground";
            break;
        case LiftPreset::HIGH:
            liftPresetString = "High";
            break;
        case LiftPreset::INTAKE:
            liftPresetString = "Intake";
            break;
        case LiftPreset::MID:
            liftPresetString = "Mid";
            break;
        case LiftPreset::TRAVEL:
            liftPresetString = "Travel";
            break;
        }

     frc::SmartDashboard::PutString("GamePiece_liftpreset", liftPresetString);
}