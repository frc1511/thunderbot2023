#include <GamePiece/GamePiece.h>

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

}

void GamePiece::setLiftPreset(LiftPreset preset) {
    lift->setEndPosition(presetMap.at(preset).first, presetMap.at(preset).second);
}

void GamePiece::prepareForAcquisition(Grabber::GamePieceType gamePieceType, AcquisitionPosition aquisitionPosition) {

}

void GamePiece::setManualAngleSpeed(double speed) {
    lift->setManualAngleSpeed(speed);
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

Grabber::GamePieceType GamePiece::getGamePiece() {
    return grabber->getGamePieceType();
}

void GamePiece::overrideHasGamePiece(bool hasGamePiece) {
    grabber->overrideHasGamePiece();
}

void GamePiece::placeGamePiece() {
    grabber->placeGamePiece();
}

void GamePiece::sendFeedback() {

}
