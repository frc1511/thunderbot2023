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

}

void GamePiece::prepareForAcquisition(Grabber::GamePieceType gamePieceType, AcquisitionPosition aquisitionPosition) {

}

void GamePiece::setManualAngleSpeed(double speed) {

}
void GamePiece::setManualExtensionSpeed(double speed) {

}

void GamePiece::setGrabberAction(Grabber::Action action) {

}

Grabber::GamePieceType GamePiece::getGamePiece() {

}

void GamePiece::overrideHasGamePiece(bool hasGamePiece) {

}

void GamePiece::placeGamePiece() {
    
}

void GamePiece::sendFeedback() {

}
