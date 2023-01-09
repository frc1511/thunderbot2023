#include <Control/Controls.h>
#include <Drive/Drive.h>
#include <GamePiece/GamePiece.h>

Controls::Controls(Drive* _drive, GamePiece* _gamePiece)
: drive(_drive), gamePiece(_gamePiece) {

}

Controls::~Controls() {

}

void Controls::resetToMode(MatchMode mode) {

}

void Controls::doPersistentConfiguration() {

}

void Controls::process() {

}

void Controls::processInDisabled() {
    // Disabled stuff...
}

bool Controls::getShouldPersistConfig() {
    return false;
}

void Controls::sendFeedback() {

}
