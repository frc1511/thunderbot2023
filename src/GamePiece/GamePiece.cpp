#include <GamePiece/GamePiece.h>
#include <frc/smartdashboard/SmartDashboard.h>


GamePiece::GamePiece(Grabber* _grabber, Lift* _lift) 
: grabber(_grabber), lift(_lift) {

}

GamePiece::~GamePiece() {

}

void GamePiece::resetToMode(MatchMode mode) {

}

void GamePiece::doPersistentConfiguration() {

}

void GamePiece::process() {
    if (getGamePieceType() != Grabber::GamePieceType::NONE) {
        if (liftPreset == LiftPreset::INTAKE || liftPreset == LiftPreset::INTAKE_FUNKY) {
            setLiftPreset(LiftPreset::TRAVEL);
        }
    }

    if (!manualWrist) {
        if (wristTipped) {
            // We don't want the grabber to hit the ground, so when the wrist is tipped the lowest preset is the 'INTAKE_FUNKY' preset.
            if (liftPreset == LiftPreset::INTAKE) {
                setLiftPreset(LiftPreset::INTAKE_FUNKY);
                grabber->setWristPosition(false);
            }
            else {
                // Only move the wrist if the lift is at the correct position.
                grabber->setWristPosition(lift->isAtPosition() && liftPreset == LiftPreset::INTAKE_FUNKY);
            }
        }
        else {
            grabber->setWristPosition(false);
        }
    }
}

void GamePiece::setLiftPreset(LiftPreset preset) {
    const auto& [angle, extension] = presetMap.at(preset);
    lift->setPosition(angle, extension);
    liftPreset = preset;
}

void GamePiece::setWrist(bool tipped) {
    wristTipped = tipped;
    manualWrist = false;
}

void GamePiece::setWristManual(bool tipped) {
    grabber->setWristPosition(tipped);
    wristTipped = tipped;
    manualWrist = true;
}

GamePiece::LiftPreset GamePiece::getLiftPreset() {
    return liftPreset;
}

void GamePiece::setManualPivotSpeed(double speed) {
    lift->setManualPivotSpeed(speed);
}

void GamePiece::setManualExtensionSpeed(double speed) {
    lift->setManualExtensionSpeed(speed);
}

void GamePiece::setGrabberAction(Grabber::Action action) {
    grabber->setAction(action);

    if (action == Grabber::Action::OUTTAKE) {
        grabber->overrideHasGamePiece();
    }
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
        case LiftPreset::BALCONY_PIVOT:
            liftPresetString = "Balcony Pivot";
            break;
        case LiftPreset::GROUND:
            liftPresetString = "Ground";
            break;
        case LiftPreset::HIGH:
            liftPresetString = "High";
            break;
        case LiftPreset::HIGH_PIVOT:
            liftPresetString = "High Pivot";
            break;
        case LiftPreset::INTAKE:
            liftPresetString = "Intake Cone or Cube";
            break;
        case LiftPreset::INTAKE_FUNKY:
            liftPresetString = "Intake Funky Cone";
            break;
        case LiftPreset::MID:
            liftPresetString = "Mid";
            break;
        case LiftPreset::MID_PIVOT:
            liftPresetString = "Mid Pivot";
            break;
        case LiftPreset::TRAVEL:
            liftPresetString = "Travel";
            break;
        }

     frc::SmartDashboard::PutString("GamePiece_LiftPreset", liftPresetString);
}