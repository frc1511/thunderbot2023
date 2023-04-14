#include <GamePiece/GamePiece.h>
#include <frc/smartdashboard/SmartDashboard.h>


GamePiece::GamePiece(Grabber* _grabber, Lift* _lift) 
: grabber(_grabber), lift(_lift) {
    // Pivot down after scoring.
    grabber->onScore([this]() {
        setWrist(false);
        setLiftPreset(LiftPreset::INTAKE);
    });
    // Pivot up after acquisition.
    grabber->onAcquire([this](Grabber::GamePieceType type) {
        setWrist(false);
        if (liftPreset == LiftPreset::BALCONY || liftPreset == LiftPreset::SLIDE) {
            balconyWaiting = true;
            balconyWaitingTimer.Reset();
            balconyWaitingTimer.Start();
        }
        else {
            setLiftPreset(LiftPreset::TRAVEL);
        }
    });
}

GamePiece::~GamePiece() {

}

void GamePiece::resetToMode(MatchMode mode) {

}

void GamePiece::doPersistentConfiguration() {

}

void GamePiece::process() {
    if (balconyWaiting && balconyWaitingTimer.Get() > 0.75_s) {
        balconyWaiting = false;
        setLiftPreset(LiftPreset::TRAVEL);
    }

    if (!manualWrist) {
        if (wristTipped) {
            // We don't want the grabber to hit the ground, so when the wrist is tipped the lowest preset is the 'TIPPED_CONE' preset.
            if (liftPreset == LiftPreset::INTAKE) {
                setLiftPreset(LiftPreset::TIPPED_CONE);
                grabber->setWristPosition(false);
            }
            else {
                if (liftPreset == LiftPreset::TIPPED_CONE) {
                    grabber->setWristPosition(lift->isAtPosition());
                }
                else if (liftPreset == LiftPreset::BALCONY) {
                    grabber->setWristPosition(lift->getPivotAngle() > -10_deg);
                }
                else {
                    grabber->setWristPosition(false);
                }
            }
        }
        else {
            grabber->setWristPosition(false);
        }
    }
}

void GamePiece::setLiftPreset(LiftPreset preset) {
    if (preset == LiftPreset::GROUND) {
        preset = getGamePieceType() == Grabber::GamePieceType::CUBE ? LiftPreset::GROUND_CUBE : LiftPreset::GROUND_CONE;
    }
    if (preset == LiftPreset::MID) {
        preset = getGamePieceType() == Grabber::GamePieceType::CUBE ? LiftPreset::MID_CUBE : LiftPreset::MID_CONE;
    }

    if (preset == LiftPreset::HIGH) {
        preset = getGamePieceType() == Grabber::GamePieceType::CUBE ? LiftPreset::HIGH_CUBE : LiftPreset::HIGH_CONE;
    }

    if (preset == LiftPreset::MID_PIVOT) {
        preset = getGamePieceType() == Grabber::GamePieceType::CUBE ? LiftPreset::MID_CUBE_PIVOT : LiftPreset::MID_CONE_PIVOT;
    }

    if (preset == LiftPreset::HIGH_PIVOT) {
        preset = getGamePieceType() == Grabber::GamePieceType::CUBE ? LiftPreset::HIGH_CUBE_PIVOT : LiftPreset::HIGH_CONE_PIVOT;
    }

    const auto& [angle, extension] = presetMap.at(preset);

    if (preset == LiftPreset::BALCONY) {
        setWrist(true);
    }
    else if (preset != LiftPreset::TIPPED_CONE) {
        setWrist(false);
    }

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
    switch (liftPreset) {
        case LiftPreset::GROUND_CONE:
        case LiftPreset::GROUND_CUBE:
            return LiftPreset::GROUND;
        case LiftPreset::MID_CONE:
        case LiftPreset::MID_CUBE:
            return LiftPreset::MID;
        case LiftPreset::HIGH_CONE:
        case LiftPreset::HIGH_CUBE:
            return LiftPreset::HIGH;
        case LiftPreset::MID_CONE_PIVOT:
        case LiftPreset::MID_CUBE_PIVOT:
            return LiftPreset::MID_PIVOT;
        case LiftPreset::HIGH_CONE_PIVOT:
        case LiftPreset::HIGH_CUBE_PIVOT:
            return LiftPreset::HIGH_PIVOT;
        default:
            return liftPreset;
    }
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
        grabber->overrideHasGamePiece(false);
    }
}

void GamePiece::setGrabberPosition(Grabber::Position position) {
    grabber->setPosition(position);
}

Grabber::GamePieceType GamePiece::getGamePieceType() {
    return grabber->getGamePieceType();
}

void GamePiece::overrideHasGamePiece(bool hasGamePiece) {
    grabber->overrideHasGamePiece(hasGamePiece);
}

void GamePiece::placeGamePiece() {
    grabber->placeGamePiece();
}

void GamePiece::setGamePiece(Grabber::GamePieceType type) {
    grabber->setGamePiece(type);
}

bool GamePiece::liftAtPosition() {
    return lift->isAtPosition();
}

bool GamePiece::isFinishedScoring() {
    return !grabber->isPlacingGamePiece();
}

bool GamePiece::isFinishedIntaking() {
    return grabber->isFinishedIntaking();
}

void GamePiece::overrideLiftKindaBroken() {
    lift->resetLiftBrokenKinda();
}

void GamePiece::resetLiftPIDController() {
    lift->resetPIDController();
}

void GamePiece::setLiftMaxPivotEncoder(double rotations) {
    lift->setMaxPivotEncoder(rotations);
}

void GamePiece::intakeGamePiece() {
    grabber->intakeGamePiece();
}

void GamePiece::sendFeedback() {
    std::string liftPresetString = "";

    switch (liftPreset) {
        case LiftPreset::INTAKE:
            liftPresetString = "Intake";
            break;
        case LiftPreset::INTAKE_FUNKY:
            liftPresetString = "Intake Funky";
            break;
        case LiftPreset::TIPPED_CONE:
            liftPresetString = "Tipped Cone";
            break;
        case LiftPreset::GROUND:
            liftPresetString = "Ground";
            break;
        case LiftPreset::GROUND_CONE:
            liftPresetString = "Ground Cone";
            break;
        case LiftPreset::GROUND_CUBE:
            liftPresetString = "Ground Cube";
            break;
        case LiftPreset::MID:
            liftPresetString = "Mid";
            break;
        case LiftPreset::MID_PIVOT:
            liftPresetString = "Mid Pivot";
            break;
        case LiftPreset::HIGH:
            liftPresetString = "High";
            break;
        case LiftPreset::HIGH_PIVOT:
            liftPresetString = "High Pivot";
            break;
        case LiftPreset::SLIDE:
            liftPresetString = "Slide";
            break;
        case LiftPreset::BALCONY:
            liftPresetString = "Balcony";
            break;
        case LiftPreset::BALCONY_PIVOT:
            liftPresetString = "Balcony Pivot";
            break;
        case LiftPreset::TRAVEL:
            liftPresetString = "Travel";
            break;
        case LiftPreset::MID_CONE:
            liftPresetString = "Mid Cone";
            break;
        case LiftPreset::MID_CUBE:
            liftPresetString = "Mid Cube";
            break;
        case LiftPreset::MID_CONE_PIVOT:
            liftPresetString = "Mid Cone Pivot";
            break;
        case LiftPreset::MID_CUBE_PIVOT:
            liftPresetString = "Mid Cube Pivot";
            break;
        case LiftPreset::HIGH_CONE:
            liftPresetString = "High Cone";
            break;
        case LiftPreset::HIGH_CUBE:
            liftPresetString = "High Cube";
            break;
        case LiftPreset::HIGH_CONE_PIVOT:
            liftPresetString = "High Cone Pivot";
            break;
        case LiftPreset::HIGH_CUBE_PIVOT:
            liftPresetString = "High Cube Pivot";
            break;
        case LiftPreset::AUTO_JANKY:
            liftPresetString = "Auto Janky";
            break;
        case LiftPreset::AUTO_JANKY_LOWER:
            liftPresetString = "Auto Janky Lower";
            break;
        }

     frc::SmartDashboard::PutString("GamePiece_LiftPreset", liftPresetString);
}