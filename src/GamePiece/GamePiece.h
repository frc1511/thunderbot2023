#pragma once

#include <Basic/Mechanism.h>
#include <GamePiece/Grabber.h>
#include <GamePiece/Lift.h>
#include <map>
#include <utility>

class GamePiece : public Mechanism {
public:
    GamePiece(Grabber* grabber, Lift* lift);
    ~GamePiece();

    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

    // The different presets for the lift.
    enum class LiftPreset {
        INTAKE, // Starting Config / Normal intake.
        INTAKE_FUNKY, // Intake for tipped over cone
        GROUND, // Hybrid Node
        MID,
        HIGH,
        BALCONY, //I dont like the balcony
        TRAVEL, //Just a bit different than intake
    };

    // Sets the pivot and extension of the lift to a preset.
    void setLiftPreset(LiftPreset preset);

    // Returns the current lift preset.
    LiftPreset getLiftPreset();

    // Sets the wrist position (safe - will not break things).
    void setWrist(bool tipped);

    // Sets the wrist position (manual control - is not safe).
    void setWristManual(bool tipped);

    // Sets the state of the grabber rollers to INTAKE, OUTTAKE, or IDLE.
    void setGrabberAction(Grabber::Action action);

    // Sets the position of the grabber sides.
    void setGrabberPosition(Grabber::Position position);

    // Manually set the speed of the motor to control the angle of the lift.
    void setManualPivotSpeed(double speed);

    // Manually set the speed of the motor to control the extension distance of the lift.
    void setManualExtensionSpeed(double speed);

    // Returns which GamePiece is currently being held (or NONE if nothing).
    Grabber::GamePieceType getGamePieceType();

    // Resets the known GamePiece count to zero.
    void overrideHasGamePiece();

    // Performs the actions to place a GamePiece on the aligned grid node.
    void placeGamePiece();

private:
    Grabber* grabber;
    Lift* lift;

    const std::map<LiftPreset, std::pair<units::meter_t, units::meter_t>> presetMap {//ALL TBD!!!!
        { LiftPreset::INTAKE, std::make_pair(1_m, 1_m) },
        { LiftPreset::INTAKE_FUNKY, std::make_pair(1_m, 1_m) },
        { LiftPreset::GROUND, std::make_pair(1_m, 1_m) },
        { LiftPreset::MID, std::make_pair(1_m, 1_m) },
        { LiftPreset::HIGH, std::make_pair(1_m, 1_m) },
        { LiftPreset::BALCONY, std::make_pair(1_m, 1_m) },
        { LiftPreset::TRAVEL, std::make_pair(1_m, 1_m) },
    };

    LiftPreset liftPreset = LiftPreset::INTAKE;
    bool manualWrist = false;
    bool wristTipped = false;
};
