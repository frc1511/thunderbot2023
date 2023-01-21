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

    enum class LiftPreset {
        GROUND,
        MID,
        HIGH,
        BALCONY,
    };

    void setLiftPreset(LiftPreset preset);


    // Represents the possible positions for acquisition.
    enum class AcquisitionPosition {
        GROUND, // Normal ground intake - cubes and upright cones.
        GROUND_TOP_DOWN, // Top-down ground intake - tipped over cones.
        BALCONY, // Intake from Human Player station balcony.
    };

    /**
     * Configures the states of the grabbber and lift mechanisms to prepare the
     * robot for accquisition of a specified GamePiece. Sets the spacing between
     * grabbers, position of lift, etc. to make it ready to aquire.
     * 
     * gamePieceType: The type of GamePiece to configure for (Cube or Cone).
     * acquisitionPosition: The position of acquisition.
     */
    void prepareForAcquisition(Grabber::GamePieceType gamePieceType, AcquisitionPosition acquisitionPosition);

    // Sets the state of the grabber rollers to INTAKE, OUTTAKE, or IDLE.
    void setGrabberAction(Grabber::Action action);

    // Manually set the speed of the motor to control the angle of the lift.
    void setManualAngleSpeed(double speed);

    // Manually set the speed of the motor to control the extension distance of the lift.
    void setManualExtensionSpeed(double speed);

    // Returns which GamePiece is currently being held (or NONE if nothing).
    Grabber::GamePieceType getGamePiece();

    void overrideHasGamePiece(bool hasGamePiece);

    /**
     * Performs the actions to place a GamePiece on the aligned grid node.
     */
    void placeGamePiece();

private:
    Grabber* grabber;
    Lift* lift;
    const std::map<LiftPreset, std::pair<units::meter_t, units::meter_t>> presetMap;
};
