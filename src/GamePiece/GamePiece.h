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
        INTAKE,        // Starting position & cone/cube intake.
        INTAKE_FUNKY,  // Intake position for preparing to intake a tipped cone.
        TIPPED_CONE,   // Tipped over cone intake position.
        GROUND,        // Low scoring position.
        MID,           // Mid scoring position.
        MID_PIVOT,     // Mid scoring position - only pivot.
        HIGH,          // High scoring position.
        HIGH_PIVOT,    // High scoring position - only pivot.
        BALCONY,       // Balcony scoring position.
        BALCONY_PIVOT, // Balcony scoring position - only pivot.
        TRAVEL,        // Travel position.
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

    bool liftAtPosition();

    void setGamePiece(Grabber::GamePieceType gamePieceType);

    bool isFinishedScoring();

    void intakeGamePiece();

    bool isFinishedIntaking();

private:
    Grabber* grabber;
    Lift* lift;

    const std::map<LiftPreset, std::pair<units::degree_t, units::meter_t>> presetMap {
        { LiftPreset::INTAKE,       std::make_pair(-40_deg, 0_m)      }, // Starting position & cone/cube intake.
        { LiftPreset::INTAKE_FUNKY, std::make_pair(-28_deg, 0.168_m)  }, // Intake position for preparing to intake a tipped cone.
        { LiftPreset::TIPPED_CONE,  std::make_pair(-28_deg, 0.168_m)  }, // Tipped over cone intake position.
        { LiftPreset::GROUND,       std::make_pair(-25_deg, 0.1_m)    }, // Low scoring position.
        { LiftPreset::MID,          std::make_pair(0_deg,   38.25_in) }, // Mid scoring position.
        { LiftPreset::MID_PIVOT,    std::make_pair(0_deg,   0_m)      }, // Mid scoring position - only pivot.
        { LiftPreset::HIGH,         std::make_pair(20_deg,  38.25_in) }, // High scoring position.
        { LiftPreset::HIGH_PIVOT,   std::make_pair(20_deg,  0_m)      }, // High scoring position - only pivot.
        { LiftPreset::BALCONY,      std::make_pair(18_deg,  30_in)    }, // Balcony scoring position.
        { LiftPreset::BALCONY_PIVOT,std::make_pair(18_deg,  0_m)      }, // Balcony scoring position - only pivot.
        { LiftPreset::TRAVEL,       std::make_pair(-25_deg, 0_m)      }, // Travel position.
    };

    // hi jeff

    LiftPreset liftPreset = LiftPreset::INTAKE;
    bool manualWrist = false;
    bool wristTipped = false;
};
