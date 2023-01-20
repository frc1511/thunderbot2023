#pragma once

#include <Basic/Mechanism.h>
#include <GamePiece/Grabber.h>
#include <GamePiece/Lift.h>

class GamePiece : public Mechanism {
public:
    GamePiece(Grabber* grabber, Lift* lift);
    ~GamePiece();

    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

    enum class Preset {
        GROUND,
        MID,
        HIGH,
        BALCONY,
    };

    void setPreset(Preset preset);

    enum class GamePieceType {
        CONE,
        CUBE,
        NONE,
    };

    //Sets the configuration of the intake for acquisition for the specified game piece.
    void setGrabberGamePiece(GamePieceType gamepiece, bool topDown);

    //Manually set the speed of the motor to control the angle of the lift.
    void setManualAngleSpeed(double speed);
    //Manually set the speed of the motor to control the extension distance of the lift.
    void setManualExtensionSpeed(double speed);

    //Sets the state of the grabber rollers to INTAKE, OUTTAKE, or IDLE.
    void setGrabberAction(Grabber::Action action);

    GamePieceType getGamePiece();

    void overrideHasGamePiece(bool hasGamePiece);

    //Tells the end effector to perform the action of placing a game piece on the alligned grid node.
    void placeGamePiece();
private:

    Grabber* grabber;
    Lift* lift;
};
