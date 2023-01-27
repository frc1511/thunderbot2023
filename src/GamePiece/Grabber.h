#pragma once

#include <Basic/Mechanism.h>
#include <frc/Timer.h>
#include <Hardware/HardwareManager.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>

class Grabber : public Mechanism {
public:
    Grabber();
    ~Grabber();

    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

    enum class Action {
        IDLE,
        INTAKE,
        OUTTAKE,
    };

    void setAction(Action action);

    Action getAction();

    enum class Position {
        OPEN, //All the way open - cube intake, placement
        AGAPE, //Semi open - cone intake
        AJAR, //Not open - carrying cone tightly
    };

    void setPosition(Position position);

    enum class GamePieceType {
        CONE,
        CUBE,
        NONE,
    };

    GamePieceType getGamePieceType();
    void overrideHasGamePiece();
    void placeGamePiece();

private:
    Action currentAction;
    Position currentPosition;
    GamePieceType gamePieceType;
    bool placingGamePiece;
    frc::Timer placingGamePieceTimer;
    HardwareManager::GrabberIntakeMotor leftIntakeMotor;
    HardwareManager::GrabberIntakeMotor rightIntakeMotor;
    frc::DigitalInput intakeSensor;
    frc::DoubleSolenoid grabberPiston1;
    frc::DoubleSolenoid grabberPiston2;
    void configureMotors();
};
