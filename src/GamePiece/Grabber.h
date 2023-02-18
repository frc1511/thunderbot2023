#pragma once

#include <Basic/Mechanism.h>
#include <frc/Timer.h>
#include <Hardware/HardwareManager.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>

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
    void setWristPosition(bool tipped);
    enum class GamePieceType {
        CONE,
        CUBE,
        NONE,
    };

    GamePieceType getGamePieceType();
    void overrideHasGamePiece();
    void placeGamePiece();
    void setGamePiece(Grabber::GamePieceType gamePieceType);
    bool isPlacingGamePiece();
    void startIntaking();
    bool isFinishedIntaking();
    void intakeGamePiece();


private:
    Action currentAction;
    Position currentPosition;
    GamePieceType gamePieceType;
    bool placingGamePiece;
    bool autoIntaking;
   // void intakingGamePiece;
    frc::Timer placingGamePieceTimer;
    frc::Timer autoIntakingTimer;
    frc::Timer finishIntakingTimer;
    bool finishIntaking;
    HardwareManager::GrabberIntakeMotor leftIntakeMotor;
    HardwareManager::GrabberIntakeMotor rightIntakeMotor;
    frc::DigitalInput intakeSensor;
    // frc::DoubleSolenoid grabberPiston1;
    // frc::DoubleSolenoid grabberPiston2;
    // frc::DoubleSolenoid wristPiston;
    void configureMotors();
    bool tipped;
};
