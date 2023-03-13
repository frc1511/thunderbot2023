#pragma once

#include <Basic/Mechanism.h>
#include <frc/Timer.h>
#include <Hardware/HardwareManager.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <functional>

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
        OPEN  = 0, // All the way open - cube intake, placement
        AGAPE = 1, // Semi open - cone intake
        AJAR  = 2, // Not open - carrying cone tightly
    };

    void setPosition(Position position);
    Position getPosition();

    void setWristPosition(bool tipped);
    enum class GamePieceType {
        NONE = -1,
        CUBE = 0,
        CONE = 1,
    };

    GamePieceType getGamePieceType();
    void overrideHasGamePiece(bool hasGamePiece);
    void placeGamePiece();
    void setGamePiece(Grabber::GamePieceType gamePieceType);
    bool isPlacingGamePiece();
    void startIntaking();
    bool isFinishedIntaking();
    void intakeGamePiece();

    void onScore(std::function<void()> callback);
    void onAcquire(std::function<void(GamePieceType)> callback);

private:
    Action currentAction;
    Position currentPosition;
    GamePieceType gamePieceType;
    bool placingGamePiece;
    bool autoIntaking;
    frc::Timer placingGamePieceTimer;
    frc::Timer autoIntakingTimer;
    frc::Timer finishIntakingTimer;
    bool finishIntaking;
    HardwareManager::GrabberIntakeMotor leftIntakeMotor;
    HardwareManager::GrabberIntakeMotor rightIntakeMotor;
#if WHICH_ROBOT != 2022
    frc::DoubleSolenoid grabberPiston1;
    frc::DoubleSolenoid grabberPiston2;
    frc::DoubleSolenoid wristPiston;
#endif
    frc::DigitalInput intakeSensor;
    void configureMotors();
    bool tipped;

    std::function<void()> scoreCallback;
    std::function<void(GamePieceType)> acquireCallback;

    frc::Timer intakeSensorTimer;

    bool intakeSensorTripped = false;
};
