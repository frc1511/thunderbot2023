#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <frc/GenericHID.h>
#include <Hardware/IOMap.h>

class Drive;
class GamePiece;
class UltraBrickMode;
class BlinkyBlinky;
class Autonomous;

class Controls : public Mechanism {
public:
    Controls(Drive* drive, GamePiece* gamePiece, UltraBrickMode* ultraBrickMode, BlinkyBlinky* blinkyBlinky
#if WHICH_ROBOT == 2023
    , Autonomous* autonomous
#endif
    );
    ~Controls();

    void resetToMode(MatchMode mode) override;
    void process() override;
    void sendFeedback() override;

    void processInDisabled();
    void processInAutonomous();
    bool getShouldPersistConfig();

private:
    Drive* drive;
    GamePiece* gamePiece;
    UltraBrickMode* ultraBrickMode;
    BlinkyBlinky* blinkyBlinky;
#if WHICH_ROBOT == 2023
    Autonomous* autonomous;
#endif
    HardwareManager::DriveGameController driveController { ThunderGameController::Controller::DRIVER };
    HardwareManager::AuxGameController auxController { ThunderGameController::Controller::AUX };
    frc::GenericHID switchPanel {2};
    void doDrive();
    void doAux();
    void doAuxManual();
    void doSwitchPanel();
    bool driveLockX = false;
    bool manualAux = false;
    bool doUltraBrickMode = false;

    bool wasScoring = false;

    bool driveAligning = false;

    bool driveRobotCentric = false;
    unsigned driveCtrlFlags = 0;

    bool driveRecording = false;
    bool driveAbsRotation = false;
    units::radian_t driveAbsAngle = 0_deg;

    bool manualWrist = false;

    bool callaDisable = false;
    bool haileyDisable = false;
};
