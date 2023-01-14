#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <frc/GenericHID.h>

class Drive;
class GamePiece;

class Controls : public Mechanism {
public:
    Controls(Drive* drive, GamePiece* gamePiece);
    ~Controls();

    void resetToMode(MatchMode mode) override;
    void process() override;
    void sendFeedback() override;

    void processInDisabled();
    bool getShouldPersistConfig();

private:
    Drive* drive;
    GamePiece* gamePiece;
    frc::GenericHID driveController {0};
    frc::GenericHID auxController {1};
    frc::GenericHID switchPanel {2};
    void doDrive();
    void doAux();
    void doSwitchPanel();
    bool driveLockX = false;

    bool driveRobotCentric = false;
    unsigned driveCtrlFlags = 0;

    bool driveRecording = false;
    bool driveAbsRotation = false;
    units::radian_t driveAbsAngle = 0_deg;


};
