#pragma once

#include <frc/TimedRobot.h>
#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <vector>

#include <Autonomous/Autonomous.h>
#include <WhooshWhoosh/WhooshWhoosh.h>
#include <Control/Controls.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <Drive/Drive.h>
#include <GamePiece/GamePiece.h>
#include <Basic/RobotChess.h>
#include <Drive/UltraBrickMode.h>
#include <Illumination/BlinkyBlinky.h>
#include <Vision/IntakeCamera.h>

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

private:
    void reset(Mechanism::MatchMode mode);

    Mechanism::MatchMode lastMode = Mechanism::MatchMode::DISABLED;

    RobotChess robotChess;

    WhooshWhoosh whooshWhoosh;

    RollingRaspberry rollingRaspberry;
    Drive drive { &whooshWhoosh, &rollingRaspberry };
    Grabber grabber;
    Lift lift;
    GamePiece gamePiece { &grabber, &lift};
    UltraBrickMode ultraBrickMode;
    BlinkyBlinky blinkyBlinky;
    Controls controls { &drive, &gamePiece, &ultraBrickMode, &blinkyBlinky };
    Autonomous autonomous { &whooshWhoosh, &drive, &gamePiece };

    IntakeCamera intakeCamera;

    // Every mechanism on the robot.
    std::vector<Mechanism*> allMechanisms {
        &whooshWhoosh, &drive, &gamePiece, &controls, &rollingRaspberry, &grabber, &lift, &ultraBrickMode, &autonomous, &blinkyBlinky, &intakeCamera
    };

    // Mechanisms that are run universally when the robot is running.
    std::vector<Mechanism*> universalMechanisms {
        &whooshWhoosh, &rollingRaspberry, &blinkyBlinky, &intakeCamera
    };
};
