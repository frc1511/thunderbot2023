#pragma once

#include <frc/TimedRobot.h>
#include <Basic/Mechanism.h>
#include <vector>

#include <Control/Controls.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <Drive/Drive.h>
#include <GamePiece/GamePiece.h>

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

    RollingRaspberry rollingRaspberry;
    Drive drive { &rollingRaspberry };
    GamePiece gamePiece;
    Controls controls { &drive, &gamePiece };

    // Every mechanism on the robot.
    std::vector<Mechanism*> allMechanisms {
        &drive, &gamePiece, &controls, &rollingRaspberry
    };

    // Mechanisms that are run universally when the robot is running.
    std::vector<Mechanism*> universalMechanisms {
        &rollingRaspberry,
    };
};
