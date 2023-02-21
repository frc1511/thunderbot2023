#include <Basic/Robot.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

void Robot::RobotInit() {
    // Send feedback every 40 ms.
    AddPeriodic([&]() {
        for (Mechanism* mech : allMechanisms) {
            mech->sendFeedback();
        }

        frc::SmartDashboard::PutNumber("thunderdashboard_match_remaining", frc::DriverStation::GetMatchTime());
    }, 40_ms);
}

void Robot::RobotPeriodic() {
    for (Mechanism* mech : universalMechanisms) {
        mech->process();
    }
}

void Robot::AutonomousInit() {
    reset(Mechanism::MatchMode::AUTO);
}

void Robot::AutonomousPeriodic() {
    drive.process();
    gamePiece.process();
    lift.process();
    grabber.process();
}

void Robot::TeleopInit() {
    reset(Mechanism::MatchMode::TELEOP);
}

void Robot::TeleopPeriodic() {
    controls.process();
    drive.process();
    gamePiece.process();
    lift.process();
    grabber.process();
}

void Robot::DisabledInit() {
    reset(Mechanism::MatchMode::DISABLED);
}

void Robot::DisabledPeriodic() {
    controls.processInDisabled();
}

void Robot::TestInit() {
    if (controls.getShouldPersistConfig()) {
        fmt::print("*** Persistent configuration activating...\n");
        for (Mechanism* mech : allMechanisms) {
          mech->doPersistentConfiguration();
        }
        fmt::print("*** Persistent configuration complete!\n");
    }
    reset(Mechanism::MatchMode::TEST);
}

void Robot::TestPeriodic() { }

void Robot::reset(Mechanism::MatchMode mode) {
    for (Mechanism* mech : allMechanisms) {
        mech->callResetToMode(lastMode);
    }

    lastMode = mode;
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
