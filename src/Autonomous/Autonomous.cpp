#include <Autonomous/Autonomous.h>
#include <Drive/Drive.h>
#include <WhooshWhoosh/WhooshWhoosh.h>
#include <GamePiece/GamePiece.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <fmt/core.h>
#include <string>

Autonomous::Autonomous(WhooshWhoosh* _whooshWhoosh, Drive* _drive, GamePiece* _gamePiece)
: whooshWhoosh(_whooshWhoosh), drive(_drive), gamePiece(_gamePiece) { }

Autonomous::~Autonomous() = default;

void Autonomous::resetToMode(MatchMode mode) {
    delayTimer.Reset();
    delayTimer.Start();
    autoTimer.Reset();
    autoTimer.Start();

    step = 0;
}

void Autonomous::process() {
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        paths = &redPaths;
    }
    else {
        paths = &bluePaths;
    }

    // Autonomous delay.
    if (delayTimer.Get().value() <= frc::SmartDashboard::GetNumber("thunderdashboard_auto_start_delay", 0.0)) {
        return;
    }

    selectedAutoMode = static_cast<AutoMode>(frc::SmartDashboard::GetNumber("Auto_Mode", 0.0));

    switch (selectedAutoMode) {
        case AutoMode::DO_NOTHING:
            doNothing();
            break;
        case AutoMode::BARRIER_2GP:
            barrier2GP(false);
            break;
        case AutoMode::BARRIER_2GP_CS:
            barrier2GP(true);
            break;
        case AutoMode::CENTER_1GP:
            center1GP(false);
            break;
        case AutoMode::CENTER_1GP_CS:
            center1GP(true);
            break;
        case AutoMode::EDGE_1GP:
            edge1GP(false);
            break;
        case AutoMode::EDGE_1GP_MOB:
            edge1GP(false);
            break;
    }

}

void Autonomous::doNothing() {
    // If it does nothing is it doing something or nothing? - trevor(2020)
        //it does something because it is doing nothing - ishan(2022)
        //I disagree - peter(2022)
        //I agree with peter -L Wrench
        //I still disagree with ishan - peter(2023)

    // Good function.
    // Very good function. - jeff downs
    // Very bad function. - jeff ups

    // Well technically it's doing something - chris(2023)
}

void Autonomous::barrier2GP(bool withCS) {
    // Score preloaded cube high.
    if (step == 0) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
        gamePiece->overrideHasGamePiece(true);
        step += (scoreAction.process() == Action::Result::DONE);
    }
    else if (step >= 1 && step <= 5) {
        step++; // Give it some time...
    }
    // Reset odometry and drive to field cone 1.
    else if (step == 6) {
        auto init = paths->at(Path::BARRIER_1).getInitialPose();
        drive->resetOdometry(frc::Pose2d(init.X(), init.Y(), init.Rotation().Degrees() - 90_deg));
        drive->runTrajectory(&paths->at(Path::BARRIER_1), actions);
        step++;
    }
    // Start intaking cone while driving.
    else if (step == 7 && !drive->isFinished()) {
        if (drive->getTrajectoryTime() > 2_s && drive->getTrajectoryTime() < 2.1_s) {
            gamePiece->setGrabberPosition(Grabber::Position::AGAPE);
            gamePiece->setGrabberAction(Grabber::Action::INTAKE);
        }
    }
    // Stop intaking, drive back to grid with acquired cone.
    else if (step == 7 && drive->isFinished()) {
        gamePiece->overrideHasGamePiece(true);
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
        drive->runTrajectory(&paths->at(Path::BARRIER_2), actions);
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);
        step++;
    }
    // Score cone.
    else if (step == 8 && drive->isFinished()) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
        step += (scoreAction.process() == Action::Result::DONE);
    }
    else if (step >= 9 && step <= 13) {
        step++; // Give it some time...
    }
    // Drive to field cone 2.
    else if (step == 14) {
        drive->runTrajectory(&paths->at(Path::BARRIER_3), actions);
        step++;
    }
    // Start intaking cone while driving.
    else if (step == 15 && !drive->isFinished()) {
        if (drive->getTrajectoryTime() > 2_s && drive->getTrajectoryTime() < 2.1_s) {
            gamePiece->setGrabberPosition(Grabber::Position::AGAPE);
            gamePiece->setGrabberAction(Grabber::Action::INTAKE);
        }
    }
    else if (step == 15 && drive->isFinished()) {
        if (withCS) {
            drive->runTrajectory(&paths->at(Path::BARRIER_FINAL_BALANCE), actions);
            step++;
        }
        else {
            step++;
        }
    }
    else if (step == 16 && drive->isFinished()) {
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
        auto pose = drive->getEstimatedPose();
        drive->resetOdometry(frc::Pose2d(pose.X(), pose.Y(), pose.Rotation().Degrees() + 90_deg));
    }
}

void Autonomous::center1GP(bool withCS) {
    // This will be such a HUGE help at FLR this week! Best of luck and great work so far - you got this! CG
    // Score preloaded cube high, reset odometry.
    if (step == 0) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
        gamePiece->overrideHasGamePiece(true);
        step += (scoreAction.process() == Action::Result::DONE);

        auto init = paths->at(Path::BARRIER_1).getInitialPose();
        drive->resetOdometry(frc::Pose2d(init.X(), init.Y(), init.Rotation().Degrees() - 90_deg));

        liftTimer.Reset();
        liftTimer.Start();
    }
    // Wait a little bit for the lift.
    else if (step == 1 && liftTimer.Get() >= 0.5_s) {
        step++;
    }
    // Balance with mobility.
    else if (step == 2) {
        balanceMobilityAction.process();
    }
}

void Autonomous::edge1GP(bool withMob) {
    if (step == 0) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
        gamePiece->overrideHasGamePiece(true);
        step += (scoreAction.process() == Action::Result::DONE);
    }
    else if (step >= 1 && step <= 5) {
        step++; // Give it some time...
    }
    // Reset odometry and drive to field cone 1.
    else if (step == 6) {
        auto init = paths->at(Path::EDGE_1).getInitialPose();
        drive->resetOdometry(frc::Pose2d(init.X(), init.Y(), init.Rotation().Degrees() - 90_deg));
        drive->runTrajectory(&paths->at(Path::EDGE_1), actions);
        step++;
    }
}

// --- Score Action ---

Autonomous::ScoreAction::ScoreAction(GamePiece* _gamePiece)
: gamePiece(_gamePiece) { }

Autonomous::ScoreAction::~ScoreAction() = default;

Action::Result Autonomous::ScoreAction::process() {
    if (step == 0) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);
        step++;
    }
    else if (step == 1 && gamePiece->liftAtPosition()) {
        gamePiece->placeGamePiece();
        step++;
    }
    else if (step >= 2 && step <= 6) {
        step++;
    }
    else if (step == 7 && gamePiece->isFinishedScoring()) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
        step = 0;
        return Result::DONE;
    }

    return Result::WORKING;
}

// --- Balance Action ---

Autonomous::BalanceAction::BalanceAction(Drive* _drive, WhooshWhoosh* _whooshWhoosh)
: drive(_drive), whooshWhoosh(_whooshWhoosh) { }

Autonomous::BalanceAction::~BalanceAction() = default;

Action::Result Autonomous::BalanceAction::process() {
    // Drive forwards until tilted.
    if (step == 0) {
        if (whooshWhoosh->getTiltAngle() <= -10_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(-1.4_mps, 0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Enable balancing.
    else if (step == 1) {
        static bool firstRun = true;
        if (firstRun) {
            // whooshWhoosh->resetAntiTiltPIDController();
            firstRun = false;
        }

        units::meters_per_second_t antiTiltVel = whooshWhoosh->calculateAntiTiltDriveVelocity();
        fmt::print("tilt: {}, vel: {}\n", whooshWhoosh->getTiltAngle().value(), -antiTiltVel.value());
        unsigned flags = Drive::ControlFlag::FIELD_CENTRIC;
        if (units::math::abs(antiTiltVel) <= 0.1_mps) {
            antiTiltVel = 0.0_mps;
            flags |= Drive::ControlFlag::BRICK;
        }

        drive->velocityControlAbsRotation(-antiTiltVel, 0_mps, 90_deg, flags);
    }

    return Result::WORKING;
}

// --- Balance w/ Mobility Action ---

Autonomous::BalanceMobilityAction::BalanceMobilityAction(Drive* _drive, WhooshWhoosh* _whooshWhoosh, BalanceAction* _balanceAction)
: drive(_drive), whooshWhoosh(_whooshWhoosh), balanceAction(_balanceAction) { }

Autonomous::BalanceMobilityAction::~BalanceMobilityAction() = default;

Action::Result Autonomous::BalanceMobilityAction::process() {
    // Drive forwards until almost off charge station.
    if (step == 0) {
        if (whooshWhoosh->getTiltAngle() >= 10_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(1.7_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Check when grounded on other side.
    else if (step == 1) {
        if (whooshWhoosh->getTiltAngle() < 0_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(1.3_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    else if (step == 2) {
        if (units::math::abs(whooshWhoosh->getTiltAngle()) <= 0.1_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(0.9_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Drive back on the Charge Station.
    else if (step == 3) {
        stopTimer.Reset();
        stopTimer.Start();
        drive->velocityControlAbsRotation(0.2_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        step++;
    }
    else if (step == 4 && (stopTimer.Get() > 1_s)) {
        step++;
    }
    else if (step == 5) {
        balanceAction->process();
    }

    return Result::WORKING;
}

void Autonomous::sendFeedback() {
    frc::SmartDashboard::PutNumber("Autonomous_Step", step);
    frc::SmartDashboard::PutBoolean("Autonomous_DriveFinished", drive->isFinished());

    std::string buffer = "";

    auto handleDashboardString = [&](AutoMode mode, const char* description) {
        int mode_index = static_cast<int>(mode);
        // Put mode index in buffer.
        buffer += fmt::format(",{}", mode_index);
        // Send description.
        frc::SmartDashboard::PutString(fmt::format("thunderdashboard_auto_{}", mode_index), description);
    };

    for (auto [mode, name] : autoModeNames) {
        handleDashboardString(mode, name);
    }

    frc::SmartDashboard::PutString("thunderdashboard_auto_list", buffer);
}
