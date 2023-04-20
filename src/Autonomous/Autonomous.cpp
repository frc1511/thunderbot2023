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

    if (mode == MatchMode::AUTO) {
        whooshWhoosh->resetHeading();
        whooshWhoosh->resetTilt();
    }

    static bool wasAuto = false;

    if (getLastMode() == Mechanism::MatchMode::AUTO && mode == Mechanism::MatchMode::DISABLED) {
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed && selectedAutoMode == AutoMode::CENTER_1GP_CS) {
            frc::Pose2d currPose(drive->getEstimatedPose());
            drive->resetOdometry(frc::Pose2d(currPose.X(), currPose.Y(), 180_deg + currPose.Rotation().Degrees()));
        }
    }
}

void Autonomous::process() {
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed && selectedAutoMode != AutoMode::CENTER_1GP_CS) {
        paths = &redPaths;
    }
    else {
        paths = &bluePaths;
    }

    // Autonomous delay.
    if (delayTimer.Get().value() <= frc::SmartDashboard::GetNumber("thunderdashboard_auto_start_delay", 0.0)) {
        return;
    }

    switch (selectedAutoMode) {
        case AutoMode::DO_NOTHING:
            doNothing();
            break;
        case AutoMode::DRIVE_FORWARDS:
            driveForwards();
            break;
        case AutoMode::SCORE:
            score();
            break;
        case AutoMode::BARRIER_2GP:
        // case AutoMode::BARRIER_2GP_CS:
        // case AutoMode::BARRIER_3GP:
            barrier();
            break;
        case AutoMode::CENTER_1GP_CS:
            center();
            break;
        case AutoMode::EDGE_2GP:
            edge();
            break;
        case AutoMode::SCORE_BACKUP:
            scoreBackup();
            break;
    }
}

bool Autonomous::isBalancing() {
    if (getCurrentMode() != MatchMode::AUTO) {
        return false;
    }

    switch (selectedAutoMode) {
        // case AutoMode::BARRIER_2GP_CS:
        //     return step >= 18;
        case AutoMode::CENTER_1GP_CS:
            return step >= 2;
        default:
            return false;
    }
    return false;



    // Hi Larry!!!
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

void Autonomous::barrier() {
    // Score preloaded cube high.
    if (step == 0) {
        if (selectedAutoMode == AutoMode::BARRIER_2GP) {
            scoreAction.setLevel(2);
        }
        else {
            scoreAction.setLevel(0);
        }
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
        gamePiece->overrideHasGamePiece(true);
        step += (scoreAction.process() == Action::Result::DONE);
    }
    else if (step >= 1 && step <= 5) {
        step++; // Give it some time...
    }
    // Reset odometry and drive to field cone 1.
    else if (step == 6) {
        scoreAction.setLevel(2);
        frc::Pose2d initPose(paths->at(Path::BARRIER_1).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
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
        gamePiece->overrideHasGamePiece(true);
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
        step++;
    }
    else if (step >= 16) {
        // switch (selectedAutoMode) {
        //     case AutoMode::BARRIER_2GP_CS:
        //         barrierFinish2GPCS();
        //         break;
        //     case AutoMode::BARRIER_3GP:
        //         barrierFinish3GP();
        //         break;
        //     default:
        //         break;
        // }
    }
}

void Autonomous::barrierFinish2GPCS() {
    if (step == 16) {
        drive->runTrajectory(&paths->at(Path::BARRIER_FINAL_BALANCE), actions);
        step++;
    }
    else if (step == 17 && drive->isFinished()) {
        step++;
    }
    else if (step == 18) {
        balanceAction.process();
    }
}

void Autonomous::barrierFinish3GP() {
    if (step == 16) {
        drive->runTrajectory(&paths->at(Path::BARRIER_FINAL_SCORE), actions);
        gamePiece->setLiftPreset(GamePiece::LiftPreset::MID);
        step++;
    }
    else if (step == 17 && drive->isFinished()) {
        step++;
    }
    else if (step == 18) {
        step += (scoreAction.process() == Action::Result::DONE);
    }
    else if (step == 19) {
        gamePiece->overrideHasGamePiece(false);
    }
}

void Autonomous::center() {
    // This will be such a HUGE help at FLR this week! Best of luck and great work so far - you got this! CG
    // Score preloaded cube high, reset odometry.
    if (step == 0) {
        if (settings.liftActive) {
            gamePiece->setGrabberPosition(Grabber::Position::OPEN);
            gamePiece->overrideHasGamePiece(true);
            step += (scoreAction.process() == Action::Result::DONE);

            liftTimer.Reset();
            liftTimer.Start();
        }
        else {
            step = 2;
            // Hi Ishan!!!
        }

        frc::Pose2d initPose(paths->at(Path::BARRIER_1).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
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

void Autonomous::edge() {
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
        frc::Pose2d initPose(paths->at(Path::EDGE_1).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        drive->runTrajectory(&paths->at(Path::EDGE_1), actions);
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
        drive->runTrajectory(&paths->at(Path::EDGE_2), actions);
        step++;
    }
    else if (step == 8 && !drive->isFinished()) {
        if (drive->getTrajectoryTime() > 2.5_s && drive->getTrajectoryTime() < 2.6_s) {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);
        }
    }
    // Score cone.
    else if (step == 8 && drive->isFinished()) {
        step += (scoreAction.process() == Action::Result::DONE);
    }
    else if (step >= 9 && step <= 13) {
        step++; // Give it some time...
    }
    // Drive to field cone 2.
    else if (step == 14) {
        gamePiece->overrideHasGamePiece(false);
        gamePiece->setGrabberAction(Grabber::Action::IDLE);
        step++;
    }
}

void Autonomous::driveForwards() {
    if (step == 0) {
        gamePiece->overrideHasGamePiece(false);
        mobilityTimer.Reset();
        mobilityTimer.Start();
        drive->velocityControlAbsRotation(0_mps, 0.6_mps, 0_deg, Drive::ControlFlag::FIELD_CENTRIC);
        step++;
    }
    else if (step == 1 && mobilityTimer.Get() >= 3_s) {
        drive->velocityControlAbsRotation(0.0_mps, 0_mps, 0_deg, Drive::ControlFlag::FIELD_CENTRIC);
        step++;
        // Hi Trevor!!!!
    }
    else if (step == 2) {
        drive->velocityControlRelRotation(0.0_mps, 0_mps, 0_deg_per_s, Drive::ControlFlag::FIELD_CENTRIC);
        step++;
    }
}

void Autonomous::scoreBackup() {
    if (step == 0) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
        gamePiece->overrideHasGamePiece(true);
        step += (scoreAction.process() == Action::Result::DONE);
    }
    else if (step == 1) {
        frc::Pose2d initPose(paths->at(Path::BARRIER_1).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        step++;
    }
    else if (step == 2) {
        gamePiece->overrideHasGamePiece(false);
        mobilityTimer.Reset();
        mobilityTimer.Start();
        drive->velocityControlAbsRotation(0.6_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        step++;
    }
    else if (step == 3 && mobilityTimer.Get() >= 6_s) {
        drive->velocityControlAbsRotation(0.0_mps, 0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        step++;
        // Hi Trevor!!!!
    }
    else if (step == 4) {
        drive->velocityControlRelRotation(0.0_mps, 0_mps, 0_deg_per_s, Drive::ControlFlag::FIELD_CENTRIC);
        step++;
    }
}

void Autonomous::score() {
    if (step == 0) {
        gamePiece->setGrabberPosition(Grabber::Position::OPEN);
        gamePiece->overrideHasGamePiece(true);
        step += (scoreAction.process() == Action::Result::DONE);
    }
    else if (step == 1) {
        frc::Pose2d initPose(paths->at(Path::BARRIER_1).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        step++;
    }
}

// --- Score Action ---

Autonomous::ScoreAction::ScoreAction(GamePiece* _gamePiece)
: gamePiece(_gamePiece) { }

Autonomous::ScoreAction::~ScoreAction() = default;

void Autonomous::ScoreAction::setLevel(int _lvl) {
    lvl = _lvl;
}

Action::Result Autonomous::ScoreAction::process() {
    if (step == 0) {
        if (lvl == 2) {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);
        }
        else if (lvl == 1) {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::MID);
        }
        else if (lvl == 0) {
            gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);
        }
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
        gamePiece->overrideHasGamePiece(false);
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
            drive->velocityControlAbsRotation(-2.2_mps, 0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
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
    // Make sure it doesn't go too far.
    if (drive->getEstimatedPose().X() > 7.5_m) {
        drive->velocityControlAbsRotation(0_mps, 0_mps, 90_deg, Drive::ControlFlag::BRICK);
        return Action::Result::WORKING;
    }

    // Drive backwards until tilted greater than 10 degrees.
    if (step == 0) {
        if (whooshWhoosh->getTiltAngle() >= 10_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(2_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Drive backwards slower until tilt becomes negative.
    else if (step == 1) {
        if (whooshWhoosh->getTiltAngle() < 0_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(1.3_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Drive backwards slower until tilt is near zero (on the ground on opposite side).
    else if (step == 2) {
        if (units::math::abs(whooshWhoosh->getTiltAngle()) <= 3_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(0.9_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Start driving slowly off the Charge Station to get mobility.
    else if (step == 3) {
        stopTimer.Reset();
        stopTimer.Start();
        drive->velocityControlAbsRotation(0.5_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        step++;
    }
    // Wait 2 seconds.
    else if (step == 4 && (stopTimer.Get() > 1.5_s)) {
        forwardsTimer.Reset();
        forwardsTimer.Start();
        step++;
    }
    // Start driving back onto the Charge Station.
    else if (step == 5) {
        drive->velocityControlAbsRotation(-1.7_mps, 0.0_mps, 90_deg, Drive::ControlFlag::FIELD_CENTRIC);
        step += forwardsTimer.Get() > 0.5_s;
    }
    // Balance.
    else if (step == 6) {
        balanceAction->process();
    }

    return Result::WORKING;
}

void Autonomous::sendFeedback() {
    int desiredAutoMode = static_cast<int>(frc::SmartDashboard::GetNumber("Auto_Mode", 0.0));
    if (desiredAutoMode < autoModeNames.size() && desiredAutoMode >= 0) {
        selectedAutoMode = static_cast<AutoMode>(desiredAutoMode);
    }
    else {
        selectedAutoMode = AutoMode::DO_NOTHING;
    }


    frc::SmartDashboard::PutNumber("Autonomous_Step", step);
    frc::SmartDashboard::PutBoolean("Autonomous_DriveFinished", drive->isFinished());
    frc::SmartDashboard::PutString("Autonomous_ModeName", autoModeNames.at(selectedAutoMode));

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
