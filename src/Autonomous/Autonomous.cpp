#include <Autonomous/Autonomous.h>
#include <Drive/Drive.h>
#include <WhooshWhoosh/WhooshWhoosh.h>
#include <GamePiece/GamePiece.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <fmt/core.h>

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

    if (step == 0) {
        drive->resetOdometry(testing_line.getInitialPose());
        step++;
    }
    else if (step == 1) {
        drive->runTrajectory(&testing_line, actions);
        step++;
    }
    else if (step == 2 && drive->isFinished()) {
        fmt::print("Auto finished!\n");
        step++;
    }
    else {
        return;
    }

    switch (selectedAutoMode) {
        case AutoMode::DO_NOTHING:
            doNothing();
            break;
        case AutoMode::BARRIER_3GP:
            barrier3GP();
            break;
        case AutoMode::BARRIER_2GP_CS:
            barrier2GP_CS();
            break;
        case AutoMode::CENTER_1GP:
            center1GP();
            break;
        case AutoMode::CENTER_1GP_CS:
            center1GP_CS();
            break;
        case AutoMode::EDGE_3GP:
            edge3GP();
            break;
        case AutoMode::EDGE_2GP_CS:
            edge2GP_CS();
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
}

void Autonomous::barrier3GP() {
    if (step == 0) {
        drive->runTrajectory(&paths->at(Path::BARRIER_START), actions);
        step++;
    }
    else if (step == 1 && drive->isFinished()) {
        drive->runTrajectory(&paths->at(Path::BARRIER_FINAL_SCORE), actions);
        step++;
    }
}

void Autonomous::barrier2GP_CS() {
    if (step == 0) {
        drive->runTrajectory(&paths->at(Path::BARRIER_START), actions);
        step++;
    }
    else if (step == 1 && drive->isFinished()) {
        drive->runTrajectory(&paths->at(Path::BARRIER_FINAL_BALANCE), actions);
        step++;
    }
}

void Autonomous::center1GP() {
    if (step == 0) {
        drive->runTrajectory(&paths->at(Path::CENTER_START), actions);
        step++;
    }
}

void Autonomous::center1GP_CS() {
    if (step == 0) {
        drive->runTrajectory(&paths->at(Path::CENTER_START), actions);
        step++;
    }
    else if (step == 1 && drive->isFinished()) {
        step++;
    }
    else if (step == 2) {
        balanceMobilityAction.process();
    }
}

void Autonomous::edge3GP() {
    if (step == 0) {
        drive->runTrajectory(&paths->at(Path::EDGE_START), actions);
        step++;
    }
    else if (step == 1 && drive->isFinished()) {
        drive->runTrajectory(&paths->at(Path::EDGE_FINAL_SCORE), actions);
        step++;
    }
}

void Autonomous::edge2GP_CS() {
    if (step == 0) {
        drive->runTrajectory(&paths->at(Path::EDGE_START), actions);
        step++;
    }
    else if (step == 1 && drive->isFinished()) {
        drive->runTrajectory(&paths->at(Path::EDGE_FINAL_BALANCE), actions);
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
    else if (step == 2 && gamePiece->isFinishedScoring()) {
        step = 0;
        return Result::DONE;
    }

    return Result::WORKING;
}

// --- Intake to Cone Action ---

Autonomous::IntakeToConeAction::IntakeToConeAction(GamePiece* _gamePiece)
: gamePiece(_gamePiece) { }

Autonomous::IntakeToConeAction::~IntakeToConeAction() = default;

Action::Result Autonomous::IntakeToConeAction::process() {
    gamePiece->setGrabberAction(Grabber::Action::INTAKE);
    gamePiece->setGrabberPosition(Grabber::Position::AGAPE);

    return Result::DONE;
}

// --- Intake Wait Action ---

Autonomous::IntakeWaitAction::IntakeWaitAction(GamePiece* _gamePiece)
: gamePiece(_gamePiece) { }

Autonomous::IntakeWaitAction::~IntakeWaitAction() = default;

Action::Result Autonomous::IntakeWaitAction::process() {
    if (gamePiece->getGamePieceType() != Grabber::GamePieceType::NONE) {
        return Result::DONE;
    }

    return Result::WORKING;
}

// --- Lift to High Action ---

Autonomous::LiftHighAction::LiftHighAction(GamePiece* _gamePiece)
: gamePiece(_gamePiece) { }

Autonomous::LiftHighAction::~LiftHighAction() = default;

Action::Result Autonomous::LiftHighAction::process() {
    gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);

    return Result::DONE;
}

// --- Balance Action ---

Autonomous::BalanceAction::BalanceAction(Drive* _drive, WhooshWhoosh* _whooshWhoosh)
: drive(_drive), whooshWhoosh(_whooshWhoosh) { }

Autonomous::BalanceAction::~BalanceAction() = default;

Action::Result Autonomous::BalanceAction::process() {
    // Drive forwards until tilted.
    if (step == 0) {
        if (whooshWhoosh->getTiltAngle() > 8_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(0_mps, -0.4_mps, 0_deg, Drive::ControlFlag::FIELD_CENTRIC);
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
        if (antiTiltVel <= 0.1_mps) {
            antiTiltVel = 0.0_mps;
            flags |= Drive::ControlFlag::BRICK;
        }

        drive->velocityControlAbsRotation(0_mps, -antiTiltVel, 0_deg, flags);
    }

    return Result::WORKING;
}

// --- Balance w/ Mobility Action ---

Autonomous::BalanceMobilityAction::BalanceMobilityAction(Drive* _drive, WhooshWhoosh* _whooshWhoosh)
: drive(_drive), whooshWhoosh(_whooshWhoosh) { }

Autonomous::BalanceMobilityAction::~BalanceMobilityAction() = default;

Action::Result Autonomous::BalanceMobilityAction::process() {
    // Drive forwards until almost off charge station.
    if (step == 0) {
        if (whooshWhoosh->getTiltAngle() > 8_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(0_mps, 0.4_mps, 180_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Check when grounded on other side.
    else if (step == 1) {
        if (whooshWhoosh->getTiltAngle() < 2_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(0_mps, 0.2_mps, 180_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Drive back on the Charge Station.
    else if (step == 2) {
        if (whooshWhoosh->getTiltAngle() > 8_deg) {
            step++;
        }
        else {
            drive->velocityControlAbsRotation(0_mps, -0.4_mps, 0_deg, Drive::ControlFlag::FIELD_CENTRIC);
        }
    }
    // Balance!
    else if (step == 3) {
        static bool firstRun = true;
        if (firstRun) {
            // whooshWhoosh->resetAntiTiltPIDController();
            firstRun = false;
        }

        units::meters_per_second_t antiTiltVel = whooshWhoosh->calculateAntiTiltDriveVelocity();
        unsigned flags = Drive::ControlFlag::FIELD_CENTRIC;
        if (antiTiltVel <= 0.1_mps) {
            antiTiltVel = 0.0_mps;
            flags |= Drive::ControlFlag::BRICK;
        }

        drive->velocityControlAbsRotation(0_mps, -antiTiltVel, 0_deg, flags);
    }

    return Result::WORKING;
}

void Autonomous::sendFeedback() {
    frc::SmartDashboard::PutNumber("Autonomous_Step", step);
    frc::SmartDashboard::PutBoolean("Autonomous_DriveFinished", drive->isFinished());
}
