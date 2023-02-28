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

    doing_auto = frc::SmartDashboard::GetBoolean("thunderdashboard_auto_doing_auto", false);

    // Autonomous delay.
    if (delayTimer.Get().value() <= frc::SmartDashboard::GetNumber("thunderdashboard_auto_start_delay", 0.0)) {
        return;
    }

    // The starting location of the robot (0 = Barrier, 1 = Center, 2 = Edge).
    startingLocation = static_cast<StartingLocation>(frc::SmartDashboard::GetNumber("thunderdashboard_auto_starting_location", -1.0));
    // The GamePiece that the robot is starting with (0 = Cube, 1 = Cone).
    startingGamePiece = frc::SmartDashboard::GetNumber("thunderdashboard_auto_starting_gamepiece", -1.0);
    // The starting action (If center start, 0 = preload & balance, 1 = preload & traverse).
    startingAction = frc::SmartDashboard::GetNumber("thunderdashboard_auto_starting_action", -1.0);
    // The field GamePiece to collect (0 = Cube, 1 = Cone).
    fieldGamePiece = frc::SmartDashboard::GetNumber("thunderdashboard_auto_field_gamepiece", -1.0);
    // The final action (If center start, 0 = do nothing, 1 = balance. If edge or barrier start, 0 = do nothing, 1 = score, 2 = balance).
    finalAction = frc::SmartDashboard::GetNumber("thunderdashboard_auto_final_action", -1.0);

    if (!doing_auto) return;
    
    scorePreloadedGamePiece();

    return;

    if (!doing_auto) {
        startingLocation = StartingLocation::MARS;
    }

    switch (startingLocation) {
        case StartingLocation::MARS:
            doNothing();
            break;
        case StartingLocation::BARRIER_SIDE:
            barrierSideAuto();
            break;
        case StartingLocation::MIDDLE:
            middleAuto();
            break;
        case StartingLocation::EDGE_SIDE:
            edgeSideAuto();
            break;
    }
}

void Autonomous::scorePreloadedGamePiece() {
    if (step == 0) {
        switch (startingLocation) {
            case StartingLocation::MARS:
                break;
            case StartingLocation::BARRIER_SIDE:
                if (startingGamePiece == 0) { // Cube
                    // Grid 1
                    drive->resetOdometry(paths->at(GRID1_TO_GP1).getInitialPose());
                }
                else { // Cone
                    // Grid 2
                    drive->resetOdometry(paths->at(GRID2_TO_GP1).getInitialPose());
                }
                break;
            case StartingLocation::MIDDLE:
                if (startingGamePiece == 0) { // Cube
                    // Start at Grid 4.
                    drive->resetOdometry(paths->at(GRID4_TO_CS).getInitialPose());
                }
                else { // Cone
                    // Start at Grid 5.
                    drive->resetOdometry(paths->at(GRID5_TO_CS).getInitialPose());
                }
                break;
            case StartingLocation::EDGE_SIDE:
                if (startingGamePiece == 0) { // Cube
                    // Start at Grid 7.
                    drive->resetOdometry(paths->at(GRID7_TO_GP4).getInitialPose());
                }
                else { // Cone
                    // Start at Grid 6.
                    drive->resetOdometry(paths->at(GRID6_TO_GP4).getInitialPose());
                }
                break;
        }
        if (startingGamePiece == 0) {
            gamePiece->setGamePiece( Grabber::GamePieceType::CUBE);
        }
        step++;
    }
    else if (step == 1) {
        if (startingGamePiece == 0) { // Cube
            gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_CUBE);
        }
        else if (startingGamePiece == 1) { // Cone
            gamePiece->setLiftPreset(GamePiece::LiftPreset::AUTO_JANKY);
        }
        step++;
    }
    else if (step == 2 && gamePiece->liftAtPosition()) {
        if (startingGamePiece == 1) { // Cone
            gamePiece->setLiftPreset(GamePiece::LiftPreset::AUTO_JANKY_LOWER);
        }
        step++;
    }
    else if (step == 3 && gamePiece->liftAtPosition()) {
        gamePiece->placeGamePiece();
        step++;
    }
    else if (step == 4 && gamePiece->isFinishedScoring()) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);
        step++;
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

void Autonomous::barrierSideAuto() {
    // 0: Set start pose and starting GamePiece.
    if (step == 0) {
        // Cube
        if (startingGamePiece == 0) {
            // Start at Grid 1.
            drive->resetOdometry(paths->at(GRID1_TO_GP1).getInitialPose());
        }
        // Cone
        else {
            // Start at Grid 2.
            drive->resetOdometry(paths->at(GRID2_TO_GP1).getInitialPose());
        }

        // Set the starting GamePiece.
        gamePiece->setGamePiece(startingGamePiece == 0 ? Grabber::GamePieceType::CUBE : Grabber::GamePieceType::CONE);

        step++;
    }
    // 1: Score preloaded GamePiece.
    else if (step == 1) {
        if (scoreGamePiece()) {
            step++;
        }
    }
    // 2: Drive to GamePiece 1 and begin lowering lift.
    else if (step == 2) {
        // Cube
        if (startingGamePiece == 0) {
            // Drive from Grid 1 to GP 1.
            drive->runTrajectory(&paths->at(GRID1_TO_GP1), actions);
        }
        // Cone
        else {
            // Drive from Grid 2 to GP 1.
            drive->runTrajectory(&paths->at(GRID2_TO_GP1), actions);
        }

        // Move lift down to intake.
        gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);

        // Prepare the grabber to intake GamePiece 1.
        gamePiece->setGrabberPosition(fieldGamePiece == 0 ? Grabber::Position::OPEN : Grabber::Position::AGAPE);

        step++;
    }
    // 3: Intake GamePiece 1.
    else if (step == 3 && drive->isFinished() && gamePiece->liftAtPosition()) {
        gamePiece->intakeGamePiece();
    }
    // 4: Wait for intake to finish.
    else if (step == 4 && gamePiece->isFinishedIntaking()) {
        step++;
    }
    // 5: Delegate the final action.
    else if (step >= 5) {
        // Delegate the final action.
        switch (finalAction) {
            case 1:
                barrierSideAuto_finalScore();
                break;
            case 2:
                barrierSideAuto_finalBalance();
                break;
            case 0:
            default:
                doNothing();
                break;
        }
    }
}

void Autonomous::barrierSideAuto_finalScore() {
    auto get_step = [&]() { return step - BARRIER_SIDE_AUTO_END_STEP; };

    // 0: Drive to grid depending on current GamePiece in possession.
    if (get_step() == 0) {
        // Cube
        if (fieldGamePiece == 0) {
            drive->runTrajectory(&paths->at(GP1_TO_GRID1), actions);
        }
        // Cone
        else {
            drive->runTrajectory(&paths->at(GP1_TO_GRID2), actions);
        }

        // Raise the lift!
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);

        step++;
    }
    // 1: Score current GamePiece.
    else if (get_step() == 1 && drive->isFinished()) {
        if (scoreGamePiece()) {
            step++;
        }
    }
    // 2: Retract the lift.
    else if (get_step() == 2) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);

        step++;
    }
}

void Autonomous::barrierSideAuto_finalBalance() {
    auto get_step = [&]() { return step - BARRIER_SIDE_AUTO_END_STEP; };

    // 0: Drive to Charge Station.
    if (get_step() == 0) {
        drive->runTrajectory(&paths->at(GP1_TO_CS), actions);
        step++;
    }
    // 1: Wait for drive.
    else if (get_step() == 1 && drive->isFinished()) {
        step++;
    }
    // 2: Balance on the charge station.
    else if (get_step() == 2) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);
        balanceOnChargeStation();
    }
}

void Autonomous::middleAuto() {
    // 0: Set start pose and starting GamePiece.
    if (step == 0) {
        // Cube
        if (startingGamePiece == 0) {
            // Start at Grid 4.
            drive->resetOdometry(paths->at(GRID4_TO_CS).getInitialPose());
        }
        // Cone
        else {
            // Start at Grid 5.
            drive->resetOdometry(paths->at(GRID5_TO_CS).getInitialPose());
        }

        // Set the starting GamePiece.
        gamePiece->setGamePiece(startingGamePiece == 0 ? Grabber::GamePieceType::CUBE : Grabber::GamePieceType::CONE);

        step++;
    }
    // 1: Score preloaded GamePiece.
    else if (step == 1) {
        if (scoreGamePiece()) {
            step++;
        }
    }
    // 2: Drive to Charge Station and begin lowering lift.
    else if (step == 2) {
        // Cube
        if (startingGamePiece == 0) {
            // Drive from Grid 4 to Charge Station.
            drive->runTrajectory(&paths->at(GRID4_TO_CS), actions);
        }
        // Cone
        else {
            // Drive from Grid 5 to Charge Station.
            drive->runTrajectory(&paths->at(GRID5_TO_CS), actions);
        }

        step++;
    }
    // 3: Delegate the starting action.
    else if (step == 3) {
        // Delegate the starting action.
        switch (startingAction) {
            case 0: // Balance on the Charge Station.
                gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);
                balanceOnChargeStation();
                break;
            case 1: // Traverse the Charge Station and collect GamePiece 2.
                step++;
                break;
            default: // Problems...
                gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);
                doNothing();
                break;
        }
    }
    // 4: Starting Action is 1, traverse charge station.
    else if (step == 4) {
        // Move lift down to intake.
        gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);

        if (traverseChargeStation(paths->at(CS_TO_GP3).getInitialPose())) {
            step++;
        }
    }
    // 5: Drive to GamePiece 3 and begin lowering lift.
    else if (step == 5) {
        // Drive from Charge Station to GamePiece 3.
        drive->runTrajectory(&paths->at(CS_TO_GP3), actions);

        // Prepare the grabber to intake GamePiece 3.
        gamePiece->setGrabberPosition(fieldGamePiece == 0 ? Grabber::Position::OPEN : Grabber::Position::AGAPE);

        step++;
    }
    // 6: Intake GamePiece 3.
    else if (step == 6 && drive->isFinished() && gamePiece->liftAtPosition()) {
        gamePiece->intakeGamePiece();
    }
    // 7: Delegate the final action.
    else if (step == 7 && gamePiece->isFinishedIntaking()) {
        // Delegate the final action.
        switch (finalAction) {
            case 1: // Balance on the Charge Station.
                step++;
                break;
            case 0: // Do nothing.
            default: // Problem...
                doNothing();
                break;
        }
    }
    // 8: Final Action is 1, drive to the Charge Station.
    else if (step == 8) {
        drive->runTrajectory(&paths->at(GP3_TO_CS), actions);
    }
    // 9: Balance on the Charge Station.
    else if (step == 9 && drive->isFinished()) {
        // Put lift up high.
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);

        balanceOnChargeStation();
    }
}

void Autonomous::edgeSideAuto() {
    // 0: Set start pose and starting GamePiece.
    if (step == 0) {
        // Cube
        if (startingGamePiece == 0) {
            // Start at Grid 7.
            drive->resetOdometry(paths->at(GRID7_TO_GP4).getInitialPose());
        }
        // Cone
        else {
            // Start at Grid 6.
            drive->resetOdometry(paths->at(GRID6_TO_GP4).getInitialPose());
        }

        // Set the starting GamePiece.
        gamePiece->setGamePiece(startingGamePiece == 0 ? Grabber::GamePieceType::CUBE : Grabber::GamePieceType::CONE);
    
        step++;
    }
    // 1: Score preloaded GamePiece.
    else if (step == 1) {
        if (scoreGamePiece()) {
            step++;
        }
    }
    // 2: Drive to GamePiece 4 and begin lowering lift.
    else if (step == 2) {
        // Cube
        if (startingGamePiece == 0) {
            // Drive from Grid 7 to GP 4.
            drive->runTrajectory(&paths->at(GRID7_TO_GP4), actions);
        }
        // Cone
        else {
            // Drive from Grid 6 to GP 4.
            drive->runTrajectory(&paths->at(GRID6_TO_GP4), actions);
        }

        // Move lift down to intake.
        gamePiece->setLiftPreset(GamePiece::LiftPreset::INTAKE);

        // Prepare the grabber to intake GamePiece 1.
        gamePiece->setGrabberPosition(fieldGamePiece == 0 ? Grabber::Position::OPEN : Grabber::Position::AGAPE);

        step++;
    }
    // 3: Intake GamePiece 4.
    else if (step == 3 && drive->isFinished() && gamePiece->liftAtPosition()) {
        gamePiece->intakeGamePiece();
    }
    // 4: Wait for intake to finish.
    else if (step == 4 && gamePiece->isFinishedIntaking()) {
        step++;
    }
    // 5: Delegate the final action.
    else if (step >= 5) {
        // Delegate the final action.
        switch (finalAction) {
            case 1:
                edgeSideAuto_finalScore();
                break;
            case 2:
                edgeSideAuto_finalBalance();
                break;
            case 0:
            default:
                doNothing();
                break;
        }
    }
}

void Autonomous::edgeSideAuto_finalScore() {
    auto get_step = [&]() { return step - EDGE_SIDE_AUTO_END_STEP; };

    // 0: Drive to grid depending on current GamePiece in possession.
    if (get_step() == 0) {
        // Cube
        if (fieldGamePiece == 0) {
            drive->runTrajectory(&paths->at(GP4_TO_GRID7), actions);
        }
        // Cone
        else {
            drive->runTrajectory(&paths->at(GP4_TO_GRID6), actions);
        }

        // Raise the lift!
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);

        step++;
    }
    // 1: Score current GamePiece.
    else if (get_step() == 1 && drive->isFinished()) {
        if (scoreGamePiece()) {
            step++;
        }
    }
    // 2: Retract the lift.
    else if (get_step() == 2) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);

        step++;
    }
}

void Autonomous::edgeSideAuto_finalBalance() {
    auto get_step = [&]() { return step - EDGE_SIDE_AUTO_END_STEP; };

    // 0: Drive to Charge Station.
    if (get_step() == 0) {
        drive->runTrajectory(&paths->at(GP4_TO_CS), actions);
        step++;
    }
    // 1: Wait for drive.
    else if (get_step() == 1 && drive->isFinished()) {
        step++;
    }
    // 2: Balance on the charge station.
    else if (get_step() == 2) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);
        balanceOnChargeStation();
    }
}

bool Autonomous::traverseChargeStation(frc::Pose2d resetPose) {
    // 0: Drive forwards over the Charge Station.
    if (traverseChargeStationStep == 0) {
        drive->runTrajectory(&traverseChargeStationTrajectory, actions);
        step++;
    }
    // 1: Wait for drive to finish, then reset the odometry to the specified pose.
    else if (traverseChargeStationStep == 1 && drive->isFinished()) {
        traverseChargeStationStep++;
        drive->resetOdometry(resetPose);
    }
    else return true;

    return false;
}

void Autonomous::balanceOnChargeStation() {
    // 0: Drive forwards until tilt angular velocity is non-zero.
    if (balanceChargeStationStep == 0) {
        drive->runTrajectory(&balanceChargeStationTrajectory, actions);
    }
    // 1: Calculate the optimal drive velocity for the current tilt angle.
    else if (balanceChargeStationStep == 1 && drive->isFinished()) {
        units::meters_per_second_t antiTiltVel = whooshWhoosh->calculateAntiTiltDriveVelocity();
        unsigned flags = Drive::ControlFlag::FIELD_CENTRIC;
        if (antiTiltVel <= 0.1_mps) {
            antiTiltVel = 0.0_mps;
            flags |= Drive::ControlFlag::BRICK;
        }

        drive->velocityControlAbsRotation(0_mps, antiTiltVel, 0_deg, flags);
    }
}

void Autonomous::runTrajectory(Trajectory* trajectory) {
    if (step == 0) {
        drive->runTrajectory(trajectory, actions);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

bool Autonomous::scoreGamePiece() {
    // No point in moving it down to HIGH_PIVOT if it's already at high.
    if (scoreGamePieceStep == 0 && gamePiece->getLiftPreset() == GamePiece::LiftPreset::HIGH) {
        step = 2;
    }

    if (scoreGamePieceStep == 0) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH_PIVOT);
        scoreGamePieceStep++;
    }
    else if (scoreGamePieceStep == 1 && gamePiece->liftAtPosition()) {
        gamePiece->setLiftPreset(GamePiece::LiftPreset::HIGH);
        scoreGamePieceStep++;
    }
    else if (scoreGamePieceStep == 2 && gamePiece->liftAtPosition()) {
        gamePiece->placeGamePiece();
        scoreGamePieceStep++;
    }
    else if (scoreGamePieceStep == 3 && gamePiece->isFinishedScoring()) {
        scoreGamePieceStep = 0;
        return true;
    }

    return false;
}

void Autonomous::sendFeedback() {
    frc::SmartDashboard::PutNumber("Autonomous_Step", step);
    frc::SmartDashboard::PutBoolean("Autonomous_DriveFinished", drive->isFinished());
}

Autonomous::PauseAction::PauseAction(units::second_t dur)
: duration(dur) {
    timer.Reset();
    timer.Stop();
}

Autonomous::PauseAction::~PauseAction() = default;

Action::Result Autonomous::PauseAction::process() {
    timer.Start();

    if (timer.Get() >= duration) {
        timer.Reset();
        timer.Stop();
        return Result::DONE;
    }

    return Result::WORKING;
}

Autonomous::MessageAction::MessageAction(const char* _msg)
: msg(_msg) { }

Autonomous::MessageAction::~MessageAction() = default;

Action::Result Autonomous::MessageAction::process() {
    fmt::print("{}", msg);
    return Action::Result::DONE;
}
