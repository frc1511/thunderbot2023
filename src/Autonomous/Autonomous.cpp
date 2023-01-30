#include <Autonomous/Autonomous.h>
#include <Drive/Drive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>

Autonomous::Autonomous(Drive* drive)
: drive(drive) { }

Autonomous::~Autonomous() = default;

void Autonomous::resetToMode(MatchMode mode) {
    delayTimer.Reset();
    delayTimer.Start();
    autoTimer.Reset();
    autoTimer.Start();

    step = 0;
}

void Autonomous::process() {
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
    // 0: Set start pose and raise lift.
    if (step == 0) {
        // Cube
        if (startingGamePiece == 0) {
            // Start at Grid 1.
            drive->resetOdometry(grid1_to_gp1_traj.getInitialPose());
        }
        // Cone
        else {
            // Start at Grid 2.
            drive->resetOdometry(grid2_to_gp1_traj.getInitialPose());
        }

        // TODO: Begin raising lift.

        step++;
    }
    // 1: Score preloaded GamePiece.
    else if (step == 1 /* && gamePiece->liftAtPosition() */) {
        // TODO: Score.

        step++;
    }
    // 2: Drive to GamePiece 1 and begin lowering lift.
    else if (step == 2 /* && gamePiece->isFinishedScoring() */) {
        // Cube
        if (startingGamePiece == 0) {
            // Drive from Grid 1 to GP 1.
            drive->runTrajectory(&grid1_to_gp1_traj, actions);
        }
        // Cone
        else {
            // Drive from Grid 2 to GP 1.
            drive->runTrajectory(&grid2_to_gp1_traj, actions);
        }

        // TODO: Begin lowering lift.

        step++;
    }
    // 3: Intake GamePiece 1.
    else if (step == 3 && drive->isFinished() /* && gamePiece->liftAtPosition() */) {
        // Intake GamePiece.

        // Cube
        if (fieldGamePiece == 0) {
            // TODO: Intake Cube
        }
        // Cone
        else {
            // TODO: Intake Cone
        }
    }
    // 4: Wait for intake to finish.
    else if (step == 4 /* && gamePiece->isFinishedIntaking() */) {
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
            drive->runTrajectory(&gp1_to_grid1_traj, actions);
        }
        // Cone
        else {
            drive->runTrajectory(&gp1_to_grid2_traj, actions);
        }

        // TODO: Begin raising lift.

        step++;
    }
    // 1: Score current GamePiece.
    else if (get_step() == 1 && drive->isFinished() /* && gamePiece->liftAtPosition() */) {
        // TODO: Score GamePiece

        step++;
    }
    // 2: Lower lift.
    else if (get_step() == 2 /* && gamePiece->isFinishedScoring() */) {
        // TODO: Lower lift

        step++;
    }
}

void Autonomous::barrierSideAuto_finalBalance() {
    auto get_step = [&]() { return step - BARRIER_SIDE_AUTO_END_STEP; };

    // 0: Drive to Charge Station.
    if (get_step() == 0) {
        drive->runTrajectory(&gp1_to_cs_traj, actions);
        step++;
    }
    // 1: Wait for drive.
    else if (get_step() == 1 && drive->isFinished()) {
        step++;
    }
    // 2: Balance on the charge station.
    else if (get_step() == 2) {
        balanceOnChargeStation();
    }
}

void Autonomous::middleAuto() {
    // 0: Set start pose and raise lift.
    if (step == 0) {
        // Cube
        if (startingGamePiece == 0) {
            // Start at Grid 4.
            drive->resetOdometry(grid4_to_cs_traj.getInitialPose());
        }
        // Cone
        else {
            // Start at Grid 5.
            drive->resetOdometry(grid5_to_cs_traj.getInitialPose());
        }

        // TODO: Begin raising lift.

        step++;
    }
    // 1: Score preloaded GamePiece.
    else if (step == 1 /* && gamePiece->liftAtPosition() */) {
        // TODO: Score.

        step++;
    }
    // 2: Drive to Charge Station and begin lowering lift.
    else if (step == 2 /* && gamePiece->isFinishedScoring() */) {
        // Cube
        if (startingGamePiece == 0) {
            // Drive from Grid 4 to Charge Station.
            drive->runTrajectory(&grid4_to_cs_traj, actions);
        }
        // Cone
        else {
            // Drive from Grid 5 to Charge Station.
            drive->runTrajectory(&grid5_to_cs_traj, actions);
        }

        // TODO: Begin lowering lift.

        step++;
    }
    // 3: Delegate the starting action.
    else if (step == 3) {
        // Delegate the starting action.
        switch (startingAction) {
            case 0: // Balance on the Charge Station.
                balanceOnChargeStation();
                break;
            case 1: // Traverse the Charge Station and collect GamePiece 2.
                step++;
                break;
            default: // Problems...
                doNothing();
                break;
        }
    }
    // 4: Starting Action is 1, traverse charge station.
    else if (step == 4) {
        if (traverseChargeStation(cs_to_gp3_traj.getInitialPose())) {
            step++;
        }
    }
    // 5: Drive to GamePiece 3 and begin lowering lift.
    else if (step == 5) {
        // Drive from Charge Station to GamePiece 3.
        drive->runTrajectory(&cs_to_gp3_traj, actions);

        // TODO: Begin lowering lift.

        step++;
    }
    // 6: Intake GamePiece 3.
    else if (step == 6 && drive->isFinished() /* && gamePiece->liftAtPosition() */) {
        // Intake GamePiece.

        // Cube
        if (fieldGamePiece == 0) {
            // TODO: Intake Cube
        }
        // Cone
        else {
            // TODO: Intake Cone
        }
    }
    // 7: Delegate the final action.
    else if (step == 7 /* && gamePiece->isFinishedIntaking() */) {
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
        drive->runTrajectory(&gp3_to_cs_traj, actions);
    }
    // 9: Balance on the Charge Station.
    else if (step == 9 && drive->isFinished()) {
        balanceOnChargeStation();
    }
}

void Autonomous::edgeSideAuto() {
    // 0: Set start pose and raise lift.
    if (step == 0) {
        // Cube
        if (startingGamePiece == 0) {
            // Start at Grid 7.
        }
        // Cone
        else {
            // Start at Grid 6.
        }

        // TODO: Begin raising lift.
    
        step++;
    }
    // 1: Score preloaded GamePiece.
    else if (step == 1 /* && gamePiece->liftAtPosition() */) {
        // TODO: Score

        step++;
    }
    // 2: Drive to GamePiece 4 and begin lowering lift.
    else if (step == 2 /* && gamePiece->isFinishedScoring() */) {
        // Cube
        if (startingGamePiece == 0) {
            // Drive from Grid 7 to GP 4.
            drive->runTrajectory(&grid7_to_gp4_traj, actions);
        }
        // Cone
        else {
            // Drive from Grid 6 to GP 4.
            drive->runTrajectory(&grid6_to_gp4_traj, actions);
        }

        // TODO: Begin lowering lift.

        step++;
    }
    // 3: Intake GamePiece 4.
    else if (step == 3 && drive->isFinished() /* && gamePiece->liftAtPosition() */) {
        // Intake GamePiece.

        // Cube
        if (fieldGamePiece == 0) {
            // TODO: Intake Cube
        }
        // Cone
        else {
            // TODO: Intake Cone
        }
    }
    // 4: Wait for intake to finish.
    else if (step == 4 /* && gamePiece->isFinishedIntaking() */) {
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
            drive->runTrajectory(&gp4_to_grid7_traj, actions);
        }
        // Cone
        else {
            drive->runTrajectory(&gp4_to_grid6_traj, actions);
        }

        // TODO: Begin raising lift.

        step++;
    }
    // 1: Score current GamePiece.
    else if (get_step() == 1 && drive->isFinished() /* && gamePiece->liftAtPosition() */) {
        // TODO: Score GamePiece

        step++;
    }
    // 2: Lower lift.
    else if (get_step() == 2 /* && gamePiece->isFinishedScoring() */) {
        // TODO: Lower lift

        step++;
    }

}

void Autonomous::edgeSideAuto_finalBalance() {
    auto get_step = [&]() { return step - EDGE_SIDE_AUTO_END_STEP; };

    // 0: Drive to Charge Station.
    if (get_step() == 0) {
        drive->runTrajectory(&gp4_to_cs_traj, actions);
        step++;
    }
    // 1: Wait for drive.
    else if (get_step() == 1 && drive->isFinished()) {
        step++;
    }
    // 2: Balance on the charge station.
    else if (get_step() == 2) {
        balanceOnChargeStation();
    }
}

bool Autonomous::traverseChargeStation(frc::Pose2d resetPose) {
    // TODO: Traverse Charge Station.

    if (traverseChargeStationStep == 0) {

    }

    return true;
}

void Autonomous::balanceOnChargeStation() {
    // TODO: Balance on Charge Station.

    // 0: Drive forwards until angle ~11 deg.
    if (balanceChargeStationStep == 0) {
        
    }
}

void Autonomous::runTrajectory(CSVTrajectory& trajectory) {
    if (step == 0) {
        drive->runTrajectory(&trajectory, actions);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
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
