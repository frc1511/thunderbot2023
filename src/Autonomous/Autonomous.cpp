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
    // The starting location of the robot.
    startingLocation = static_cast<StartingLocation>(frc::SmartDashboard::GetNumber("thunderdashboard_auto_starting_location", -1.0));

    // Whether we are actually running an auto mode.
    if (frc::SmartDashboard::GetNumber("thunderdashboard_auto_do_nothing", 0.0)) {
        startingLocation = StartingLocation::MARS;
    }

    // Starting GamePiece (Cube = 0, Cone = 1).
    startingGamePiece = frc::SmartDashboard::GetNumber("thunderdashboard_auto_starting_gamepiece", -1.0);
    // The GamePiece to be collecting on the field (Cube = 0, Cone = 1).
    fieldGamePiece = frc::SmartDashboard::GetNumber("thunderdashboard_auto_field_gamepiece", -1.0);

    // The final action of the autonomous mode (dependant on starting location).
    finalAction = frc::SmartDashboard::GetNumber("thunderdashboard_auto_final_action", -1.0);

    // Autonomous delay.
    if (delayTimer.Get().value() <= frc::SmartDashboard::GetNumber("thunderdashboard_auto_start_delay", 0.0)) {
        return;
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
    /**
     * Auto mode for starting at side of the community closest to barrier.
     * 1. Score preloaded GamePiece on grid.
     * 2. Drive around the Charge Station and collect GamePiece 1.
     * 3. Configurable final action:
     *    a. Do nothing
     *    b. Score collected GamePiece 1
     *    c. Balance on Charge Station
     */

    // Code ...
}

void Autonomous::middleAuto() {
    /**
     * Auto mode for starting in the middle of the community behind the charging station.
     * 1. Score preloaded GamePiece on grid.
     * 2. Configurable final action:
     *    a. Do nothing
     *    b. Balance on Charge Station
     *    c. Go over Charge Station and collect GamePiece 2/3
     *    d. Go over Charge Station, collect GamePiece 2/3, then balance on Charge Station
     */

    // Code ...
}

void Autonomous::edgeSideAuto() {
    /**
     * Auto mode for starting at the side of the community closest to wall.
     * 1. Score preloaded GamePiece on grid.
     * 2. Drive around the Charge Station and collect GamePiece 4.
     * 3. Configure final action:
     *    a. Do nothing
     *    b. Score collected GamePiece 4
     *    c. Balance on Charge Station
     */

    // Code ...
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
    frc::SmartDashboard::PutNumber("Autonomous_step", step);
    frc::SmartDashboard::PutBoolean("Autonomous_drive_finished", drive->isFinished());
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
