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
    currentMode = static_cast<AutoMode>(frc::SmartDashboard::GetNumber("Auto_Mode", 0.0));

    // Autonomous delay.
    if (delayTimer.Get().value() <= frc::SmartDashboard::GetNumber("thunderdashboard_auto_start_delay", 0.0)) {
        return;
    }

    switch (currentMode) {
        case AutoMode::DO_NOTHING:
            doNothing();
            break;
        case AutoMode::RECORDED:
            runTrajectory(drive->getRecordedTrajectory());
            break;
    }
}

void Autonomous::doNothing() {
    // If it does nothing is it doing something or nothing? - trevor(2020)
        //it does something because it is doing nothing - ishan(2022)
        //I disagree - peter(2022)
        //I agree with peter -L Wrench

    // Good function.
    // Very good function. - jeff downs
    // Very bad function. - jeff ups
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

    std::string buffer;

    auto sendAutoMode = [&](AutoMode mode, const char* description) {
        // Append mode number to the end of the buffer.
        buffer.append(fmt::format(",{}", static_cast<int>(mode)));

        frc::SmartDashboard::PutString(fmt::format("thunderdashboard_auto_{}", fmt::format("{}", static_cast<int>(mode))), description);
    };

    sendAutoMode(AutoMode::DO_NOTHING, "Do Nothing");
    sendAutoMode(AutoMode::RECORDED, "Recorded Trajectory");

    frc::SmartDashboard::PutString("thunderdashboard_auto_list", buffer);
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
