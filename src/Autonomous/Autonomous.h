#pragma once

#include <Basic/Mechanism.h>
#include <frc/Timer.h>
#include <Trajectory/CSVTrajectory.h>
#include <Autonomous/Action.h>
#include <map>
#include <cstdint>

#define DEPLOY_DIR "/home/lvuser/deploy/"

class Drive;

class Autonomous : public Mechanism {
public:
    Autonomous(Drive* drive);
    ~Autonomous();

    void process() override;
    void sendFeedback() override;
    void resetToMode(MatchMode mode) override;

private:
    enum class AutoMode {
        DO_NOTHING = 0, // Do nothing or something.
        RECORDED = 1, // Run the recorded trajectory.
    };

    void doNothing();
    void runTrajectory(CSVTrajectory& trajectory);

    AutoMode currentMode = AutoMode::DO_NOTHING;

    frc::Timer delayTimer,
               autoTimer;

    std::size_t step = 0;

    Drive* drive;

    /**
     * An action that pauses the path for a specified number of seconds.
     */
    class PauseAction : public Action {
    public:
        PauseAction(units::second_t duration);
        ~PauseAction();

        Result process() override;

    private:
        frc::Timer timer;
        units::second_t duration;
    };

    class MessageAction : public Action {
    public:
        MessageAction(const char* msg = "Hello, Ishan!\n");
        ~MessageAction();

        Result process() override;

    private:
        const char* msg;
    };

    PauseAction pause3sAction { 3_s };
    MessageAction msgAction;

    /**
     * A map of the actions that are available to each autonomous mode.
     */
    std::map<u_int32_t, Action*> actions {
        { 1 << 0, &pause3sAction },
        { 1 << 1, &msgAction },
    };
};
