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
    void doNothing();

    /**
     * Auto mode for starting at side of the community closest to barrier.
     * 1. Score preloaded GamePiece on grid.
     * 2. Drive around the Charge Station and collect GamePiece 1.
     * 3. Configurable final action:
     *    a. Do nothing
     *    b. Score collected GamePiece 1
     *    c. Balance on Charge Station
     */
    void barrierSideAuto();

    /**
     * Auto mode for starting in the middle of the community behind the charging station.
     * 1. Score preloaded GamePiece on grid.
     * 2. Configurable final action:
     *    a. Do nothing
     *    b. Balance on Charge Station
     *    c. Go over Charge Station and collect GamePiece 2/3
     *    d. Go over Charge Station, collect GamePiece 2/3, then balance on Charge Station
     */
    void middleAuto();

    /**
     * Auto mode for starting at the side of the community closest to wall.
     * 1. Score preloaded GamePiece on grid.
     * 2. Drive around the Charge Station and collect GamePiece 4.
     * 3. Configure final action:
     *    a. Do nothing
     *    b. Score collected GamePiece 4
     *    c. Balance on Charge Station
     */
    void edgeSideAuto();

    void runTrajectory(CSVTrajectory& trajectory);

    enum class StartingLocation {
        MARS = -1, // Not really mars - signifies that we aren't running any auto.
        BARRIER_SIDE = 0,
        MIDDLE = 1,
        EDGE_SIDE = 2,
    };

    StartingLocation startingLocation;
    int startingGamePiece,
        fieldGamePiece,
        finalAction;

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
