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

    enum {
        BARRIER_SIDE_AUTO_END_STEP = 5
    };

    /**
     * Performs the final scoring action for barrier side auto.
     */
    void barrierSideAuto_finalScore();

    /**
     * Performs the final balancing action for the barrier side auto.
     */
    void barrierSideAuto_finalBalance();

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

    enum {
        EDGE_SIDE_AUTO_END_STEP = 5
    };

    /**
     * Performs the final scoring action for edge side auto.
     */
    void edgeSideAuto_finalScore();

    /**
     * Performs the final balancing action for the edge side auto.
     */
    void edgeSideAuto_finalBalance();

    /**
     * Drive over the Charge Station.
     * resetPose: The pose to reset once charge station cleared.
     * Returns true once finished.
     */
    bool traverseChargeStation(frc::Pose2d resetPose);

    /**
     * Balances the robot on the Charge Station.
     */
    void balanceOnChargeStation();

    /**
     * Runs a trajectory from start to finish.
     */
    void runTrajectory(CSVTrajectory& trajectory);

    enum class StartingLocation {
        MARS = -1, // Not really mars - signifies that we aren't running any auto.
        BARRIER_SIDE = 0,
        MIDDLE = 1,
        EDGE_SIDE = 2,
    };

    bool doing_auto = false;
    StartingLocation startingLocation = StartingLocation::MARS;
    int startingGamePiece = -1,
        startingAction = -1,
        fieldGamePiece = -1,
        finalAction = -1;

    frc::Timer delayTimer,
               autoTimer;

    std::size_t step = 0;

    Drive* drive;

    CSVTrajectory grid1_to_gp1_traj { DEPLOY_DIR "ThunderAuto/grid1_to_gp1.csv" };
    CSVTrajectory grid2_to_gp1_traj { DEPLOY_DIR "ThunderAuto/grid2_to_gp1.csv" };
    CSVTrajectory gp1_to_grid1_traj { DEPLOY_DIR "ThunderAuto/gp1_to_grid1.csv" };
    CSVTrajectory gp1_to_grid2_traj { DEPLOY_DIR "ThunderAuto/gp1_to_grid2.csv" };
    CSVTrajectory gp1_to_cs_traj    { DEPLOY_DIR "ThunderAuto/gp1_to_cs.csv"    };
    CSVTrajectory grid4_to_cs_traj  { DEPLOY_DIR "ThunderAuto/grid4_to_cs.csv"  };
    CSVTrajectory grid5_to_cs_traj  { DEPLOY_DIR "ThunderAuto/grid5_to_cs.csv"  };
    CSVTrajectory cs_to_gp3_traj    { DEPLOY_DIR "ThunderAuto/cs_to_gp3.csv"    };
    CSVTrajectory gp3_to_cs_traj    { DEPLOY_DIR "ThunderAuto/gp3_to_cs.csv"    };
    CSVTrajectory grid6_to_gp4_traj { DEPLOY_DIR "ThunderAuto/grid6_to_gp4.csv" };
    CSVTrajectory grid7_to_gp4_traj { DEPLOY_DIR "ThunderAuto/grid7_to_gp4.csv" };
    CSVTrajectory gp4_to_grid6_traj { DEPLOY_DIR "ThunderAuto/gp4_to_grid6.csv" };
    CSVTrajectory gp4_to_grid7_traj { DEPLOY_DIR "ThunderAuto/gp4_to_grid7.csv" };
    CSVTrajectory gp4_to_cs_traj    { DEPLOY_DIR "ThunderAuto/gp4_to_cs.csv"    };

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
