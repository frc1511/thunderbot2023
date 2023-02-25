#pragma once

#include <Basic/Mechanism.h>
#include <frc/Timer.h>
#include <Trajectory/CSVTrajectory.h>
#include <Trajectory/LinearTrajectory.h>
#include <Autonomous/Action.h>
#include <map>
#include <cstdint>
#include <numbers>
#include <array>

#define DEPLOY_DIR "/home/lvuser/deploy/"

#define DIST_TO_CS_CENTER ((0.97_m / 2.0) + (80.4_in / 2.0))

class WhooshWhoosh;
class Drive;
class GamePiece;

class Autonomous : public Mechanism {
public:
    Autonomous(WhooshWhoosh* whooshWhoosh, Drive* drive, GamePiece* gamePiece);
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
    void runTrajectory(Trajectory* trajectory);

    bool scoreGamePiece();

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

    int step = 0;
    int traverseChargeStationStep = 0;
    int balanceChargeStationStep = 0;
    int scoreGamePieceStep = 0;

    LinearTrajectory balanceChargeStationTrajectory { frc::Pose2d(5.35_m, 2.7_m, 0_deg), frc::Pose2d(5.35_m, 2.7_m + DIST_TO_CS_CENTER, 0_deg), 1.5_mps, 2_mps_sq };
    LinearTrajectory traverseChargeStationTrajectory { frc::Pose2d(5.35_m, 2.7_m, 0_deg), frc::Pose2d(5.35_m, 2.7_m + (2 * DIST_TO_CS_CENTER), 0_deg), 1.5_mps, 2_mps_sq };

    WhooshWhoosh* whooshWhoosh;
    Drive* drive;
    GamePiece* gamePiece;

    enum Path {
        GRID1_TO_GP1 = 0,
        GRID2_TO_GP1 = 1,
        GP1_TO_GRID1 = 2,
        GP1_TO_GRID2 = 3,
        GP1_TO_CS    = 4,
        GRID4_TO_CS  = 5,
        GRID5_TO_CS  = 6,
        CS_TO_GP3    = 7,
        GP3_TO_CS    = 8,
        GRID6_TO_GP4 = 9,
        GRID7_TO_GP4 = 10,
        GP4_TO_GRID6 = 11,
        GP4_TO_GRID7 = 12,
        GP4_TO_CS    = 13,

        PATH_NUM = 14,
    };

    std::array<CSVTrajectory, Path::PATH_NUM> bluePaths = {
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid1_to_gp1.csv", false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid2_to_gp1.csv", false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp1_to_grid1.csv", false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp1_to_grid2.csv", false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp1_to_cs.csv",    false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid4_to_cs.csv",  false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid5_to_cs.csv",  false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/cs_to_gp3.csv",    false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp3_to_cs.csv",    false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid6_to_gp4.csv", false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid7_to_gp4.csv", false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp4_to_grid6.csv", false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp4_to_grid7.csv", false },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp4_to_cs.csv",    false },
    };

    std::array<CSVTrajectory, Path::PATH_NUM> redPaths = {
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid1_to_gp1.csv", true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid2_to_gp1.csv", true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp1_to_grid1.csv", true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp1_to_grid2.csv", true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp1_to_cs.csv",    true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid4_to_cs.csv",  true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid5_to_cs.csv",  true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/cs_to_gp3.csv",    true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp3_to_cs.csv",    true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid6_to_gp4.csv", true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/grid7_to_gp4.csv", true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp4_to_grid6.csv", true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp4_to_grid7.csv", true },
        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/gp4_to_cs.csv",    true },
    };

    std::array<CSVTrajectory, Path::PATH_NUM>* paths = &bluePaths;

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
