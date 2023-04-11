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

    bool isBalancing();

private:
    enum class AutoMode {
        DO_NOTHING     = 0,
        DRIVE_FORWARDS = 1,
        SCORE          = 2,
        BARRIER_2GP    = 3,
        BARRIER_2GP_CS = 4,
        BARRIER_3GP    = 5,
        CENTER_1GP_CS  = 6,
        EDGE_2GP       = 7,
        SCORE_BACKUP   = 8,
    };

    AutoMode selectedAutoMode = AutoMode::DO_NOTHING;

    // Does nothing or something.
    void doNothing();
    void scoreBackup();
    // Barrier autos.
    void barrier();
    void barrierFinish2GPCS();
    void barrierFinish3GP();
    // Center autos.
    void center();
    // Edge autos.
    void edge();
    // Drives forwards for a bit.
    void driveForwards();
    // Scores a cube and then we are happy.
    void score();

    const std::map<AutoMode, const char*> autoModeNames {
        { AutoMode::DO_NOTHING,     "Do Nothing"        },
        { AutoMode::DRIVE_FORWARDS, "Drive Forwards"    },
        { AutoMode::SCORE,          "Score"             },
        { AutoMode::BARRIER_2GP,    "Barrier: 2.5GP"    },
        { AutoMode::BARRIER_2GP_CS, "Barrier: 2.5GP+CS" },
        { AutoMode::BARRIER_3GP,    "Barrier: 3GP+CS"   },
        { AutoMode::CENTER_1GP_CS,  "Center: 1GP+CS"    },
        { AutoMode::EDGE_2GP,       "Edge: 2GP"         },
    };

    frc::Timer delayTimer,
               autoTimer;

    frc::Timer mobilityTimer;

    int step = 0;

    WhooshWhoosh* whooshWhoosh;
    Drive* drive;
    GamePiece* gamePiece;

    enum class Path {
        BARRIER_1,
        BARRIER_2,
        BARRIER_3,
        BARRIER_FINAL_SCORE,
        BARRIER_FINAL_BALANCE,
        EDGE_1,
        EDGE_2,
    };

    const std::map<Path, CSVTrajectory> bluePaths {
        { Path::BARRIER_1,             CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_1.csv",             false } },
        { Path::BARRIER_2,             CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_2.csv",             false } },
        { Path::BARRIER_3,             CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_3.csv",             false } },
        { Path::BARRIER_FINAL_SCORE,   CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_final_score.csv",   false } },
        { Path::BARRIER_FINAL_BALANCE, CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_final_balance.csv", false } },
        { Path::EDGE_1,                CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_1.csv",                false } },
        { Path::EDGE_2,                CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_2.csv",                false } },
    };

    const std::map<Path, CSVTrajectory> redPaths {
        { Path::BARRIER_1,             CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_1.csv",             true } },
        { Path::BARRIER_2,             CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_2.csv",             true } },
        { Path::BARRIER_3,             CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_3.csv",             true } },
        { Path::BARRIER_FINAL_SCORE,   CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_final_score.csv",   true } },
        { Path::BARRIER_FINAL_BALANCE, CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_final_balance.csv", true } },
        { Path::EDGE_1,                CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_1.csv",                true } },
        { Path::EDGE_2,                CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_2.csv",                true } },
    };

    CSVTrajectory testing_line { DEPLOY_DIR "ThunderAuto/test_line.csv", false };

    const std::map<Path, CSVTrajectory>* paths = nullptr;

    /**
     * An action that scores the current game piece.
    */
    class ScoreAction : public Action {
    public:
        ScoreAction(GamePiece* gamePiece);
        ~ScoreAction();

        void setLevel(int lvl);

        Result process() override;
    private:
        GamePiece* gamePiece;
        int step = 0;
        int lvl = 2;
    };

    /**
     * Balances the robot on the charge station.
     */
    class BalanceAction : public Action {
    public:
        BalanceAction(Drive* drive, WhooshWhoosh* whooshWhoosh);
        ~BalanceAction();

        Result process() override;
    private:
        Drive* drive;
        WhooshWhoosh* whooshWhoosh;
        int step = 0;
    };

    /**
     * Traverses the charge station, then balances the robot on it.
     */
    class BalanceMobilityAction : public Action {
    public:
        BalanceMobilityAction(Drive* drive, WhooshWhoosh* whooshWhoosh, BalanceAction* balanceAction);
        ~BalanceMobilityAction();

        Result process() override;
    private:
        Drive* drive;
        WhooshWhoosh* whooshWhoosh;
        BalanceAction* balanceAction;
        int step = 0;
        frc::Timer stopTimer;
        frc::Timer forwardsTimer;
    };

    ScoreAction scoreAction { gamePiece };
    BalanceAction balanceAction { drive, whooshWhoosh };
    BalanceMobilityAction balanceMobilityAction { drive, whooshWhoosh, &balanceAction };

    frc::Timer liftTimer;

    /**
     * A map of the actions that are available to each autonomous mode.
     */
    std::map<u_int32_t, Action*> actions {
        { 1 << 0, &scoreAction },
        { 1 << 1, &balanceAction },
        { 1 << 2, &balanceMobilityAction },
    };
};
