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

#define DIST_TO_CS_CENTER ((0.95_m / 2.0) + (80.4_in / 2.0))

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
    enum class AutoMode {
        DO_NOTHING     = 0,
        BARRIER_3GP    = 1,
        BARRIER_2GP_CS = 2,
        CENTER_1GP     = 3,
        CENTER_1GP_CS  = 4,
        EDGE_3GP       = 5,
        EDGE_2GP_CS    = 6,
    };

    AutoMode selectedAutoMode = AutoMode::DO_NOTHING;

    void doNothing();
    void barrier3GP();
    void barrier2GP_CS();
    void center1GP();
    void center1GP_CS();
    void edge3GP();
    void edge2GP_CS();

    const std::map<AutoMode, const char*> autoModeNames {
        { AutoMode::DO_NOTHING,     "Do Nothing"      },
        { AutoMode::BARRIER_3GP,    "Barrier: 3GP"    },
        { AutoMode::BARRIER_2GP_CS, "Barrier: 2GP+CS" },
        { AutoMode::CENTER_1GP,     "Center: 1GP"     },
        { AutoMode::CENTER_1GP_CS,  "Center: 1GP+CS"  },
        { AutoMode::EDGE_3GP,       "Edge: 3GP"       },
        { AutoMode::EDGE_2GP_CS,    "Edge: 2GP+CS"    },
    };

    frc::Timer delayTimer,
               autoTimer;

    int step = 0;

    LinearTrajectory traverseChargeStationTrajectory { frc::Pose2d(5.35_m, 2.7_m, 0_deg), frc::Pose2d(5.35_m, 2.7_m + (2 * DIST_TO_CS_CENTER), 0_deg), 1.5_mps, 2_mps_sq };

    WhooshWhoosh* whooshWhoosh;
    Drive* drive;
    GamePiece* gamePiece;

    enum class Path {
        BARRIER_START,
        BARRIER_FINAL_SCORE,
        BARRIER_FINAL_BALANCE,
        CENTER_START,
        EDGE_START,
        EDGE_FINAL_SCORE,
        EDGE_FINAL_BALANCE,
    };

    const std::map<Path, CSVTrajectory> bluePaths {
        { Path::BARRIER_START,         CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_start.csv",         false } },
        { Path::BARRIER_FINAL_SCORE,   CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_final_score.csv",   false } },
        { Path::BARRIER_FINAL_BALANCE, CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_final_balance.csv", false } },
        { Path::CENTER_START,          CSVTrajectory{ DEPLOY_DIR "ThunderAuto/center_start.csv",          false } },
        { Path::EDGE_START,            CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_start.csv",            false } },
        { Path::EDGE_FINAL_SCORE,      CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_final_score.csv",      false } },
        { Path::EDGE_FINAL_BALANCE,    CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_final_balance.csv",    false } },
    };

    const std::map<Path, CSVTrajectory> redPaths {
        { Path::BARRIER_START,         CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_start.csv",         true } },
        { Path::BARRIER_FINAL_SCORE,   CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_final_score.csv",   true } },
        { Path::BARRIER_FINAL_BALANCE, CSVTrajectory{ DEPLOY_DIR "ThunderAuto/barrier_final_balance.csv", true } },
        { Path::CENTER_START,          CSVTrajectory{ DEPLOY_DIR "ThunderAuto/center_start.csv",          true } },
        { Path::EDGE_START,            CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_start.csv",            true } },
        { Path::EDGE_FINAL_SCORE,      CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_final_score.csv",      true } },
        { Path::EDGE_FINAL_BALANCE,    CSVTrajectory{ DEPLOY_DIR "ThunderAuto/edge_final_balance.csv",    true } },
    };

    const std::map<Path, CSVTrajectory>* paths = nullptr;

    /**
     * An action that scores the current game piece.
    */
    class ScoreAction : public Action {
    public:
        ScoreAction(GamePiece* gamePiece);
        ~ScoreAction();

        Result process() override;
    private:
        GamePiece* gamePiece;
        int step = 0;
    };

    /**
     * An action that scores the prepares the grabber to intake a cone.
     */
    class IntakeToConeAction : public Action {
    public:
        IntakeToConeAction(GamePiece* gamePiece);
        ~IntakeToConeAction();

        Result process() override;
    private:
        GamePiece* gamePiece;
    };

    /**
     * An action that waits for the grabber to intake a GamePiece.
     */
    class IntakeWaitAction : public Action {
    public:
        IntakeWaitAction(GamePiece* gamePiece);
        ~IntakeWaitAction();

        Result process() override;
    private:
        GamePiece* gamePiece;
    };

    /**
     * Puts the lift in the high position.
     */
    class LiftHighAction : public Action {
    public:
        LiftHighAction(GamePiece* gamePiece);
        ~LiftHighAction();

        Result process() override;
    private:
        GamePiece* gamePiece;
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
        BalanceMobilityAction(Drive* drive, WhooshWhoosh* whooshWhoosh);
        ~BalanceMobilityAction();

        Result process() override;
    private:
        Drive* drive;
        WhooshWhoosh* whooshWhoosh;
        int step = 0;
    };

    ScoreAction scoreAction { gamePiece };
    IntakeToConeAction intakeToConeAction { gamePiece };
    IntakeWaitAction intakeWaitAction { gamePiece };
    LiftHighAction liftHighAction { gamePiece };
    BalanceAction balanceAction { drive, whooshWhoosh };
    BalanceMobilityAction balanceMobilityAction { drive, whooshWhoosh };

    /**
     * A map of the actions that are available to each autonomous mode.
     */
    std::map<u_int32_t, Action*> actions {
        { 1 << 0, &scoreAction },
        { 1 << 1, &intakeToConeAction },
        { 1 << 2, &intakeWaitAction },
        { 1 << 3, &liftHighAction },
        { 1 << 4, &balanceAction },
        { 1 << 5, &balanceMobilityAction },
    };
};
