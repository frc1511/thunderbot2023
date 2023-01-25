#pragma once

#include <Trajectory/Trajectory.h>
#include <vector>

/**
 * Represents a generated cubic hermite-spline trajectory for the robot to follow.
 */
class HermiteSplineTrajectory : public Trajectory {
public:
    struct Config {
        units::meters_per_second_t maxVelocity;
        units::meters_per_second_squared_t maxAcceleration;
    };

    HermiteSplineTrajectory(State p0, units::degree_t m0, State p1, units::degree_t m1, Config config);
    HermiteSplineTrajectory(State p0, units::degree_t m0, State p1, Config config);
    HermiteSplineTrajectory(State p0, State p1, units::degree_t m1, Config config);
    ~HermiteSplineTrajectory();

    /**
     * Samples the trajectory at a specified time.
     */
    State sample(units::second_t time) override;

    /**
     * Returns the duration in seconds of the trajectory.
     */
    units::second_t getDuration() override;

    /**
     * Returns the initial position of the robot.
     */
    frc::Pose2d getInitialPose() override;

    /**
     * Returns the actions with their attributed timestamps.
     */
    inline const std::map<units::second_t, u_int32_t>& getActions() override { return actions; }

private:
    State p0, p1;
    double m0, m1;

    std::map<units::second_t, u_int32_t> actions;
};
