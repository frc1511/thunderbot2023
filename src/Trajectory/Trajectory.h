#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <units/math.h>
#include <map>

/**
 * Base class for all types of trajectories.
 */
class Trajectory {
public:
    virtual ~Trajectory() = default;

    /**
     * Represents a single point in a trajectory.
     */
    struct State {
        // The target pose of the robot.
        frc::Pose2d pose;

        // The target velocity of the robot.
        units::meters_per_second_t velocity;
    };

    /**
     * Samples the trajectory at a specified time.
     */
    virtual State sample(units::second_t time) const = 0;

    /**
     * Returns the duration in seconds of the trajectory.
     */
    virtual units::second_t getDuration() const = 0;

    /**
     * Returns the initial position of the robot.
     */
    virtual frc::Pose2d getInitialPose() const = 0;

    /**
     * Returns the actions with their attributed timestamps.
     */
    virtual const std::map<units::second_t, u_int32_t>& getActions() const { return defaultActions; }

protected:
    Trajectory() = default;

private:
    std::map<units::second_t, u_int32_t> defaultActions;
};
