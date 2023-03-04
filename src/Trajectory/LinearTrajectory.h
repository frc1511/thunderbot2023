#pragma once

#include <Trajectory/Trajectory.h>
#include <frc/geometry/Twist2d.h>

/**
 * Represents a Linear trajectory for the robot to follow.
 * 
 * The fastest path between two points is a straight line :D
 */
class LinearTrajectory : public Trajectory {
public:
    LinearTrajectory(frc::Pose2d startPose, frc::Pose2d endPose,
                     units::meters_per_second_t maxVel,
                     units::meters_per_second_squared_t maxAccel,
                     units::meters_per_second_t startVel = 0_mps,
                     units::meters_per_second_t endVel = 0_mps);
    ~LinearTrajectory();

    /**
     * Samples the trajectory at a specified time.
     */
    State sample(units::second_t time) const;

    /**
     * Returns the duration in seconds of the trajectory.
     */
    inline units::second_t getDuration() const { return duration; }

    /**
     * Returns the initial position of the robot.
     */
    inline frc::Pose2d getInitialPose() const { return startPose; }

private:
    std::map<units::second_t, u_int32_t> actions;

    frc::Pose2d startPose;
    frc::Pose2d endPose;
    units::meters_per_second_t startVel;
    units::meters_per_second_t endVel;
    units::meters_per_second_t maxVel;
    units::meters_per_second_squared_t maxAccel;

    frc::Rotation2d heading;

    // Time of the entire trajectory.
    units::second_t duration;

    // Time to accelerate from start velocity to max velocity.
    units::second_t accelTime;
    units::meter_t accelDist;
    // Time to cruise at max velocity.
    units::second_t cruiseTime;
    units::meter_t cruiseDist;
    // Time to decelerate from max velocity to end velocity.
    units::second_t decelTime;
    units::meter_t decelDist;
};
