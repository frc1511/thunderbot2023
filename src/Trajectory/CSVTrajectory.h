#pragma once

#include <Trajectory/Trajectory.h>
#include <filesystem>

/**
 * Represents a ThunderAuto-style CSV trajectory for the robot to follow.
 */
class CSVTrajectory : public Trajectory {
public:
    CSVTrajectory(std::filesystem::path path, bool inverted = false);
    ~CSVTrajectory();

    /**
     * Samples the trajectory at a specified time.
     */
    State sample(units::second_t time) const;

    /**
     * Returns the duration in seconds of the trajectory.
     */
    units::second_t getDuration() const;

    /**
     * Returns the initial position of the robot.
     */
    frc::Pose2d getInitialPose() const;

    /**
     * Returns the actions with their attributed timestamps.
     */
    inline const std::map<units::second_t, u_int32_t>& getActions() const override { return actions; }

private:
    std::map<units::second_t, State> states;
    std::map<units::second_t, u_int32_t> actions;
};
