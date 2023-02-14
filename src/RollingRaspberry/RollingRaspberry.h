#pragma once

#include <Basic/Mechanism.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/geometry/Pose2d.h>
#include <units/time.h>
#include <units/length.h>
#include <units/angle.h>
#include <thread>
#include <mutex>
#include <map>
#include <cstddef>

#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>

/**
 * Represents an estimate of the robot's pose based on vision data.
 */
struct RobotPoseEstimate {
    units::second_t timestamp;

    frc::Pose3d pose1;
    frc::Pose3d pose2;

    double error1;
    double error2;

    /**
     * Returns whether the estimate is likely to be ambiguous.
     * 
     * Calculates the ratio of pose reprojection errors and returns true
     * if the ratio is above 0.2.
    */
    bool isAmbiguous() const;
};

/**
 * Mechanism that handles communication with the Raspberry Pis on the robot.
 */
class RollingRaspberry : public Mechanism {
public:
    RollingRaspberry();
    ~RollingRaspberry();

    void process() override;

    /**
     * Returns the most recent estimated robot poses.
     */
    std::map<units::second_t, frc::Pose2d> getEstimatedRobotPoses();

private:
    /**
     * Validates a pose by checking several things:
     *   - Whether the pose is within the field boundaries.
     *   - Whether the pose is above the field or below the field.
     */
    bool validatePose(frc::Pose3d pose) const;

    bool serverRunning = false;
    void initServer();

    bool socketCreated = false;
    bool socketBound = false;
    bool socketListening = false;

    int serverFileDesc = 0;

    std::mutex connectionMutex;

    std::map<int, std::thread> connectionThreads;
    void connectionThread(std::thread& self, int clientFileDesc);

    std::vector<RobotPoseEstimate> robotPoseEstimates;
};
