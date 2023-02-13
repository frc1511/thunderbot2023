#pragma once

#include <Basic/Mechanism.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <units/length.h>
#include <units/angle.h>
#include <Util/BetterThread.h>
#include <mutex>
#include <map>
#include <cstddef>

#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>

struct RobotPoseEstimate {
    frc::Pose3d pose1;
    frc::Pose3d pose2;

    double error1;
    double error2;

    double GetAmbiguity() const;
};

class RollingRaspberry : public Mechanism {
public:
    RollingRaspberry();
    ~RollingRaspberry();

    void process() override;

    /**
     * Returns the estimated poses of the robot calculated by the Raspberry
     * Pi's vision processing pipeline paired with a FPGA Timestamp.
     */
    std::map<units::second_t, frc::Pose2d> getEstimatedRobotPoses();

private:
    bool serverRunning = false;
    void initServer();

    bool socketCreated = false;
    bool socketBound = false;
    bool socketListening = false;

    int serverFileDesc = 0;

    std::mutex connectionMutex;

    std::map<int, std::thread> connectionThreads;
    void connectionThread(std::thread& self, int client_fd);

    std::vector<RobotPoseEstimate> robotPoseEstimates;
};
