#pragma once

#include <Basic/Mechanism.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <units/time.h>
#include <mutex>
#include <thread>
#include <map>

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

    int server_fd = 0;

    std::map<int, std::thread> connection_threads;
    void connectionThread(int client_fd);
};
