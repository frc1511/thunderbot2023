#pragma once

#include <Basic/Mechanism.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableListener.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <units/time.h>
#include <map>
#include <mutex>

/**
 * Represents the Raspberry Pi Co-Processor on the Robot.
 */
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
    void poseCallback(std::string_view key, const nt::Event& event);

    // A network table used to communicate with the r-pi.
    std::shared_ptr<nt::NetworkTable> table;

    // Subtable containing pose estimates.
    std::shared_ptr<nt::NetworkTable> posesSubtable;

    bool init = false;

    std::map<units::second_t, frc::Pose2d> poses;

    NT_Listener poseListener;

    std::mutex posesMutex;
};
