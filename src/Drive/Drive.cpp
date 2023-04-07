#include <Drive/Drive.h>
#include <WhooshWhoosh/WhooshWhoosh.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <frc/geometry/Twist2d.h>
#include <frc/DriverStation.h>
#include <Util/Parser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>
#include <map>

// The file which magnetic encoder offsets are stored on the RoboRIO.
#define ENCODER_OFFSETS_PATH std::filesystem::path("/home/lvuser/drive_encoder_offsets.txt")

#define FIELD_X 16.54175_m
#define FIELD_Y 8.0137_m

static const std::map<int, frc::Pose2d> BLUE_ALIGNMENT_POSES = {
    { 0, frc::Pose2d(1.86_m, 5.09_m, 180_deg) },
    { 1, frc::Pose2d(1.86_m, 4.46_m, 180_deg) },
    { 2, frc::Pose2d(1.86_m, 3.88_m, 180_deg) },
    { 3, frc::Pose2d(1.86_m, 3.3_m,  180_deg) },
    { 4, frc::Pose2d(1.86_m, 2.77_m, 180_deg) },
    { 5, frc::Pose2d(1.86_m, 2.17_m, 180_deg) },
    { 6, frc::Pose2d(1.86_m, 1.58_m, 180_deg) },
    { 7, frc::Pose2d(1.86_m, 1.02_m, 180_deg) },
    { 8, frc::Pose2d(1.86_m, 0.4_m,  180_deg) }
};

static const std::map<int, frc::Pose2d> RED_ALIGNMENT_POSES = {
    { 0, frc::Pose2d(FIELD_X - 1.86_m, 5.09_m, 0_deg) },
    { 1, frc::Pose2d(FIELD_X - 1.86_m, 4.46_m, 0_deg) },
    { 2, frc::Pose2d(FIELD_X - 1.86_m, 3.88_m, 0_deg) },
    { 3, frc::Pose2d(FIELD_X - 1.86_m, 3.3_m,  0_deg) },
    { 4, frc::Pose2d(FIELD_X - 1.86_m, 2.77_m, 0_deg) },
    { 5, frc::Pose2d(FIELD_X - 1.86_m, 2.17_m, 0_deg) },
    { 6, frc::Pose2d(FIELD_X - 1.86_m, 1.58_m, 0_deg) },
    { 7, frc::Pose2d(FIELD_X - 1.86_m, 1.02_m, 0_deg) },
    { 8, frc::Pose2d(FIELD_X - 1.86_m, 0.4_m,  0_deg) }
};

static const std::map<int, units::meter_t> GRID_ZONES = {
    { 0, 5.47_m },
    { 1, 3.58_m },
    { 2, 1.87_m },
    { 3, 0.00_m },
}; 

Drive::Drive(WhooshWhoosh* _whooshWhoosh, RollingRaspberry* _rollingRaspberry)
: rollingRaspberry(_rollingRaspberry), whooshWhoosh(_whooshWhoosh), driveController(
    [&]() -> frc::HolonomicDriveController {
        // Set the angular PID controller range from -180 to 180 degrees.
        trajectoryThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
        // Setup the drive controller with the individual axis PID controllers.
        return frc::HolonomicDriveController(xPIDController, yPIDController, trajectoryThetaPIDController);
    } ()) {

    manualThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));

    whooshWhoosh->resetHeading();

    // Apply the magnetic encoder offsets if the config file exists.
    if (readOffsetsFile()) {
        applyOffsets();
    }

    // Enable the trajectory drive controller.
    driveController.SetEnabled(true);

    // driveController.SetTolerance(frc::Pose2d(5_in, 5_in, 5_deg));
}

Drive::~Drive() {
    for (SwerveModule* module : swerveModules) {
        delete module;
    }
}

void Drive::doPersistentConfiguration() {
    for (SwerveModule* module : swerveModules) {
        module->doPersistentConfiguration();
    }
    configMagneticEncoders();
}

void Drive::resetToMode(MatchMode mode) {
    driveMode = DriveMode::STOPPED;

    // Reset the manual control data.
    VelocityControlData lastControlData(controlData);
    controlData = { 0_mps, 0_mps, 0_rad_per_s, ControlFlag::NONE };

    trajectoryThetaPIDController.Reset(getRotation().Radians());

    // Reset the rate limiters to 0.
    // driveRateLimiter.Reset(0_mps);
    // turnRateLimiter.Reset(0_rad_per_s);

    // This seems to be necessary. Don't ask me why.
    for (SwerveModule* module : swerveModules) {
        module->stop();
    }

    if (mode == MatchMode::DISABLED) {
        /**
         * Coast all motors in disabled (good for transportation, however can
         * lead to some runaway robots).
         */
        setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);

        teleopTimer.Stop();
        trajectoryTimer.Stop();

        // If the drivetrain movement was being recorded before being disabled.
        if (lastControlData.flags & ControlFlag::RECORDING) {
            /**
             * Export the recorded trajectory to a CSV file on the RoboRIO so
             * it can be examined and/or played back later.
             */
            trajectoryRecorder.writeToCSV(RECORDED_TRAJ_PATH);

            // Reload the recorded trajectory from the file.
            reloadRecordedTrajectory();
        }
    }
    else {
        // Brake all motors when enabled to help counteract pushing.
        setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);

        /**
         * Calibrate the IMU if not already calibrated. This will cause the
         * robot to pause for 4 seconds while it waits for it to calibrate, so
         * the IMU should always be calibrated before the match begins.
         */
        if (!isIMUCalibrated()) {
            calibrateIMU();
        }

        // Reset the teleop timer.
        teleopTimer.Reset();
        lastTeleopTime = 0_s;

        if (mode == MatchMode::TELEOP) {
            teleopTimer.Start();

            // Clear the recorded trajectory.
            trajectoryRecorder.clear();
        }

        // Reset the trajectory timer.
        trajectoryTimer.Reset();

        if (mode == MatchMode::AUTO) {
            trajectoryTimer.Start();

            // Clear the motion profile CSV file.
            trajectoryMotionFile.clear();
            
            // Write the header of the CSV file.
            trajectoryMotionFile <<
                "time,x_pos,y_pos,dest_x_pos,dest_y_pos,vel_x,vel_y,vel_ang,ang,dest_ang\n";
        }

        // Reset the position and rotation on the field.
        // resetOdometry();
    }

    static bool wasAuto = false;

    // Going from Auto to Disabled to Teleop.
    if (wasAuto && mode == Mechanism::MatchMode::TELEOP) {
        wasAuto = false;
        frc::Pose2d currPose(getEstimatedPose());
        resetOdometry(frc::Pose2d(currPose.X(), currPose.Y(), currPose.Rotation().Degrees() + 90_deg));
    }
    else {
        // Check if going from Auto to Disabled.
        wasAuto = getLastMode() == Mechanism::MatchMode::AUTO && mode == Mechanism::MatchMode::DISABLED;

        // Doing something else.
        if (!wasAuto && mode != Mechanism::MatchMode::DISABLED) {
            // Stuff to reset normally.
        }
    }
}

void Drive::process() {
    updateOdometry();

    switch (driveMode) {
        case DriveMode::STOPPED:
            execStopped();
            break;
        case DriveMode::VELOCITY:
            execVelocityControl();
            break;
        case DriveMode::TRAJECTORY:
            execTrajectory();
            break;
    }
}

void Drive::manualControlRelRotation(double xPct, double yPct, double angPct, unsigned flags) {
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel    = xPct * DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel    = yPct * DRIVE_MANUAL_MAX_VEL;
    units::radians_per_second_t angVel = angPct * DRIVE_MANUAL_MAX_ANG_VEL;

    // Pass the velocities to the velocity control function.
    velocityControlRelRotation(xVel, yVel, angVel, flags);
}

void Drive::manualControlAbsRotation(double xPct, double yPct, units::radian_t angle, unsigned flags) {
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel    = xPct * DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel    = yPct * DRIVE_MANUAL_MAX_VEL;

    // Pass the velocities to the velocity control function.
    velocityControlAbsRotation(xVel, yVel, angle, flags);
}

void Drive::velocityControlRelRotation(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t angVel, unsigned flags) {
    units::meters_per_second_t newXVel    = xVel;
    units::meters_per_second_t newYVel    = yVel;
    units::radians_per_second_t newAngVel = angVel;
    
    // Apply the locked axis flags.
    if (flags & ControlFlag::LOCK_X) newXVel = 0_mps;
    if (flags & ControlFlag::LOCK_Y) newYVel = 0_mps;
    if (flags & ControlFlag::LOCK_ROT) newAngVel = 0_rad_per_s;

    // Stop the robot in brick mode no matter what.
    if (flags & ControlFlag::BRICK) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot isn't being told to do move, sooo.... stop??
    else if ((!newXVel && !newYVel && !newAngVel)) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot is being told to do stuff, so start doing stuff.
    else {
        driveMode = DriveMode::VELOCITY;
    }

    controlData = { newXVel, newYVel, newAngVel, flags };
}

void Drive::velocityControlAbsRotation(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radian_t angle, unsigned flags) {
    units::radian_t currAngle = getEstimatedPose().Rotation().Radians();

    // Reset the PID controller if this is the first run.
    static bool firstRun = true;
    if (firstRun) {
        manualThetaPIDController.Reset(currAngle);
        firstRun = false;
    }

    auto angVel = units::radians_per_second_t(
        manualThetaPIDController.Calculate(currAngle, angle)
    );

    velocityControlRelRotation(xVel, yVel, angVel, flags);
}

void Drive::runTrajectory(const Trajectory* _trajectory, const std::map<u_int32_t, Action*>& actionMap) {
    driveMode = DriveMode::TRAJECTORY;
    // Set the trajectory.
    trajectory = _trajectory;

    // Set the initial action.
    trajectoryActionIter = trajectory->getActions().cbegin();

    trajectoryActions = &actionMap;

    // Reset done trajectory actions.
    doneTrajectoryActions.clear();

    // Reset the trajectory timer.
    trajectoryTimer.Reset();
    trajectoryTimer.Start();

    // Get the initial pose of the robot.
    // frc::Pose2d initialPose(trajectory->getInitialPose());
    // // Adjust the rotation because everything about this robot is 90 degrees off D:
    // initialPose = frc::Pose2d(initialPose.X(), initialPose.Y(), (initialPose.Rotation().Degrees() - 90_deg));

    // // Reset the odometry to the initial pose.
    // resetOdometry(initialPose);
}

void Drive::goToPose(frc::Pose2d pose) {
    goToPoseTrajectory = std::make_unique<LinearTrajectory>(
        getEstimatedPose(), pose, DRIVE_AUTO_MAX_VEL, DRIVE_AUTO_MAX_ACCEL
    );

    runTrajectory(goToPoseTrajectory.get(), *trajectoryActions);
}

bool Drive::alignToGrid(AlignmentDirection direction) {
    // Get the alliance color.
    int isBlue = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? 1 : -1;

    // The index of the alignment position (0-8).
    aligningGridNode = -1;

    // Determine which grid the robot is in front of.
    if (alignedGrid == 0) {
        // Positions 0-2.
        aligningGridNode = 1 + static_cast<int>(direction) * isBlue;
    }
    else if (alignedGrid == 1) {
        // Positions 3-5.
        aligningGridNode = 4 + static_cast<int>(direction) * isBlue;
    }
    else if (alignedGrid == 2) {
        // Positions 6-8.
        aligningGridNode = 7 + static_cast<int>(direction) * isBlue;
    }

    if (aligningGridNode == -1) {
        // No alignment for you.
        return false;
    }

    // Which poses to use.
    const auto& alignment_poses = isBlue == 1 ? BLUE_ALIGNMENT_POSES : RED_ALIGNMENT_POSES;

    // Go to the alignment pose.
    goToPose(alignment_poses.at(aligningGridNode));

    return true;
}

bool Drive::isFinished() const {
    // Stopped is as 'finished' as it gets I guess.
    return driveMode == DriveMode::STOPPED || driveMode == DriveMode::VELOCITY;
}

void Drive::calibrateIMU() {
    whooshWhoosh->calibrateIMU();
    
    imuCalibrated = true;

    resetOdometry();
}

bool Drive::isIMUCalibrated() {
    return imuCalibrated;
}

void Drive::resetOdometry(frc::Pose2d pose) {
    /**
     * Resets the position and rotation of the robot to a given pose
     * while ofsetting for the IMU's recorded rotation.
     */

    poseEstimator.ResetPosition(getRotation(), getModulePositions(), pose);

    whooshWhoosh->setHeadingAngle(0_deg);

    for (SwerveModule* module : swerveModules) {
        module->resetDrivePosition();
    }
}

frc::Pose2d Drive::getEstimatedPose() {
    return poseEstimator.GetEstimatedPosition();
}

frc::Rotation2d Drive::getRotation() {
    // The raw rotation from the IMU.
    return frc::Rotation2d(whooshWhoosh->getHeadingAngle());
}

void Drive::reloadRecordedTrajectory() {
    recordedTrajectory = CSVTrajectory(RECORDED_TRAJ_PATH);
}

void Drive::updateOdometry() {
    /**
     * Update the pose estimator with encoder measurements from
     * the swerve modules.
     */
    poseEstimator.Update(getRotation(), getModulePositions());

    /**
     * Update the pose estimator with the estimated poses from
     * RollingRaspberry.
     */
    const auto estimatedPoses = rollingRaspberry->getEstimatedRobotPoses();

    for (const auto& [timestamp, pose] : estimatedPoses) {
        poseEstimator.AddVisionMeasurement(pose, timestamp);
    }
}

void Drive::updateGridAlignment() {
    alignedGrid = -1;

    frc::Pose2d pose = getEstimatedPose();

    // Determine which grid the robot is in front of.
    if (pose.Y() < GRID_ZONES.at(0) && pose.Y() >= GRID_ZONES.at(1)) {
        // Positions 0-2.
        alignedGrid = 0;
    }
    else if (pose.Y() < GRID_ZONES.at(1) && pose.Y() >= GRID_ZONES.at(2)) {
        // Positions 3-5.
        alignedGrid = 1;
    }
    else if (pose.Y() < GRID_ZONES.at(2) && pose.Y() >= GRID_ZONES.at(3)) {
        // Positions 6-8.
        alignedGrid = 2;
    }
}

void Drive::execStopped() {
    // Set the speeds to 0.
    setModuleStates({ 0_mps, 0_mps, 0_deg_per_s });

    // Just for feedback.
    targetPose = getEstimatedPose();

    aligningGridNode = -1;

    // Put the drivetrain into brick mode if the flag is set.
    if (controlData.flags & ControlFlag::BRICK) {
        makeBrick();
    }

    // If recording, record the current state of the robot.
    if (controlData.flags & ControlFlag::RECORDING) {
        record_state();
    }
}

void Drive::execVelocityControl() {
    // The velocity of the robot using the component velocities.
    // units::meters_per_second_t vel = units::math::hypot(controlData.xVel, controlData.yVel);

    // The heading of the robot's velocity.
    // units::radian_t head = units::math::atan2(controlData.yVel, controlData.xVel);

    // Adjust the velocity using the configured acceleration and deceleration limits.
    // vel = driveRateLimiter.Calculate(vel);

    // Calculate the new component velocities.
    // xVel = units::math::cos(head) * vel;
    // yVel = units::math::sin(head) * vel;

    // Adjust the angular velocity using the configured acceleration and deceleration limits.
    // angVel = turnRateLimiter.Calculate(angVel);

    frc::Pose2d currPose(getEstimatedPose());
    
    frc::ChassisSpeeds velocities;

    // Generate chassis speeds depending on the control mode.
    if (controlData.flags & ControlFlag::FIELD_CENTRIC) {
        // Generate chassis speeds based on the rotation of the robot relative to the field.
        velocities = frc::ChassisSpeeds::FromFieldRelativeSpeeds(controlData.xVel, controlData.yVel, controlData.angVel, currPose.Rotation());//whooshWhoosh->getHeadingAngle());// currPose.Rotation());
    }
    else {
        // Chassis speeds are robot-centric.
        velocities = { controlData.xVel, controlData.yVel, controlData.angVel };
    }

    // Just for feedback.
    targetPose = currPose;

    // Set the modules to drive at the given velocities.
    setModuleStates(velocities);

    // If recording, record the current state of the robot.
    if (controlData.flags & ControlFlag::RECORDING) {
        record_state();
    }
}

void Drive::execTrajectory() {
    units::second_t time(trajectoryTimer.Get());

    int actionRes = 0;
    bool execAction = false;

    // If we've got another action to go.
    if (trajectoryActionIter != trajectory->getActions().cend()) {
        const auto& [action_time, actions] = *trajectoryActionIter;

        // Check if it's time to execute the action.
        if (time >= action_time) {
            execAction = true;

            // Loop through the available actions.
            for (auto it(trajectoryActions->cbegin()); it != trajectoryActions->cend(); ++it) {
                const auto& [id, action] = *it;

                // Narrow the list down to only actions that have not been completed yet.
                if (std::find(doneTrajectoryActions.cbegin(), doneTrajectoryActions.cend(), id) == doneTrajectoryActions.cend()) {
                    // If the action's bit is set in the bit field.
                    if (actions & id) {
                        // Execute the action.
                        Action::Result res = action->process();

                        // If the action has completed.
                        if (res == Action::Result::DONE) {
                            // Remember that it's done.
                            doneTrajectoryActions.push_back(id);
                        }

                        actionRes += res;
                    }
                }
            }
        }
    }

    // Stop the trajectory because an action is still running.
    if (actionRes) {
        trajectoryTimer.Stop();
    }
    // Continue/Resume the trajectory because the actions are done.
    else {
        // Increment the action if an action was just finished.
        if (execAction) {
            ++trajectoryActionIter;
            doneTrajectoryActions.clear();
        }
        trajectoryTimer.Start();
    }

    // If the trajectory is done, then stop it.
    if (time > (trajectory->getDuration() + 0.0_s)) {// && driveController.AtReference()) { 
        driveMode = DriveMode::STOPPED;
        return;
    }

    // Sample the trajectory at the current time for the desired state of the robot.
    Trajectory::State state(trajectory->sample(time));

    // Don't be moving if an action is being worked on.
    if (actionRes) {
        state.velocity = 0_mps;
    }

    // Adjust the rotation because everything about this robot is 90 degrees off D:
    state.pose = frc::Pose2d(state.pose.Translation(), state.pose.Rotation() - 90_deg);

    // The current pose of the robot.
    frc::Pose2d currentPose(getEstimatedPose());

    // The desired change in position.
    frc::Twist2d twist(currentPose.Log(state.pose));

    // The angle at which the robot should be driving at.
    frc::Rotation2d heading;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        heading = frc::Rotation2d(units::math::atan2(twist.dy, twist.dx) + 90_deg);
    }
    else {
        heading = frc::Rotation2d(units::math::atan2(twist.dy, twist.dx) - 90_deg);
    }
    

    /**
     * Calculate the chassis velocities based on the error between the current
     * pose and the desired pose.
     */
    frc::ChassisSpeeds velocities(
        driveController.Calculate(
            currentPose,
            frc::Pose2d(state.pose.X(), state.pose.Y(), heading),
            state.velocity,
            state.pose.Rotation()
        )
    );

    // Keep target pose for feedback.
    targetPose = state.pose;

    // Log motion to CSV file for debugging.
    trajectoryMotionFile << time.value() << ','
                         << currentPose.X().value() << ','
                         << currentPose.Y().value() << ','
                         << targetPose.X().value() << ','
                         << targetPose.Y().value() << ','
                         << velocities.vx.value() << ','
                         << velocities.vy.value() << ','
                         << velocities.omega.value() << ','
                         << currentPose.Rotation().Radians().value() << ','
                         << targetPose.Rotation().Radians().value() << '\n';

    // Make the robot go vroom :D
    setModuleStates(velocities);
}

void Drive::record_state() {
    units::second_t currTime(teleopTimer.Get());
    frc::Pose2d currPose(getEstimatedPose());

    // Add the state to the trajectory recorder.
    trajectoryRecorder.addState(
        // The delta time between logged states.
        currTime - lastTeleopTime,
        // Add 90 degrees because everything on this robot is 90 degrees off.
        frc::Pose2d(currPose.X(), currPose.Y(), currPose.Rotation().Degrees() + 90_deg)
    );

    lastTeleopTime = currTime;
}

void Drive::makeBrick() {
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        units::degree_t angle;
        // If the index is even.
        if (i % 2 == 0) {
            angle = -45_deg;
        }
        // If the index is odd.
        else {
            angle = 45_deg;
        }
        
        // Stop the robot. It should already be stopped tho.
        driveMode = DriveMode::STOPPED;
        
        // Turn the swerve module to point towards the center of the robot.
        swerveModules.at(i)->setTurningMotor(angle);
    }
}

void Drive::configMagneticEncoders() {
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        /**
         * Get the current rotation of the swerve modules to save as the
         * forward angle of the module. This is a necessary step because
         * when installed, each magnetic encoder has its own offset.
         */
        units::radian_t angle(swerveModules.at(i)->getRawRotation());
        
        // Set the current angle as the new offset angle.
        offsets.at(i) = angle;
    }

    fmt::print("0: {}, 1: {}, 2: {}, 3: {}\n", offsets.at(0).value(), offsets.at(1).value(), offsets.at(2).value(), offsets.at(3).value());

    // Write the new offsets to the offsets file.
    writeOffsetsFile();

    // Apply the new offsets to the swerve modules.
    applyOffsets();
}

bool Drive::readOffsetsFile() {
    std::string fileStr = Parser::getFile(ENCODER_OFFSETS_PATH);
    if (fileStr.empty()) exit(1);

    Parser::Iter currIter = fileStr.cbegin();

    std::string line;
    for (int i = 0; i < 4; ++i) {
        offsets.at(i) = units::radian_t(Parser::parseNumber(currIter, fileStr.cend()));
        ++currIter;
    }

    return true;
}

void Drive::writeOffsetsFile() {
    // Open the file (Will create a new file if it does not already exist).
    std::ofstream offsetsFile(ENCODER_OFFSETS_PATH);

    // Clear the contents of the file.
    offsetsFile.clear();

    // Write each offset to the file (in radians).
    for (units::radian_t offset : offsets) {
        offsetsFile << offset.value() << '\n';
    }
}

void Drive::applyOffsets() {
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->setOffset(offsets.at(i) - 90_deg);
    }
}

void Drive::setIdleMode(ThunderCANMotorController::IdleMode mode) {
    for (SwerveModule* module : swerveModules) {
        module->setIdleMode(mode);
    }
}

void Drive::setModuleStates(frc::ChassisSpeeds speeds) {
    // Store velocities for feedback.
    chassisSpeeds = speeds;

    // Generate individual module states using the chassis velocities.
    wpi::array<frc::SwerveModuleState, 4> moduleStates(kinematics.ToSwerveModuleStates(speeds));
    
    // Set the states of the individual modules.
    for(std::size_t i = 0; i < swerveModules.size(); i++) {
      swerveModules.at(i)->setState(moduleStates.at(i));
    }
}

wpi::array<frc::SwerveModuleState, 4> Drive::getModuleStates() {
    return { swerveModules.at(0)->getState(), swerveModules.at(1)->getState(),
             swerveModules.at(2)->getState(), swerveModules.at(3)->getState() };
}

wpi::array<frc::SwerveModulePosition, 4> Drive::getModulePositions() {
    return { swerveModules.at(0)->getPosition(), swerveModules.at(1)->getPosition(),
             swerveModules.at(2)->getPosition(), swerveModules.at(3)->getPosition() };
}

void Drive::sendFeedback() {
    // Module feedback.
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->sendFeedback(i);
    }

    frc::Pose2d pose(getEstimatedPose());

    // Drive feedback.
    frc::SmartDashboard::PutNumber("Drive_PoseY_m",                 pose.X().value());
    frc::SmartDashboard::PutNumber("Drive_PoseX_m",                 pose.Y().value());
    frc::SmartDashboard::PutNumber("Drive_PoseRot_deg",             getRotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelX_mps",         controlData.xVel.value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelY_mps",         controlData.yVel.value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelRot_rad_per_s", controlData.angVel.value());
    frc::SmartDashboard::PutBoolean("Drive_FieldCentric",           controlData.flags & ControlFlag::FIELD_CENTRIC);
    frc::SmartDashboard::PutBoolean("Drive_Brick",                  controlData.flags & ControlFlag::BRICK);
    frc::SmartDashboard::PutBoolean("Drive_Recording",              controlData.flags & ControlFlag::RECORDING);
    frc::SmartDashboard::PutBoolean("Drive_LockX",                  controlData.flags & ControlFlag::LOCK_X);
    frc::SmartDashboard::PutBoolean("Drive_LockY",                  controlData.flags & ControlFlag::LOCK_Y);
    frc::SmartDashboard::PutBoolean("Drive_LockRot",                controlData.flags & ControlFlag::LOCK_ROT);

    // ThunderDashboard things.
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_x_pos",        pose.X().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_y_pos",        pose.Y().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_target_x_pos", targetPose.X().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_target_y_pos", targetPose.Y().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_x_vel",        chassisSpeeds.vx.value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_y_vel",        chassisSpeeds.vy.value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_ang_vel",      chassisSpeeds.omega.value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_ang",          pose.Rotation().Radians().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_target_ang",   targetPose.Rotation().Radians().value());

    frc::SmartDashboard::PutNumber("thunderdashboard_score_grid",        alignedGrid);
    frc::SmartDashboard::PutNumber("thunderdashboard_score_grid_column", aligningGridNode % 3);

    frc::SmartDashboard::PutBoolean("thunderdashboard_gyro", !imuCalibrated);
}
