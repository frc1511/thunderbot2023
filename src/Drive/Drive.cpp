#include <Drive/Drive.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <frc/geometry/Twist2d.h>
#include <Util/Parser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>

// The file which magnetic encoder offsets are stored on the RoboRIO.
#define ENCODER_OFFSETS_PATH std::filesystem::path("/home/lvuser/drive_encoder_offsets.txt")

Drive::Drive(RollingRaspberry* rollingRaspberry)
: rollingRaspberry(rollingRaspberry), driveController(
    [&]() -> frc::HolonomicDriveController {
        // Set the angular PID controller range from -180 to 180 degrees.
        trajectoryThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
        // Setup the drive controller with the individual axis PID controllers.
        return frc::HolonomicDriveController(xPIDController, yPIDController, trajectoryThetaPIDController);
    } ()) {

    manualThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));

    // 4s is good?
    imu.configCalTime(ThunderIMU::CalibrationTime::_4s);
    imu.setYawAxis(ThunderIMU::Axis::Z);
    imu.reset();

    // Apply the magnetic encoder offsets if the config file exists.
    if (readOffsetsFile()) {
        applyOffsets();
    }

    // Enable the trajectory drive controller.
    driveController.SetEnabled(true);
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

    ManualControlData lastManualData(manualData);
    // Reset the manual control data.
    manualData = {};

    // Reset the rate limiters to 0.
    driveRateLimiter.Reset(0_mps);
    turnRateLimiter.Reset(0_rad_per_s);

    // This seems to be necessary. Don't ask me why.
    for (SwerveModule* module : swerveModules) {
        module->stop();
    }

    if (mode == MatchMode::DISABLED) {
        /**
         * Coast all motors in disabled (good for transportation, however can
         * lead to some runaway robots).
         */
        setIdleMode(ThunderCANMotorController::IdleMode::COAST);

        teleopTimer.Stop();
        trajectoryTimer.Stop();

        // If the drivetrain movement was being recorded before being disabled.
        if (lastManualData.flags & ControlFlag::RECORDING) {
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
        resetOdometry();
    }
}

void Drive::process() {
    updateOdometry();

    switch (driveMode) {
        case DriveMode::STOPPED:
            execStopped();
            break;
        case DriveMode::MANUAL:
            execManual();
            break;
        case DriveMode::TRAJECTORY:
            execTrajectory();
            break;
    }
}

void Drive::manualControlRelRotation(double xPct, double yPct, double angPct, unsigned flags) {
    double newXPct = xPct, newYPct = yPct, newAngPct = angPct;

    // Apply the locked axis flags.
    if (flags & ControlFlag::LOCK_X) newXPct = 0;
    if (flags & ControlFlag::LOCK_Y) newYPct = 0;
    if (flags & ControlFlag::LOCK_ROT) newAngPct = 0;

    // Stop the robot in brick mode no matter what.
    if (flags & ControlFlag::BRICK) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot isn't being told to do move, sooo.... stop??
    else if ((!newXPct && !newYPct && !newAngPct)) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot is being told to do stuff, so start doing stuff.
    else {
        driveMode = DriveMode::MANUAL;
    }

    manualData = { newXPct, newYPct, newAngPct, flags };
}

void Drive::manualControlAbsRotation(double xPct, double yPct, units::radian_t angle, unsigned flags) {
    units::radian_t currAngle = getEstimatedPose().Rotation().Radians();

    // Reset the PID controller.
    static bool firstRun = true;
    if (firstRun) {
        manualThetaPIDController.Reset(currAngle);
        firstRun = false;
    }

    auto angVel = units::radians_per_second_t(
        manualThetaPIDController.Calculate(currAngle, angle)
    );

    double angPct = (angVel / DRIVE_MANUAL_MAX_ANG_VEL).value();

    manualControlRelRotation(xPct, yPct, angPct, flags);
}

void Drive::runTrajectory(Trajectory* _trajectory, const std::map<u_int32_t, Action*>& actionMap) {
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
    frc::Pose2d initialPose(trajectory->getInitialPose());
    // Adjust the rotation because everything about this robot is 90 degrees off D:
    initialPose = frc::Pose2d(initialPose.X(), initialPose.Y(), (initialPose.Rotation().Degrees() - 90_deg));

    // Reset the odometry to the initial pose.
    resetOdometry(initialPose);
}

void Drive::goToPose(frc::Pose2d pose) {
    goToPoseTrajectory = std::make_unique<LinearTrajectory>(
        getEstimatedPose(), pose, DRIVE_AUTO_MAX_VEL, DRIVE_AUTO_MAX_ACCEL
    );

    runTrajectory(goToPoseTrajectory.get(), *trajectoryActions);
}

bool Drive::alignToGrid(AlignmentDirection direction) {
    // TODO: Implement.
}

bool Drive::isFinished() const {
    // Stopped is as 'finished' as it gets I guess.
    return driveMode == DriveMode::STOPPED || driveMode == DriveMode::MANUAL;
}

void Drive::calibrateIMU() {
    imu.calibrate();

    /**
     * Sleep the current thread manually because the IMU library doesn't
     * do it automatically for some reason D:
     */
    sleep(4);
    
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

    for (SwerveModule* module : swerveModules) {
        module->resetDrivePosition();
    }
}

frc::Pose2d Drive::getEstimatedPose() {
    return poseEstimator.GetEstimatedPosition();
}

frc::Rotation2d Drive::getRotation() {
    // The raw rotation from the IMU.
    units::degree_t imuAngle = imu.getAngle();

    return frc::Rotation2d(imuAngle);
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
        // poseEstimator.AddVisionMeasurement(pose, timestamp);
        poseEstimator.ResetPosition(getRotation(), getModulePositions(), pose);
    }
}

void Drive::execStopped() {
    // Set the speeds to 0.
    setModuleStates({ 0_mps, 0_mps, 0_deg_per_s });

    // Just for feedback.
    targetPose = getEstimatedPose();

    // Put the drivetrain into brick mode if the flag is set.
    if (manualData.flags & ControlFlag::BRICK) {
        makeBrick();
    }

    // If recording, record the current state of the robot.
    if (manualData.flags & ControlFlag::RECORDING) {
        record_state();
    }
}

void Drive::execManual() {
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel = manualData.xPct * DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel = manualData.yPct * DRIVE_MANUAL_MAX_VEL;
    units::radians_per_second_t angVel = manualData.angPct * DRIVE_MANUAL_MAX_ANG_VEL;

    // The velocity of the robot using the component velocities.
    units::meters_per_second_t vel = units::math::hypot(xVel, yVel);

    // The heading of the robot's velocity.
    units::radian_t head = units::math::atan2(yVel, xVel);

    // Adjust the velocity using the configured acceleration and deceleration limits.
    vel = driveRateLimiter.Calculate(vel);

    // Calculate the new component velocities.
    xVel = units::math::cos(head) * vel;
    yVel = units::math::sin(head) * vel;

    // Adjust the angular velocity using the configured acceleration and deceleration limits.
    angVel = turnRateLimiter.Calculate(angVel);

    frc::Pose2d currPose(getEstimatedPose());
    
    frc::ChassisSpeeds velocities;

    // Generate chassis speeds depending on the control mode.
    if (manualData.flags & ControlFlag::FIELD_CENTRIC) {
        // Generate chassis speeds based on the rotation of the robot relative to the field.
        velocities = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, angVel, currPose.Rotation());
    }
    else {
        // Chassis speeds are robot-centric.
        velocities = { xVel, yVel, angVel };
    }

    // Just for feedback.
    targetPose = currPose;

    // Set the modules to drive at the given velocities.
    setModuleStates(velocities);

    // If recording, record the current state of the robot.
    if (manualData.flags & ControlFlag::RECORDING) {
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
    if (time > trajectory->getDuration() && driveController.AtReference()) {
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
    frc::Rotation2d heading(units::math::atan2(twist.dy, twist.dx));

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
        /**
         * The offsets will be 90 degrees off because EVERYTHING about
         * this robot is 90 degrees off!
         */
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
    frc::SmartDashboard::PutNumber("Drive_PoseY_m",          pose.X().value());
    frc::SmartDashboard::PutNumber("Drive_PoseX_m",          pose.Y().value());
    frc::SmartDashboard::PutNumber("Drive_PoseRot_deg",      getRotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Drive_ManualPercentX",   manualData.xPct);
    frc::SmartDashboard::PutNumber("Drive_ManualPercentY",   manualData.yPct);
    frc::SmartDashboard::PutNumber("Drive_ManualPercentRot", manualData.angPct);
    frc::SmartDashboard::PutBoolean("Drive_FieldCentric",    manualData.flags & ControlFlag::FIELD_CENTRIC);
    frc::SmartDashboard::PutBoolean("Drive_Brick",           manualData.flags & ControlFlag::BRICK);
    frc::SmartDashboard::PutBoolean("Drive_Recording",       manualData.flags & ControlFlag::RECORDING);
    frc::SmartDashboard::PutBoolean("Drive_LockX",           manualData.flags & ControlFlag::LOCK_X);
    frc::SmartDashboard::PutBoolean("Drive_LockY",           manualData.flags & ControlFlag::LOCK_Y);
    frc::SmartDashboard::PutBoolean("Drive_LockRot",         manualData.flags & ControlFlag::LOCK_ROT);

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
}
