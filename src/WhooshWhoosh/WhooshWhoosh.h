#pragma once

#include <Basic/Mechanism.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <Hardware/HardwareManager.h>
#include <frc/controller/PIDController.h>

// Comment out if no ADIS16470 IMU.
#define HAS_IMU

/**
 * This mechanism is responsible for handling the robot's rotational
 * information.
 *
 * Contains the functionality to balance the robot on the Charge Station by
 * using the roll axis angle and angular velocity to calculate the optimal drive
 * velocity to balance the robot.
 */
class WhooshWhoosh : public Mechanism {
public:
    WhooshWhoosh();
    ~WhooshWhoosh();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    /**
     * Returns the yaw axis angle of the robot.
     */
    units::degree_t getHeadingAngle();

    /**
     * Returns the yaw axis angular velocity of the robot.
     */
    units::radians_per_second_t getHeadingRate();

    /**
     * Returns the roll axis angle of the robot.
     */
    units::degree_t getTiltAngle();

    /**
     * Returns the roll axis angular velocity of the robot.
     */
    units::radians_per_second_t getTiltRate();

    /**
     * Calculates the optimal drive velocity to balance the robot on the Charge
     * Station.
     */
    units::meters_per_second_t calculateAntiTiltDriveVelocity();

    /**
     * Calibrates the IMU (sleeps the program for 4 seconds).
     */
    void calibrateIMU();

    /**
     * Resets the yaw axis angle and rate to zero.
     */
    void resetHeading();

    /**
     * Resets the roll axis angle and rate to zero.
     */
    void resetTilt();

private:

#ifdef HAS_IMU
    frc1511::ADIS16470_IMU imu {
        frc1511::ADIS16470_IMU::kY,
        frc1511::ADIS16470_IMU::kZ,
        frc1511::ADIS16470_IMU::kX,
        frc::SPI::Port::kOnboardCS0,
        frc1511::ADIS16470_IMU::CalibrationTime::_4s
    };
#endif

    frc::PIDController balancePIDController;
};
