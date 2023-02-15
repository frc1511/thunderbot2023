#pragma once

#include <Basic/Mechanism.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/math.h>
#include <Hardware/HardwareManager.h>
#include <frc/DigitalInput.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/velocity.h>

class Lift : public Mechanism {
public:
    Lift();
    ~Lift();
    bool isAtPosition();
    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;

    // Manually set the speed of the motor to control the angle of the lift.
    void setManualPivotSpeed(double speed);

    // Manually set the speed of the motor to control the extension distance of the lift.
    void setManualExtensionSpeed(double speed);

    // Set the arm angle (from horizontal) and extension length.
    void setPosition(units::degree_t angle, units::meter_t extensionLength);

private:
    double manualPivotSpeed;
    double manualExtensionSpeed;

    enum class ControlType{
        MANUAL,
        POSITION,
    };
    ControlType controlType;

    // Target extension length of the lift arm.
    units::meter_t positionalExtensionLength;
    // Target angle of the lift arm.
    units::degree_t positionalAngle;

    // Whether the lift is at it's desired position.
    bool atPosition;

    // Motor for extending the lift arm.
    HardwareManager::LiftExtensionMotor extensionMotor;

    // Motors for pivoting the lift arm (lead screw).
    HardwareManager::LiftLeftPivotMotor pivotMotorLeft;
    HardwareManager::LiftRightPivotMotor pivotMotorRight;

    // Sensor detecting if the lift is at the home position.
    frc::DigitalInput homeSensor;

    // Sensor detecting if the lift is at the extension limit.
    frc::DigitalInput extensionSensor;

    frc::PIDController extensionPIDController;
    frc::SlewRateLimiter<units::meters_per_second> extensionSlewRateLimiter;

    void configureMotors();
};
