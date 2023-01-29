#pragma once

#include <Basic/Mechanism.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/math.h>
#include <Hardware/HardwareManager.h>
#include <frc/DigitalInput.h>

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

    // Set the coordinate position of where the end effector should be.
    void setEndPosition(units::meter_t y, units::meter_t z);

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

    // Positional coordinates of the end effector (For feedback use only).
    units::meter_t positionalY;
    units::meter_t positionalZ;

    // Whether the lift is at it's desired position.
    bool atPosition;

    // Motor for extending the lift arm.
    HardwareManager::LiftExtensionMotor extensionMotor;

    // Motors for pivoting the lift arm (lead screw).
    HardwareManager::LiftPivotMotor pivotMotorLeft;
    HardwareManager::LiftPivotMotor pivotMotorRight;

    // Sensor detecting if the lift is at the home position.
    frc::DigitalInput homeSensor;

    // Sensor detecting if the lift is at the extension limit.
    frc::DigitalInput extensionSensor;

    void configureMotors();
};
