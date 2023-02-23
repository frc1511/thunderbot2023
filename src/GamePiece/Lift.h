#pragma once

#include <Basic/Mechanism.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/math.h>
#include <Hardware/HardwareManager.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>

#define EXTENSION_P (0.926354793886058 * 2)
#define EXTENSION_I 0.0
#define EXTENSION_D 0.0
#define EXTENSION_I_ZONE 0 
#define EXTENSION_FF 0

// These values are made up (mean nothing in terms of actual units).
#define EXTENSION_MAX_VEL 2_mps
#define EXTENSION_MAX_ACCEL 6_mps_sq

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

    // Whether the lift is at its desired position.
    bool atPosition;

    // Motor for extending the lift arm.
    HardwareManager::LiftExtensionMotor extensionMotor;

    // Motors for pivoting the lift arm (ball screws).
    HardwareManager::LiftLeftPivotMotor pivotMotorLeft;
    HardwareManager::LiftRightPivotMotor pivotMotorRight;

    // Sensor detecting if the lift is at the home position (fully retracted).
    frc::DigitalInput homeSensor;

    // Sensor detecting if the lift is at the extension limit (fully extended).
    frc::DigitalInput extensionSensor;

    // Profiled PID Controller for the lift extension.
    frc::ProfiledPIDController<units::meters> extensionPIDController {
        EXTENSION_P, EXTENSION_I, EXTENSION_D,
        frc::TrapezoidProfile<units::meters>::Constraints(EXTENSION_MAX_VEL, EXTENSION_MAX_ACCEL)
    };

    struct LiftPosition {
        double pivotPosition;
        double extensionPosition;
        double extensionOffset;
    };

    LiftPosition getCurrentPosition();

    void configureMotors();

    double _targetPivotPosition = 0;
    double _targetExtensionPosition = 0;
};
