#include <GamePiece/Lift.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <frc/geometry/Twist2d.h>
#include <Util/Parser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>
#define ENCODER_TOLERANCE 0.1


#define ENCODER_TOLERANCE 0.1
#define PIVOT_POINT_HEIGHT 0.5_m

#define MIN_PIVOT_ANGLE -20_deg
#define MAX_PIVOT_ANGLE 45_deg
#define MAX_PIVOT_ENCODER 100
#define MAX_EXTENSION_LENGTH 4_ft
#define MAX_EXTENSION_ENCODER 100

#define MANUAL_PIVOT_SPEED_COEFF 0.5
#define MANUAL_EXTENSION_SPEED_COEFF 0.

// The max amperage of the pivot and extension motors.
#define PIVOT_MAX_AMPERAGE 40_A
#define EXTENSION_MAX_AMPERAGE 40_A

// --- PID Values ---
#define PIVOT_P 0.1
#define PIVOT_I 0
#define PIVOT_D 0
#define PIVOT_I_ZONE 0
#define PIVOT_FF 0

#define EXTENSION_P 0.1
#define EXTENSION_I 0
#define EXTENSION_D 0
#define EXTENSION_I_ZONE 0 
#define EXTENSION_FF 0

Lift::Lift()
: extensionMotor(HardwareManager::IOMap::CAN_LIFT_EXTENSION),
  pivotMotorLeft(HardwareManager::IOMap::CAN_LIFT_PIVOT_LEFT), 
  pivotMotorRight(HardwareManager::IOMap::CAN_LIFT_PIVOT_RIGHT),
  homeSensor(HardwareManager::IOMap::DIO_LIFT_HOME),
  extensionSensor(HardwareManager::IOMap::DIO_LIFT_EXTENSION) {

}

Lift::~Lift() {

}

void Lift::resetToMode(MatchMode mode) {
    controlType = ControlType::MANUAL;
    manualPivotSpeed = 0;
    manualExtensionSpeed = 0;

    extensionMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
    pivotMotorLeft.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
    pivotMotorRight.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
}

void Lift::doPersistentConfiguration() {

}

void Lift::process() {
    if (controlType == ControlType::MANUAL) {
        // Limits for the manual control of the lift.
        if (extensionSensor.Get() && manualExtensionSpeed > 0) {
            manualExtensionSpeed = 0;
        }
        if (homeSensor.Get() && manualExtensionSpeed < 0) {
            manualExtensionSpeed = 0;
        }

        // Set the motor speeds.
        extensionMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, manualExtensionSpeed);
        pivotMotorLeft.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, manualPivotSpeed);
        pivotMotorRight.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, manualPivotSpeed);
    }
    else {
        // --- Pivot ---

        // Convert the positional angle to a percent of the range of motion.
        double pivotPercent = positionalAngle / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);
        // Convert the percent to an encoder position.
        double pivotPosition = pivotPercent * MAX_PIVOT_ENCODER;

        // Set the PID reference of the pivot motors to the encoder position.
        pivotMotorLeft.set(ThunderCANMotorController::ControlMode::POSITION, pivotPosition);
        pivotMotorRight.set(ThunderCANMotorController::ControlMode::POSITION, pivotPosition);

        // --- Extension ---

        // Convert the positional extension length to a percent of the range of motion.
        double extensionPercent = positionalExtensionLength / MAX_EXTENSION_LENGTH;
        // Convert the percent to an encoder position.
        double extensionPosition = extensionPercent * MAX_EXTENSION_ENCODER;

        // Set the PID reference of the extension motor to the encoder position.
        extensionMotor.set(ThunderCANMotorController::ControlMode::POSITION, extensionPosition);

        // --- Check if at position ---

        double currentAnglePosition = pivotMotorLeft.getEncoderPosition();
        double currentExtensionPosition = extensionMotor.getEncoderPosition();

        atPosition = false;
        if ((pivotPosition + ENCODER_TOLERANCE >= currentAnglePosition && pivotPosition - ENCODER_TOLERANCE <= currentAnglePosition) &&
            (extensionPosition + ENCODER_TOLERANCE >= currentExtensionPosition && extensionPosition - ENCODER_TOLERANCE <= currentExtensionPosition)) {
            atPosition = true;
        }
    }
}

void Lift::setManualPivotSpeed(double speed) {
    speed *= MANUAL_PIVOT_SPEED_COEFF;
    manualPivotSpeed = speed;
    controlType = ControlType::MANUAL;

    positionalY = -1_m;
    positionalZ = -1_m;
    positionalAngle = -1_deg;
    positionalExtensionLength = -1_m;
}

void Lift::setManualExtensionSpeed(double speed) {
    speed *= MANUAL_EXTENSION_SPEED_COEFF;
    manualExtensionSpeed = speed;
    controlType = ControlType::MANUAL;

    positionalY = -1_m;
    positionalZ = -1_m;
    positionalAngle = -1_deg;
    positionalExtensionLength = -1_m;
}

void Lift::setEndPosition(units::meter_t y, units::meter_t z) {
    z -= PIVOT_POINT_HEIGHT;
    controlType = ControlType::POSITION;
    positionalAngle = units::math::atan2(z, y);
    positionalExtensionLength = z / units::math::sin(positionalAngle);

    positionalY = y;
    positionalZ = z;
}

bool Lift::isAtPosition(){
    if (controlType == ControlType::POSITION){
        return atPosition;
    }
    return true;
}

void Lift::configureMotors() {
    // Left Pivot Motor Configuration
    pivotMotorLeft.configFactoryDefault();
    pivotMotorLeft.configFactoryDefault();
    pivotMotorLeft.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    pivotMotorLeft.configSmartCurrentLimit(PIVOT_MAX_AMPERAGE);
    pivotMotorLeft.setInverted(false);

    pivotMotorLeft.configP(PIVOT_P);
    pivotMotorLeft.configI(PIVOT_I);
    pivotMotorLeft.configD(PIVOT_D);
    pivotMotorLeft.configIZone(PIVOT_I_ZONE);
    pivotMotorLeft.configFF(PIVOT_FF);

    // Right Pivot Motor Configuration
    pivotMotorRight.configFactoryDefault();
    pivotMotorRight.configFactoryDefault();
    pivotMotorRight.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    pivotMotorRight.configSmartCurrentLimit(PIVOT_MAX_AMPERAGE);
    pivotMotorRight.setInverted(false);

    pivotMotorRight.configP(PIVOT_P);
    pivotMotorRight.configI(PIVOT_I);
    pivotMotorRight.configD(PIVOT_D);
    pivotMotorRight.configIZone(PIVOT_I_ZONE);
    pivotMotorRight.configFF(PIVOT_FF);
    
    // Extension Motor Configuration
    extensionMotor.configFactoryDefault();
    extensionMotor.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    extensionMotor.configSmartCurrentLimit(EXTENSION_MAX_AMPERAGE);
    extensionMotor.setInverted(false);

    extensionMotor.configP(EXTENSION_P);
    extensionMotor.configI(EXTENSION_I);
    extensionMotor.configD(EXTENSION_D);
    extensionMotor.configIZone(EXTENSION_I_ZONE);
    extensionMotor.configFF(EXTENSION_FF);
}

void Lift::sendFeedback() {
    std::string controlTypeString;
    switch (controlType) {
        case ControlType::MANUAL:
            controlTypeString = "Manual";
            break;
        case ControlType::POSITION:
            controlTypeString = "Position";
            break;
    }
    frc::SmartDashboard::PutString("Lift_ControlType", controlTypeString);

    // Target Positions
    frc::SmartDashboard::PutNumber("Lift_TargetExtension_m", positionalExtensionLength.value());
    frc::SmartDashboard::PutNumber("Lift_TargetAngle_deg", positionalAngle.value());
    frc::SmartDashboard::PutBoolean("Lift_TargetReached", atPosition);
    frc::SmartDashboard::PutNumber("Lift_TargetY_m", positionalY.value());
    frc::SmartDashboard::PutNumber("Lift_TargetZ_m", positionalZ.value());

    // Encoder Positions
    double extensionPosition = extensionMotor.getEncoderPosition();
    double pivotLeftPosition = pivotMotorLeft.getEncoderPosition();
    double pivotRightPosition = pivotMotorRight.getEncoderPosition();

    frc::SmartDashboard::PutNumber("Lift_EncoderExtension", extensionPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderPivotLeft", pivotLeftPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderPivotRight", pivotRightPosition);

    // End Effector Position
    double extensionPercent = extensionPosition / MAX_EXTENSION_ENCODER;
    units::meter_t currentExtensionLength = extensionPercent * MAX_EXTENSION_LENGTH;

    double pivotPercent = pivotLeftPosition / MAX_PIVOT_ENCODER;
    units::radian_t currentAngle = pivotPercent * (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);

    units::meter_t currentY = currentExtensionLength * units::math::cos(currentAngle);
    units::meter_t currentZ = currentExtensionLength * units::math::sin(currentAngle) + PIVOT_POINT_HEIGHT;

    frc::SmartDashboard::PutNumber("Lift_Y_m", currentY.value());
    frc::SmartDashboard::PutNumber("Lift_Z_m", currentZ.value());

    // Digital Inputs
    frc::SmartDashboard::PutBoolean("Lift_SensorHome", homeSensor.Get());
    frc::SmartDashboard::PutBoolean("Lift_SensorExtension", extensionSensor.Get());

    // Temperature
    frc::SmartDashboard::PutNumber("Lift_TempRightPivot_F", pivotMotorRight.getTemperature().value());
    frc::SmartDashboard::PutNumber("Lift_TempLeftPivot_F", pivotMotorLeft.getTemperature().value());
    frc::SmartDashboard::PutNumber("Lift_TempExtension_F", extensionMotor.getTemperature().value());

    // Current
    frc::SmartDashboard::PutNumber("Lift_CurrentRightPivot_A", pivotMotorRight.getOutputCurrent().value());
    frc::SmartDashboard::PutNumber("Lift_CurrentLeftPivot_A", pivotMotorLeft.getOutputCurrent().value());
    frc::SmartDashboard::PutNumber("Lift_CurrentExtension_A", extensionMotor.getOutputCurrent().value());
}
