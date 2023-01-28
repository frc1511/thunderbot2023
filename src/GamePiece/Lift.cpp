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
#define PIVOT_P 0
#define PIVOT_I 0
#define PIVOT_D 0
#define PIVOT_I_ZONE 0
#define PIVOT_FF 0

#define EXTENSION_P 0
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
    if (controlType == ControlType::MANUAL){
        if (extensionSensor.Get() && manualExtensionSpeed > 0) {
            manualExtensionSpeed = 0;
        }
        if (homeSensor.Get() && manualExtensionSpeed < 0) {
            manualExtensionSpeed = 0;
        }
        extensionMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, manualExtensionSpeed);
        pivotMotorLeft.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, manualPivotSpeed);
        pivotMotorRight.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, manualPivotSpeed);
    }
    else{
        double pivotPercent = positionalAngle / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);
        double pivotPosition = pivotPercent * MAX_PIVOT_ENCODER;
        pivotMotorLeft.set(ThunderCANMotorController::ControlMode::POSITION, pivotPosition);
        pivotMotorRight.set(ThunderCANMotorController::ControlMode::POSITION, pivotPosition);

        double extensionPercent = positionalExtensionLength / MAX_EXTENSION_LENGTH;
        double extensionPosition = extensionPercent * MAX_EXTENSION_ENCODER;
        extensionMotor.set(ThunderCANMotorController::ControlMode::POSITION, extensionPosition);

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
}

void Lift::setManualExtensionSpeed(double speed) {
    speed *= MANUAL_EXTENSION_SPEED_COEFF;
    manualExtensionSpeed = speed;
    controlType = ControlType::MANUAL;
}

void Lift::setEndPosition(units::meter_t y, units::meter_t z) {
    z -= PIVOT_POINT_HEIGHT;
    controlType = ControlType::POSITION;
    positionalAngle = units::math::atan2(z, y);
    positionalExtensionLength = z / units::math::sin(positionalAngle);
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
    // Lift Feedback
    frc::SmartDashboard::PutNumber("Lift_Positional_Extension_Length", positionalExtensionLength.value());
    frc::SmartDashboard::PutNumber("Lift_Positional_Angle", positionalAngle.value());
    frc::SmartDashboard::PutBoolean("Lift_At_Position", atPosition);
    frc::SmartDashboard::PutNumber("Lift_Extension_Motor_Encoder", extensionMotor.getEncoderPosition());
    frc::SmartDashboard::PutNumber("Lift_Pivot_Left_Motor_Encoder", pivotMotorLeft.getEncoderPosition());
    frc::SmartDashboard::PutNumber("Lift_Pivot_Motor_Right_Encoder", pivotMotorRight.getEncoderPosition());
    frc::SmartDashboard::PutBoolean("Lift_Home_Sensor", homeSensor.Get());
    frc::SmartDashboard::PutBoolean("Lift_Extension_Sensor", extensionSensor.Get());

}
