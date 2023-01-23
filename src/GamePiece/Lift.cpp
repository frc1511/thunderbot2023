#include <GamePiece/Lift.h>

#define ENCODER_TOLERANCE 0.1
#define PIVOT_POINT_HEIGHT 0.5_m

#define MIN_PIVOT_ANGLE -20_deg
#define MAX_PIVPT_ANGLE 45_deg
#define MAX_PIVOT_ENCODER 100
#define MAX_EXTENSION_LENGTH 4_ft
#define MAX_EXTENSION_ENCODER 100

#define MANUAL_PIVOT_SPEED_COEFF 0.5
#define MANUAL_EXTENSION_SPEED_COEFF 0.5

Lift::Lift()
: extensionMotor((int)HardwareManager::IOMap::CAN_LIFT_EXTENSION),
  pivotMotorLeft((int)HardwareManager::IOMap::CAN_LIFT_PIVOT_LEFT), 
  pivotMotorRight((int)HardwareManager::IOMap::CAN_LIFT_PIVOT_RIGHT),
  homeSensor((int)HardwareManager::IOMap::DIO_LIFT_HOME),
  extensionSensor((int)HardwareManager::IOMap::DIO_LIFT_EXTENSION) {

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
        double pivotPercent = positionalAngle / (MAX_PIVPT_ANGLE - MIN_PIVOT_ANGLE);
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

void Lift::sendFeedback() {

}
