#include <GamePiece/Lift.h>
#define ENCODER_TOLERANCE 0.1


Lift::Lift() :
extensionMotor((int)HardwareManager::IOMap::CAN_LIFT_EXTENSION),
pivotMotorLeft((int)HardwareManager::IOMap::CAN_LIFT_PIVOT_LEFT), 
pivotMotorRight((int)HardwareManager::IOMap::CAN_LIFT_PIVOT_RIGHT),
homeSensor((int)HardwareManager::IOMap::DIO_LIFT_HOME),
extensionSensor((int)HardwareManager::IOMap::DIO_LIFT_EXTENSION)

 {

}

Lift::~Lift() {

}

void Lift::resetToMode(MatchMode mode) {

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
        pivotMotorLeft.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, manualAngleSpeed);
        pivotMotorRight.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, manualAngleSpeed);
    }
    else{
        double anglePercent = positionalAngle/(maxAngle - minAngle);
        double anglePosition = anglePercent * maxAnglePosition;
        pivotMotorLeft.set(ThunderCANMotorController::ControlMode::POSITION, anglePosition);
        pivotMotorRight.set(ThunderCANMotorController::ControlMode::POSITION, anglePosition);

        double extensionPercent = positionalExtensionLength/maxExtension;
        double extensionPosition = extensionPercent * maxExtensionPosition;
        pivotMotorLeft.set(ThunderCANMotorController::ControlMode::POSITION, extensionPosition);
        pivotMotorRight.set(ThunderCANMotorController::ControlMode::POSITION, extensionPosition);

        extensionMotor.set(ThunderCANMotorController::ControlMode::POSITION, extensionPosition);
        double currentAnglePosition = pivotMotorLeft.getEncoderPosition();
        double currentExtensionPosition = extensionMotor.getEncoderPosition();

        if ((anglePosition + ENCODER_TOLERANCE >= currentAnglePosition && anglePosition - ENCODER_TOLERANCE <= currentAnglePosition) &&
        (extensionPosition + ENCODER_TOLERANCE >= currentExtensionPosition && extensionPosition - ENCODER_TOLERANCE <= currentExtensionPosition)) {
            atPosition = true;
        }
        else {
            atPosition = false;
        }
    }

}

void Lift::setManualAngleSpeed(double speed) {
    manualAngleSpeed = speed;
    controlType = ControlType::MANUAL;
}

void Lift::setManualExtensionSpeed(double speed) {
    manualAngleSpeed = speed;
    controlType = ControlType::MANUAL;
}

void Lift::setEndPosition(units::meter_t y, units::meter_t z) {
    controlType = ControlType::POSITION;   
    positionalAngle = units::math::atan2(z,y);
    positionalExtensionLength =z/units::math::sin(positionalAngle);
}

bool Lift::isAtPosition(){
    if (controlType == ControlType::POSITION){
        return atPosition;
    
    }
    return true;
}

void Lift::sendFeedback() {

}
