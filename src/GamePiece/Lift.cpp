#include <GamePiece/Lift.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <frc/geometry/Twist2d.h>
#include <Util/Parser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>
#define ENCODER_TOLERANCE 0.1


#define ENCODER_TOLERANCE 0.1
#define PIVOT_POINT_HEIGHT 0.5_m

#define MIN_PIVOT_ANGLE -30_deg
#define MAX_PIVOT_ANGLE 40_deg
#define MAX_PIVOT_ENCODER 100
#define MAX_EXTENSION_LENGTH 42_in
#define MAX_EXTENSION_ENCODER 100

#define MANUAL_PIVOT_SPEED_COEFF 0.5
#define MANUAL_EXTENSION_SPEED_COEFF 0.5

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

#define EXTENSION_MPS_P 0.1
#define EXTENSION_MPS_I 0
#define EXTENSION_MPS_D 0

#define EXTENSION_MAX_ACCEL 8_mps_sq
#define EXTENSION_MAX_DECEL -16_mps_sq

#define EXTENSION_METER_TO_ENCODER_FACTOR 1

Lift::Lift()
: extensionMotor(HardwareManager::IOMap::CAN_LIFT_EXTENSION),
  pivotMotorLeft(HardwareManager::IOMap::CAN_LIFT_PIVOT_LEFT), 
  pivotMotorRight(HardwareManager::IOMap::CAN_LIFT_PIVOT_RIGHT),
  homeSensor(HardwareManager::IOMap::DIO_LIFT_HOME),
  extensionSensor(HardwareManager::IOMap::DIO_LIFT_EXTENSION),
  extensionPIDController(EXTENSION_MPS_P, EXTENSION_MPS_I, EXTENSION_MPS_D),
  extensionSlewRateLimiter(EXTENSION_MAX_ACCEL, EXTENSION_MAX_DECEL) {

    configureMotors();
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

    extensionSlewRateLimiter.Reset(0_mps);

    if (!(mode == Mechanism::MatchMode::DISABLED && getLastMode() == Mechanism::MatchMode::AUTO)
     && !(mode == Mechanism::MatchMode::TELEOP && getLastMode() == Mechanism::MatchMode::DISABLED)) {
        // Stuff to reset normally.
        extensionMotor.setEncoderPosition(0);
        pivotMotorLeft.setEncoderPosition(0);
        pivotMotorRight.setEncoderPosition(0);
    }
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
        pivotPercent = std::clamp(pivotPercent, 0.0, 1.0);

        // Convert the percent to an encoder position.
        double pivotPosition = pivotPercent * MAX_PIVOT_ENCODER;

        // Set the PID reference of the pivot motors to the encoder position.
        pivotMotorLeft.set(ThunderCANMotorController::ControlMode::POSITION, pivotPosition);
        pivotMotorRight.set(ThunderCANMotorController::ControlMode::POSITION, pivotPosition);

        // --- Extension ---

        // Convert the positional extension length to a percent of the range of motion.
        double extensionPercent = positionalExtensionLength / MAX_EXTENSION_LENGTH;
        extensionPercent = std::clamp(extensionPercent, 0.0, 1.0);

        // Convert the percent to an encoder position.
        double extensionPosition = extensionPercent * MAX_EXTENSION_ENCODER;

        units::meters_per_second_t extensionVelocity(extensionPIDController.Calculate(extensionPosition));
        extensionVelocity = extensionSlewRateLimiter.Calculate(extensionVelocity);
        
        double rpm = extensionVelocity.value() * 60 * EXTENSION_METER_TO_ENCODER_FACTOR;
        extensionMotor.set(ThunderCANMotorController::ControlMode::VELOCITY, rpm);
        
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
    manualPivotSpeed = speed * MANUAL_PIVOT_SPEED_COEFF;
    controlType = ControlType::MANUAL;

    positionalAngle = -1_deg;
    positionalExtensionLength = -1_m;
}

void Lift::setManualExtensionSpeed(double speed) {
    manualExtensionSpeed = speed * MANUAL_EXTENSION_SPEED_COEFF;
    controlType = ControlType::MANUAL;

    positionalAngle = -1_deg;
    positionalExtensionLength = -1_m;
}

void Lift::setPosition(units::degree_t angle, units::meter_t extension) {
    controlType = ControlType::POSITION;
    positionalAngle = angle;
    positionalExtensionLength = extension;
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

    // Encoder Positions
    double extensionPosition = extensionMotor.getEncoderPosition();
    double pivotLeftPosition = pivotMotorLeft.getEncoderPosition();
    double pivotRightPosition = pivotMotorRight.getEncoderPosition();

    frc::SmartDashboard::PutNumber("Lift_EncoderExtension", extensionPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderPivotLeft", pivotLeftPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderPivotRight", pivotRightPosition);

    frc::SmartDashboard::PutNumber("Lift_ManualPivotSpeed", manualPivotSpeed);
    frc::SmartDashboard::PutNumber("Lift_ManualExtensionSpeed", manualExtensionSpeed);

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
