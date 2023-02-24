#include <GamePiece/Lift.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <frc/geometry/Twist2d.h>
#include <Util/Parser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>

// The allowable error in the pivot angle and extension length.
#define PIVOT_ENCODER_TOLERANCE 5.0
#define EXTENSION_ENCODER_TOLERANCE 4.0

// The minimum angle of the arm.
#define MIN_PIVOT_ANGLE -47_deg

// The maximum angle of the arm.
#define MAX_PIVOT_ANGLE 20_deg

// The encoder value at the maximum angle of the arm.
#define MAX_PIVOT_ENCODER 81.645897

// The maximum extension length of the arm (delta from starting position).
#define MAX_EXTENSION_LENGTH 38.25_in

// The encoder value at the maximum extension length of the arm.
#define MAX_EXTENSION_ENCODER 57.929295

// The minimum pivot position at which the arm can be extended.
#define MIN_PIVOT_EXTEND_ENCODER 3.738092

// The change in extension encoder value over the full pivot range of the arm.
#define EXTENSION_BACKDRIVE_ENCODER -2.214285

#define MANUAL_PIVOT_SPEED_COEFF 0.3
#define MANUAL_EXTENSION_SPEED_COEFF 0.2

// The max amperage of the pivot and extension motors.
#define PIVOT_MAX_AMPERAGE 40_A
#define EXTENSION_MAX_AMPERAGE 40_A

// Removes the backdrive offset from an extension encoder value.
#define NORMALIZE_EXTENSION_POSITION(x, offset) (x - offset)

// Adds back the backdrive offset to an extension encoder value.
#define DENORMALIZE_EXTENSION_POSITION(x, offset) (x + offset)

Lift::Lift()
: extensionMotor(HardwareManager::IOMap::CAN_LIFT_EXTENSION),
  pivotMotorLeft(HardwareManager::IOMap::CAN_LIFT_PIVOT_LEFT), 
  pivotMotorRight(HardwareManager::IOMap::CAN_LIFT_PIVOT_RIGHT),
  homeSensor(HardwareManager::IOMap::DIO_LIFT_HOME),
  extensionSensor(HardwareManager::IOMap::DIO_LIFT_EXTENSION) {

    configureMotors();

    pivotPIDController.Reset(0_deg);
    extensionPIDController.Reset(0_m);
    positionalExtensionLength = 0_m;
    positionalAngle = -40_deg;
    controlType = ControlType::POSITION;
}

Lift::~Lift() {

}

void Lift::resetToMode(MatchMode mode) {
    controlType = ControlType::POSITION;
    manualPivotSpeed = 0;
    manualExtensionSpeed = 0;

    atPosition = false;

    extensionMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
    pivotMotorLeft.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);
    pivotMotorRight.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, 0);

    static bool wasAuto = false;

    // Going from Auto to Disabled to Teleop.
    if (wasAuto && mode == Mechanism::MatchMode::TELEOP) {
        wasAuto = false;
    }
    else {
        // Check if going from Auto to Disabled.
        wasAuto = getLastMode() == Mechanism::MatchMode::DISABLED && mode == Mechanism::MatchMode::DISABLED;

        // Doing something else.
        if (!wasAuto && mode != Mechanism::MatchMode::DISABLED) {
            // Stuff to reset normally.
        }
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

        return;
    }

    // --- The rest of this function is for automatic control of the lift. ---

    // Get the current position of the lift.
    auto [currentPivotPosition, currentExtensionPosition, extensionOffset] = getCurrentPosition();

    units::degree_t currentLeftPivotAngle = (pivotMotorLeft.getEncoderPosition() / MAX_PIVOT_ENCODER) * (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE) + MIN_PIVOT_ANGLE;
    units::degree_t currentRightPivotAngle = (pivotMotorRight.getEncoderPosition() / MAX_PIVOT_ENCODER) * (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE) + MIN_PIVOT_ANGLE;

    /*
    // If the extension flag is set, reset the encoder value to the known maximum.
    if (extensionSensor.Get()) {
        extensionMotor.setEncoderPosition(DENORMALIZE_EXTENSION_POSITION(MAX_EXTENSION_ENCODER, extensionOffset));
        currentExtensionPosition = MAX_EXTENSION_ENCODER;

        extensionPIDController.Reset(
            (DENORMALIZE_EXTENSION_POSITION(MAX_EXTENSION_ENCODER, extensionOffset) / MAX_EXTENSION_ENCODER) * MAX_EXTENSION_LENGTH
        );
    }
    // If the home flag is set, reset the encoder value to the known minimum.
    if (homeSensor.Get()) {
        extensionMotor.setEncoderPosition(DENORMALIZE_EXTENSION_POSITION(0, extensionOffset));
        currentExtensionPosition = 0;

        extensionPIDController.Reset(
            (DENORMALIZE_EXTENSION_POSITION(0, extensionOffset) / MAX_EXTENSION_ENCODER) * MAX_EXTENSION_LENGTH
        );
    }
    */
    // Calculate the target extension position.
    double targetExtensionPosition = currentExtensionPosition;
    {
        // Convert the positional extension length to a percent of the range of motion.
        double extensionPercent = positionalExtensionLength / MAX_EXTENSION_LENGTH;
        // Don't overshoot the limits!
        extensionPercent = std::clamp(extensionPercent, 0.0, 1.0);

        targetExtensionPosition = extensionPercent * MAX_EXTENSION_ENCODER;
    }

    units::meter_t currentLength = (DENORMALIZE_EXTENSION_POSITION(currentExtensionPosition, extensionOffset) / MAX_EXTENSION_ENCODER) * MAX_EXTENSION_LENGTH;
    units::meter_t targetLength = (DENORMALIZE_EXTENSION_POSITION(targetExtensionPosition, extensionOffset) / MAX_EXTENSION_ENCODER) * MAX_EXTENSION_LENGTH;

    pivotMotorLeft.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, pivotPIDController.Calculate(currentLeftPivotAngle, positionalAngle));
    pivotMotorRight.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, pivotPIDController.Calculate(currentRightPivotAngle, positionalAngle));

    double extensionPercentOutput = extensionPIDController.Calculate(currentLength, targetLength);
    extensionMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, extensionPercentOutput);

    auto isAtPosition = [&](double current, double target, double tolerance) {
        return std::abs(current - target) < tolerance;
    };
    
    // Check if the lift is at the target position.
    atPosition = units::math::abs(currentLeftPivotAngle - positionalAngle) < 2_deg &&
                    isAtPosition(currentExtensionPosition, targetExtensionPosition, EXTENSION_ENCODER_TOLERANCE);
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

Lift::LiftPosition Lift::getCurrentPosition() {
    // The current pivot position of the lift.
    double currentPivotPosition = pivotMotorLeft.getEncoderPosition();

    // The amount of backdrive of the extension motor due to the pivot position.
    double extensionOffset = EXTENSION_BACKDRIVE_ENCODER * (currentPivotPosition / MAX_PIVOT_ENCODER);

    // The current extension position of the lift, accounting for backdrive.
    double currentExtensionPosition = NORMALIZE_EXTENSION_POSITION(extensionMotor.getEncoderPosition(), extensionOffset);

    return { currentPivotPosition, currentExtensionPosition, extensionOffset };
}

void Lift::configureMotors() {
    // Left Pivot Motor Configuration
    pivotMotorLeft.configFactoryDefault();
    pivotMotorLeft.configFactoryDefault();
    pivotMotorLeft.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    pivotMotorLeft.configSmartCurrentLimit(PIVOT_MAX_AMPERAGE);
    pivotMotorLeft.setInverted(true);

    pivotMotorLeft.configP(PIVOT_P);
    pivotMotorLeft.configI(PIVOT_I);
    pivotMotorLeft.configD(PIVOT_D);

    // Right Pivot Motor Configuration
    pivotMotorRight.configFactoryDefault();
    pivotMotorRight.configFactoryDefault();
    pivotMotorRight.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    pivotMotorRight.configSmartCurrentLimit(PIVOT_MAX_AMPERAGE);
    pivotMotorRight.setInverted(true);

    pivotMotorRight.configP(PIVOT_P);
    pivotMotorRight.configI(PIVOT_I);
    pivotMotorRight.configD(PIVOT_D);
    
    // Extension Motor Configuration
    extensionMotor.configFactoryDefault();
    extensionMotor.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    extensionMotor.configSmartCurrentLimit(EXTENSION_MAX_AMPERAGE);
    extensionMotor.setInverted(true);

    extensionMotor.configP(EXTENSION_P);
    extensionMotor.configI(EXTENSION_I);
    extensionMotor.configD(EXTENSION_D);
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

    // Raw Encoder Positions
    frc::SmartDashboard::PutNumber("Lift_EncoderRawExtension", extensionMotor.getEncoderPosition());
    frc::SmartDashboard::PutNumber("Lift_EncoderRawPivotLeft", pivotMotorLeft.getEncoderPosition());
    frc::SmartDashboard::PutNumber("Lift_EncoderRawPivotRight", pivotMotorRight.getEncoderPosition());

    // Normalized Encoder Positions
    auto [currentPivotPosition, currentExtensionPosition, extensionOffset] = getCurrentPosition();

    frc::SmartDashboard::PutNumber("Lift_EncoderExtension",       currentExtensionPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderPivot",           currentPivotPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderExtensionOffset", extensionOffset);

    frc::SmartDashboard::PutNumber("Lift_EncoderTargetPivot", _targetPivotPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderTargetExtension", _targetExtensionPosition);

    // Lift length and angle
    units::meter_t currentExtension = MAX_EXTENSION_LENGTH * (currentExtensionPosition / MAX_EXTENSION_ENCODER);
    units::degree_t currentAngle = MAX_PIVOT_ANGLE * (currentPivotPosition / MAX_PIVOT_ENCODER);

    frc::SmartDashboard::PutNumber("Lift_Length_m", currentExtension.value());
    frc::SmartDashboard::PutNumber("Lift_Angle_deg", currentAngle.value());

    frc::SmartDashboard::PutNumber("Lift_ManualPivotSpeed", manualPivotSpeed);
    frc::SmartDashboard::PutNumber("Lift_ManualExtensionSpeed", manualExtensionSpeed);

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

    double pivotPercent = currentPivotPosition / MAX_PIVOT_ENCODER;
    double extensionPercent = currentExtensionPosition / MAX_EXTENSION_ENCODER;

    // Dashboard feedback
    frc::SmartDashboard::PutNumber("thunderdashboard_lift_pivot_percent", pivotPercent);
    frc::SmartDashboard::PutNumber("thunderdashboard_lift_extension_percent", extensionPercent);

    if (controlType == ControlType::MANUAL || getCurrentMode() == MatchMode::DISABLED) {
        frc::SmartDashboard::PutNumber("thunderdashboard_lift_pivot_target_percent", -1.0);
        frc::SmartDashboard::PutNumber("thunderdashboard_lift_extension_target_percent", -1.0);
    }
    else {
        double targetPivotPercent = (positionalAngle - MIN_PIVOT_ANGLE) / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);
        double targetExtensionPercent = positionalExtensionLength / MAX_EXTENSION_LENGTH;

        if (atPosition) {
            targetPivotPercent = pivotPercent;
            targetExtensionPercent = extensionPercent;
        }

        frc::SmartDashboard::PutNumber("thunderdashboard_lift_pivot_target_percent", targetPivotPercent);
        frc::SmartDashboard::PutNumber("thunderdashboard_lift_extension_target_percent", targetExtensionPercent);
    }
}
