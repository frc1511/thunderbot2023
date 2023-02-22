#include <GamePiece/Lift.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <frc/geometry/Twist2d.h>
#include <Util/Parser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>

// The allowable error in the pivot angle and extension length.
#define ENCODER_TOLERANCE 0.1

// The minimum angle of the arm.
#define MIN_PIVOT_ANGLE -47_deg

// The maximum angle of the arm.
#define MAX_PIVOT_ANGLE 20_deg

// The encoder value at the maximum angle of the arm.
#define MAX_PIVOT_ENCODER 81.360153

// The maximum extension length of the arm (delta from starting position).
#define MAX_EXTENSION_LENGTH 38.25_in

// The encoder value at the maximum extension length of the arm.
#define MAX_EXTENSION_ENCODER 57.929295

// The minimum pivot position at which the arm can be extended.
#define MIN_PIVOT_EXTEND_ENCODER 3.738092

// The change in extension encoder value over the full pivot range of the arm.
#define EXTENSION_BACKDRIVE_ENCODER -2.50708256

#define MANUAL_PIVOT_SPEED_COEFF 0.3
#define MANUAL_EXTENSION_SPEED_COEFF 0.2

// The max amperage of the pivot and extension motors.
#define PIVOT_MAX_AMPERAGE 40_A
#define EXTENSION_MAX_AMPERAGE 40_A

// --- PID Values ---
#define PIVOT_P 0.0366
#define PIVOT_I 0
#define PIVOT_D 0
#define PIVOT_I_ZONE 0
#define PIVOT_FF 0

#define EXTENSION_P 0.0177
#define EXTENSION_I 0
#define EXTENSION_D 0
#define EXTENSION_I_ZONE 0 
#define EXTENSION_FF 0

#define EXTENSION_MPS_P 0.1
#define EXTENSION_MPS_I 0
#define EXTENSION_MPS_D 0

// These values are not in meters per second lol.
#define EXTENSION_MAX_ACCEL 2_mps_sq
#define EXTENSION_MAX_DECEL -4_mps_sq

// Removes the backdrive offset from an extension encoder value.
#define NORMALIZE_EXTENSION_POSITION(x, offset) (x - offset)

// Adds back the backdrive offset to an extension encoder value.
#define DENORMALIZE_EXTENSION_POSITION(x, offset) (x + offset)

Lift::Lift()
: extensionMotor(HardwareManager::IOMap::CAN_LIFT_EXTENSION),
  pivotMotorLeft(HardwareManager::IOMap::CAN_LIFT_PIVOT_LEFT), 
  pivotMotorRight(HardwareManager::IOMap::CAN_LIFT_PIVOT_RIGHT),
  homeSensor(HardwareManager::IOMap::DIO_LIFT_HOME),
  extensionSensor(HardwareManager::IOMap::DIO_LIFT_EXTENSION),
  extensionSlewRateLimiter(EXTENSION_MAX_ACCEL, EXTENSION_MAX_DECEL) {

    configureMotors();
}

Lift::~Lift() {

}

void Lift::resetToMode(MatchMode mode) {
    controlType = ControlType::MANUAL;
    manualPivotSpeed = 0;
    manualExtensionSpeed = 0;

    atPosition = false;

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

        return;
    }

    // --- The rest of this function is for automatic control of the lift. ---

    // Get the current position of the lift.
    auto [currentPivotPosition, currentExtensionPosition, extensionOffset] = getCurrentPosition();

    // Calculate the target pivot position.
    double targetPivotPosition = currentPivotPosition;
    {
        // Convert the positional angle to a percent of the range of motion.
        double pivotPercent = positionalAngle / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);
        // Don't overshoot the limits!
        pivotPercent = std::clamp(pivotPercent, 0.0, 1.0);

        targetPivotPosition = pivotPercent * MAX_PIVOT_ENCODER;
    }

    // If the extension flag is set, reset the encoder value to the known maximum.
    if (extensionSensor.Get()) {
        extensionMotor.setEncoderPosition(DENORMALIZE_EXTENSION_POSITION(MAX_EXTENSION_ENCODER, extensionOffset));
        currentExtensionPosition = MAX_EXTENSION_ENCODER;
    }
    // If the home flag is set, reset the encoder value to the known minimum.
    if (homeSensor.Get()) {
        extensionMotor.setEncoderPosition(DENORMALIZE_EXTENSION_POSITION(0, extensionOffset));
        currentExtensionPosition = 0;
    }

    // Calculate the target extension position.
    double targetExtensionPosition = currentExtensionPosition;
    {
        // Convert the positional extension length to a percent of the range of motion.
        double extensionPercent = positionalExtensionLength / MAX_EXTENSION_LENGTH;
        // Don't overshoot the limits!
        extensionPercent = std::clamp(extensionPercent, 0.0, 1.0);

        targetExtensionPosition = extensionPercent * MAX_EXTENSION_ENCODER;
    }

    // Constrain the lift when close to the hard stop to prevent damage.
    if (currentPivotPosition <= MIN_PIVOT_EXTEND_ENCODER) {
        targetExtensionPosition = currentExtensionPosition;

        if (homeSensor.Get() || currentExtensionPosition < ENCODER_TOLERANCE) {
            // We can pivot freely because the lift is fully retracted.
        }
        else {
            // The lift is not fully retracted! We need to stop pivoting and retract fully!

            targetPivotPosition = currentPivotPosition;
        }
    }

    // Set the PID reference of the pivot motors to the encoder position.
    pivotMotorLeft.set(ThunderCANMotorController::ControlMode::POSITION, targetPivotPosition);
    pivotMotorRight.set(ThunderCANMotorController::ControlMode::POSITION, targetPivotPosition);

    // Set the PID reference of the extension motor to the encoder position.
    extensionMotor.set(ThunderCANMotorController::ControlMode::POSITION, DENORMALIZE_EXTENSION_POSITION(targetExtensionPosition, extensionOffset));

    auto isAtPosition = [&](double current, double target) {
        return std::abs(current - target) < ENCODER_TOLERANCE;
    };
    
    // Check if the lift is at the target position.
    atPosition = isAtPosition(currentPivotPosition, targetPivotPosition) &&
                    isAtPosition(currentExtensionPosition, targetExtensionPosition);
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
    pivotMotorLeft.configIZone(PIVOT_I_ZONE);
    pivotMotorLeft.configFF(PIVOT_FF);

    // Right Pivot Motor Configuration
    pivotMotorRight.configFactoryDefault();
    pivotMotorRight.configFactoryDefault();
    pivotMotorRight.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    pivotMotorRight.configSmartCurrentLimit(PIVOT_MAX_AMPERAGE);
    pivotMotorRight.setInverted(true);

    pivotMotorRight.configP(PIVOT_P);
    pivotMotorRight.configI(PIVOT_I);
    pivotMotorRight.configD(PIVOT_D);
    pivotMotorRight.configIZone(PIVOT_I_ZONE);
    pivotMotorRight.configFF(PIVOT_FF);
    
    // Extension Motor Configuration
    extensionMotor.configFactoryDefault();
    extensionMotor.setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);
    extensionMotor.configSmartCurrentLimit(EXTENSION_MAX_AMPERAGE);
    extensionMotor.setInverted(true);

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

    frc::SmartDashboard::PutNumber("Lift_EncoderRawExtension", extensionMotor.getEncoderPosition());
    frc::SmartDashboard::PutNumber("Lift_EncoderRawPivotLeft", pivotMotorLeft.getEncoderPosition());
    frc::SmartDashboard::PutNumber("Lift_EncoderRawPivotRight", pivotMotorRight.getEncoderPosition());

    auto [currentPivotPosition, currentExtensionPosition, extensionOffset] = getCurrentPosition();

    frc::SmartDashboard::PutNumber("Lift_EncoderExtension",       currentExtensionPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderPivot",           currentPivotPosition);
    frc::SmartDashboard::PutNumber("Lift_EncoderExtensionOffset", extensionOffset);

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
        double targetPivotPercent = positionalAngle / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);
        double targetExtensionPercent = positionalExtensionLength / MAX_EXTENSION_LENGTH;

        if (atPosition) {
            targetPivotPercent = pivotPercent;
            targetExtensionPercent = extensionPercent;
        }

        frc::SmartDashboard::PutNumber("thunderdashboard_lift_pivot_target_percent", targetPivotPercent);
        frc::SmartDashboard::PutNumber("thunderdashboard_lift_extension_target_percent", targetExtensionPercent);
    }
}
