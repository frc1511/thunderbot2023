#include <GamePiece/Lift.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <frc/geometry/Twist2d.h>
#include <Util/Parser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>

#define PIVOT_FF 0.0//0.05

// The allowable error in the pivot angle and extension length.
#define PIVOT_TOLERANCE 3_deg
#define PIVOT_TOLERANCE_BIGGER 6_deg
#define EXTENSION_TOLERANCE 2_in

// The minimum angle of the arm.
#define MIN_PIVOT_ANGLE -47_deg

// The maximum angle of the arm.
#define MAX_PIVOT_ANGLE 20_deg

// The encoder value at the maximum angle of the arm.
#define MAX_PIVOT_ENCODER 89

// The maximum extension length of the arm (delta from starting position).
#define MAX_EXTENSION_LENGTH 38.25_in

// The encoder value at the maximum extension length of the arm.
#define MAX_EXTENSION_ENCODER 57.929295

// The minimum pivot position at which the arm can be extended.
#define MIN_PIVOT_EXTEND_ENCODER 3.738092

// The change in extension encoder value over the full pivot range of the arm.
#define EXTENSION_BACKDRIVE_ENCODER -2.309523

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

    // pivotLeftPIDController.Reset(-47_deg);
    pivotRightPIDController.Reset(-47_deg);
    extensionPIDController.Reset(0_m);
    targetExtension = 0_m;
    targetAngle = -40_deg;
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

    auto isAtPosition = [&](auto current, auto target, auto tolerance) {
        return units::math::abs(current - target) < tolerance;
    };

    // --- The rest of this function is for automatic control of the lift. ---

    // Get the current position of the lift.
    auto [currentExtension, currentLeftPivot, currentRightPivot, currentExtensionOffset] = getCurrentState();

    units::meter_t newTargetExtension = targetExtension;
    units::degree_t newTargetAngle = targetAngle;

    static bool isPivotingDown = false;
    static units::degree_t lockedAngle = 0_deg;

    if (!isAtPosition(currentRightPivot, targetAngle, PIVOT_TOLERANCE_BIGGER)) {
        if (currentRightPivot < targetAngle) {
            // Going up, don't extend.
            newTargetExtension = 0_m;
            isPivotingDown = false;
        }
        else {
            if (!isAtPosition(currentExtension, targetExtension, EXTENSION_TOLERANCE)) {
                if (!isPivotingDown) lockedAngle = currentRightPivot;
                // Going down, don't pivot.
                newTargetAngle = lockedAngle;
                isPivotingDown = true;
            }
        }
    }
    else {
        isPivotingDown = false;
    }

    double extensionOffsetPosition = (currentExtensionOffset / MAX_EXTENSION_LENGTH) * MAX_EXTENSION_ENCODER;

    static bool wasExtensionSensor = false;

    // If the extension flag is set, reset the encoder value to the known maximum.
    if (extensionSensor.Get() && !wasExtensionSensor) {
        extensionMotor.setEncoderPosition(DENORMALIZE_EXTENSION_POSITION(MAX_EXTENSION_ENCODER, extensionOffsetPosition));
        currentExtension = MAX_EXTENSION_LENGTH;

        extensionPIDController.Reset(DENORMALIZE_EXTENSION_POSITION(MAX_EXTENSION_LENGTH, currentExtensionOffset));
    }
    wasExtensionSensor = extensionSensor.Get();

    static bool wasHomeSensor = false;

    // If the home flag is set, reset the encoder value to the known minimum.
    if (homeSensor.Get() && !wasHomeSensor) {
        extensionMotor.setEncoderPosition(DENORMALIZE_EXTENSION_POSITION(0, extensionOffsetPosition));
        currentExtension = 0_m;

        extensionPIDController.Reset(DENORMALIZE_EXTENSION_POSITION(0_m, currentExtensionOffset));
    }
    wasHomeSensor = homeSensor.Get();

    // double leftPivotPercentOutput = pivotLeftPIDController.Calculate(currentLeftPivot, targetAngle);
    double rightPivotPercentOutput = pivotRightPIDController.Calculate(currentRightPivot, newTargetAngle);

    // leftPivotPercentOutput = std::clamp(leftPivotPercentOutput, -0.7, 0.7);
    rightPivotPercentOutput = std::clamp(rightPivotPercentOutput, -1.0, 1.0);

    double pivotPercent = (currentRightPivot - MIN_PIVOT_ANGLE) / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);

    // Control the pivot motors.
    pivotMotorLeft.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, rightPivotPercentOutput + pivotPercent * PIVOT_FF);
    pivotMotorRight.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, rightPivotPercentOutput + pivotPercent * PIVOT_FF);

    double extensionPercentOutput = extensionPIDController.Calculate(DENORMALIZE_EXTENSION_POSITION(currentExtension, currentExtensionOffset), DENORMALIZE_EXTENSION_POSITION(newTargetExtension, currentExtensionOffset));
    extensionPercentOutput = std::clamp(extensionPercentOutput, -1.0, 1.0);
    extensionMotor.set(ThunderCANMotorController::ControlMode::PERCENT_OUTPUT, extensionPercentOutput);
    
    // Check if the lift is at the target position.
    atPosition = isAtPosition(currentLeftPivot, newTargetAngle, 3_deg) &&
                 isAtPosition(currentRightPivot, newTargetAngle, 3_deg) &&
                 isAtPosition(currentExtension, newTargetExtension, 2_in);
}

void Lift::setManualPivotSpeed(double speed) {
    manualPivotSpeed = speed * MANUAL_PIVOT_SPEED_COEFF;
    controlType = ControlType::MANUAL;

    targetAngle = -1_deg;
    targetExtension = -1_m;
}

void Lift::setManualExtensionSpeed(double speed) {
    manualExtensionSpeed = speed * MANUAL_EXTENSION_SPEED_COEFF;
    controlType = ControlType::MANUAL;

    targetAngle = -1_deg;
    targetExtension = -1_m;
}

void Lift::setPosition(units::degree_t angle, units::meter_t extension) {
    controlType = ControlType::POSITION;
    targetAngle = angle;
    targetExtension = extension;
}

bool Lift::isAtPosition(){
    if (controlType == ControlType::POSITION){
        return atPosition;
    }
    return true;
}

Lift::LiftState Lift::getCurrentState() {
    // Left and right pivot encoder positions.
    double leftEncoderPosition = pivotMotorLeft.getEncoderPosition();
    double rightEncoderPosition = pivotMotorRight.getEncoderPosition();

    // Percentages of the pivot range of motion.
    double leftPercent = leftEncoderPosition / MAX_PIVOT_ENCODER;
    double rightPercent = rightEncoderPosition / MAX_PIVOT_ENCODER;

    // Angle of the pivot.
    units::degree_t leftAngle = leftPercent * (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE) + MIN_PIVOT_ANGLE;
    units::degree_t rightAngle = rightPercent * (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE) + MIN_PIVOT_ANGLE;

    // The extension offset based on the pivot angle (using left pivot angle right now).
    double extensionOffsetPosition = EXTENSION_BACKDRIVE_ENCODER * leftPercent;

    // The extension encoder position.
    double extensionPosition = extensionMotor.getEncoderPosition();
    // Remove the backdrive offset from the extension position.
    extensionPosition = NORMALIZE_EXTENSION_POSITION(extensionPosition, extensionOffsetPosition);

    // Percentage of the extension range of motion.
    double extensionPercent = extensionPosition / MAX_EXTENSION_ENCODER;

    // Extension length.
    units::meter_t extensionLength = extensionPercent * MAX_EXTENSION_LENGTH;

    units::meter_t extensionOffset = ((EXTENSION_BACKDRIVE_ENCODER * ((leftAngle - MIN_PIVOT_ANGLE) / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE))) / MAX_EXTENSION_ENCODER) * MAX_EXTENSION_LENGTH;

    return LiftState { extensionLength, leftAngle, rightAngle, extensionOffset };
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
    frc::SmartDashboard::PutNumber("Lift_TargetExtension_m", targetExtension.value());
    frc::SmartDashboard::PutNumber("Lift_TargetAngle_deg", targetAngle.value());
    frc::SmartDashboard::PutBoolean("Lift_TargetReached", atPosition);

    // Raw Encoder Positions
    frc::SmartDashboard::PutNumber("Lift_EncoderRawExtension", extensionMotor.getEncoderPosition());
    frc::SmartDashboard::PutNumber("Lift_EncoderRawPivotLeft", pivotMotorLeft.getEncoderPosition());
    frc::SmartDashboard::PutNumber("Lift_EncoderRawPivotRight", pivotMotorRight.getEncoderPosition());

    // Normalized Encoder Positions
    auto [currentExtension, currentLeftAngle, currentRightAngle, currentExtensionOffset] = getCurrentState();

    frc::SmartDashboard::PutNumber("Lift_Extension_m",       currentExtension.value());
    frc::SmartDashboard::PutNumber("Lift_PivotLeft_deg",     currentLeftAngle.value());
    frc::SmartDashboard::PutNumber("Lift_PivotRight_deg",    currentRightAngle.value());
    frc::SmartDashboard::PutNumber("Lift_ExtensionOffset_m", currentExtensionOffset.value());

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

    double extensionPercent = currentExtension / MAX_EXTENSION_LENGTH;
    double pivotPercent = (currentLeftAngle - MIN_PIVOT_ANGLE) / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);

    // Dashboard feedback
    frc::SmartDashboard::PutNumber("thunderdashboard_lift_pivot_percent", pivotPercent);
    frc::SmartDashboard::PutNumber("thunderdashboard_lift_extension_percent", extensionPercent);

    if (controlType == ControlType::MANUAL || getCurrentMode() == MatchMode::DISABLED) {
        frc::SmartDashboard::PutNumber("thunderdashboard_lift_pivot_target_percent", -1.0);
        frc::SmartDashboard::PutNumber("thunderdashboard_lift_extension_target_percent", -1.0);
    }
    else {
        double targetPivotPercent = (targetAngle - MIN_PIVOT_ANGLE) / (MAX_PIVOT_ANGLE - MIN_PIVOT_ANGLE);
        double targetExtensionPercent = targetExtension / MAX_EXTENSION_LENGTH;

        if (atPosition) {
            targetPivotPercent = pivotPercent;
            targetExtensionPercent = extensionPercent;
        }

        frc::SmartDashboard::PutNumber("thunderdashboard_lift_pivot_target_percent", targetPivotPercent);
        frc::SmartDashboard::PutNumber("thunderdashboard_lift_extension_target_percent", targetExtensionPercent);
    }
}
