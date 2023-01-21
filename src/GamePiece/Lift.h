#pragma once

#include <Basic/Mechanism.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/math.h>
#include <Hardware/HardwareManager.h>
class Lift : public Mechanism {
public:
    Lift();
    ~Lift();
    bool isAtPosition();
    void resetToMode(MatchMode mode) override;
    void doPersistentConfiguration() override;
    void process() override;
    void sendFeedback() override;


    //Manually set the speed of the motor to control the angle of the lift.
    void setManualAngleSpeed(double speed);
    //Manually set the speed of the motor to control the extension distance of the lift.
    void setManualExtensionSpeed(double speed);

    //Set the coordinate position of where the end effector should be.
    void setEndPosition(units::meter_t y, units::meter_t z);
private:
    double manualAngleSpeed;
    double manualExtensionSpeed;
    enum class ControlType{
        MANUAL,
        POSITION,
    };
    ControlType controlType;
    units::meter_t positionalExtensionLength;
    units::degree_t positionalAngle;
    units::degree_t minAngle;
    units::degree_t maxAngle;
    double maxAnglePosition;
    units::meter_t maxExtension;
    double maxExtensionPosition;
    bool atPosition;
    HardwareManager::LiftExtensionMotor extensionMotor;
    HardwareManager::LiftPivotMotor pivotMotorLeft;
    HardwareManager::LiftPivotMotor pivotMotorRight;





};
