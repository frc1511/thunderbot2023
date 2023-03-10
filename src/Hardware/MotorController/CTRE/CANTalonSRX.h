#pragma once

#include <Hardware/MotorController/CANMotorController.h>
#include <ctre/Phoenix.h>

class ThunderCANTalonSRX : public ThunderCANMotorController {
public:
    ThunderCANTalonSRX(int canID);
    ~ThunderCANTalonSRX();

    Vendor getVendor() const;
    int set(ControlMode mode, double value);
    double getPercentOutput() const;
    double getEncoderPosition() const;
    double getEncoderVelocity() const;
    int setEncoderPosition(double position);
    double getAlternateEncoderPosition();
    double getAlternateEncoderVelocity();
    int setAlternateEncoderPosition(double position);
    int setIdleMode(IdleMode mode);
    void setInverted(bool isInverted);
    bool getInverted() const;
    units::ampere_t getOutputCurrent() const;
    int configFactoryDefault(units::millisecond_t timeout = 50_ms);
    int configOpenLoopRamp(units::second_t seconds, units::millisecond_t timeout = 50_ms);
    int configClosedLoopRamp(units::second_t seconds, units::millisecond_t timeout = 50_ms);
    int configSmartCurrentLimit(units::ampere_t limit, units::millisecond_t timeout = 50_ms);
    int burnFlash();
    int configAlternateEncoder(int countsPerRev);
    units::fahrenheit_t getTemperature() const;
    int configP(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms);
    int configI(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms);
    int configD(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms);
    int configFF(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms);
    int configIZone(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms);
    int configOutputRange(double min, double max, int slotID = 0, units::millisecond_t timeout = 50_ms);
    void follow(ThunderCANMotorController* master);
    void* getRawMotorController();

private:
    mutable ctre::phoenix::motorcontrol::can::TalonSRX talon;
};
