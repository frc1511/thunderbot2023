#include <Hardware/MotorController/CTRE/CANTalonSRX.h>

ThunderCANTalonSRX::ThunderCANTalonSRX(int canID)
: ThunderCANMotorController(canID), talon(canID) { }

ThunderCANTalonSRX::~ThunderCANTalonSRX() = default;

ThunderCANMotorController::Vendor ThunderCANTalonSRX::getVendor() const {
    return ThunderCANMotorController::Vendor::CTRE;
}

int ThunderCANTalonSRX::set(ControlMode mode, double value) {
    TalonSRXControlMode talonCtrlMode = TalonSRXControlMode::PercentOutput;
    switch (mode) {
        case ControlMode::PERCENT_OUTPUT:
            talonCtrlMode = TalonSRXControlMode::PercentOutput;
            break;
        case ControlMode::POSITION:
            talonCtrlMode = TalonSRXControlMode::Position;
            break;
        case ControlMode::VELOCITY:
            talonCtrlMode = TalonSRXControlMode::Velocity;
            break;
        case ControlMode::CURRENT:
            talonCtrlMode = TalonSRXControlMode::Current;
            break;
    }
    
    talon.Set(talonCtrlMode, value);
    return 0;
}

double ThunderCANTalonSRX::getPercentOutput() const {
    return talon.GetMotorOutputPercent();
}

double ThunderCANTalonSRX::getEncoderPosition() const {
    return talon.GetSelectedSensorPosition(0);
}

double ThunderCANTalonSRX::getEncoderVelocity() const {
    return talon.GetSelectedSensorVelocity(0);
}

int ThunderCANTalonSRX::setEncoderPosition(double position) {
    talon.SetSelectedSensorPosition(position, 0);
    return 0;
}

double ThunderCANTalonSRX::getAlternateEncoderPosition() {
    return talon.GetSelectedSensorPosition(1);
}

double ThunderCANTalonSRX::getAlternateEncoderVelocity() {
    return talon.GetSelectedSensorVelocity(1);
}

int ThunderCANTalonSRX::setAlternateEncoderPosition(double position) {
    talon.SetSelectedSensorPosition(position, 1);
    return 0;
}

int ThunderCANTalonSRX::setIdleMode(IdleMode mode) {
    NeutralMode talonNeutralMode = NeutralMode::Brake;
    switch (mode) {
        case IdleMode::BRAKE:
            talonNeutralMode = NeutralMode::Brake;
            break;
        case IdleMode::COAST:
            talonNeutralMode = NeutralMode::Coast;
            break;
    }
    
    talon.SetNeutralMode(talonNeutralMode);
    return 0;
}

void ThunderCANTalonSRX::setInverted(bool isInverted) {
    talon.SetInverted(isInverted);
}

bool ThunderCANTalonSRX::getInverted() const {
    return talon.GetInverted();
}

units::ampere_t ThunderCANTalonSRX::getOutputCurrent() const {
    return units::ampere_t(talon.GetOutputCurrent());
}

int ThunderCANTalonSRX::configFactoryDefault(units::millisecond_t timeout) {
   ctre::phoenix::ErrorCode error = talon.ConfigFactoryDefault(timeout.value());

   return static_cast<int>(error);
}

int ThunderCANTalonSRX::configOpenLoopRamp(units::second_t seconds, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.ConfigOpenloopRamp(seconds.value(), timeout.value());
    
   return static_cast<int>(error);
}

int ThunderCANTalonSRX::configClosedLoopRamp(units::second_t seconds, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.ConfigClosedloopRamp(seconds.value(), timeout.value());
    
    return static_cast<int>(error);
}

int ThunderCANTalonSRX::configSmartCurrentLimit(units::ampere_t limit, units::millisecond_t timeout) {
    return 0;
}

int ThunderCANTalonSRX::burnFlash() {
    return 0;
}

int ThunderCANTalonSRX::configAlternateEncoder(int countsPerRev) {
    return 0;
}

units::fahrenheit_t ThunderCANTalonSRX::getTemperature() const {
    return units::celsius_t(talon.GetTemperature());
}

int ThunderCANTalonSRX::configP(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_kP(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonSRX::configI(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_kI(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonSRX::configD(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_kD(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonSRX::configFF(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_kF(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonSRX::configIZone(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_IntegralZone(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonSRX::configOutputRange(double min, double max, int slotID, units::millisecond_t timeout) {
    // I am pretty sure this is right, but can't be tested right now.
    ctre::phoenix::ErrorCode error1 = talon.ConfigPeakOutputForward(max, timeout.value());
    ctre::phoenix::ErrorCode error2 = talon.ConfigPeakOutputReverse(min, timeout.value());
    
    int res = static_cast<int>(error1);
    if (!res) res = static_cast<int>(error2);
    
    return res;
}

void ThunderCANTalonSRX::follow(ThunderCANMotorController* master) {
    assert(master && master->getVendor() == ThunderCANMotorController::Vendor::CTRE);
    
    auto talonMaster = reinterpret_cast<ctre::phoenix::motorcontrol::can::TalonSRX*>(master->getRawMotorController());
    talon.Follow(*talonMaster);
}

void* ThunderCANTalonSRX::getRawMotorController() {
    return reinterpret_cast<void*>(&talon);
}
