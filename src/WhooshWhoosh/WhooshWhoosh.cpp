#include <WhooshWhoosh/WhooshWhoosh.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <thread>
#include <chrono>

#define BALANCE_P 0.027 // 1m/s at 4deg
#define BALANCE_I 0.0
#define BALANCE_D 0.0

WhooshWhoosh::WhooshWhoosh()
: balancePIDController(BALANCE_P, BALANCE_I, BALANCE_D) { }

WhooshWhoosh::~WhooshWhoosh() = default;

void WhooshWhoosh::resetToMode(MatchMode mode) {
    balancePIDController.Reset();
}

void WhooshWhoosh::process() {
}

void WhooshWhoosh::setHeadingAngle(units::degree_t angle) {
#ifdef HAS_IMU
    imu.SetGyroAngle(frc1511::ADIS16470_IMU::IMUAxis::kYaw, angle);
#endif
}

units::degree_t WhooshWhoosh::getHeadingAngle() {
#ifdef HAS_IMU
    return imu.GetAngle(frc1511::ADIS16470_IMU::kYaw);
#endif
    return 0_deg;
}

units::radians_per_second_t WhooshWhoosh::getHeadingRate() {
#ifdef HAS_IMU
    return imu.GetRate(frc1511::ADIS16470_IMU::kYaw);
#endif
    return 0_deg_per_s;
}

units::degree_t WhooshWhoosh::getTiltAngle() {
#ifdef HAS_IMU
    return imu.GetAngle(frc1511::ADIS16470_IMU::kPitch);
#endif
    return 0_deg;
}

units::radians_per_second_t WhooshWhoosh::getTiltRate() {
#ifdef HAS_IMU
    return imu.GetRate(frc1511::ADIS16470_IMU::kPitch);
#endif
    return 0_deg_per_s;
}

units::meters_per_second_t WhooshWhoosh::calculateAntiTiltDriveVelocity() {
    return units::meters_per_second_t(balancePIDController.Calculate(getTiltAngle().value(), 0));
}

void WhooshWhoosh::calibrateIMU() {
    using namespace std::chrono_literals;

#ifdef HAS_IMU
    imu.Calibrate();

    // Sleep for 4 seconds as the IMU calibrates.
    std::this_thread::sleep_for(4s);
#endif
}

void WhooshWhoosh::resetHeading() {
#ifdef HAS_IMU
    imu.SetGyroAngle(frc1511::ADIS16470_IMU::kYaw, 0_deg);
#endif
}

void WhooshWhoosh::resetTilt() {
#ifdef HAS_IMU
    imu.SetGyroAngle(frc1511::ADIS16470_IMU::kRoll, 0_deg);
#endif
}

void WhooshWhoosh::sendFeedback() {
#ifdef HAS_IMU
    frc::SmartDashboard::PutNumber("IMU_x",          imu.GetAngle(frc1511::ADIS16470_IMU::kX).value());
    frc::SmartDashboard::PutNumber("IMU_y",          imu.GetAngle(frc1511::ADIS16470_IMU::kY).value());
    frc::SmartDashboard::PutNumber("IMU_z",          imu.GetAngle(frc1511::ADIS16470_IMU::kZ).value());
    frc::SmartDashboard::PutNumber("IMU_yaw",        imu.GetAngle(frc1511::ADIS16470_IMU::kYaw).value());
    frc::SmartDashboard::PutNumber("IMU_pitch",      imu.GetAngle(frc1511::ADIS16470_IMU::kPitch).value());
    frc::SmartDashboard::PutNumber("IMU_roll",       imu.GetAngle(frc1511::ADIS16470_IMU::kRoll).value());
    frc::SmartDashboard::PutNumber("IMU_rate_x",     imu.GetRate(frc1511::ADIS16470_IMU::kX).value());
    frc::SmartDashboard::PutNumber("IMU_rate_y",     imu.GetRate(frc1511::ADIS16470_IMU::kY).value());
    frc::SmartDashboard::PutNumber("IMU_rate_z",     imu.GetRate(frc1511::ADIS16470_IMU::kZ).value());
    frc::SmartDashboard::PutNumber("IMU_rate_yaw",   imu.GetRate(frc1511::ADIS16470_IMU::kYaw).value());
    frc::SmartDashboard::PutNumber("IMU_rate_pitch", imu.GetRate(frc1511::ADIS16470_IMU::kPitch).value());
    frc::SmartDashboard::PutNumber("IMU_rate_roll",  imu.GetRate(frc1511::ADIS16470_IMU::kRoll).value());
    frc::SmartDashboard::PutNumber("IMU_accel_x",    imu.GetAccelX().value());
    frc::SmartDashboard::PutNumber("IMU_accel_y",    imu.GetAccelY().value());
    frc::SmartDashboard::PutNumber("IMU_accel_z",    imu.GetAccelZ().value());
#endif
}
