#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>


class UltraBrickMode : public Mechanism {
public:
    UltraBrickMode();
    ~UltraBrickMode();
  
    void setState(bool extended);

private:
    frc::DoubleSolenoid solenoid {
        HardwareManager::IOMap::CAN_PCM, frc::PneumaticsModuleType::CTREPCM,
        HardwareManager::IOMap::PCM_ULTRA_BRICK_MODE_EXTEND, HardwareManager::IOMap::PCM_ULTRA_BRICK_MODE_RETRACT
    };
};
