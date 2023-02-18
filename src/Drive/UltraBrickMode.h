#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <frc/Solenoid.h>



class UltraBrickMode : public Mechanism {
public:
    UltraBrickMode();
    ~UltraBrickMode();
  
    void setState(bool extended);

private:
    frc::Solenoid solenoid {frc::PneumaticsModuleType::CTREPCM, HardwareManager::IOMap::PCM_ULTRA_BRICK_MODE};
};
