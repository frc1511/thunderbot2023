#include <frc/smartdashboard/SmartDashboard.h>
#include <Drive/UltraBrickMode.h>
#include <iostream>

UltraBrickMode::UltraBrickMode(){

}

UltraBrickMode::~UltraBrickMode(){

}

void UltraBrickMode::setState(bool extended){
    solenoid.Set(extended ? frc::DoubleSolenoid::kForward : frc::DoubleSolenoid::kReverse);
}

