#include <frc/smartdashboard/SmartDashboard.h>
#include <Drive/UltraBrickMode.h>

UltraBrickMode::UltraBrickMode(){

}

UltraBrickMode::~UltraBrickMode(){

}

void UltraBrickMode::setState(bool extended){
    solenoid.Set(extended);
}

