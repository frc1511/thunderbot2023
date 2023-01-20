#pragma once

#include <Basic/Mechanism.h>
#include <units/length.h>
#include <units/angle.h>

class Lift : public Mechanism {
public:
    Lift();
    ~Lift();

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

};
