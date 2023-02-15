#pragma once

#include <Basic/Mechanism.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

/**
 * This mechanism is responsible for balancing the robot on the Charge Station
 * by using the angle and angular velocity of the robot's roll axis to
 * calculate the optimal drive velocity to balance the robot.
 */
class WhooshWhoosh : public Mechanism {
public:
    WhooshWhoosh();
    ~WhooshWhoosh();

    void resetToMode(MatchMode mode) override;
    void sendFeedback() override;
    void process() override;

    /**
     * Returns the roll axis angle of the robot.
     */
    units::degree_t getWhooshAngle();

    /**
     * Returns the roll axis angular velocity of the robot.
     */
    units::radians_per_second_t getWhooshRate();

    /**
     * Calculates the optimal drive velocity to balance the robot on the Charge
     * Station.
     */
    units::meters_per_second_t calculateAntiWhooshDriveVelocity();
};
