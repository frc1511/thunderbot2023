#include <Trajectory/LinearTrajectory.h>

LinearTrajectory::LinearTrajectory(frc::Pose2d _startPose,
                                   frc::Pose2d _endPose,
                                   units::meters_per_second_t _maxVel,
                                   units::meters_per_second_squared_t _maxAccel,
                                   units::meters_per_second_t _startVel,
                                   units::meters_per_second_t _endVel)
    : startPose(_startPose),
      endPose(_endPose),
      startVel(_startVel),
      endVel(_endVel),
      maxVel(_maxVel),
      maxAccel(_maxAccel) {

    frc::Twist2d deltaPose = startPose.Log(endPose);

    // The direction in which the robot will travel.
    heading = units::math::atan2(deltaPose.dy, deltaPose.dx);

    units::meter_t totalDistance = units::math::hypot(deltaPose.dx, deltaPose.dy);

    /**
     * Calculate the max attainable velocity given the max acceleration.
     *
     * v = √((2ax + v0^2 + v1^2) / 2)
     */
    units::meters_per_second_t maxAttainableVel(
        units::math::sqrt((2 * maxAccel * totalDistance + startVel * startVel + endVel * endVel) / 2.0)
    );

    // If the max attainable velocity is less than configured max velocity, set it to that.
    if (maxAttainableVel < maxVel) {
        maxVel = maxAttainableVel;
    }

    /**
     * Calculate the time to accelerate from start velocity to max velocity.
     * v = v0 + at
     * t = (v - v0) / a
     */
    accelTime = (maxVel - startVel) / maxAccel;

    /**
     * Calculate the distance traveled during acceleration.
     * x = v0t + 1/2at^2
     */
    accelDist = startVel * accelTime + 0.5 * maxAccel * accelTime * accelTime;

    /**
     * Calculate the time to decelerate from max velocity to end velocity.
     * v = v0 + at
     * t = (v - v0) / a
     */
    decelTime = (maxVel - endVel) / maxAccel;

    /**
     * Calculate the distance traveled during deceleration.
     * x = v0t + 1/2at^2
     */
    decelDist = maxVel * decelTime + 0.5 * maxAccel * decelTime * decelTime;

    // The remaining distance to travel is cruise distance.
    cruiseDist = totalDistance - accelDist - decelDist;

    /**
     * Calculate the time to cruise at max velocity.
     * v = x / t
     */
    cruiseTime = cruiseDist / maxVel;
}

LinearTrajectory::~LinearTrajectory() { }

Trajectory::State LinearTrajectory::sample(units::second_t time) const {
    // Trajectory is complete, so return the end state.
    if (time >= duration) {
        return State(endPose, endVel);
    }

    units::meters_per_second_t vel;
    units::meter_t dist;

    if (time < accelTime) {
        // Accelerating

        // v = v0 + at
        vel = startVel + maxAccel * time;

        // x = x0 + v0t + 1/2at^2
        dist = startVel * time + 0.5 * maxAccel * time * time;
    }
    else if (time < accelTime + cruiseTime) {
        // Cruising

        time -= accelTime;

        vel = maxVel;

        dist = accelDist + maxVel * time;
    }
    else {
        // Decelerating

        time -= accelTime + cruiseTime;

        // v = v0 + at
        vel = maxVel - maxAccel * time;

        dist = accelDist + cruiseDist + maxVel * time - 0.5 * maxAccel * time * time;
    }

    // Distance to travel.
    frc::Twist2d delta(
        dist * heading.Cos(),
        dist * heading.Sin(),
        (endPose.Rotation() - startPose.Rotation()).Radians()
    );

    return State(startPose.Exp(delta), vel);
}
