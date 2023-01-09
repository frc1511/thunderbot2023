#pragma once

#include <Wrappers/GameController/PS4Controller.h>
#include <Wrappers/GameController/XboxController.h>

#include <Wrappers/MotorController/CANMotorController.h>
#include <Wrappers/MotorController/Rev/CANSparkMax.h>
#include <Wrappers/MotorController/CTRE/CANTalonFX.h>

#include <Wrappers/MagneticEncoder/CANMagneticEncoder.h>
#include <Wrappers/MagneticEncoder/CTRE/CANCoder.h>

#include <Wrappers/IMU/IMU.h>
#include <Wrappers/IMU/ADIS16470IMU.h>

#include <Hardware/IOMap.h>

class HardwareManager {
public:
// Declare motor and sensor types here (only for interchangable hardware - stuff that has a wrapper).

#ifdef TEST_BOARD
// Test board hardware.
#else
// Regular robot hardware.
#endif

    using DriveGameController = ThunderPS4Controller;
    using AuxGameController = ThunderPS4Controller;
};
