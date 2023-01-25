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
    // The I/O Map for the robot.
    using IOMap = 
#if WHICH_ROBOT == 2023
        IOMap2023;
#elif WHICH_ROBOT == 2022
        IOMap2022;
#elif WHICH_ROBOT == TEST_BOARD
        IOMapTestBoard;
#endif

    // Motor and sensor types.
#if WHICH_ROBOT == 2023 || WHICH_ROBOT == 2022
    using SwerveDriveMotor = ThunderCANSparkMax;
    using SwerveTurningMotor = ThunderCANSparkMax;
    using SwerveTurningEncoder = ThunderCANCoder;
    using DriveIMU = ThunderADIS16470IMU;
    using GrabberIntakeMotor = ThunderCANSparkMax;
    using LiftExtensionMotor = ThunderCANSparkMax;
    using LiftPivotMotor = ThunderCANSparkMax;

#elif WHICH_ROBOT == TEST_BOARD
    using SwerveDriveMotor = ThunderCANMotorController;
    using SwerveTurningMotor = ThunderCANMotorController;
    using SwerveTurningEncoder = ThunderCANMagneticEncoder;
    using DriveIMU = ThunderIMU;
    using GrabberIntakeMotor = ThunderCANMotorController;
    using LiftExtensionMotor = ThunderCANMotorController;
    using LiftPivotMotor = ThunderCANMotorController;

#endif

    // Game controller types.
    using DriveGameController = ThunderPS4Controller;
    using AuxGameController = ThunderPS4Controller;
};
