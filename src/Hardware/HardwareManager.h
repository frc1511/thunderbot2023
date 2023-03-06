#pragma once

#include <Hardware/GameController/PS5Controller.h>
#include <Hardware/GameController/PS4Controller.h>
#include <Hardware/GameController/XboxController.h>

#include <Hardware/MotorController/CANMotorController.h>
#include <Hardware/MotorController/Rev/CANSparkMax.h>
#include <Hardware/MotorController/CTRE/CANTalonFX.h>
#include <Hardware/MotorController/CTRE/CANTalonSRX.h>

#include <Hardware/MagneticEncoder/CANMagneticEncoder.h>
#include <Hardware/MagneticEncoder/CTRE/CANCoder.h>

#include <Hardware/IMU/ADIS16470_IMU.h>

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
#if WHICH_ROBOT == 2023
    using SwerveDriveMotor = ThunderCANSparkMax;
    using SwerveTurningMotor = ThunderCANSparkMax;
    using SwerveTurningEncoder = ThunderCANCoder;
    using GrabberIntakeMotor = ThunderCANSparkMax;
    using LiftExtensionMotor = ThunderCANSparkMax;
    using LiftLeftPivotMotor = ThunderCANSparkMax;
    using LiftRightPivotMotor = ThunderCANSparkMax;

#elif WHICH_ROBOT == 2022
    using SwerveDriveMotor = ThunderCANSparkMax;
    using SwerveTurningMotor = ThunderCANSparkMax;
    using SwerveTurningEncoder = ThunderCANCoder;
    using GrabberIntakeMotor = ThunderCANMotorController;
    using LiftExtensionMotor = ThunderCANMotorController;
    using LiftLeftPivotMotor = ThunderCANMotorController;
    using LiftRightPivotMotor = ThunderCANMotorController;

#elif WHICH_ROBOT == TEST_BOARD
    using SwerveDriveMotor = ThunderCANMotorController;
    using SwerveTurningMotor = ThunderCANMotorController;
    using SwerveTurningEncoder = ThunderCANMagneticEncoder;
    using GrabberIntakeMotor = ThunderCANMotorController;
    using LiftExtensionMotor = ThunderCANMotorController;
    using LiftLeftPivotMotor = ThunderCANMotorController;
    using LiftRightPivotMotor = ThunderCANMotorController;

#endif

    // Game controller types.
    using DriveGameController = ThunderPS4Controller;
    using AuxGameController = ThunderPS4Controller;
};
