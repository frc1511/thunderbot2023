#pragma once

/*
    ██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
    ██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
    ██║██║   ██║    ██╔████╔██║███████║██████╔╝
    ██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
    ██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
    ╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     v2023

    This file contains the I/O map for the robot. It is used to define the
    I/O pins for motors and sensors.
*/

// Which robot to build the robot code for.
#define WHICH_ROBOT 2023 // 2023 Robot.
// #define WHICH_ROBOT 2022 // 2022 Robot.
// #define WHICH_ROBOT TEST // Test board

#ifndef WHICH_ROBOT
# error No Robot Specified
#endif

// I/O ID for missing devices.
#define IO_MISSING -1

// I/O Map for the 2023 robot.
enum class IOMap2023 {
    // CAN
    CAN_SWERVE_DRIVE_MOTOR_FL = 6,
    CAN_SWERVE_DRIVE_MOTOR_BL = 4,
    CAN_SWERVE_DRIVE_MOTOR_BR = 7,
    CAN_SWERVE_DRIVE_MOTOR_FR = 5,

    CAN_SWERVE_ROT_MOTOR_FL = 2,
    CAN_SWERVE_ROT_MOTOR_BL = 8,
    CAN_SWERVE_ROT_MOTOR_BR = 3,
    CAN_SWERVE_ROT_MOTOR_FR = 1,

    CAN_SWERVE_ROT_CAN_CODER_FL = 11,
    CAN_SWERVE_ROT_CAN_CODER_BL = 10,
    CAN_SWERVE_ROT_CAN_CODER_BR = 13,
    CAN_SWERVE_ROT_CAN_CODER_FR = 12,

    CAN_GRABBER_INTAKE_L = 2,
    CAN_GRABBER_INTAKE_R = 3,

    CAN_LIFT_EXTENSION = 4,
    CAN_LIFT_PIVOT_LEFT = 5,
    CAN_LIFT_PIVOT_RIGHT = 6,


    // PWM
    PWM_BLINKY_BLINKY = 0,

    // DIO
    DIO_GRABBER_INTAKE = 0,
    // SPI
};

// I/O Map for the 2022 robot (select devices for testing).
enum class IOMap2022 {
    // CAN
    CAN_SWERVE_DRIVE_MOTOR_FL = 9,
    CAN_SWERVE_DRIVE_MOTOR_BL = 10,
    CAN_SWERVE_DRIVE_MOTOR_BR = 11,
    CAN_SWERVE_DRIVE_MOTOR_FR = 12,

    CAN_SWERVE_ROT_MOTOR_FL = 13,
    CAN_SWERVE_ROT_MOTOR_BL = 14,
    CAN_SWERVE_ROT_MOTOR_BR = 15,
    CAN_SWERVE_ROT_MOTOR_FR = 16,

    CAN_SWERVE_ROT_CAN_CODER_FL = 17,
    CAN_SWERVE_ROT_CAN_CODER_BL = 18,
    CAN_SWERVE_ROT_CAN_CODER_BR = 19,
    CAN_SWERVE_ROT_CAN_CODER_FR = 20,

    // PWM
    PWM_BLINKY_BLINKY = 4,

    // DIO

    // SPI
};

// I/O Map for the test board (select devices for testing).
enum class IOMapTestBoard {
    // CAN
    CAN_SWERVE_DRIVE_MOTOR_FL = IO_MISSING,
    CAN_SWERVE_DRIVE_MOTOR_BL = IO_MISSING,
    CAN_SWERVE_DRIVE_MOTOR_BR = IO_MISSING,
    CAN_SWERVE_DRIVE_MOTOR_FR = IO_MISSING,

    CAN_SWERVE_ROT_MOTOR_FL = IO_MISSING,
    CAN_SWERVE_ROT_MOTOR_BL = IO_MISSING,
    CAN_SWERVE_ROT_MOTOR_BR = IO_MISSING,
    CAN_SWERVE_ROT_MOTOR_FR = IO_MISSING,

    CAN_SWERVE_ROT_CAN_CODER_FL = IO_MISSING,
    CAN_SWERVE_ROT_CAN_CODER_BL = IO_MISSING,
    CAN_SWERVE_ROT_CAN_CODER_BR = IO_MISSING,
    CAN_SWERVE_ROT_CAN_CODER_FR = IO_MISSING,

    // PWM
    PWM_BLINKY_BLINKY = IO_MISSING,

    // DIO

    // SPI
};
