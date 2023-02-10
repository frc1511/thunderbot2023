#pragma once

/*
    ██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
    ██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
    ██║██║   ██║    ██╔████╔██║███████║██████╔╝
    ██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
    ██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
    ╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     v2023
*/

#define TEST_BOARD 1

// Which robot to build the robot code for.
// #define WHICH_ROBOT 2023 // 2023 Robot.
// #define WHICH_ROBOT 2022 // 2022 Robot.
#define WHICH_ROBOT TEST_BOARD // Test board

#ifndef WHICH_ROBOT
# error No Robot Specified
#endif

// I/O ID for missing devices.
#define IO_MISSING -1

class IOMap {
public:
    enum {
        // Network Ports
        NET_PS5_DRIVER = 5809,
        NET_PS5_AUX = 5808,
        NET_ROLLING_RASPBERRY = 5800,
    };
};

// I/O Map for the 2023 robot.
class IOMap2023 : public IOMap {
public:
    enum {
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

        CAN_GRABBER_INTAKE_LEFT  = 2,
        CAN_GRABBER_INTAKE_RIGHT = 3,

        CAN_LIFT_EXTENSION   = 4,
        CAN_LIFT_PIVOT_LEFT  = 5,
        CAN_LIFT_PIVOT_RIGHT = 6,

        // PWM
        PWM_BLINKY_BLINKY = 0,

        // DIO
        DIO_GRABBER_INTAKE = 0,
        DIO_LIFT_HOME      = 1,
        DIO_LIFT_EXTENSION = 2, 

        // SPI

        // PCM
        PCM_GRABBER_PISTON_1_EXTEND  = 0,
        PCM_GRABBER_PISTON_1_RETRACT = 1,
        PCM_GRABBER_PISTON_2_EXTEND  = 2,
        PCM_GRABBER_PISTON_2_RETRACT = 3,

        PCM_GRABBER_WRIST_EXTEND  = 4,
        PCM_GRABBER_WRIST_RETRACT = 5,
    };
};

// I/O Map for the 2022 robot (select devices for testing).
class IOMap2022 : public IOMap {
public:
    enum {
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

        CAN_GRABBER_INTAKE_LEFT  = IO_MISSING,
        CAN_GRABBER_INTAKE_RIGHT = IO_MISSING,

        CAN_LIFT_EXTENSION   = IO_MISSING,
        CAN_LIFT_PIVOT_LEFT  = IO_MISSING,
        CAN_LIFT_PIVOT_RIGHT = IO_MISSING,

        // PWM
        PWM_BLINKY_BLINKY = IO_MISSING,

        // DIO
        DIO_GRABBER_INTAKE = 5,
        DIO_LIFT_HOME      = 7,
        DIO_LIFT_EXTENSION = 0,

        // SPI

        // PCM
        PCM_GRABBER_PISTON_1_EXTEND  = IO_MISSING,
        PCM_GRABBER_PISTON_1_RETRACT = IO_MISSING,
        PCM_GRABBER_PISTON_2_EXTEND  = IO_MISSING,
        PCM_GRABBER_PISTON_2_RETRACT = IO_MISSING,
        
        PCM_GRABBER_WRIST_EXTEND  = IO_MISSING,
        PCM_GRABBER_WRIST_RETRACT = IO_MISSING,
    };
};

// I/O Map for the test board (select devices for testing).
class IOMapTestBoard : public IOMap {
public:
    enum {
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

        CAN_GRABBER_INTAKE_LEFT  = 9,
        CAN_GRABBER_INTAKE_RIGHT = 10,

        CAN_LIFT_EXTENSION   = 2,
        CAN_LIFT_PIVOT_LEFT  = 8,
        CAN_LIFT_PIVOT_RIGHT = IO_MISSING,

        // PWM
        PWM_BLINKY_BLINKY = IO_MISSING,

        // DIO
        DIO_GRABBER_INTAKE = 4,
        DIO_LIFT_HOME      = 1,
        DIO_LIFT_EXTENSION = 0,

        // SPI

        // PCM
        PCM_GRABBER_PISTON_1_EXTEND  = 5,
        PCM_GRABBER_PISTON_1_RETRACT = 2,
        PCM_GRABBER_PISTON_2_EXTEND  = 4,
        PCM_GRABBER_PISTON_2_RETRACT = 3,
        
        PCM_GRABBER_WRIST_EXTEND  = 6,
        PCM_GRABBER_WRIST_RETRACT = 1,
    };
};
