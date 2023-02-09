#pragma once

#include <Wrappers/GameController/GenericHIDGameController.h>

class ThunderPS4Controller : public ThunderGenericHIDGameController {
public:
    ThunderPS4Controller(Controller whichController)
    : ThunderGenericHIDGameController(whichController) { }

    ~ThunderPS4Controller() { }

    enum Button {
        TRIANGLE = 4,
        CIRCLE = 3,
        CROSS = 2,
        SQUARE = 1,

        Y = TRIANGLE,
        B = CIRCLE,
        A = CROSS,
        X = SQUARE,

        LEFT_BUMPER = 5,
        RIGHT_BUMPER = 6,

        SHARE = 9,
        OPTIONS = 10,

        BACK = SHARE,
        START = OPTIONS,

        LEFT_STICK = 11,
        RIGHT_STICK = 12,
    };

    enum Axis {
        LEFT_X = 0,
        LEFT_Y = 1,
        RIGHT_X = 2,
        RIGHT_Y = 5,

        LEFT_TRIGGER = 3,
        RIGHT_TRIGGER = 4,
    };

    inline bool getTriangleButton(ButtonState state = ButtonState::HELD) override { return getButton(TRIANGLE, state); }
    inline bool getCircleButton(ButtonState state = ButtonState::HELD) override { return getButton(CIRCLE, state); }
    inline bool getCrossButton(ButtonState state = ButtonState::HELD) override { return getButton(CROSS, state); }
    inline bool getSquareButton(ButtonState state = ButtonState::HELD) override { return getButton(SQUARE, state); }

    inline bool getYButton(ButtonState state = ButtonState::HELD) override { return getButton(Y, state); }
    inline bool getBButton(ButtonState state = ButtonState::HELD) override { return getButton(B, state); }
    inline bool getAButton(ButtonState state = ButtonState::HELD) override { return getButton(A, state); }
    inline bool getXButton(ButtonState state = ButtonState::HELD) override { return getButton(X, state); }

    inline bool getLeftBumper(ButtonState state = ButtonState::HELD) override { return getButton(LEFT_BUMPER, state); }
    inline bool getRightBumper(ButtonState state = ButtonState::HELD) override { return getButton(RIGHT_BUMPER, state); }

    inline bool getShareButton(ButtonState state = ButtonState::HELD) override { return getButton(SHARE, state); }
    inline bool getOptionsButton(ButtonState state = ButtonState::HELD) override { return getButton(OPTIONS, state); }

    inline bool getBackButton(ButtonState state = ButtonState::HELD) override { return getButton(BACK, state); }
    inline bool getStartButton(ButtonState state = ButtonState::HELD) override { return getButton(START, state); }

    inline bool getLeftStickButton(ButtonState state = ButtonState::HELD) override { return getButton(LEFT_STICK, state); }
    inline bool getRightStickButton(ButtonState state = ButtonState::HELD) override { return getButton(RIGHT_STICK, state); }

    inline double getLeftXAxis() override { return getAxis(LEFT_X); }
    inline double getLeftYAxis() override { return getAxis(LEFT_Y); }
    inline double getRightXAxis() override { return getAxis(RIGHT_X); }
    inline double getRightYAxis() override { return getAxis(RIGHT_Y); }

    inline double getLeftTrigger() override { return getAxis(LEFT_TRIGGER); }
    inline double getRightTrigger() override { return getAxis(RIGHT_TRIGGER); }
};
