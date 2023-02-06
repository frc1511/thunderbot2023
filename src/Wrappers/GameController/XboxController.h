#pragma once

#include <Wrappers/GameController/GenericHIDGameController.h>

class ThunderXboxController : public ThunderGenericHIDGameController {
public:
    ThunderXboxController(Controller controller)
    : ThunderGenericHIDGameController(controller) { }

    ~ThunderXboxController() { }

    enum Button {
        Y = 4,
        B = 2,
        A = 1,
        X = 3,

        TRIANGLE = Y,
        CIRCLE = B,
        CROSS = A,
        SQUARE = X,

        LEFT_BUMPER = 5,
        RIGHT_BUMPER = 6,

        BACK = 7,
        START = 8,

        SHARE = BACK,
        OPTIONS = START,

        LEFT_STICK = 11,
        RIGHT_STICK = 12,
    };

    enum Axis {
        LEFT_X = 0,
        LEFT_Y = 1,
        RIGHT_X = 4,
        RIGHT_Y = 5,

        LEFT_TRIGGER = 2,
        RIGHT_TRIGGER = 3,
    };

    inline bool getTriangleButton(ButtonState state = ButtonState::HELD) { return getButton(TRIANGLE, state); }
    inline bool getCircleButton(ButtonState state = ButtonState::HELD) { return getButton(CIRCLE, state); }
    inline bool getCrossButton(ButtonState state = ButtonState::HELD) { return getButton(CROSS, state); }
    inline bool getSquareButton(ButtonState state = ButtonState::HELD) { return getButton(SQUARE, state); }

    inline bool getYButton(ButtonState state = ButtonState::HELD) { return getButton(Y, state); }
    inline bool getBButton(ButtonState state = ButtonState::HELD) { return getButton(B, state); }
    inline bool getAButton(ButtonState state = ButtonState::HELD) { return getButton(A, state); }
    inline bool getXButton(ButtonState state = ButtonState::HELD) { return getButton(X, state); }
    
    inline bool getLeftBumper(ButtonState state = ButtonState::HELD) { return getButton(LEFT_BUMPER, state); }
    inline bool getRightBumper(ButtonState state = ButtonState::HELD) { return getButton(RIGHT_BUMPER, state); }

    inline bool getShareButton(ButtonState state = ButtonState::HELD) { return getButton(SHARE, state); }
    inline bool getOptionsButton(ButtonState state = ButtonState::HELD) { return getButton(OPTIONS, state); }

    inline bool getLeftStickButton(ButtonState state = ButtonState::HELD) { return getButton(LEFT_STICK, state); }
    inline bool getRightStickButton(ButtonState state = ButtonState::HELD) { return getButton(RIGHT_STICK, state); }

    inline bool getBackButton(ButtonState state = ButtonState::HELD) { return getButton(BACK, state); }
    inline bool getStartButton(ButtonState state = ButtonState::HELD) { return getButton(START, state); }

    inline double getLeftXAxis() { return getAxis(LEFT_X); }
    inline double getLeftYAxis() { return getAxis(LEFT_Y); }

    inline double getRightXAxis() { return getAxis(RIGHT_X); }
    inline double getRightYAxis() { return getAxis(RIGHT_Y); }

    inline double getLeftTrigger() { return getAxis(LEFT_TRIGGER); }
    inline double getRightTrigger() { return getAxis(RIGHT_TRIGGER); }
};
