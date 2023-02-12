#pragma once

#include <Hardware/GameController/GameController.h>
#include <thread>
#include <mutex>

class ThunderPS5Controller : public ThunderGameController {
public:
    ThunderPS5Controller(Controller controller);
    ~ThunderPS5Controller();

    void process() override;

    enum Button {
        TRIANGLE = 1,
        CIRCLE = 2,
        CROSS = 3,
        SQUARE = 4,

        Y = TRIANGLE,
        B = CIRCLE,
        A = CROSS,
        X = SQUARE,

        LEFT_BUMPER = 5,
        RIGHT_BUMPER = 6,

        SHARE = 7,
        OPTIONS = 8,

        BACK = SHARE,
        START = OPTIONS,

        LEFT_STICK = 9,
        RIGHT_STICK = 10,
    };

    enum Axis {
        LEFT_X = 1,
        LEFT_Y = 2,
        RIGHT_X = 3,
        RIGHT_Y = 4,

        LEFT_TRIGGER = 5,
        RIGHT_TRIGGER = 6,
    };

    bool getButton(int button, ButtonState state = ButtonState::HELD) override;
    double getAxis(int axis) override;
    DPad getDPad() override;

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

    void setRumble(double left, double right) override;
    void setAdaptiveTrigger(Trigger trigger, TriggerEffect effect, double start = 0.0, double end = 0.0, double force = 0.0) override;
    void setLightbarColor(unsigned char r, unsigned char g, unsigned char b) override;
    void setMicrophoneLed(MicrophoneLedState state) override;
    void setPlayerLed(unsigned char bitmask, bool fade) override;

private:
    char inputState[24U];
    char lastInputState[24U];

    char outputState[16U];
    bool newOutputState = false;
};
