#pragma once

#include <Wrappers/GameController/GameController.h>
#include <thread>
#include <mutex>

extern "C" {
    struct InputState {
        uint32_t axisLeftX : 8;
        uint32_t axisLeftY : 8;
        uint32_t axisRightX : 8;
        uint32_t axisRightY : 8;
        uint32_t axisLeftTrigger : 8;
        uint32_t axisRightTrigger : 8;
        uint32_t buttonsAndDPad : 8;
        uint32_t buttonsA : 8;
        uint32_t accelX : 16;
        uint32_t accelY : 16;
        uint32_t accelZ : 16;
        uint32_t gyroX : 16;
        uint32_t gyroY : 16;
        uint32_t gyroZ : 16;
        uint32_t buttonsB : 8;
        uint32_t pad : 24;
    }; // 24 bytes
    typedef struct InputState InputState;

    struct OutputState {
        uint32_t rumbleLeft : 8;
        uint32_t rumbleRight : 8;
        uint32_t micLed : 2; // (0 = off, 1 = on, 2 = pulse)
        uint32_t playerLedFade : 1;
        uint32_t playerLedBitmask : 5; // (0x01 = left, 0x02 = middle left, 0x04 = middle, 0x08 = middle right, 0x10 = right)
        uint32_t lightbarR : 8;
        uint32_t lightbarG : 8;
        uint32_t lightbarB : 8;
        uint32_t leftTriggerStartPosition : 8;
        uint32_t leftTriggerEndPosition : 8;
        uint32_t leftTriggerForce : 8;
        uint32_t rightTriggerStartPosition : 8;
        uint32_t rightTriggerEndPosition : 8;
        uint32_t rightTriggerForce : 8;
        uint32_t leftTriggerEffect : 2; // (0 = off, 1 = section, 2 = continuous)
        uint32_t rightTriggerEffect : 2; // (0 = off, 1 = section, 2 = continuous)
        uint32_t pad : 28;
    }; // 16 bytes
    typedef struct OutputState OutputState;
}

class ThunderPS5Controller : public ThunderGameController {
public:
    ThunderPS5Controller(Controller controller);
    ~ThunderPS5Controller();

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
    void threadMain();

    std::thread serverThread;
    std::mutex inputMutex;
    std::mutex outputMutex;

    InputState inputState;
    InputState lastInputState;

    OutputState outputState;
    bool newOutput = false;

    char inputBuffer[1024];
    char outputBuffer[1024];
};
