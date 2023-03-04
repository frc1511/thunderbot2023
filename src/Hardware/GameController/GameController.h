#pragma once

class ThunderGameController {
public:
    enum class Controller {
        DRIVER = 0,
        AUX = 1,
    };

    ThunderGameController(Controller controller) : controller(controller) { }
    virtual ~ThunderGameController() { }

    virtual void process() { }

    enum class ButtonState {
        PRESSED,
        HELD,
        RELEASED,
    };

    virtual bool getButton(int button, ButtonState state = ButtonState::HELD) { return false; }
    virtual double getAxis(int axis) { return 0.0; }

    enum class DPad {
        NONE = -1,
        UP = 0,
        UP_RIGHT = 45,
        RIGHT = 90,
        DOWN_RIGHT = 135,
        DOWN = 180,
        DOWN_LEFT = 225,
        LEFT = 270,
        UP_LEFT = 315,
    };

    virtual DPad getDPad() { return DPad::NONE; }

    virtual bool getTriangleButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getCircleButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getCrossButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getSquareButton(ButtonState state = ButtonState::HELD) { return false; }

    virtual bool getYButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getBButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getAButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getXButton(ButtonState state = ButtonState::HELD) { return false; }

    virtual bool getLeftBumper(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getRightBumper(ButtonState state = ButtonState::HELD) { return false; }

    virtual bool getShareButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getOptionsButton(ButtonState state = ButtonState::HELD) { return false; }

    virtual bool getBackButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getStartButton(ButtonState state = ButtonState::HELD) { return false; }

    virtual bool getLeftStickButton(ButtonState state = ButtonState::HELD) { return false; }
    virtual bool getRightStickButton(ButtonState state = ButtonState::HELD) { return false; }

    virtual double getLeftXAxis() { return 0.0; }
    virtual double getLeftYAxis() { return 0.0; }
    virtual double getRightXAxis() { return 0.0; }
    virtual double getRightYAxis() { return 0.0; }

    virtual double getLeftTrigger() { return 0.0; }
    virtual double getRightTrigger() { return 0.0; }

    virtual void setRumble(double left, double right) { }

    enum class TriggerEffect {
        NONE      = 0x00,
        SECTION    = 0x01,
        CONTINUOUS = 0x02,
    };

    enum class Trigger {
        LEFT  = 0x00,
        RIGHT = 0x01,
    };

    virtual void setAdaptiveTrigger(Trigger trigger, TriggerEffect effect, double start = 0.0, double end = 0.0, double force = 0.0) { }

    virtual void setLightbarColor(unsigned char r, unsigned char g, unsigned char b) { }

    enum class MicrophoneLedState {
        OFF   = 0x00,
        ON    = 0x01,
        PULSE = 0x02,
    };

    virtual void setMicrophoneLed(MicrophoneLedState state) { }

    enum PlayerLED {
        PLAYER_LEFT         = 0x01,
        PLAYER_MIDDLE_LEFT  = 0x02,
        PLAYER_MIDDLE       = 0x04,
        PLAYER_MIDDLE_RIGHT = 0x08,
        PLAYER_RIGHT        = 0x10,
    };

    virtual void setPlayerLed(unsigned char bitmask, bool fade) { }

protected:
    Controller controller;
};
