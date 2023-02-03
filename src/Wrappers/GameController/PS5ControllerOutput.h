#pragma once

class PS5ControllerOutput {
public:
    enum class Controller {
        DRIVER = 0,
        AUX = 1,
    };

    static void set_rumble(Controller controller, double left, double right);

    enum class TriggerEffect {
        NONE = 0,
        SECTION = 1,
        CONTINUOUS = 2,
    };

    enum class Trigger {
        LEFT = 0,
        RIGHT = 1,
    };

    static void set_adaptive_trigger(Controller controller, Trigger trigger, TriggerEffect effect, double start = 0.0, double end = 0.0, double force = 0.0);

    static void set_lightbar_color(Controller controller, unsigned char r, unsigned char g, unsigned char b);

    enum class MicrophoneLedState {
        OFF = 0,
        ON = 1,
        PULSE = 2,
    };

    static void set_microphone_led(Controller controller, MicrophoneLedState state);

    enum PlayerLED {
        PLAYER_LEFT         = 0x01,
        PLAYER_MIDDLE_LEFT  = 0x02,
        PLAYER_MIDDLE       = 0x04,
        PLAYER_MIDDLE_RIGHT = 0x08,
        PLAYER_RIGHT        = 0x10,
    };

    static void set_player_led(Controller controller, unsigned char bitmask, bool fade);
};