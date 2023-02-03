#include <Wrappers/GameController/PS5ControllerOutput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cstdint>
#include <cstring>
#include <fmt/core.h>

extern "C" {
  struct PS5RumbleOptions {
    uint32_t left : 8;
    uint32_t right : 8;
    uint32_t pad : 16;
  };
  typedef struct PS5RumbleOptions PS5RumbleOptions;
  
  struct PS5TriggerOptions {
    uint32_t start_position : 8;
    uint32_t end_position : 8;
    uint32_t force : 8;
    uint32_t trigger_effect : 2; // (0 = off, 1 = section, 2 = continuous)
    uint32_t pad : 6;
  };
  typedef struct PS5TriggerOptions PS5TriggerOptions;

  struct PS5LEDOptions {
    uint32_t lightbar_r : 8;
    uint32_t lightbar_g : 8;
    uint32_t lightbar_b : 8;
    uint32_t mic : 2; // (0 = off, 1 = on, 2 = pulse)
    uint32_t player_fade : 1;
    uint32_t player_bitmask : 5; // (0x01 = left, 0x02 = middle left, 0x04 = middle, 0x08 = middle right, 0x10 = right)
  };
  typedef struct PS5LEDOptions PS5LEDOptions;
}

auto get_prefix = [](PS5ControllerOutput::Controller controller) {
    return controller == PS5ControllerOutput::Controller::DRIVER ? "thunderdashboard_driver" : "thunderdashboard_aux";
};

void PS5ControllerOutput::set_rumble(Controller controller, double leftPct, double rightPct) {
    std::string key = fmt::format("{}_rumble", get_prefix(controller));

    unsigned char left = (unsigned char)(leftPct * 0xFF);
    unsigned char right = (unsigned char)(rightPct * 0xFF);

    PS5RumbleOptions rumbleOptions;

    rumbleOptions.left = left;
    rumbleOptions.right = right;

    int32_t rumbleOptionsInt = 0;
    std::memcpy(&rumbleOptionsInt, &rumbleOptions, sizeof(PS5RumbleOptions));

    frc::SmartDashboard::PutNumber(key, rumbleOptionsInt);
}

void PS5ControllerOutput::set_adaptive_trigger(Controller controller, Trigger trigger, TriggerEffect effect, double start, double end, double force) {
    std::string key = fmt::format("{}_{}_trigger", get_prefix(controller), trigger == Trigger::LEFT ? "left" : "right");

    int32_t triggerOptionsInt = frc::SmartDashboard::GetNumber(key, 0.0);

    PS5TriggerOptions triggerOptions;
    std::memcpy(&triggerOptions, &triggerOptionsInt, sizeof(PS5TriggerOptions));

    triggerOptions.start_position = (unsigned char)(start * 0xFF);
    triggerOptions.end_position   = (unsigned char)(end * 0xFF);
    triggerOptions.force          = (unsigned char)(force * 0xFF);
    triggerOptions.trigger_effect = (uint64_t)effect;

    std::memcpy(&triggerOptionsInt, &triggerOptions, sizeof(PS5TriggerOptions));
    frc::SmartDashboard::PutNumber(key, triggerOptionsInt);
}

void PS5ControllerOutput::set_lightbar_color(Controller controller, unsigned char r, unsigned char g, unsigned char b) {
    std::string key = fmt::format("{}_leds", get_prefix(controller));

    int32_t ledOptionsInt = frc::SmartDashboard::GetNumber(key, 0.0);

    PS5LEDOptions ledOptions;
    std::memcpy(&ledOptions, &ledOptionsInt, sizeof(PS5LEDOptions));

    ledOptions.lightbar_r = r;
    ledOptions.lightbar_g = g;
    ledOptions.lightbar_b = b;

    std::memcpy(&ledOptionsInt, &ledOptions, sizeof(PS5LEDOptions));
    frc::SmartDashboard::PutNumber(key, ledOptionsInt);
}

void PS5ControllerOutput::set_microphone_led(Controller controller, MicrophoneLedState state) {
    std::string key = fmt::format("{}_leds", get_prefix(controller));

    int32_t ledOptionsInt = frc::SmartDashboard::GetNumber(key, 0.0);

    PS5LEDOptions ledOptions;
    std::memcpy(&ledOptions, &ledOptionsInt, sizeof(PS5LEDOptions));

    ledOptions.mic = (uint64_t)state;

    std::memcpy(&ledOptionsInt, &ledOptions, sizeof(PS5LEDOptions));
    frc::SmartDashboard::PutNumber(key, ledOptionsInt);
}

void PS5ControllerOutput::set_player_led(Controller controller, unsigned char bitmask, bool fade) {
    std::string key = fmt::format("{}_leds", get_prefix(controller));

    int32_t ledOptionsInt = frc::SmartDashboard::GetNumber(key, 0.0);

    PS5LEDOptions ledOptions;
    std::memcpy(&ledOptions, &ledOptionsInt, sizeof(PS5LEDOptions));

    ledOptions.player_bitmask = bitmask;
    ledOptions.player_fade = fade;

    std::memcpy(&ledOptionsInt, &ledOptions, sizeof(PS5LEDOptions));
    frc::SmartDashboard::PutNumber(key, ledOptionsInt);
}