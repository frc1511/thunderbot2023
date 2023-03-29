#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <array>
#include <map>

#define LED_TOTAL 60
#define LED_STRIP 30

#define KITT_TIME 0.8_s
#define KITT_LOOPS (KITT_TIME / 20_ms)

#define FIRE_TIME 0.3_s
#define FIRE_MAX_LOOPS (FIRE_TIME / 20_ms)

class WhooshWhoosh;

class BlinkyBlinky : public Mechanism {
public:
    BlinkyBlinky(WhooshWhoosh* whooshWhoosh);
    ~BlinkyBlinky();

    void sendFeedback() override;
    void resetToMode(MatchMode mode) override;
    void process() override;

    enum class LEDMode {
        OFF,
        RAINBOW,
        ALLIANCE,
        CUBE,
        CONE,
        HAS_GAMEPIECE,
        FIRE,
        CRATER_MODE,
        CALIBRATING,
        KNIGHT_RIDER,
        HOME_DEPOT,
        BALANCING,
        CUSTOM,
    };

    void setLEDMode(LEDMode mode);

    void setCustomColor(frc::Color color);

    void playScoreAnimation();

private:
    frc::AddressableLED strip { HardwareManager::IOMap::PWM_BLINKY_BLINKY };

    WhooshWhoosh* whooshWhoosh;

    std::array<frc::AddressableLED::LEDData, LED_TOTAL> stripBuffer;

    enum class Strip : std::size_t {
        LEFT = 0,
        RIGHT = LED_STRIP,
    };

    void setPixel(std::size_t index, frc::Color color);
    // Sets a pixel at index to color and mirrors it on the second strip.
    void setMirroredPixel(std::size_t index, frc::Color color);
    void setColor(frc::Color color);
    void setStrip(Strip strip, frc::Color color);

    void interpolateHue(int lowHue, int highHue, int offset);

    void rainbow();
    void balancing();
    void kitt();
    void fire();

    LEDMode ledMode = LEDMode::ALLIANCE;

    int rainbowOffset = 0;
    int kittIter = 0;
    int kittDir = 1;
    int fireIter = 0;
    int fireDir = 1;
    int fireLoops = 20;
    int fireRange = (LED_STRIP - 1) * 0.3;

    frc::Color customColor;

    bool scoreAnimation = false;
    frc::Timer scoreAnimationTimer;
};
