#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <array>
#include <map>

#define LED_TOTAL 60

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
        CRATER_MODE,
        CALIBRATING,
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
        RIGHT = 30,
    };

    void setPixel(std::size_t index, frc::Color color);
    void setColor(frc::Color color);
    void setStrip(Strip strip, frc::Color color);

    void interpolateHue(int lowHue, int highHue, int offset);

    void rainbow();
    void balancing();

    LEDMode ledMode = LEDMode::ALLIANCE;

    int rainbowOffset = 0;

    frc::Color customColor;

    bool scoreAnimation = false;
    frc::Timer scoreAnimationTimer;
};
