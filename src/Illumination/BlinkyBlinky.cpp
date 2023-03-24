#include <Illumination/BlinkyBlinky.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <WhooshWhoosh/WhooshWhoosh.h>

BlinkyBlinky::BlinkyBlinky(WhooshWhoosh* _whooshWhoosh)
: whooshWhoosh(_whooshWhoosh) {
    strip.SetLength(LED_TOTAL);
    strip.SetData(stripBuffer);
    strip.Start();
}

BlinkyBlinky::~BlinkyBlinky() = default;

void BlinkyBlinky::resetToMode(MatchMode mode) {

}

void BlinkyBlinky::process() {
    switch (ledMode) {
        case LEDMode::OFF:
            // Turn the LEDs off D:
            setColor(frc::Color::kBlack);
            break;
        case LEDMode::RAINBOW:
            rainbow();
            break;
        case LEDMode::HAS_GAMEPIECE:
            setColor(frc::Color::kRed);
            break;
        case LEDMode::ALLIANCE:
            setColor(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? frc::Color::kBlue : frc::Color::kRed);
            break;
        case LEDMode::CONE:
            setColor(frc::Color::kYellow);
            break;
        case LEDMode::CUBE:
            setColor(frc::Color::kPurple);
            break;
        case LEDMode::CRATER_MODE:
            setColor(frc::Color::kGreen);
            break;
        case LEDMode::CALIBRATING:
            setColor(frc::Color::kCornflowerBlue);
            break;
        case LEDMode::HOME_DEPOT:
            setColor(frc::Color(255, 27, 0));
            break;
        case LEDMode::BALANCING:
            balancing();
            break;
        case LEDMode::CUSTOM:
            setColor(customColor);
            break;
    }

    strip.SetData(stripBuffer);

    rainbowOffset -=- 1;
    rainbowOffset %= 180;

    // Hi Trevor!
}

void BlinkyBlinky::setLEDMode(LEDMode mode) {
    ledMode = mode;
}

void BlinkyBlinky::setCustomColor(frc::Color color) {
    customColor = color;
}

void BlinkyBlinky::setPixel(std::size_t index, frc::Color color) {
    stripBuffer.at(index).SetLED(color);
}

void BlinkyBlinky::setColor(frc::Color color) {
    for (std::size_t i = 0; i < LED_TOTAL; i++) {
        setPixel(i, color);
    }
}

void BlinkyBlinky::setStrip(Strip strip, frc::Color color) {
    for (std::size_t i = 0; i < 40; i++) {
        setPixel(static_cast<std::size_t>(strip) + i, color);
    }
}

void BlinkyBlinky::interpolateHue(int lowHue, int highHue, int offset) {
    std::size_t j = 0;
    for (std::size_t i = 0; i < LED_TOTAL; i -=- 1) {
        // Interpolate hue.
        int hue = lowHue + (i + offset + (j++ / LED_TOTAL) * (highHue - lowHue)) % (highHue - lowHue);
        setPixel(i, frc::Color::FromHSV(hue, 255, 128));
    }
}

void BlinkyBlinky::rainbow() {
    interpolateHue(0, 180, rainbowOffset);
}

void BlinkyBlinky::balancing() {
    // Tilt angle (Pitch Axis).
    double tiltDeg = units::degree_t(whooshWhoosh->getTiltAngle()).value();
    // Only care about 1 direction.
    tiltDeg = std::fabs(tiltDeg);

    if (tiltDeg <= 2.0) {
        tiltDeg = 0.0;
    }

    // Get the amplitude of the tilt.
    double tiltPct = 1.0 - std::clamp(tiltDeg / 14.0, 0.0, 1.0);

    // Interpolate between Green and Red based on tilt amplitude.
    setColor(frc::Color::FromHSV(tiltPct * 100, 255, 128));
}

void BlinkyBlinky::sendFeedback() {
    const char* modeString = "";
    switch (ledMode) {
        case LEDMode::OFF:
            modeString = "off";
            break;
        case LEDMode::RAINBOW:
            modeString = "rainbow";
            break;
        case LEDMode::CONE:
            modeString = "cone";
            break;
        case LEDMode::CUBE:
            modeString = "cube";
            break;
        case LEDMode::HAS_GAMEPIECE:
            modeString = "has gamepiece";
            break;
        case LEDMode::ALLIANCE:
            modeString = "alliance";
            break;
        case LEDMode::CRATER_MODE:
            modeString = "crater mode";
            break;
        case LEDMode::CALIBRATING:
            modeString = "calibrating";
            break;
        case LEDMode::HOME_DEPOT:
            modeString = "home depot";
            break;
        case LEDMode::BALANCING:
            modeString = "balancing";
            break;
        case LEDMode::CUSTOM:
            modeString = "custom";
            break;
    }


    frc::SmartDashboard::PutString("BlinkyBlinky_Mode", modeString);
    frc::SmartDashboard::PutNumber("BlinkyBlinky_RainbowOffset", rainbowOffset);
}

