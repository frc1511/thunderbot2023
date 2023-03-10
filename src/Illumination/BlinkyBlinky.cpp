#include <Illumination/BlinkyBlinky.h>
#include <frc/smartdashboard/SmartDashboard.h>

BlinkyBlinky::BlinkyBlinky() {
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
        case LEDMode::DISABLED:
            setColor(frc::Color(255, 27, 0));
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
        int hue = lowHue + (offset + (j++ / LED_TOTAL) * (highHue - lowHue)) % (highHue - lowHue);
        setPixel(i, frc::Color::FromHSV(hue, 255, 128));
    }
}

void BlinkyBlinky::rainbow() {
    interpolateHue(0, 180, rainbowOffset);
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
        case LEDMode::DISABLED:
            modeString = "disabled";
            break;
        case LEDMode::CUSTOM:
            modeString = "custom";
            break;
    }


    frc::SmartDashboard::PutString("BlinkyBlinky_Mode", modeString);
    frc::SmartDashboard::PutNumber("BlinkyBlinky_RainbowOffset", rainbowOffset);
}

