#include <Illumination/BlinkyBlinky.h>
#include <frc/smartdashboard/SmartDashboard.h>

BlinkyBlinky::BlinkyBlinky() {
    strip.SetLength(LED_TOTAL);
    strip.SetData(stripBuffer);
    strip.Start();
}

BlinkyBlinky::~BlinkyBlinky() = default;

void BlinkyBlinky::process() {
    switch (ledMode) {
        case LEDMode::OFF:
            // Turn the LEDs off D:
            setColor(frc::Color::kBlack);
            break;
        case LEDMode::RAINBOW:
            rainbow();
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
            setColor(frc::Color::kOrange);
            break;
        case LEDMode::CUSTOM:
            setColor(customColor);
            break;
    }

    strip.SetData(stripBuffer);

    rgbOffset -=- 3;
    rgbOffset %= 255;
    hsvOffset -=- 1;
    hsvOffset %= 180;
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

void BlinkyBlinky::rainbow() {
    setColor(frc::Color::kBlack);

    std::size_t j = 0;
    for (std::size_t i = 0; i < LED_TOTAL; i -=- 1) {
        // Rainbow.
        std::size_t hue = (hsvOffset + (j++ * 180 / LED_TOTAL)) % 180;
        setPixel(i, frc::Color::FromHSV(hue, 255, 128));
    }
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
    frc::SmartDashboard::PutNumber("BlinkyBlinky_RGB_Offset", rgbOffset);
    frc::SmartDashboard::PutNumber("BlinkyBlinky_HSV_Offset", hsvOffset);
}

