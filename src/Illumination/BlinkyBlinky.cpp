#include <Illumination/BlinkyBlinky.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <WhooshWhoosh/WhooshWhoosh.h>
#include <random>

BlinkyBlinky::BlinkyBlinky(WhooshWhoosh* _whooshWhoosh)
: whooshWhoosh(_whooshWhoosh) {
    strip.SetLength(LED_TOTAL);
    strip.SetData(stripBuffer);
    strip.Start();
    
	srand((unsigned)time(nullptr));
}

BlinkyBlinky::~BlinkyBlinky() = default;

void BlinkyBlinky::resetToMode(MatchMode mode) {

}

void BlinkyBlinky::process() {
    if (scoreAnimation) {
        double percent = scoreAnimationTimer.Get() / 0.25_s;
        if (percent >= 1.0) {
            scoreAnimation = false;
            scoreAnimationTimer.Stop();
        }
        else {
            setColor(frc::Color::kRed);
            int end = static_cast<int>(percent * LED_STRIP);
            int start = std::clamp(end - 10.0, 0.0, (double)LED_STRIP);

            for (int i = start; i < end; i++) {
                setMirroredPixel(i, frc::Color::kYellow);
            }
        }
    }

    if (!scoreAnimation) {
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
            case LEDMode::FIRE:
                fire();
                break;
            case LEDMode::ALLIANCE:
                setColor(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? frc::Color::kBlue : frc::Color::kRed);
                break;
            case LEDMode::CONE:
                setColor(frc::Color(255, 100, 0));
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
            case LEDMode::KNIGHT_RIDER:
                kitt();
                break;
            case LEDMode::BALANCING:
                balancing();
                break;
            case LEDMode::CUSTOM:
                setColor(customColor);
                break;
        }
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

void BlinkyBlinky::playScoreAnimation() {
    scoreAnimation = true;
    scoreAnimationTimer.Reset();
    scoreAnimationTimer.Start();
}

void BlinkyBlinky::setPixel(std::size_t index, frc::Color color) {
    stripBuffer.at(index).SetLED(color);
}

void BlinkyBlinky::setMirroredPixel(std::size_t index, frc::Color color) {
    setPixel(index, color);
    setPixel(LED_TOTAL - 1 - index, color);
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
    setColor(frc::Color::FromHSV(tiltPct * 45, 255, 128));
}

void BlinkyBlinky::kitt() {
    kittIter += kittDir;

    // Reverse direction when reaches end.
    if (kittIter >= KITT_LOOPS || kittIter <= 0) {
        kittDir = -kittDir;
    }

    double percent = kittIter / KITT_LOOPS;

    int pixel = static_cast<int>(percent * (LED_STRIP - 1));

    double fadeRange = LED_STRIP * 0.3;

    setColor(frc::Color::kBlack);

    // Fade up.
    for (int i = pixel; i <= (pixel + fadeRange > LED_STRIP - 1 ? LED_STRIP - 1 : pixel + fadeRange); i++) {
        double percent = static_cast<double>(i - pixel) / fadeRange;

        double value = (1 / percent) * 128;

        setMirroredPixel(i, frc::Color::FromHSV(percent * 5, value * 0.5 + 192, value));
    }
    // Fade down.
    for (int i = pixel; i >= (pixel - fadeRange < 0 ? 0 : pixel - fadeRange); i--) {
        double percent = static_cast<double>(pixel - i) / fadeRange;

        double value = (1 / percent) * 128;

        setMirroredPixel(i, frc::Color::FromHSV(percent * 5, value * 0.5 + 192, value));
    }

    // Middle.
    setMirroredPixel(pixel, frc::Color::kRed);
}

void BlinkyBlinky::fire() {
    fireIter += fireDir;

    if (fireIter >= fireLoops || fireIter <= 0) {
        fireDir = -fireDir;
        fireLoops = FIRE_MAX_LOOPS * ((60 + (static_cast<double>(rand() % 40))) / 100.0);
        fireRange = (LED_STRIP - 1) * ((50 + (static_cast<double>(rand() % 51))) / 100.0);
        if (fireDir == 1) {
            fireIter = fireLoops;
        }
        else {
            fireIter = 0;
        }
        fmt::print("fireRange: {}, fireLoops: {}\n", fireRange, fireLoops);
    }

    double percent = fireIter / fireLoops;
    percent = std::clamp(percent, 0.0, 1.0);

    // int pixel = static_cast<int>(percent * fireRange);
    int pixel = fireRange;

    for (int i = 0; i < pixel; i++) {
        setMirroredPixel(i, frc::Color::FromHSV((static_cast<double>(i) / (LED_STRIP - 1)) * 5.0, 255 /*(1.0 / (static_cast<double>(i) / pixel)) * 255*/, 128));
    }
    for (int i = pixel; i < LED_STRIP; i++) {
        setMirroredPixel(i, frc::Color::kBlack);
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
        case LEDMode::HAS_GAMEPIECE:
            modeString = "has gamepiece";
            break;
        case LEDMode::FIRE:
            modeString = "fire";
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
        case LEDMode::KNIGHT_RIDER:
            modeString = "knight rider";
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

