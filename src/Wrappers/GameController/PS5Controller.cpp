#include <Wrappers/GameController/PS5Controller.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cerrno>
#include <fmt/core.h>

ThunderPS5Controller::ThunderPS5Controller(Controller controller)
: ThunderGameController(controller) {

}

ThunderPS5Controller::~ThunderPS5Controller() {

}

bool ThunderPS5Controller::getButton(int button, ButtonState state) {

}

double ThunderPS5Controller::getAxis(int axis) {

}

ThunderGameController::DPad ThunderPS5Controller::getDPad() {

}

void ThunderPS5Controller::setRumble(double left, double right) {

}

void ThunderPS5Controller::setAdaptiveTrigger(Trigger trigger, TriggerEffect effect, double start, double end, double force) {

}

void ThunderPS5Controller::setLightbarColor(unsigned char r, unsigned char g, unsigned char b) {

}

void ThunderPS5Controller::setMicrophoneLed(MicrophoneLedState state) {

}

void ThunderPS5Controller::setPlayerLed(unsigned char bitmask, bool fade) {

}

void ThunderPS5Controller::threadMain() {
    // Thread stuff.
}
