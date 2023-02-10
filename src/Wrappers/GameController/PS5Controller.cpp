#include <Wrappers/GameController/PS5Controller.h>
#include <Hardware/HardwareManager.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cerrno>
#include <fmt/core.h>
#include <cstdint>
#include <map>
#include <utility>

enum class ButtonInterface {
    BUTTONS_AND_DPAD,
    BUTTONS_A,
    BUTTONS_B,
};

std::map<ThunderPS5Controller::Button, std::pair<ButtonInterface, int>> buttonMappings {
    { ThunderPS5Controller::Button::TRIANGLE,     std::make_pair(ButtonInterface::BUTTONS_AND_DPAD, 0x80) },
    { ThunderPS5Controller::Button::CIRCLE,       std::make_pair(ButtonInterface::BUTTONS_AND_DPAD, 0x40) },
    { ThunderPS5Controller::Button::CROSS,        std::make_pair(ButtonInterface::BUTTONS_AND_DPAD, 0x20) },
    { ThunderPS5Controller::Button::SQUARE,       std::make_pair(ButtonInterface::BUTTONS_AND_DPAD, 0x10) },
    { ThunderPS5Controller::Button::LEFT_BUMPER,  std::make_pair(ButtonInterface::BUTTONS_A, 0x01) },
    { ThunderPS5Controller::Button::RIGHT_BUMPER, std::make_pair(ButtonInterface::BUTTONS_A, 0x02) },
    { ThunderPS5Controller::Button::SHARE,        std::make_pair(ButtonInterface::BUTTONS_A, 0x10) },
    { ThunderPS5Controller::Button::OPTIONS,      std::make_pair(ButtonInterface::BUTTONS_A, 0x20) },
    { ThunderPS5Controller::Button::LEFT_STICK,   std::make_pair(ButtonInterface::BUTTONS_A, 0x40) },
    { ThunderPS5Controller::Button::RIGHT_STICK,  std::make_pair(ButtonInterface::BUTTONS_A, 0x80) },
};

std::map<ThunderGameController::DPad, int> dpadMappings {
    { ThunderGameController::DPad::UP,    0x08 },
    { ThunderGameController::DPad::RIGHT, 0x04 },
    { ThunderGameController::DPad::DOWN,  0x02 },
    { ThunderGameController::DPad::LEFT,  0x01 },
};

ThunderPS5Controller::ThunderPS5Controller(Controller controller)
: ThunderGameController(controller), serverThread([this] { this->threadMain(); }) { }

ThunderPS5Controller::~ThunderPS5Controller() { }

bool ThunderPS5Controller::getButton(int _button, ButtonState state) {
    if (_button < 1 || _button > 10) {
        return false;
    }
    Button button = static_cast<Button>(_button);
    if (!buttonMappings.count(button)) {
        return false;
    }

    auto [interface, bit] = buttonMappings[button];

    unsigned char currentButtonsBitmask = 0;
    unsigned char oldButtonsBitmask = 0;
    {
        std::lock_guard<std::mutex> lk(inputMutex);
        switch (interface) {
            case ButtonInterface::BUTTONS_AND_DPAD:
                currentButtonsBitmask = inputState.buttonsAndDPad;
                oldButtonsBitmask = lastInputState.buttonsAndDPad;
                break;
            case ButtonInterface::BUTTONS_A:
                currentButtonsBitmask = inputState.buttonsA;
                oldButtonsBitmask = lastInputState.buttonsA;
                break;
            case ButtonInterface::BUTTONS_B:
                currentButtonsBitmask = inputState.buttonsB;
                oldButtonsBitmask = lastInputState.buttonsB;
                break;
        }
    }

    bool buttonPressed = currentButtonsBitmask & bit;
    bool buttonWasPressed = oldButtonsBitmask & bit;

    switch (state) {
        case ButtonState::PRESSED:
            return buttonPressed && !buttonWasPressed;
        case ButtonState::HELD:
            return buttonPressed;
        case ButtonState::RELEASED:
            return !buttonPressed && buttonWasPressed;
    }
    return false;
}

double ThunderPS5Controller::getAxis(int _axis) {
    if (_axis < 1 || _axis > 6) {
        return 0.0;
    }

    Axis axis = static_cast<Axis>(_axis);

    std::lock_guard<std::mutex> lk(inputMutex);

    switch (axis) {
        case LEFT_X:
            return static_cast<double>(inputState.axisLeftX) / 0xFF - 0x80;
        case LEFT_Y:
            return static_cast<double>(inputState.axisLeftY) / 0xFF - 0x80;
        case RIGHT_X:
            return static_cast<double>(inputState.axisRightX) / 0xFF - 0x80;
        case RIGHT_Y:
            return static_cast<double>(inputState.axisRightY) / 0xFF - 0x80;
        case LEFT_TRIGGER:
            return static_cast<double>(inputState.axisLeftTrigger) / 0xFF;
        case RIGHT_TRIGGER:
            return static_cast<double>(inputState.axisRightTrigger) / 0xFF;
    }

    return 0.0;
}

ThunderGameController::DPad ThunderPS5Controller::getDPad() {
    unsigned char bitmask = 0;
    {
        std::lock_guard<std::mutex> lk(inputMutex);
        bitmask = inputState.buttonsAndDPad;
    }

    bool up    = bitmask & dpadMappings[DPad::UP];
    bool right = bitmask & dpadMappings[DPad::RIGHT];
    bool down  = bitmask & dpadMappings[DPad::DOWN];
    bool left  = bitmask & dpadMappings[DPad::LEFT];

    if (up) {
        if (right) {
            return DPad::UP_RIGHT;
        } else if (left) {
            return DPad::UP_LEFT;
        }
        return DPad::UP;
    }
    else if (down) {
        if (right) {
            return DPad::DOWN_RIGHT;
        } else if (left) {
            return DPad::DOWN_LEFT;
        }
        return DPad::DOWN;
    }
    else if (right) {
        return DPad::RIGHT;
    }
    else if (left) {
        return DPad::LEFT;
    }
    return DPad::NONE;
}

void ThunderPS5Controller::setRumble(double leftPct, double rightPct) {
    unsigned char left  = static_cast<unsigned char>(leftPct * 0xFF);
    unsigned char right = static_cast<unsigned char>(rightPct * 0xFF);

    std::lock_guard<std::mutex> lk(outputMutex);

    outputState.rumbleLeft = left;
    outputState.rumbleRight = right;
    newOutput = true;
}

void ThunderPS5Controller::setAdaptiveTrigger(Trigger trigger, TriggerEffect effect, double startPct, double endPct, double forcePct) {
    unsigned char start = static_cast<unsigned char>(startPct * 0xFF);
    unsigned char end   = static_cast<unsigned char>(endPct * 0xFF);
    unsigned char force = static_cast<unsigned char>(forcePct * 0xFF);

    std::lock_guard<std::mutex> lk(outputMutex);

    if (trigger == Trigger::LEFT) {
        outputState.leftTriggerEffect = static_cast<unsigned char>(effect);
        outputState.leftTriggerStartPosition = start;
        outputState.leftTriggerEndPosition = end;
        outputState.leftTriggerForce = force;
    }
    else {
        outputState.rightTriggerEffect = static_cast<unsigned char>(effect);
        outputState.rightTriggerStartPosition = start;
        outputState.rightTriggerEndPosition = end;
        outputState.rightTriggerForce = force;
    }

    newOutput = true;
}

void ThunderPS5Controller::setLightbarColor(unsigned char r, unsigned char g, unsigned char b) {
    std::lock_guard<std::mutex> lk(outputMutex);

    outputState.lightbarR = r;
    outputState.lightbarG = g;
    outputState.lightbarB = b;

    newOutput = true;
}

void ThunderPS5Controller::setMicrophoneLed(MicrophoneLedState state) {
    std::lock_guard<std::mutex> lk(outputMutex);

    outputState.micLed = static_cast<unsigned char>(state);

    newOutput = true;
}

void ThunderPS5Controller::setPlayerLed(unsigned char bitmask, bool fade) {
    std::lock_guard<std::mutex> lk(outputMutex);

    outputState.playerLedBitmask = bitmask;
    outputState.playerLedFade = fade;

    newOutput = true;
}

void ThunderPS5Controller::threadMain() {
    using namespace std::chrono_literals;

SOCKET_CREATE:
    // Create server socket.
    int serverFD = socket(AF_INET, SOCK_STREAM, 0);
    if (!serverFD) {
        fmt::print("BIG PROBLEM: Failed to create socket: {}\n", strerror(errno));
        std::this_thread::sleep_for(1s);
        goto SOCKET_CREATE;
    }

    // Setup non-blocking I/O for the server socket.
    int flags = fcntl(serverFD, F_GETFL, 0);
    fcntl(serverFD, F_SETFL, flags | O_NONBLOCK);

    // hi jeff!

    // Create IP socket address.
    struct sockaddr_in server_addr { AF_INET, htons(controller == Controller::DRIVER ? HardwareManager::IOMap::NET_PS5_DRIVER : HardwareManager::IOMap::NET_PS5_AUX), INADDR_ANY, {} };
    // Set address zero padding.
    std::memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));

SOCKET_BIND:
    // Bind the address to the socket.
    if (bind(serverFD, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        fmt::print("BIG PROBLEM: Failed to bind socket to port {}: {}\n", (int)(controller == Controller::DRIVER ? HardwareManager::IOMap::NET_PS5_DRIVER : HardwareManager::IOMap::NET_PS5_AUX), strerror(errno));
        std::this_thread::sleep_for(0.1s);
        goto SOCKET_BIND;
    }

SOCKET_LISTEN:
    // Listen for connections (backlog of 1 b/c we only need one connection).
    if (listen(serverFD, 1) < 0) {
        fmt::print("BIG PROBLEM: Failed to listen on socket: {}\n", strerror(errno));
        std::this_thread::sleep_for(0.1s);
        goto SOCKET_LISTEN;
    }

    char inputBuf[sizeof(InputState)];
    char outputBuf[sizeof(OutputState)];

    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(struct sockaddr_in);

RETRY_CONNECTION:
    // Accept connection.
    int clientFD = accept(serverFD, (struct sockaddr*)&client_addr, &client_addr_len);
    if (clientFD < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No incoming connections.
        }
        else {
            fmt::print("Accept failed: {}\n", strerror(errno));
        }
        goto RETRY_CONNECTION;
    }

    // Setup non-blocking I/O for the client socket.
    flags = fcntl(clientFD, F_GETFL, 0);
    fcntl(clientFD, F_SETFL, flags | O_NONBLOCK);

    for (;;) {
        // Handle input.
        ssize_t inputLen = read(clientFD, inputBuf, sizeof(inputBuf));
        if (inputLen <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || !inputLen) {
                // No incoming data.
            }
            else if (errno == EBADF || errno == ECONNRESET) {
                fmt::print("Client disconnected. Attempting to reconnect...\n");
                close(clientFD);
                goto RETRY_CONNECTION;
            }
            else {
                fmt::print("read() failed: {}\n", strerror(errno));
                close(clientFD);
                goto RETRY_CONNECTION;
            }
        }

        if (inputLen) {
            InputState newInputState;
            std::memcpy(&newInputState, inputBuf, sizeof(newInputState));

            {
                std::lock_guard<std::mutex> lk(inputMutex);
                lastInputState = inputState;
                inputState = newInputState;
            }
        }

        OutputState newOutputState;
        bool shouldOutput;

        {
            std::lock_guard<std::mutex> lk(outputMutex);
            shouldOutput = newOutput;
            newOutputState = outputState;
        }

        if (shouldOutput) {
            std::memcpy(outputBuf, &newOutputState, sizeof(newOutputState));

WRITE_OUTPUT:
            // Handle output.
            ssize_t outputLen = write(clientFD, outputBuf, sizeof(outputBuf));
            if (outputLen < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // Needs some time to write.
                    goto WRITE_OUTPUT;
                }
                else {
                    fmt::print("write() failed: {}\n", strerror(errno));
                    close(clientFD);
                    goto RETRY_CONNECTION;
                }
            }
        }
    }
}
