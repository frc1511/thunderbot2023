#include <Hardware/GameController/PS5Controller.h>
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

extern "C" {
    struct InputState {
        uint32_t axisLeftX : 8;
        uint32_t axisLeftY : 8;
        uint32_t axisRightX : 8;
        uint32_t axisRightY : 8;
        uint32_t axisLeftTrigger : 8;
        uint32_t axisRightTrigger : 8;
        uint32_t buttonsAndDPad : 8;
        uint32_t buttonsA : 8;
        uint32_t accelX : 16;
        uint32_t accelY : 16;
        uint32_t accelZ : 16;
        uint32_t gyroX : 16;
        uint32_t gyroY : 16;
        uint32_t gyroZ : 16;
        uint32_t buttonsB : 8;
        uint32_t pad : 24;
    }; // 24 bytes
    typedef struct InputState InputState;

    struct OutputState {
        uint32_t rumbleLeft : 8;
        uint32_t rumbleRight : 8;
        uint32_t micLed : 2; // (0 = off, 1 = on, 2 = pulse)
        uint32_t playerLedFade : 1;
        uint32_t playerLedBitmask : 5; // (0x01 = left, 0x02 = middle left, 0x04 = middle, 0x08 = middle right, 0x10 = right)
        uint32_t lightbarR : 8;
        uint32_t lightbarG : 8;
        uint32_t lightbarB : 8;
        uint32_t leftTriggerStartPosition : 8;
        uint32_t leftTriggerEndPosition : 8;
        uint32_t leftTriggerForce : 8;
        uint32_t rightTriggerStartPosition : 8;
        uint32_t rightTriggerEndPosition : 8;
        uint32_t rightTriggerForce : 8;
        uint32_t leftTriggerEffect : 2; // (0 = off, 1 = section, 2 = continuous)
        uint32_t rightTriggerEffect : 2; // (0 = off, 1 = section, 2 = continuous)
        uint32_t pad : 28;
    }; // 16 bytes
    typedef struct OutputState OutputState;

    struct InputMessage {
        InputState driverInputState;
        InputState auxInputState;
    }; // 48 bytes
    typedef struct InputMessage InputMessage;

    struct OutputMessage {
        OutputState driverOutputState;
        OutputState auxOutputState;
    }; // 32 bytes
} // extern "C"

class PS5ControllerNetworking {
public:
    static inline PS5ControllerNetworking& getInstance() { return instance; }

    /**
     * Initializes the networking thread.
     */
    void init() {
        if (hasInit) return;

        std::memset(&driverInputState, 0, sizeof(InputState));
        std::memset(&auxInputState, 0, sizeof(InputState));
        lastDriverInputState = driverInputState;
        lastAuxInputState = auxInputState;

        networkingThread = std::thread([this]() { this->threadMain(); });

        hasInit = true;
    }

    /**
     * Returns the current input state for a controller.
     */
    InputState getInputState(ThunderGameController::Controller controller) {
        std::lock_guard<std::mutex> lock(inputMutex);
        return controller == ThunderGameController::Controller::DRIVER ?
               driverInputState : auxInputState;
    }

    /**
     * Returns the last input state for a controller.
     */
    InputState getLastInputState(ThunderGameController::Controller controller) {
        std::lock_guard<std::mutex> lock(inputMutex);
        return controller == ThunderGameController::Controller::DRIVER ?
               lastDriverInputState : lastAuxInputState;
    }

    void setOutputState(ThunderGameController::Controller controller, OutputState state) {
        std::lock_guard<std::mutex> lock(outputMutex);
        if (controller == ThunderGameController::Controller::DRIVER) {
            driverOutputState = state;
        }
        else {
            auxOutputState = state;
        }
        newOutputState = true;
    }

private:
    InputState driverInputState;
    InputState auxInputState;
    InputState lastDriverInputState;
    InputState lastAuxInputState;

    bool newOutputState = false;
    OutputState driverOutputState;
    OutputState auxOutputState;

    bool hasInit = false;

    std::thread networkingThread;
    std::mutex inputMutex;
    std::mutex outputMutex;

    void threadMain() {
        using namespace std::chrono_literals;

        // --- Create the server socket. ---

SOCKET_CREATE:
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            fmt::print("PS5Controller: socket() failed: {}\n", strerror(errno));
            std::this_thread::sleep_for(1s);
            goto SOCKET_CREATE;
        }

        fmt::print("PS5Controller: Created socket.\n");

        // Setup non-blocking I/O for the server socket.
        int flags = fcntl(server_fd, F_GETFL, 0);
        fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

        // hi jeff!!

        // Server address.
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(HardwareManager::IOMap::TCP_PS5_CONTROL);
        std::memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));

        // --- Bind the server socket to the port. ---

SOCKET_BIND:
        if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            fmt::print("PS5Controller: bind() failed: {}\n", strerror(errno));
            std::this_thread::sleep_for(1s);
            goto SOCKET_BIND;
        }

        fmt::print("PS5Controller: Bound socket to port {}\n", (int)HardwareManager::IOMap::TCP_PS5_CONTROL);

        // --- Listen for incoming connections. ---

SOCKET_LISTEN:
        if (listen(server_fd, 1) < 0) {
            fmt::print("PS5Controller: listen() failed: {}\n", strerror(errno));
            goto SOCKET_LISTEN;
        }

        fmt::print("PS5Controller: Listening on port {}\n", (int)HardwareManager::IOMap::TCP_PS5_CONTROL);

        // --- Accept incoming connections. ---

        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
SOCKET_CONNECT:
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_len);
        if (client_fd < 0) {
            if (errno != EWOULDBLOCK && errno != EAGAIN) {
                fmt::print("PS5Controller: accept() failed: {}\n", strerror(errno));
                goto SOCKET_CONNECT;
            }
        }

        fmt::print("PS5Controller: Accepted connection from {}\n", inet_ntoa(client_addr.sin_addr));

        // --- Read/Write stuff. ---

        char input_buf[256];
        char output_buf[256];
        while (true) {
            // --- Read input. ---

            ssize_t bytes_read = read(client_fd, input_buf, sizeof(input_buf));
            if (bytes_read < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    // No incoming data D:
                }
                else if (errno == ECONNRESET) {
                    fmt::print("PS5Controller: Connection reset by peer.\n");
                    close(client_fd);
                    goto SOCKET_CONNECT;
                }
                else {
                    fmt::print("PS5Controller: read() failed: {}\n", strerror(errno));
                }
            }   
            else if (bytes_read) {
                // We got some data!
                fmt::print("PS5Controller: Read {} bytes.\n", bytes_read);

                InputMessage msg;
                std::memcpy(&msg, input_buf, sizeof(msg));

                std::lock_guard<std::mutex> lock(inputMutex);
                lastDriverInputState = driverInputState;
                lastAuxInputState = auxInputState;
                driverInputState = msg.driverInputState;
                auxInputState = msg.auxInputState;
            }

            // --- Write output. ---

            bool _newOutputState = false;
            {
                std::lock_guard<std::mutex> lock(outputMutex);
                _newOutputState = newOutputState;
            }

            if (_newOutputState) {
                OutputMessage msg;
                {
                    std::lock_guard<std::mutex> lock(outputMutex);
                    newOutputState = false;
                    msg.driverOutputState = driverOutputState;
                    msg.auxOutputState = auxOutputState;
                }

                std::memcpy(output_buf, &msg, sizeof(msg));
                ssize_t bytes_written = write(client_fd, output_buf, sizeof(output_buf));
                if (bytes_written < 0) {
                    if (errno == ECONNRESET) {
                        fmt::print("PS5Controller: Connection reset by peer.\n");
                        close(client_fd);
                        goto SOCKET_CONNECT;
                    }
                    else {
                        fmt::print("PS5Controller: write() failed: {}\n", strerror(errno));
                    }
                }
                else if (bytes_written) {
                    fmt::print("PS5Controller: Wrote {} bytes.\n", bytes_written);
                }
            }
        }
    }

    static PS5ControllerNetworking instance;
};

PS5ControllerNetworking PS5ControllerNetworking::instance;

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
: ThunderGameController(controller) {
    PS5ControllerNetworking::getInstance().init();

    std::memset(&inputState, 0, sizeof(inputState));
    std::memset(&lastInputState, 0, sizeof(lastInputState));
}

ThunderPS5Controller::~ThunderPS5Controller() { }

void ThunderPS5Controller::process() {
    InputState input = PS5ControllerNetworking::getInstance().getInputState(controller);
    std::memcpy(&inputState, &input, sizeof(inputState));

    InputState lastInput = PS5ControllerNetworking::getInstance().getLastInputState(controller);
    std::memcpy(&lastInputState, &lastInput, sizeof(lastInputState));

    if (newOutputState) {
        OutputState output;
        std::memcpy(&output, &outputState, sizeof(outputState));
        PS5ControllerNetworking::getInstance().setOutputState(controller, output);
        newOutputState = false;
    }
}

bool ThunderPS5Controller::getButton(int _button, ButtonState state) {
    if (_button < 1 || _button > 10) {
        return false;
    }
    Button button = static_cast<Button>(_button);
    if (!buttonMappings.count(button)) {
        return false;
    }

    InputState input;
    std::memcpy(&input, &inputState, sizeof(inputState));
    InputState lastInput;
    std::memcpy(&lastInput, &lastInputState, sizeof(lastInputState));

    auto [interface, bit] = buttonMappings[button];

    unsigned char currentButtonsBitmask = 0;
    unsigned char oldButtonsBitmask = 0;
    switch (interface) {
        case ButtonInterface::BUTTONS_AND_DPAD:
            currentButtonsBitmask = input.buttonsAndDPad;
            oldButtonsBitmask = lastInput.buttonsAndDPad;
            break;
        case ButtonInterface::BUTTONS_A:
            currentButtonsBitmask = input.buttonsA;
            oldButtonsBitmask = lastInput.buttonsA;
            break;
        case ButtonInterface::BUTTONS_B:
            currentButtonsBitmask = input.buttonsB;
            oldButtonsBitmask = lastInput.buttonsB;
            break;
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

    InputState input;
    std::memcpy(&input, &inputState, sizeof(inputState));

    Axis axis = static_cast<Axis>(_axis);

    switch (axis) {
        case LEFT_X:
            return static_cast<double>(input.axisLeftX) / 0xFF - 0x80;
        case LEFT_Y:
            return static_cast<double>(input.axisLeftY) / 0xFF - 0x80;
        case RIGHT_X:
            return static_cast<double>(input.axisRightX) / 0xFF - 0x80;
        case RIGHT_Y:
            return static_cast<double>(input.axisRightY) / 0xFF - 0x80;
        case LEFT_TRIGGER:
            return static_cast<double>(input.axisLeftTrigger) / 0xFF;
        case RIGHT_TRIGGER:
            return static_cast<double>(input.axisRightTrigger) / 0xFF;
    }

    return 0.0;
}

ThunderGameController::DPad ThunderPS5Controller::getDPad() {
    InputState input;
    std::memcpy(&input, &inputState, sizeof(inputState));

    unsigned char bitmask = input.buttonsAndDPad;

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

    OutputState output;
    std::memcpy(&output, &outputState, sizeof(outputState));

    output.rumbleLeft = left;
    output.rumbleRight = right;

    newOutputState = true;
    std::memcpy(&outputState, &output, sizeof(outputState));
}

void ThunderPS5Controller::setAdaptiveTrigger(Trigger trigger, TriggerEffect effect, double startPct, double endPct, double forcePct) {
    unsigned char start = static_cast<unsigned char>(startPct * 0xFF);
    unsigned char end   = static_cast<unsigned char>(endPct * 0xFF);
    unsigned char force = static_cast<unsigned char>(forcePct * 0xFF);

    OutputState output;
    std::memcpy(&output, &outputState, sizeof(outputState));

    if (trigger == Trigger::LEFT) {
        output.leftTriggerEffect = static_cast<unsigned char>(effect);
        output.leftTriggerStartPosition = start;
        output.leftTriggerEndPosition = end;
        output.leftTriggerForce = force;
    }
    else {
        output.rightTriggerEffect = static_cast<unsigned char>(effect);
        output.rightTriggerStartPosition = start;
        output.rightTriggerEndPosition = end;
        output.rightTriggerForce = force;
    }

    newOutputState = true;
    std::memcpy(&outputState, &output, sizeof(outputState));
}

void ThunderPS5Controller::setLightbarColor(unsigned char r, unsigned char g, unsigned char b) {
    OutputState output;
    std::memcpy(&output, &outputState, sizeof(outputState));

    output.lightbarR = r;
    output.lightbarG = g;
    output.lightbarB = b;

    newOutputState = true;
    std::memcpy(&outputState, &output, sizeof(outputState));
}

void ThunderPS5Controller::setMicrophoneLed(MicrophoneLedState state) {
    OutputState output;
    std::memcpy(&output, &outputState, sizeof(outputState));

    output.micLed = static_cast<unsigned char>(state);

    newOutputState = true;
    std::memcpy(&outputState, &output, sizeof(outputState));
}

void ThunderPS5Controller::setPlayerLed(unsigned char bitmask, bool fade) {
    OutputState output;
    std::memcpy(&output, &outputState, sizeof(outputState));

    output.playerLedBitmask = bitmask;
    output.playerLedFade = fade;

    newOutputState = true;
    std::memcpy(&outputState, &output, sizeof(outputState));
}
