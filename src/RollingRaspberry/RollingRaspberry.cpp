#include <RollingRaspberry/RollingRaspberry.h>
#include <Hardware/HardwareManager.h>
#include <frc/Timer.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cerrno>
#include <fmt/core.h>
#include <cstdint>

#define VISION_MESSAGE_SIZE (sizeof(double) * 16)

RollingRaspberry::RollingRaspberry() {
    initServer();
}

RollingRaspberry::~RollingRaspberry() {
    close(serverFileDesc);
}

void RollingRaspberry::initServer() {
    using namespace std::chrono_literals;

    if (serverRunning) return;

    if (!socketCreated) {
        serverFileDesc = socket(AF_INET, SOCK_STREAM, 0);
        if (serverFileDesc <= 0) {
            fmt::print("RollingRaspberry: socket() failed: {}\n", strerror(errno));
            return;
        }

        socketCreated = true;
        fmt::print("RollingRaspberry: Created socket.\n");

        // Setup non-blocking I/O for the server socket.
        int flags = fcntl(serverFileDesc, F_GETFL, 0);
        fcntl(serverFileDesc, F_SETFL, flags | O_NONBLOCK);
    }

    if (!socketCreated) return;

    if (!socketBound) {
        // Server address.
        struct sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons((int)HardwareManager::IOMap::TCP_ROLLING_RASPBERRY);
        std::memset(serverAddr.sin_zero, 0, sizeof(serverAddr.sin_zero));

        // --- Bind the server socket to the port. ---

        if (bind(serverFileDesc, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            fmt::print("RollingRaspberry: bind() to port {} failed: {}\n", (int)HardwareManager::IOMap::TCP_ROLLING_RASPBERRY, strerror(errno));
            return;
        }

        socketBound = true;
        fmt::print("RollingRaspberry: Bound socket to port {}\n", (int)HardwareManager::IOMap::TCP_ROLLING_RASPBERRY);
    }

    if (!socketBound) return;

    // --- Listen for incoming connections. ---

    if (!socketListening) {
        // Backlog of 3 connections.
        if (listen(serverFileDesc, 3) < 0) {
            fmt::print("RollingRaspberry: listen() failed: {}\n", strerror(errno));
            return;
        }

        socketListening = true;
        fmt::print("RollingRaspberry: Listening on port {}\n", (int)HardwareManager::IOMap::TCP_ROLLING_RASPBERRY);
    }

    if (!socketListening) return;

    if (socketCreated && socketBound && socketListening) {
        serverRunning = true;
        fmt::print("RollingRaspberry: Server running.\n");
    }
}

void RollingRaspberry::process() {
    if (!serverRunning) initServer();
    if (!serverRunning) return;

    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    // --- Accept incoming connections. ---

    int clientFileDesc = accept(serverFileDesc, (struct sockaddr*)&clientAddr, &clientAddrLen);
    if (clientFileDesc <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No incoming connections.
        }
        else {
            fmt::print("RollingRaspberry: accept() failed: {}\n", strerror(errno));
        }
        return;
    }

    fmt::print("RollingRaspberry: Accepted connection from {}\n", inet_ntoa(clientAddr.sin_addr));

    std::thread th([this, clientFileDesc, &th]() { this->connectionThread(th, clientFileDesc); });

    connectionThreads.emplace(clientFileDesc, std::move(th));

    for (auto& [fd, th] : connectionThreads) {
        if (!th.joinable()) {
            connectionThreads.erase(fd);
        }
    }
}

std::map<units::second_t, frc::Pose2d> RollingRaspberry::getEstimatedRobotPoses() {
}

void RollingRaspberry::connectionThread(std::thread& self, int clientFileDesc) {

    char buf[VISION_MESSAGE_SIZE];
    while (true) {
        ssize_t bytesRead = read(clientFileDesc, buf, VISION_MESSAGE_SIZE);
        if (bytesRead <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data to read.
            }
            else if (errno == ECONNRESET) {
                fmt::print("RollingRaspberry: Connection reset by peer.\n");
                break;
            }
            else {
                fmt::print("RollingRaspberry: read() failed: {}\n", strerror(errno));
                break;
            }
        }
        else if (bytesRead == VISION_MESSAGE_SIZE) {
            char* iter = buf;

            auto get_double = [&iter]() -> double {
                double value;
                std::memcpy(&value, iter, sizeof(double));
                iter += sizeof(double);
                return value;
            };

            units::meter_t pose1TranslationX = units::meter_t(get_double());
            units::meter_t pose1TranslationY = units::meter_t(get_double());
            units::meter_t pose1TranslationZ = units::meter_t(get_double());
            frc::Translation3d pose1Translation(pose1TranslationX, pose1TranslationY, pose1TranslationZ);
            frc::Rotation3d pose1Rotation(frc::Quaternion(get_double(), get_double(), get_double(), get_double()));
            double pose1Error = get_double();
            units::meter_t pose2TranslationX = units::meter_t(get_double());
            units::meter_t pose2TranslationY = units::meter_t(get_double());
            units::meter_t pose2TranslationZ = units::meter_t(get_double());
            frc::Translation3d pose2Translation(pose2TranslationX, pose2TranslationY, pose2TranslationZ);
            frc::Rotation3d pose2Rotation(frc::Quaternion(get_double(), get_double(), get_double(), get_double()));
            double pose2Error = get_double();

            {
                std::scoped_lock lock(connectionMutex);
                robotPoseEstimates.emplace_back(
                    frc::Pose3d(pose1Translation, pose1Rotation), frc::Pose3d(pose2Translation, pose2Rotation),
                    pose1Error, pose2Error
                );
            }
        }
    }


    self.detach();
}