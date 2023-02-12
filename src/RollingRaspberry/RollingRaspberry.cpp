#include <RollingRaspberry/RollingRaspberry.h>
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

extern "C" {
    struct VisionMessage {
        uint32_t pose_1_translation_x : 32;
        uint32_t pose_1_translation_y : 32;
        uint32_t pose_1_translation_z : 32;
        uint32_t pose_1_rotation_w : 32;
        uint32_t pose_1_rotation_x : 32;
        uint32_t pose_1_rotation_y : 32;
        uint32_t pose_1_rotation_z : 32;
        uint32_t pose_1_error : 32;

        uint32_t pose_2_translation_x : 32;
        uint32_t pose_2_translation_y : 32;
        uint32_t pose_2_translation_z : 32;
        uint32_t pose_2_rotation_w : 32;
        uint32_t pose_2_rotation_x : 32;
        uint32_t pose_2_rotation_y : 32;
        uint32_t pose_2_rotation_z : 32;
        uint32_t pose_2_error : 32;
    }; // 64 bytes
    typedef struct VisionMessage VisionMessage;
}

RollingRaspberry::RollingRaspberry() {
    initServer();
}

RollingRaspberry::~RollingRaspberry() {
    close(server_fd);
}

void RollingRaspberry::initServer() {
    using namespace std::chrono_literals;

    if (serverRunning) return;

    if (!socketCreated) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd <= 0) {
            fmt::print("RollingRaspberry: socket() failed: {}\n", strerror(errno));
            return;
        }

        socketCreated = true;
        fmt::print("RollingRaspberry: Created socket.\n");

        // Setup non-blocking I/O for the server socket.
        int flags = fcntl(server_fd, F_GETFL, 0);
        fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);
    }

    if (!socketCreated) return;

    if (!socketBound) {
        // Server address.
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons((int)HardwareManager::IOMap::TCP_ROLLING_RASPBERRY);
        std::memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));

        // --- Bind the server socket to the port. ---

        if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
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
        if (listen(server_fd, 3) < 0) {
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

    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    // --- Accept incoming connections. ---

    int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_len);
    if (client_fd <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No incoming connections.
        }
        else {
            fmt::print("RollingRaspberry: accept() failed: {}\n", strerror(errno));
        }
        return;
    }

    fmt::print("RollingRaspberry: Accepted connection from {}\n", inet_ntoa(client_addr.sin_addr));

    connection_threads.emplace(client_fd, std::thread([this, client_fd]() { this->connectionThread(client_fd); }));

    for (auto& [fd, th] : connection_threads) {
        if (th.joinable()) {
            connection_threads.erase(fd);
        }
    }
}

std::map<units::second_t, frc::Pose2d> RollingRaspberry::getEstimatedRobotPoses() {
}

void RollingRaspberry::connectionThread(int client_fd) {
    // TODO: Handle connection :D
}