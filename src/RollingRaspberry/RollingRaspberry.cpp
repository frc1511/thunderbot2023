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

/*
  Message structure:
  
  time_since_data - 4 bytes (float)
  pose1_x         - 4 bytes (float)
  pose1_y         - 4 bytes (float)
  pose1_z         - 4 bytes (float)
  pose1_qw        - 4 bytes (float)
  pose1_qx        - 4 bytes (float)
  pose1_qy        - 4 bytes (float)
  pose1_qz        - 4 bytes (float)
  pose1_error     - 4 bytes (float)
  pose2_x         - 4 bytes (float)
  pose2_y         - 4 bytes (float)
  pose2_z         - 4 bytes (float)
  pose2_qw        - 4 bytes (float)
  pose2_qx        - 4 bytes (float)
  pose2_qy        - 4 bytes (float)
  pose2_qz        - 4 bytes (float)
  pose2_error     - 4 bytes (float)
*/
#define VISION_MESSAGE_SIZE (sizeof(float) * 17)

// Field dimensions.

#define FIELD_X 16.54175_m
#define FIELD_Y 8.0137_m

// Areas of elevation.

#define BLUE_CS_MIN_X 2.7_m
#define BLUE_CS_MAX_X 4.91_m
#define RED_CS_MIN_X (FIELD_X - BLUE_CS_MAX_X)
#define RED_CS_MAX_X (FIELD_X - BLUE_CS_MIN_X)

#define CS_MIN_Y 1.28_m
#define CS_MAX_Y 4.07_m

#define CS_ELEVATION 0.5_m

// Tolerances

#define VERTICAL_TOLERANCE 5_in
#define HORIZONTAL_TOLERANCE 5_in

bool RobotPoseEstimate::isAmbiguous() const {
    double minError = std::min(error1, error2);
    double maxError = std::max(error1, error2);

    double ratio = -1;

    if (maxError > 0.0) {
        ratio = minError / maxError;
    }

    return ratio > 0.2 || ratio < 0;
}

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
    std::vector<RobotPoseEstimate> estimates;
    {
        std::lock_guard<std::mutex> lk(connectionMutex);

        // Get the latest robot pose estimates and clear the list.
        estimates = robotPoseEstimates;
        robotPoseEstimates.clear();
    }

    std::map<units::second_t, frc::Pose2d> finalPoses;

    for (const auto& estimate : estimates) {
        // If no ambiguity, use the pose with the lower reprojection error.
        if (!estimate.isAmbiguous()) {
            frc::Pose3d goodPose = estimate.error1 < estimate.error2 ? estimate.pose1 : estimate.pose2;
            finalPoses.emplace(estimate.timestamp, frc::Pose2d(goodPose.X(), goodPose.Y(), frc::Rotation2d(goodPose.Rotation().Z())));
            continue;
        }

        /**
         * Ambiguous D:
         *
         * Some ways to resolve the ambiguity include:
         * - Check whether the robot pose is outside of the field boundaries.
         * - Check whether the robot pose is above the field or below the field.
         * - Compare the pose against the previous known pose and see if it's a reasonable change.
         * - Check to see whether other estimated poses are similar.
         */
    }

}

bool RollingRaspberry::validatePose(frc::Pose3d pose) const {
    units::meter_t x = pose.X(), y = pose.Y(), z = pose.Z();

    // Check field boundaries.
    const units::meter_t min_x = 0_m - HORIZONTAL_TOLERANCE,
                         max_x = FIELD_X + HORIZONTAL_TOLERANCE;
    const units::meter_t min_y = 0_m - HORIZONTAL_TOLERANCE,
                         max_y = FIELD_Y + HORIZONTAL_TOLERANCE;

    // Check if on Charge Station.
    bool onChargeStation = false;
    if (y >= CS_MIN_Y && y <= CS_MAX_Y) {
        if ((x >= BLUE_CS_MIN_X && x <= BLUE_CS_MAX_X) ||
            (x >= RED_CS_MIN_X  && x <= RED_CS_MAX_X)) {
            onChargeStation = true;
        }
    }

    // Check elevation.
    const units::meter_t min_z = 0_m - VERTICAL_TOLERANCE,
                         max_z = (onChargeStation ? CS_ELEVATION : 0_m) + VERTICAL_TOLERANCE;

    return (x >= min_x && x <= max_x) &&
           (y >= min_y && y <= max_y) &&
           (z >= min_z && z <= max_z);
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

            auto get_float = [&iter]() -> float {
                float value;
                std::memcpy(&value, iter, sizeof(float));
                iter += sizeof(float);
                return value;
            };

            units::second_t timestamp(frc::Timer::GetFPGATimestamp() - units::second_t(get_float()));
            units::meter_t pose1X(get_float());
            units::meter_t pose1Y(get_float());
            units::meter_t pose1Z(get_float());
            frc::Translation3d pose1Translation(pose1X, pose1Y, pose1Z);
            frc::Rotation3d pose1Rotation(frc::Quaternion(get_float(), get_float(), get_float(), get_float()));
            double pose1Error = get_float();
            units::meter_t pose2X(get_float());
            units::meter_t pose2Y(get_float());
            units::meter_t pose2Z(get_float());
            frc::Translation3d pose2Translation(pose2X, pose2Y, pose2Z);
            frc::Rotation3d pose2Rotation(frc::Quaternion(get_float(), get_float(), get_float(), get_float()));
            double pose2Error = get_float();

            RobotPoseEstimate estimate(timestamp, frc::Pose3d(pose1Translation, pose1Rotation), frc::Pose3d(pose2Translation, pose2Rotation), pose1Error, pose2Error);

            {
                std::lock_guard<std::mutex> lk(connectionMutex);
                robotPoseEstimates.push_back(std::move(estimate));
            }
        }
    }

    self.detach();
}