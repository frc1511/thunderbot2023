#include <Trajectory/HermiteSplineTrajectory.h>

// https://en.wikipedia.org/wiki/Cubic_Hermite_spline

// --- Hermite Basis Functions ---

// h00(t) = 2t^3 - 3t^2 + 1
constexpr double h00(double t) { return 2.0 * std::pow(t, 3.0) - 3.0 * std::pow(t, 2.0) + 1.0; }
// h10(t) = t^3 - 2t^2 + t
constexpr double h10(double t) { return std::pow(t, 3.0) - 2.0 * std::pow(t, 2.0) + t; }
// h01(t) = -2t^3 + 3t^2
constexpr double h01(double t) { return -2.0 * std::pow(t, 3.0) + 3.0 * std::pow(t, 2.0); }
// h11(t) = t^3 - t^2
constexpr double h11(double t) { return std::pow(t, 3.0) - std::pow(t, 2.0); }

HermiteSplineTrajectory::HermiteSplineTrajectory(State p0, units::degree_t m0, State p1, units::degree_t m1, Config config) {

}

HermiteSplineTrajectory::HermiteSplineTrajectory(State p0, units::degree_t m0, State p1, Config config) {

}

HermiteSplineTrajectory::HermiteSplineTrajectory(State p0, State p1, units::degree_t m1, Config config) {

}

HermiteSplineTrajectory::~HermiteSplineTrajectory() {

}

HermiteSplineTrajectory::State HermiteSplineTrajectory::sample(units::second_t time) {

}

units::second_t HermiteSplineTrajectory::getDuration() {

}

frc::Pose2d HermiteSplineTrajectory::getInitialPose() {

}