/**
 * @file pose.cpp
 * @brief Pose struct implementation.
 */

#include "parrotlib/odom/pose.hpp"
#include "parrotlib/core/math_utils.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>

namespace parrot {

Pose::Pose(double x, double y, double heading)
    : x(x), y(y), heading(heading) {}

double Pose::distance_to(const Pose& other) const {
    return math::distance(x, y, other.x, other.y);
}

double Pose::angle_to(const Pose& other) const {
    return math::angle_to_point(x, y, other.x, other.y);
}

std::string Pose::to_string() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1)
       << "(" << x << ", " << y << ", " << heading << "°)";
    return ss.str();
}

Pose Pose::operator+(const Pose& other) const {
    return {x + other.x, y + other.y, heading + other.heading};
}

Pose Pose::operator-(const Pose& other) const {
    return {x - other.x, y - other.y, heading - other.heading};
}

Pose Pose::operator*(double scalar) const {
    return {x * scalar, y * scalar, heading * scalar};
}

} // namespace parrot
