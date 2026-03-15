/**
 * @file math_utils.cpp
 * @brief Math utility implementations.
 */

#include "parrotlib/core/math_utils.hpp"
#include <cmath>

namespace parrot {
namespace math {

double deg_to_rad(double deg) {
    return deg * PI / 180.0;
}

double rad_to_deg(double rad) {
    return rad * 180.0 / PI;
}

double wrap_angle_180(double angle) {
    // fmod gives us the remainder, but we need to handle the sign correctly.
    angle = std::fmod(angle, 360.0);
    if (angle > 180.0) angle -= 360.0;
    if (angle <= -180.0) angle += 360.0;
    return angle;
}

double wrap_angle_360(double angle) {
    angle = std::fmod(angle, 360.0);
    if (angle < 0.0) angle += 360.0;
    return angle;
}

double angle_error(double target, double current) {
    return wrap_angle_180(target - current);
}

double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double angle_to_point(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;

    // Avoid returning garbage when the points are on top of each other.
    if (std::fabs(dx) < 1e-9 && std::fabs(dy) < 1e-9) {
        return 0.0;
    }

    // atan2(dx, dy) because our 0° is +Y (forward), not +X.
    // This directly gives us heading in our compass-style convention.
    double angle = rad_to_deg(std::atan2(dx, dy));
    return wrap_angle_360(angle);
}

double clamp(double value, double limit) {
    limit = std::fabs(limit);
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

double clamp(double value, double low, double high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

double sign(double value) {
    if (value > 0.0) return 1.0;
    if (value < 0.0) return -1.0;
    return 0.0;
}

double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

} // namespace math
} // namespace parrot
