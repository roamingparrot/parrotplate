/**
 * @file tracking_wheel.cpp
 * @brief Tracking wheel encoder abstraction implementation.
 */

#include "parrotlib/odom/tracking_wheel.hpp"
#include "parrotlib/core/math_utils.hpp"
#include <cmath>

namespace parrot {

TrackingWheel::TrackingWheel(int port, double wheel_diameter, double offset,
                             double gear_ratio)
    : port_(port),
      wheel_diameter_(wheel_diameter),
      offset_(offset),
      gear_ratio_(gear_ratio),
      is_motor_(false) {
    // inches per degree = (circumference / 360) * gear_ratio
    inches_per_degree_ =
        (wheel_diameter_ * math::PI / 360.0) * gear_ratio_;
}

TrackingWheel::TrackingWheel(int port, double wheel_diameter, double offset,
                             double gear_ratio, bool is_motor)
    : port_(port),
      wheel_diameter_(wheel_diameter),
      offset_(offset),
      gear_ratio_(gear_ratio),
      is_motor_(is_motor) {
    inches_per_degree_ =
        (wheel_diameter_ * math::PI / 360.0) * gear_ratio_;
}

double TrackingWheel::get_distance() const {
    return get_raw_degrees() * inches_per_degree_;
}

double TrackingWheel::get_raw_degrees() const {
    if (is_motor_) {
        // Motor get_position returns degrees (centidegrees / 100 internally,
        // but the PROS API gives us degrees as a double).
        pros::Motor motor(std::abs(port_));
        double pos = motor.get_position();
        return (port_ < 0) ? -pos : pos;
    } else {
        pros::Rotation rotation(std::abs(port_));
        // Rotation sensor returns centidegrees; convert.
        double pos = rotation.get_position() / 100.0;
        return (port_ < 0) ? -pos : pos;
    }
}

void TrackingWheel::reset() {
    if (is_motor_) {
        pros::Motor motor(std::abs(port_));
        motor.tare_position();
    } else {
        pros::Rotation rotation(std::abs(port_));
        rotation.reset_position();
    }
}

double TrackingWheel::get_offset() const {
    return offset_;
}

bool TrackingWheel::is_connected() const {
    // PROS returns PROS_ERR (INT32_MAX) when a device isn't plugged in.
    // We check by attempting to read and seeing if we get garbage.
    if (is_motor_) {
        pros::Motor motor(std::abs(port_));
        return motor.get_position() != PROS_ERR_F;
    } else {
        pros::Rotation rotation(std::abs(port_));
        return rotation.get_position() != PROS_ERR;
    }
}

} // namespace parrot
