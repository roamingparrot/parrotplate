/**
 * @file drivetrain.cpp
 * @brief Low-level drivetrain motor control implementation.
 */

#include "parrotlib/chassis/drivetrain.hpp"
#include "parrotlib/core/math_utils.hpp"
#include <cmath>
#include <numeric>

namespace parrot {

Drivetrain::Drivetrain(const MotorPorts& left_ports,
                       const MotorPorts& right_ports,
                       int motor_rpm, double gear_ratio,
                       double wheel_diameter)
    : left_motors_(left_ports),
      right_motors_(right_ports),
      wheel_diameter_(wheel_diameter),
      gear_ratio_(gear_ratio) {
    // Configure motor cartridge gearing
    pros::motor_gearset_e_t gearset;
    switch (motor_rpm) {
        case 100: gearset = pros::E_MOTOR_GEARSET_36; break;
        case 200: gearset = pros::E_MOTOR_GEARSET_18; break;
        case 600: gearset = pros::E_MOTOR_GEARSET_06; break;
        default:  gearset = pros::E_MOTOR_GEARSET_18; break;
    }
    left_motors_.set_gearing(gearset);
    right_motors_.set_gearing(gearset);

    // Default to coast so drivers get the feel they expect
    left_motors_.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    right_motors_.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

    // Precompute conversion factor: motor degrees → inches traveled
    // Motor reports in degrees. After external gearing and wheel circumference:
    inches_per_degree_ = (wheel_diameter_ * math::PI / 360.0) * gear_ratio_;
}

void Drivetrain::move(double left, double right) {
    left  = math::clamp(left, 127.0);
    right = math::clamp(right, 127.0);
    left_motors_.move(static_cast<int>(left));
    right_motors_.move(static_cast<int>(right));
}

void Drivetrain::move(double voltage) {
    move(voltage, voltage);
}

void Drivetrain::stop() {
    left_motors_.move(0);
    right_motors_.move(0);
}

void Drivetrain::arcade(double forward, double turn,
                        double curve, int deadzone) {
    forward = apply_deadzone(forward, deadzone);
    turn    = apply_deadzone(turn, deadzone);
    forward = apply_curve(forward, curve);
    turn    = apply_curve(turn, curve);

    double left  = forward + turn;
    double right = forward - turn;

    // Scale both sides proportionally if we exceed [-127, 127].
    // This preserves the turn ratio instead of clipping one side.
    double max_val = std::max(std::fabs(left), std::fabs(right));
    if (max_val > 127.0) {
        double scale = 127.0 / max_val;
        left  *= scale;
        right *= scale;
    }

    move(left, right);
}

void Drivetrain::tank(double left, double right,
                      double curve, int deadzone) {
    left  = apply_curve(apply_deadzone(left, deadzone), curve);
    right = apply_curve(apply_deadzone(right, deadzone), curve);
    move(left, right);
}

double Drivetrain::get_left_distance() const {
    // MotorGroup get_positions() returns a vector of positions in degrees.
    // Average them for a smoother reading.
    auto positions = left_motors_.get_positions();
    if (positions.empty()) return 0.0;
    double sum = 0;
    for (double p : positions) sum += p;
    return (sum / positions.size()) * inches_per_degree_;
}

double Drivetrain::get_right_distance() const {
    auto positions = right_motors_.get_positions();
    if (positions.empty()) return 0.0;
    double sum = 0;
    for (double p : positions) sum += p;
    return (sum / positions.size()) * inches_per_degree_;
}

double Drivetrain::get_average_distance() const {
    return (get_left_distance() + get_right_distance()) / 2.0;
}

void Drivetrain::reset_encoders() {
    left_motors_.tare_position();
    right_motors_.tare_position();
}

double Drivetrain::get_average_temp() const {
    auto left_temps  = left_motors_.get_temperature_all();
    auto right_temps = right_motors_.get_temperature_all();

    double sum = 0;
    int count  = 0;
    for (double t : left_temps) { sum += t; count++; }
    for (double t : right_temps) { sum += t; count++; }

    return count > 0 ? sum / count : 0.0;
}

void Drivetrain::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    left_motors_.set_brake_modes(mode);
    right_motors_.set_brake_modes(mode);
}

int Drivetrain::motors_per_side() const {
    return left_motors_.size();
}

double Drivetrain::apply_curve(double input, double curve) {
    if (curve == 0.0 || input == 0.0) return input;

    // Attempt to preserve a sane curve behavior for the driver.
    // This exponential curve maps [-127, 127] → [-127, 127] with
    // reduced sensitivity near the center. curve=5 is a good default.
    // Formula adapted from the one EZ-Template and JAR-Template use,
    // since drivers are already familiar with that feel.
    double normalized = input / 127.0;
    double curved = std::pow(std::fabs(normalized), curve) *
                    math::sign(normalized) * 127.0;
    return curved;
}

double Drivetrain::apply_deadzone(double input, int deadzone) {
    if (std::fabs(input) < deadzone) return 0.0;
    return input;
}

} // namespace parrot
