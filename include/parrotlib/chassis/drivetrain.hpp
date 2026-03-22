/**
 * @file drivetrain.hpp
 * @brief Low-level drivetrain motor control.
 *
 * This handles the "metal" side of the chassis — motor groups, voltage
 * output, and driver control curves. It deliberately does NOT include
 * PID or odometry; those are handled at the Chassis level.
 *
 * The separation exists because during debugging you sometimes want to
 * drive raw voltage to test if a motor is working, without fighting
 * the PID system.
 */

#ifndef PARROTLIB_CHASSIS_DRIVETRAIN_HPP
#define PARROTLIB_CHASSIS_DRIVETRAIN_HPP

#include "parrotlib/chassis/chassis_config.hpp"
#include "api.h"
#include <vector>

namespace parrot {

class Drivetrain {
public:
    /**
     * @param left_ports    Motor ports for left side (negative = reversed)
     * @param right_ports   Motor ports for right side (negative = reversed)
     * @param motor_rpm     Cartridge RPM (100, 200, or 600)
     * @param gear_ratio    External gear ratio (driven/driving)
     * @param wheel_diameter  Drive wheel diameter in inches
     */
    Drivetrain(const MotorPorts& left_ports, const MotorPorts& right_ports,
               int motor_rpm, double gear_ratio, double wheel_diameter);

    // ─── Raw motor control ───────────────────────────────────────────────

    /** Set voltage for left and right sides. Range: [-127, 127]. */
    void move(double left, double right);

    /** Set voltage for both sides equally. */
    void move(double voltage);

    /** Brake all motors (coast mode by default, or hold if set). */
    void stop();

    // ─── Driver control ──────────────────────────────────────────────────

    /**
     * Arcade drive: one stick controls forward/backward, another controls turning.
     *
     * @param forward  Forward/backward input [-127, 127]
     * @param turn     Turn input [-127, 127] (positive = right)
     * @param curve    Joystick curve exponent (0 = linear). Higher values
     *                 make the center of the stick less sensitive, giving
     *                 more precise control at low speeds.
     * @param deadzone Input values below this magnitude are treated as 0.
     */
    void arcade(double forward, double turn,
                double curve = 0.0, int deadzone = 5);

    /**
     * Tank drive: each stick controls one side directly.
     *
     * @param left     Left side input [-127, 127]
     * @param right    Right side input [-127, 127]
     * @param curve    Joystick curve exponent
     * @param deadzone Input values below this magnitude are treated as 0.
     */
    void tank(double left, double right,
              double curve = 0.0, int deadzone = 5);

    // ─── Encoder access ──────────────────────────────────────────────────

    /**
     * Average distance traveled by the drive motors, in inches.
     * Uses the left motor group's average encoder value.
     * Only useful when no tracking wheels are configured.
     */
    double get_left_distance() const;
    double get_right_distance() const;
    double get_average_distance() const;

    /** Zero all motor encoders. */
    void reset_encoders();

    // ─── Motor info ──────────────────────────────────────────────────────

    /** Average temperature across all drive motors (Celsius). */
    double get_average_temp() const;

    /** Set brake mode for all drive motors. */
    void set_brake_mode(pros::motor_brake_mode_e_t mode);

    /** Get the number of motors per side. */
    int motors_per_side() const;

private:
    pros::MotorGroup left_motors_;
    pros::MotorGroup right_motors_;

    double wheel_diameter_;
    double gear_ratio_;

    // Precomputed: inches per degree of motor rotation
    double inches_per_degree_;

    /** Apply exponential curve to joystick input. */
    static double apply_curve(double input, double curve);

    /** Apply deadzone threshold. */
    static double apply_deadzone(double input, int deadzone);
};

} // namespace parrot

#endif // PARROTLIB_CHASSIS_DRIVETRAIN_HPP
