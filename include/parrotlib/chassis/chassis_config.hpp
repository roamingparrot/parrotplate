/**
 * @file chassis_config.hpp
 * @brief Configuration structs for the Chassis class.
 *
 * All robot-specific hardware details go here. This is designed so you
 * configure once in robot/config.cpp and pass the struct to the Chassis
 * constructor. Keeps the wiring details out of your autonomous code.
 */

#ifndef PARROTLIB_CHASSIS_CONFIG_HPP
#define PARROTLIB_CHASSIS_CONFIG_HPP

#include "parrotlib/core/pid.hpp"
#include "parrotlib/odom/tracking_wheel.hpp"
#include <initializer_list>
#include <vector>

namespace parrot {

/**
 * Which side of the drivetrain a swing turn pivots around.
 */
enum class Side {
    LEFT,
    RIGHT
};

/**
 * Motor port group. Negative port number = reversed.
 * Example: {-1, 2, -3} means port 1 reversed, port 2 normal, port 3 reversed.
 */
using MotorPorts = std::vector<int>;

/**
 * Optional parameters for linear (forward/backward) motions.
 *
 * Every field defaults to -1 or 0 which means "use the chassis default."
 * Override any field per-call to tweak behavior for a specific motion
 * without touching the global config.
 *
 * @code
 *   // Use defaults for everything
 *   chassis.move_forward(24, 2000);
 *
 *   // Override just the speed and tolerance for a precision move
 *   chassis.move_forward(6, 1500, {.max_speed = 60, .tolerance = 0.25});
 *
 *   // Override PID gains for a particularly tricky movement
 *   chassis.move_forward(48, 3000, {.gains = {8.0, 0, 50.0}});
 * @endcode
 */
struct MoveParams {
    double max_speed   = 127;   // Cap output voltage
    double min_speed   = 0;     // Minimum speed (helps overcome static friction)
    double slew_rate   = -1;    // -1 = use chassis default, >0 = override
    double tolerance   = -1;    // -1 = use chassis default (inches)
    int    settle_time = -1;    // -1 = use chassis default (ms)
    PIDGains gains     = {-1, -1, -1};  // -1 per-field = use chassis default
    PIDGains heading_gains = {-1, -1, -1};  // Heading correction gains override
    bool   async       = false;
};

/**
 * Parameters for turn motions.
 * Same override convention: -1 means "use chassis default."
 */
struct TurnParams {
    double max_speed   = 127;
    double min_speed   = 0;
    double tolerance   = -1;    // degrees, -1 = use default
    int    settle_time = -1;    // ms, -1 = use default
    PIDGains gains     = {-1, -1, -1};
    bool   async       = false;
};

/**
 * Parameters for swing turns (one side drives, the other holds).
 */
struct SwingParams {
    double max_speed   = 127;
    double min_speed   = 0;
    double tolerance   = -1;    // degrees, -1 = use default
    int    settle_time = -1;    // ms, -1 = use default
    PIDGains gains     = {-1, -1, -1};
    bool   async       = false;
};

/**
 * Parameters for point-to-point movement.
 */
struct MoveToParams {
    double max_speed       = 127;
    double min_speed       = 0;
    double lead_distance   = 6.0;   // Carrot distance for curved path following (inches)
    double settle_radius   = -1;    // -1 = use chassis default, >0 = override (inches)
    double drive_tolerance = -1;    // -1 = use default (inches)
    int    settle_time     = -1;    // ms, -1 = use default
    PIDGains drive_gains   = {-1, -1, -1};
    PIDGains turn_gains    = {-1, -1, -1};
    bool   forward         = true;  // false = drive backwards to the point
    bool   async           = false;
};

/**
 * Complete chassis hardware and tuning configuration.
 *
 * @code
 *   parrot::ChassisConfig config = {
 *       .left_motors  = {-1, -2, -3},
 *       .right_motors = {4, 5, 6},
 *       .imu_port     = 10,
 *       .drive_pid    = {{4.5, 0.0, 35.0}, .tolerance = 0.5, .settle_time = 200},
 *       .turn_pid     = {{2.0, 0.0, 15.0}, .tolerance = 1.0, .settle_time = 150},
 *       // ...
 *   };
 * @endcode
 */
struct ChassisConfig {
    // ─── Motors ──────────────────────────────────────────────────────────
    MotorPorts left_motors;     // Left side motor ports (negative = reversed)
    MotorPorts right_motors;    // Right side motor ports (negative = reversed)

    /**
     * Motor cartridge RPM. Must match what's physically in the motors.
     *   - 100: red cartridge (torque)
     *   - 200: green cartridge (standard)
     *   - 600: blue cartridge (speed)
     */
    int motor_rpm = 200;

    /**
     * External gear ratio: driven / driving.
     * Example: 60T driven by 36T = 60.0/36.0 = 1.667
     * Set to 1.0 for direct drive (no external gears).
     */
    double gear_ratio = 1.0;

    /** Drive wheel diameter in inches (3.25" and 4" are common). */
    double wheel_diameter = 3.25;

    // ─── Sensors ─────────────────────────────────────────────────────────
    /** IMU port (1-21). Set to 0 for no IMU. */
    int imu_port = 0;

    // ─── Tracking Wheels (optional) ──────────────────────────────────────
    // If nullptr, chassis uses integrated motor encoders for distance.
    TrackingWheel* left_tracker  = nullptr;
    TrackingWheel* right_tracker = nullptr;
    TrackingWheel* back_tracker  = nullptr;

    // ─── PID Configurations ──────────────────────────────────────────────
    PIDConfig drive_pid;           // Forward/backward distance control
    PIDConfig turn_pid;            // Point-turn heading control
    PIDConfig swing_pid;           // Swing-turn heading control
    PIDConfig heading_pid;         // Heading correction during straight drives

    /**
     * PID config for the move_to() lateral heading correction.
     * This controls how aggressively the robot steers toward the target
     * point during point-to-point movement.
     */
    PIDConfig move_to_drive_pid;
    PIDConfig move_to_turn_pid;

    // ─── Drive Behavior ──────────────────────────────────────────────────

    /** Joystick curve for driver control (0 = linear, 5-10 = typical). */
    double drive_curve = 0.0;

    /**
     * Deadzone for joystick input. V5 controller sticks can drift a few
     * counts even when centered. 5-10 is usually enough to handle it.
     */
    int joystick_deadzone = 5;
};

} // namespace parrot

#endif // PARROTLIB_CHASSIS_CONFIG_HPP
