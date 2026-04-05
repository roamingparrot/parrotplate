/**
 * @file config.cpp
 * @brief Robot hardware configuration — construct global objects here.
 *
 * ============================================================================
 *  USER FILE: Edit this to match your robot's wiring and tuning.
 * ============================================================================
 *
 * This is the single source of truth for your robot's hardware setup.
 * When you rewire a motor or change a gear ratio, this is the only file
 * you should need to touch.
 *
 * PID Tuning Quick Reference:
 *   kP: Start here. Increase until the robot reaches the target.
 *       Too high → oscillation. Too low → undershoots.
 *   kD: Add this to dampen oscillation. Usually 5-20x kP.
 *       Too high → jittery. Too low → overshoot persists.
 *   kI: Add last, only if there's steady-state error. Keep it small.
 *       Too high → slow windup oscillation. Usually 0.001-0.01x kP.
 *
 * General tuning order: drive_pid → heading_pid → turn_pid → swing_pid
 * Get straight-line driving solid first, then work on turns.
 */

#include "robot/config.hpp"
#include "robot/autons.hpp"

namespace robot {

// ─── Tracking Wheels (optional) ──────────────────────────────────────────────
// Uncomment and configure if you have dedicated tracking wheels.
// If not, the chassis will fall back to motor encoders + IMU.

// parrot::TrackingWheel left_tracker(1, 2.75, -2.5);     // port 1, 2.75" wheel, 2.5" left of center
// parrot::TrackingWheel right_tracker(2, 2.75, 2.5);     // port 2, 2.75" wheel, 2.5" right of center
// parrot::TrackingWheel back_tracker(3, 2.75, -4.5);     // port 3, 2.75" wheel, 4.5" behind center

// ─── Chassis ─────────────────────────────────────────────────────────────────
parrot::Chassis chassis({
    // Motor ports — negative means reversed.
    // Adjust these to match your wiring. Check motor directions with
    // `pros terminal` and raw voltage commands before tuning PID.
    .left_motors  = {-1, -2, -3},
    .right_motors = {4, 5, 6},

    // Motor hardware
    .motor_rpm      = 200,         // Green cartridge
    .gear_ratio     = 60.0 / 36.0, // 60T driven by 36T (1.667:1)
    .wheel_diameter = 3.25,        // inches

    // IMU
    .imu_port = 10,

    // Tracking wheels (set to nullptr if using motor encoders)
    .left_tracker  = nullptr,  // &left_tracker if using tracking wheels
    .right_tracker = nullptr,  // &right_tracker
    .back_tracker  = nullptr,  // &back_tracker

    // ── PID Tuning ───────────────────────────────────────────────────────
    // These are STARTING VALUES. You MUST tune these for your robot.
    // Every robot is different — weight, friction, center of gravity,
    // wheel wear all affect tuning. Budget at least 1-2 hours for this.

    .drive_pid = {
        .gains       = {5.0, 0.0, 35.0},
        .slew_rate   = 8.0,       // Prevents wheel spin on starts
        .windup_range = 3.0,      // inches
        .output_max  = 127,
        .tolerance   = 0.5,       // inches — how close is "close enough"
        .settle_time = 200,       // ms within tolerance
    },

    .turn_pid = {
        .gains       = {2.5, 0.0, 18.0},
        .output_max  = 100,       // Cap turn speed for consistency
        .tolerance   = 1.0,       // degrees
        .settle_time = 150,
    },

    .swing_pid = {
        .gains       = {2.0, 0.0, 15.0},
        .output_max  = 100,
        .tolerance   = 1.5,       // degrees — swings are less precise
        .settle_time = 150,
    },

    .heading_pid = {
        .gains       = {3.0, 0.0, 20.0},
        .output_max  = 40,        // Small corrections only
        .tolerance   = 0.5,       // degrees
    },

    .move_to_drive_pid = {
        .gains       = {5.0, 0.0, 35.0},
        .output_max  = 127,
        .tolerance   = 1.0,       // inches
        .settle_time = 200,
    },

    .move_to_turn_pid = {
        .gains       = {3.0, 0.0, 20.0},
        .output_max  = 80,
        .tolerance   = 2.0,       // degrees — looser because we're also driving
    },

    // ── Driver control ───────────────────────────────────────────────────
    .drive_curve       = 5.0,     // Joystick exponential curve (0 = linear)
    .joystick_deadzone = 8,       // Ignore stick values below this
});

// ─── Controller ──────────────────────────────────────────────────────────────
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ─── Autonomous Selector ─────────────────────────────────────────────────────
// These show up as buttons on the brain screen. Keep names short and clear.
parrot::AutonSelector auton_selector({
    {"Showcase",   auton::showcase},
    {"Do Nothing", auton::do_nothing},
    // Add your routines here:
    // {"Left AWP",   auton::left_side_awp},
    // {"Right Elim", auton::right_side_elim},
    // {"Skills",     auton::skills},
});

// ─── Screen ──────────────────────────────────────────────────────────────────
parrot::Screen screen;

} // namespace robot
