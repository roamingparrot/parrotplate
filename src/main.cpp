/**
 * @file main.cpp
 * @brief PROS entry point — initialize, autonomous, and opcontrol.
 *
 * This file contains the three functions the PROS kernel calls during
 * a VEX competition match:
 *   - initialize()         → runs when the brain turns on
 *   - autonomous()         → runs during the autonomous period
 *   - opcontrol()          → runs during driver control
 *
 * The robot hardware is configured in robot/config.cpp.
 * Autonomous routines are in robot/autons.cpp.
 * Subsystem logic is in robot/subsystems.cpp.
 *
 * This file should stay fairly minimal — it's just the glue between
 * PROS and your robot code.
 */

#include "main.h"

/**
 * Called once when the V5 brain powers on or the program starts.
 * Use this for calibration, GUI setup, and anything that needs to
 * happen before a match.
 */
void initialize() {
    // Calibrate IMU and start odometry tracking.
    // This blocks for ~2 seconds — the robot must be stationary.
    robot::chassis.calibrate();

    // Set up the brain screen GUI
    robot::screen.add_page(&robot::auton_selector);
    robot::screen.add_page(new parrot::OdomPage(
        robot::chassis.get_odometry()));
    robot::screen.add_page(new parrot::InfoPage({
        {"L Drive", {1, 2, 3}},    // Match your motor ports from config.cpp
        {"R Drive", {4, 5, 6}},
        // {"Intake",  {7}},
        // {"Lift",    {8}},
    }, 10));  // IMU port for sensor display
    robot::screen.start();

    parrot::Logger::info("Init", "Initialization complete");
}

/**
 * Called when the robot is in disabled state (connected to field control
 * but not in autonomous or driver control). Not commonly used, but
 * available if you need it.
 */
void disabled() {}

/**
 * Called between initialize() and the start of autonomous when connected
 * to competition control. Use this for last-minute setup that depends
 * on the selected autonomous (e.g., adjusting starting pose).
 */
void competition_initialize() {}

/**
 * Called at the start of the autonomous period (15 seconds in a match,
 * 60 seconds in skills).
 */
void autonomous() {
    // Run whichever autonomous was selected on the brain screen
    auto selected = robot::auton_selector.get_selected();
    if (selected) {
        parrot::Logger::info("Auton", "Running: %s",
                             robot::auton_selector.get_selected_name().c_str());
        selected();
    } else {
        parrot::Logger::info("Auton", "No autonomous selected");
    }
}

/**
 * Called at the start of the driver control period (1:45 in a match).
 *
 * This function runs a simple control loop at ~50Hz. The loop reads
 * joystick inputs and passes them to the chassis and subsystems.
 * Keep this loop lean — don't do heavy computation here.
 */
void opcontrol() {
    // Switch drive motors to coast for driver feel.
    // Some teams prefer hold — try both and see what your driver likes.
    robot::chassis.get_drivetrain().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        // ── Drivetrain ───────────────────────────────────────────────────
        // Arcade control: left stick Y = forward/back, right stick X = turn
        int forward = robot::controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn    = robot::controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        robot::chassis.arcade(forward, turn);

        // ── Subsystems ───────────────────────────────────────────────────
        // Add your subsystem controls here. Example:
        //
        // if (robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        //     robot::intake::spin(127);
        // } else if (robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        //     robot::intake::spin(-127);
        // } else {
        //     robot::intake::stop();
        // }
        //
        // if (robot::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        //     robot::clamp::toggle();
        // }
        //
        // robot::lift::control(
        //     robot::controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        // ── Loop timing ──────────────────────────────────────────────────
        // 20ms = 50Hz control loop. The V5 motor controller updates at 50Hz
        // internally, so faster than this doesn't help and just wastes CPU.
        parrot::delay(20);
    }
}
