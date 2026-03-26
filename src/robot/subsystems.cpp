/**
 * @file subsystems.cpp
 * @brief User-defined subsystem implementations.
 *
 * ============================================================================
 *  USER FILE: Implement your robot's subsystems here.
 * ============================================================================
 *
 * Define motors, pistons, and control logic for each mechanism.
 * Keep subsystems independent when possible — an intake shouldn't
 * need to know about the lift's state unless they genuinely interact.
 *
 * Example:
 * @code
 *   #include "robot/subsystems.hpp"
 *   #include "robot/config.hpp"
 *
 *   // Motor on port 7, not reversed
 *   static pros::Motor intake_motor(7);
 *
 *   namespace robot {
 *   namespace intake {
 *       void spin(int voltage) { intake_motor.move(voltage); }
 *       void stop() { intake_motor.move(0); }
 *   }
 *   }
 * @endcode
 */

#include "robot/subsystems.hpp"
#include "robot/config.hpp"

namespace robot {

// ─── Define subsystem motors/pistons here ────────────────────────────────────
// static pros::Motor intake_motor(7);
// static pros::Motor lift_motor(-8);  // Negative = reversed
// static pros::adi::DigitalOut clamp_piston('A');

// ─── Implement subsystem functions here ──────────────────────────────────────

// namespace intake {
//     void spin(int voltage) { intake_motor.move(voltage); }
//     void stop() { intake_motor.move(0); }
// }

// namespace lift {
//     static parrot::PID lift_pid({
//         .gains       = {1.5, 0.0, 8.0},
//         .output_max  = 127,
//         .tolerance   = 5.0,  // degrees
//         .settle_time = 100,
//     }, "Lift");
//
//     void move_to(double degrees) {
//         // Simple blocking PID move. For non-blocking,
//         // use a background task pattern.
//         lift_pid.reset();
//         while (!lift_pid.is_settled()) {
//             double error = degrees - lift_motor.get_position();
//             double output = lift_pid.update(error);
//             lift_motor.move(output);
//             pros::delay(10);
//         }
//         lift_motor.move(0);
//     }
//
//     void hold() {
//         lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//         lift_motor.move(0);
//     }
//
//     void control(int joystick_input) {
//         if (std::abs(joystick_input) > 10) {
//             lift_motor.move(joystick_input);
//         } else {
//             hold();
//         }
//     }
// }

// namespace clamp {
//     static bool is_open = false;
//     void toggle() { is_open = !is_open; clamp_piston.set_value(is_open); }
//     void open()   { is_open = true;  clamp_piston.set_value(true); }
//     void close()  { is_open = false; clamp_piston.set_value(false); }
// }

} // namespace robot
