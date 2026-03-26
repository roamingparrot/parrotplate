/**
 * @file subsystems.hpp
 * @brief User-defined subsystem declarations.
 *
 * ============================================================================
 *  USER FILE: Add your robot's subsystems here.
 * ============================================================================
 *
 * A "subsystem" is any mechanism on the robot beyond the drivetrain:
 * intakes, lifts, clamps, catapults, flywheels, etc.
 *
 * Declare your subsystem control functions here and implement them
 * in src/robot/subsystems.cpp. Keep each subsystem in its own namespace
 * or use descriptive function names.
 *
 * Example:
 * @code
 *   namespace robot {
 *   namespace intake {
 *       void spin(int voltage);
 *       void stop();
 *   }
 *   namespace lift {
 *       void move_to_position(double target);
 *       void hold();
 *   }
 *   }
 * @endcode
 */

#ifndef ROBOT_SUBSYSTEMS_HPP
#define ROBOT_SUBSYSTEMS_HPP

namespace robot {

// ─── Example: Intake ─────────────────────────────────────────────────────────
// Uncomment and modify for your robot's mechanisms.

// namespace intake {
//     void spin(int voltage);
//     void stop();
// }

// ─── Example: Lift / Arm ─────────────────────────────────────────────────────

// namespace lift {
//     void move_to(double degrees);
//     void hold();
//     void control(int joystick_input);
// }

// ─── Example: Clamp / Pneumatic ──────────────────────────────────────────────

// namespace clamp {
//     void toggle();
//     void open();
//     void close();
// }

} // namespace robot

#endif // ROBOT_SUBSYSTEMS_HPP
