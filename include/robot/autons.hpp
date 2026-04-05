/**
 * @file autons.hpp
 * @brief Autonomous routine declarations.
 *
 * ============================================================================
 *  USER FILE: Define your autonomous routines here.
 * ============================================================================
 *
 * Each autonomous routine is a void function. Declare them here and
 * implement them in src/robot/autons.cpp.
 *
 * Register them with the AutonSelector in config.cpp so the drive team
 * can pick one from the brain screen before each match.
 *
 * Naming convention: name them by what they do and where they start.
 * Your drive team will thank you when they're picking autons under
 * match pressure.
 */

#ifndef ROBOT_AUTONS_HPP
#define ROBOT_AUTONS_HPP

namespace robot {
namespace auton {

/**
 * Do-nothing autonomous. Always have this as a fallback option
 * in case the alliance partner asks you not to move.
 */
void do_nothing();

/**
 * Comprehensive showcase routine that demonstrates every major motion
 * function in the framework. Reads like a realistic skills-style run.
 *
 * Use this as a reference when building your own routines — every
 * function call is annotated with what it does, why you'd use it,
 * and what to watch out for.
 */
void showcase();

// ─── Add your autonomous routines below ──────────────────────────────────
// void left_side_awp();
// void right_side_elim();
// void skills();

} // namespace auton
} // namespace robot

#endif // ROBOT_AUTONS_HPP
