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
 * Example autonomous — drives forward, turns, drives again.
 * Replace this with your actual competition routines.
 */
void example_auton();

// ─── Add your autonomous routines below ──────────────────────────────────
// void left_side_awp();
// void right_side_elim();
// void skills();

} // namespace auton
} // namespace robot

#endif // ROBOT_AUTONS_HPP
