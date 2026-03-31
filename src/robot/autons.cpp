/**
 * @file autons.cpp
 * @brief Autonomous routine implementations.
 *
 * ============================================================================
 *  USER FILE: Write your autonomous routines here.
 * ============================================================================
 *
 * Tips for writing good autonomous routines:
 *
 *   1. ALWAYS use timeouts on every motion command. A stuck robot with no
 *      timeout will burn all 15 seconds doing nothing.
 *
 *   2. Test each motion command individually before chaining them.
 *      Get move_forward working before you try a 10-step routine.
 *
 *   3. Use set_pose() at the start to set your known starting position.
 *      This makes move_to() work correctly.
 *
 *   4. Call reset_pid_defaults() at the top of each auton so previous
 *      settle tweaks from another routine don't carry over.
 *
 *   5. Use set_drive_settle() / set_turn_settle() at the top of an auton
 *      to tune settle behavior for the whole routine, or pass overrides
 *      per-motion for individual moves that need different tuning.
 */

#include "robot/autons.hpp"
#include "robot/config.hpp"

namespace robot {
namespace auton {

void do_nothing() {
    // Intentionally empty. Sometimes the best autonomous is no autonomous.
}

void example_auton() {
    // Always start by resetting PID defaults and setting the starting pose.
    // This guarantees a clean slate regardless of what ran before.
    robot::chassis.reset_pid_defaults();
    robot::chassis.set_pose({0, 0, 0});

    // ── Per-auton settle tuning ──────────────────────────────────────────
    // These apply to every motion in this routine until changed.
    // Use tighter values for precision routes, looser for speed.
    robot::chassis.set_drive_settle(0.5, 200);  // 0.5 inches, 200ms
    robot::chassis.set_turn_settle(1.0, 150);   // 1.0 degrees, 150ms

    // Drive forward 24 inches (one field tile) with a 2-second timeout
    robot::chassis.move_forward(24, 2000);

    parrot::delay(100);

    // Turn to face 90 degrees
    robot::chassis.turn_to(90, 1500);

    parrot::delay(100);

    // ── Per-motion overrides ─────────────────────────────────────────────
    // This short move needs tighter tolerance than the rest of the routine,
    // so override just for this call. The auton-level defaults are restored
    // automatically after the motion completes.
    robot::chassis.move_forward(6, 1500, {
        .max_speed  = 60,           // Slower for precision
        .tolerance  = 0.25,         // Quarter-inch accuracy
        .settle_time = 300,         // Wait longer to confirm settled
    });

    // This move uses the auton-level defaults again (0.5in, 200ms)
    robot::chassis.move_forward(12, 1500);

    // ── More override examples (uncomment to try) ────────────────────────

    // Override PID gains for a single move (heavier load, different tuning)
    // robot::chassis.move_forward(24, 2000, {
    //     .gains = {8.0, 0.0, 50.0},  // Stiffer P and D for heavy pushing
    //     .max_speed = 100,
    // });

    // Loose turn with custom settle (just need to be roughly facing right)
    // robot::chassis.turn_to(180, 1500, {
    //     .tolerance = 3.0,    // 3 degrees is close enough
    //     .settle_time = 100,  // Don't wait long
    // });

    // Swing turn with per-motion gains
    // robot::chassis.swing_to(180, parrot::Side::LEFT, 1500, {
    //     .gains = {3.0, 0.0, 20.0},
    //     .tolerance = 2.0,
    // });

    // Move-to with custom settle radius and drive gains
    // robot::chassis.move_to(24, 48, 3000, {
    //     .settle_radius = 0.5,     // Get within half an inch
    //     .drive_gains = {6.0, 0.0, 40.0},
    // });
}

// ─── Add your competition autonomous routines below ──────────────────────────

// void left_side_awp() {
//     robot::chassis.reset_pid_defaults();
//     robot::chassis.set_pose({-48, -48, 90});
//
//     // Tighter settle for a precision AWP route
//     robot::chassis.set_drive_settle(0.3, 250);
//     robot::chassis.set_turn_settle(0.8, 150);
//
//     // ... your routine here ...
// }

// void right_side_elim() {
//     robot::chassis.reset_pid_defaults();
//     robot::chassis.set_pose({48, -48, 270});
//
//     // Looser settle for a fast elimination route — speed matters more
//     robot::chassis.set_drive_settle(1.0, 100);
//     robot::chassis.set_turn_settle(2.0, 100);
//
//     // ... your routine here ...
// }

// void skills() {
//     robot::chassis.reset_pid_defaults();
//     robot::chassis.set_pose({0, 0, 0});
//
//     // Skills has 60 seconds so we can afford tighter tolerances
//     robot::chassis.set_drive_settle(0.3, 300);
//     robot::chassis.set_turn_settle(0.5, 200);
//     robot::chassis.set_move_to_settle(0.5, 250);
//
//     // ... your routine here ...
// }

} // namespace auton
} // namespace robot
