/**
 * @file autons.cpp
 * @brief Autonomous routine implementations.
 *
 * ============================================================================
 *  USER FILE: Write your autonomous routines here.
 * ============================================================================
 *
 * This file contains the showcase routine that demonstrates every major
 * motion function in the framework. Study it, copy pieces from it, then
 * delete it and write your own routes.
 *
 * Ground rules for writing autonomous routines:
 *
 *   1. ALWAYS use timeouts. A stuck PID loop with no timeout will burn
 *      your entire autonomous period doing nothing.
 *
 *   2. Call reset_pid_defaults() at the top. PID state carries over
 *      between routines if the brain doesn't power cycle (common when
 *      re-running autons from the selector during testing).
 *
 *   3. Call set_pose() at the top with your known starting position.
 *      Every move_to() call depends on the coordinate frame being right.
 *      Get this wrong and the robot drives to the wrong spot on the field.
 *
 *   4. Test each motion individually before chaining. If move_forward(24)
 *      doesn't go 24 inches accurately by itself, a 10-step routine
 *      won't magically fix that — the errors compound.
 *
 *   5. You do NOT need delays between motions. Each motion command blocks
 *      until the robot's error has stayed within tolerance for the full
 *      settle time. That IS the "the robot has stopped" guarantee.
 *      If you're tempted to add a delay, your settle params are probably
 *      too loose — tighten tolerance or increase settle time instead.
 */

#include "robot/autons.hpp"
#include "robot/config.hpp"

namespace robot {
namespace auton {

void do_nothing() {
    // Intentionally empty. Sometimes the best autonomous is no autonomous.
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SHOWCASE AUTONOMOUS
// ═══════════════════════════════════════════════════════════════════════════════
//
//  A realistic skills-style routine that intentionally uses every major
//  motion function. The route simulates:
//
//    1. Leave the starting tile
//    2. Drive to the first scoring zone (odom navigation)
//    3. Align and score
//    4. Swing turn to face the next objective
//    5. Cross the field (point-to-point)
//    6. Precision approach for a second score
//    7. Reposition with heading correction
//    8. Navigate backward to a pickup
//    9. Final scoring run
//   10. Park in the elevation zone
//
//  Coordinate convention reminder:
//    X = right (+),  Y = forward (+)
//    Heading 0° = facing +Y (forward), clockwise positive
//    All distances in inches, all angles in degrees
//
//  The field is 144x144 inches (6x6 tiles, 24 inches per tile).
//  We place the origin at the robot's starting position.
//
// ═══════════════════════════════════════════════════════════════════════════════

void showcase() {

// ─── PHASE 0: INITIALIZATION ─────────────────────────────────────────────────
//
// Every autonomous routine should start with these three calls. They
// guarantee the robot is in a known state regardless of what ran before.

    // reset_pid_defaults() restores all PID configs to whatever you set in
    // config.cpp. This matters because set_drive_settle(), set_turn_settle(),
    // and per-motion overrides all modify the active PID configs at runtime.
    // If you ran a different auton first (or re-ran this one), those changes
    // persist. This wipes the slate.
    robot::chassis.reset_pid_defaults();

    // set_pose() tells the odometry system "I am HERE right now."
    // Everything the robot does after this is measured relative to this pose.
    //
    // For this showcase we start at field position (-48, -60) facing 0°.
    // In a real match, measure where you place the robot and put those
    // coordinates here. If the starting pose is wrong, every move_to()
    // call drives to the wrong place.
    //
    // Units: x = inches (+right), y = inches (+forward), heading = degrees
    robot::chassis.set_pose({-48, -60, 0});

    // Per-auton settle tuning. These set the settle error (tolerance) and
    // settle time for the ENTIRE routine. Individual motions can still
    // override these per-call (shown later), but these are the baseline.
    //
    // For a skills run with 60 seconds, we can afford to be precise.
    // For a 15-second match auton, you'd use looser values (1.0in, 100ms).
    //
    // How settle works:
    //   tolerance (settle error) = maximum acceptable error. The robot must
    //     be within this range of the target for settle to begin counting.
    //     For drives this is in inches, for turns in degrees.
    //   settle_time = how many milliseconds the robot must CONTINUOUSLY
    //     stay within tolerance before the motion is considered done.
    //     This prevents exiting on the peak of an oscillation — the robot
    //     might briefly pass through the tolerance zone while overshooting,
    //     but settle_time forces it to actually be stable there.
    //
    // Example: tolerance = 0.5in, settle_time = 200ms
    //   The robot hits 23.8in (within 0.5 of 24) → timer starts
    //   Overshoots to 24.3in (still within 0.5) → timer keeps counting
    //   Bounces to 24.6in (outside 0.5) → timer RESETS
    //   Settles to 24.1in (within 0.5) → timer starts over
    //   Stays within 0.5 for 200ms straight → motion complete
    robot::chassis.set_drive_settle(0.5, 200);   // 0.5 inches, 200ms
    robot::chassis.set_turn_settle(1.0, 150);    // 1.0 degrees, 150ms
    robot::chassis.set_swing_settle(1.5, 150);   // swings are less precise
    robot::chassis.set_move_to_settle(1.0, 200); // point-to-point

// ─── PHASE 1: LEAVE THE STARTING TILE ────────────────────────────────────────
//
//  move_forward(distance, timeout)
//
//  Drives straight forward by the given distance in inches. A heading
//  correction PID runs in parallel to keep the robot from veering left
//  or right — even small motor imbalances cause drift over 24+ inches.
//
//  The function blocks until BOTH conditions are met:
//    - The error is within tolerance (settle error)
//    - It has stayed there for settle_time ms
//  OR the timeout expires, whichever comes first.
//
//  Parameters:
//    distance — inches to travel (positive = forward)
//    timeout  — maximum time in milliseconds. If the PID hasn't settled
//               by this time, the motion aborts and the routine continues.
//               ALWAYS set this in competition. 2000ms per tile is safe.
//
//  Common mistakes:
//    - Forgetting the timeout. The robot stalls against a wall and the
//      PID loop runs forever. 15 seconds wasted.
//    - Setting tolerance too tight for the hardware. If your drivetrain
//      can't reliably hold 0.1-inch accuracy, the PID will oscillate
//      forever trying. Start at 0.5in and tighten from there.

    robot::chassis.move_forward(24, 2000);
    // The robot drove one tile forward. It didn't return from this call
    // until it was within 0.5in of the target for 200ms straight (or
    // until the 2000ms timeout expired). No delay needed — the robot
    // is confirmed stable before we continue.

// ─── PHASE 2: TURN TO FACE THE SCORING ZONE ─────────────────────────────────
//
//  turn_to(heading, timeout)
//
//  Turns the robot IN PLACE to face the given absolute heading. The heading
//  is field-absolute — 90° always means facing right, regardless of where
//  the robot is or what heading it started at.
//
//  The controller automatically picks the shortest rotation direction.
//  If you're at 350° and target 10°, it turns 20° clockwise, not 340° CCW.
//
//  Parameters:
//    heading — absolute target heading in degrees (0° = forward, CW+)
//    timeout — max time in ms
//
//  When to use turn_to() vs swing_to():
//    - turn_to() spins both sides in opposite directions (tank turn).
//      Tight turning radius, stays roughly in place. Use when space is
//      limited or you need to face an exact heading from a known spot.
//    - swing_to() locks one side and drives the other. Wider arc, but
//      faster and smoother. Use when you have room and need speed.

    robot::chassis.turn_to(90, 1500);
    // Now facing 90° (right). Confirmed within 1.0° for 150ms before returning.

// ─── PHASE 3: ODOM NAVIGATION — DRIVE TO A FIELD COORDINATE ─────────────────
//
//  move_to(x, y, timeout)
//
//  Drives to a global (x, y) coordinate on the field. This is the most
//  powerful motion command — it handles both steering and distance in one
//  call using two PID loops (one for heading, one for distance).
//
//  The robot steers toward the target point as it drives, correcting its
//  heading continuously. This means it can approach from any angle — you
//  don't need to be facing the target before calling move_to().
//
//  Parameters:
//    x, y    — target position in inches (field-absolute coordinates)
//    timeout — max time in ms
//
//  CRITICAL: move_to() only works correctly if the odometry knows where
//  the robot is. That's why set_pose() at the start matters so much.
//  If the odom drifts over a long routine, move_to() accuracy degrades.
//  Re-reading the pose and sanity-checking it against known field features
//  (walls, lines) helps catch drift problems.
//
//  The settle_radius parameter controls how close is "close enough."
//  Default comes from set_move_to_settle(). Override per-call if needed.

    robot::chassis.move_to(-24, -12, 3000);
    // The robot navigated from roughly (-48, -36) to (-24, -12).
    // It steered itself to face the target and drove there in one motion.

// ─── PHASE 4: PRECISION ALIGNMENT ───────────────────────────────────────────
//
//  Per-motion overrides let you tweak behavior for a single call.
//  Here we need a precise short forward move to align for scoring.
//
//  Any field set to -1 (or omitted) uses the auton-level default.
//  Only specify what you want to change.

    // Slow, precise forward nudge — tighter settle than the baseline.
    robot::chassis.move_forward(6, 1500, {
        .max_speed   = 60,     // Half speed for control
        .tolerance   = 0.25,   // Quarter-inch settle error
        .settle_time = 300,    // Must hold within 0.25in for 300ms
    });
    // After this call, the PID config reverts to the auton defaults (0.5in, 200ms).
    // The per-motion overrides are temporary — they don't leak into later calls.

    // (In a real routine you'd actuate a scoring mechanism here)
    // robot::intake::spin(127);
    // parrot::delay(500);   ← delay for mechanism actuation is fine — that's
    //                          waiting for the MECHANISM, not waiting for the
    //                          drivetrain to settle.
    // robot::intake::stop();

// ─── PHASE 5: BACK AWAY FROM THE SCORING ZONE ───────────────────────────────
//
//  move_backward(distance, timeout)
//
//  Drives in reverse. Internally this is just move_forward(-distance),
//  but it reads clearer in a routine. The heading correction still runs,
//  so the robot backs up straight.
//
//  Backing away before turning prevents the robot from clipping the
//  scoring zone as it rotates. Give yourself at least 6–8 inches of
//  clearance for a point turn, more for a swing.

    robot::chassis.move_backward(12, 1500);

// ─── PHASE 6: SWING TURN LEFT ───────────────────────────────────────────────
//
//  swing_to(heading, side, timeout)
//
//  One side of the drivetrain stays still while the other drives. The
//  robot pivots around the stationary side, creating an arcing turn.
//
//  Side::LEFT  = left wheels stay still, right wheels drive.
//                The robot pivots around its left side.
//  Side::RIGHT = right wheels stay still, left wheels drive.
//                The robot pivots around its right side.
//
//  Parameters:
//    heading — absolute target heading in degrees
//    side    — which side stays locked (parrot::Side::LEFT or RIGHT)
//    timeout — max time in ms
//
//  When to use swing turns:
//    - Faster than point turns for large heading changes (45°+)
//    - Smoother motion — less jerk, less wheel scrub
//    - The robot moves sideways during the turn, which you need to account
//      for in your route planning. A left swing pivots your right side forward.
//    - Don't use swings in tight spaces — the arc needs room.
//
//  Common mistake: forgetting that a swing changes the robot's XY position.
//  After a swing, the odom will show the robot has shifted. If the next
//  command is a move_to(), that's fine — it re-navigates. But if the next
//  command is move_forward(), the robot starts from the post-swing position.

    robot::chassis.swing_to(180, parrot::Side::LEFT, 1500);
    // The right side drove forward while the left held still, pivoting
    // the robot to face 180° (backward from starting orientation).
    // The robot's position shifted slightly to the right during the arc.

// ─── PHASE 7: CROSS THE FIELD — MULTI-POINT NAVIGATION ──────────────────────
//
//  Chaining multiple move_to() calls creates a multi-waypoint path.
//  The robot navigates to each point in sequence. Each call handles
//  its own steering — you don't need turn_to() between them unless
//  you need a specific facing direction at the waypoint.
//
//  No delays between waypoints. Each move_to() settles before returning,
//  so the robot is stationary when the next one starts.
//
//  Tip: for long traversals, add an intermediate waypoint to avoid
//  obstacles. move_to() drives in a roughly straight line to the target.
//  It won't path-plan around things.

    // Waypoint 1: cross toward center field
    robot::chassis.move_to(0, 0, 3000);

    // Waypoint 2: arrive at the far scoring zone
    robot::chassis.move_to(36, 24, 3000);

// ─── PHASE 8: SWING TURN RIGHT ──────────────────────────────────────────────
//
//  Same idea as Phase 6, but swinging the other direction.

    robot::chassis.swing_to(270, parrot::Side::RIGHT, 1500);
    // The left side drove while the right held, pivoting to face 270° (left).

// ─── PHASE 9: READING THE ODOMETRY — RUNTIME POSE CHECKS ────────────────────
//
//  get_pose() returns the robot's current estimated position at any time.
//  Use it for runtime decisions, logging, or debugging.
//
//  The Pose struct has three fields: x, y, heading (all doubles).
//  It also has helper methods:
//    pose.distance_to(other_pose) — euclidean distance in inches
//    pose.angle_to(other_pose)    — heading to the other pose in degrees
//    pose.to_string()             — formatted "(x, y, heading°)" for logging

    parrot::Pose current = robot::chassis.get_pose();
    parrot::Logger::info("Auton", "Currently at %s", current.to_string().c_str());

    // The Odometry object has its own convenience helpers that do the same
    // thing but read from the live tracking data directly:
    //
    //   chassis.get_odometry().distance_to(x, y)
    //   chassis.get_odometry().angle_to(x, y)
    //
    // These are handy for conditional logic. For example, checking if
    // we're close enough to skip a waypoint:

    parrot::Pose target_zone = {48, 48};
    double dist_remaining = current.distance_to(target_zone);
    double heading_to_target = current.angle_to(target_zone);

    parrot::Logger::info("Auton",
        "%.1f inches to target, need heading %.1f°",
        dist_remaining, heading_to_target);

    // You can use these for conditional routing:
    // if (dist_remaining < 6.0) {
    //     // Already close, skip the drive
    // } else {
    //     robot::chassis.move_to(48, 48, 3000);
    // }

// ─── PHASE 10: TURN TO FACE A POINT (CALCULATED HEADING) ────────────────────
//
//  Sometimes you want to face a specific field coordinate before driving
//  forward (instead of letting move_to() handle steering). This is useful
//  when you need to be precisely aimed — like lining up a shooter.
//
//  Use angle_to() to calculate the heading, then turn_to() that value.

    double aim_heading = current.angle_to({48, 48});
    robot::chassis.turn_to(aim_heading, 1500);

// ─── PHASE 11: FORWARD DRIVE WITH CUSTOM PID GAINS ──────────────────────────
//
//  Per-motion PID gain overrides. If one specific movement needs stiffer
//  or softer control than your baseline, override the gains for just
//  that call. The originals are restored after the motion completes.
//
//  Scenario: we're pushing a game object and need more aggressive control
//  to maintain speed against the resistance.

    robot::chassis.move_forward(24, 2500, {
        .gains = {8.0, 0.0, 50.0},   // Stiffer P and D for pushing
        .max_speed = 100,             // Not full power — leave headroom for heading correction
    });

// ─── PHASE 12: MOVE TO POINT DRIVING BACKWARD ───────────────────────────────
//
//  move_to() with .forward = false drives to the target in reverse.
//  The robot still steers toward the point, but it backs into it
//  instead of driving front-first.
//
//  When to use:
//    - The back of your robot has a mechanism that needs to face the target
//      (e.g., a rear clamp approaching a mobile goal)
//    - You'd waste time turning around just to drive a short distance
//    - You want to back into a wall for alignment

    robot::chassis.move_to(24, 0, 3000, {
        .forward = false,
    });
    // The robot backed up to (24, 0), steering in reverse the whole way.

// ─── PHASE 13: HEADING RESET — CORRECTING DRIFT ─────────────────────────────
//
//  reset_heading(heading) overwrites the odometry heading without touching
//  the XY position. This is useful when you know the robot's heading from
//  an external reference (e.g., after squaring against a wall) but the
//  odom has drifted.
//
//  After a long routine, IMU drift or tracking wheel slip can accumulate
//  a few degrees of heading error. If you have a chance to square up
//  against a known wall or barrier, you can correct the heading.
//
//  Example: the robot just backed into a flat wall that runs along the
//  0° axis. You know the heading is 180° (facing the wall behind you).

    // Wall-square: drive backward slowly until the timeout expires.
    // The robot pushes against the wall for the full 800ms, squaring up.
    // This intentionally times out — we don't care about settling, we
    // want wall contact. The timeout IS the "wait" here.
    robot::chassis.move_backward(3, 800, {.max_speed = 40});

    // We know we're squared against a wall at heading 180°
    robot::chassis.reset_heading(180);
    parrot::Logger::info("Auton", "Heading corrected to 180° after wall square");

    // The odom heading is now exactly 180°, even if it had drifted.
    // XY position is unchanged — only the heading was corrected.
    // All subsequent turn_to() and move_to() calls now work from
    // this corrected heading.

// ─── PHASE 14: SECOND MOVE_TO WITH PER-MOTION SETTLE OVERRIDE ───────────────
//
//  Here we need to arrive at a very precise location for scoring.
//  Override settle_radius and settle_time for just this call.

    robot::chassis.move_to(48, 48, 4000, {
        .max_speed   = 80,              // Controlled approach
        .settle_radius = 0.5,           // Get within half an inch
        .settle_time = 300,             // Hold for 300ms to confirm
    });

// ─── PHASE 15: TURN WITH LOOSE TOLERANCE (FAST ALIGNMENT) ───────────────────
//
//  Not every turn needs to be precise. When you just need to roughly
//  face a direction before a long drive (where move_to will correct
//  the heading anyway), use a loose tolerance and short settle time
//  to save autonomous seconds.

    robot::chassis.turn_to(0, 1000, {
        .tolerance   = 3.0,   // 3 degrees settle error — close enough
        .settle_time = 50,    // Only needs to hold for 50ms
    });

// ─── PHASE 16: FULL POSE RESET ──────────────────────────────────────────────
//
//  reset_pose() sets everything back to (0, 0, 0°). It's equivalent
//  to set_pose({0, 0, 0}).
//
//  When to use:
//    - You've driven back to a known landmark and want to zero out
//      accumulated drift for the next section of the routine.
//    - You're switching from a "relative to start" coordinate frame to a
//      "relative to this new position" frame mid-routine.
//
//  Caution: after reset_pose(), all previous coordinates are invalid.
//  If you were using move_to({48, 48}) before the reset, that same call
//  now targets a completely different place because (0,0) moved.
//  Only reset when you intentionally want a new coordinate origin.

    robot::chassis.reset_pose();
    // The robot's position is now (0, 0, 0°) regardless of where it actually is.
    // All subsequent move_to() calls are relative to this new origin.

    parrot::Logger::info("Auton", "Pose reset — new origin established");

// ─── PHASE 17: NAVIGATION FROM THE NEW ORIGIN ──────────────────────────────
//
//  After the reset, coordinates are relative to the current position.
//  (24, 24) means "24 inches right and 24 inches forward from where I am now."

    robot::chassis.move_to(24, 24, 3000);
    // This drives to a point one tile right and one tile forward from the
    // reset position — NOT from the original starting position.

// ─── PHASE 18: ASYNC MOTION — DO SOMETHING WHILE DRIVING ────────────────────
//
//  All motion commands block by default. Setting .async = true makes the
//  function return immediately while the motion runs in a background task.
//  Use wait_until_settled() to block until it finishes.
//
//  This lets you actuate mechanisms while the robot is still moving.
//  For example, spin up an intake while driving to the next game object.

    robot::chassis.move_forward(24, 2500, {.async = true});
    // The function returned immediately. The robot is driving right now.

    // Do something else in parallel:
    // robot::intake::spin(127);
    parrot::Logger::info("Auton", "Driving forward + running intake simultaneously");

    // Wait for the motion to complete before the next step.
    // If you forget this and issue another motion command, cancel_motion()
    // is called automatically — the new motion takes over.
    robot::chassis.wait_until_settled();
    // robot::intake::stop();

// ─── PHASE 19: CANCEL MOTION — EMERGENCY STOP ───────────────────────────────
//
//  cancel_motion() immediately stops all motors and kills any running
//  motion (sync or async). Use it for emergency stops or when teleop
//  needs to take over from a running auton motion.
//
//  In practice, the chassis calls cancel_motion() internally when you
//  call arcade() or tank() during opcontrol, so driver input always
//  overrides any leftover autonomous movement.

    robot::chassis.move_forward(48, 3000, {.async = true});
    parrot::delay(500);  // Let it drive for half a second...
    // We changed our mind — stop the motion early
    robot::chassis.cancel_motion();
    parrot::Logger::info("Auton", "Motion cancelled after 500ms");

// ─── PHASE 20: FINAL PARKING — SWING + PRECISION DRIVE ──────────────────────
//
//  Typical endgame: swing to face the elevation zone, drive in, stop.
//  The swing is faster than a point turn here because we're turning
//  a large angle (let's say ~90°) and have room to arc.

    robot::chassis.swing_to(90, parrot::Side::LEFT, 1500, {
        .tolerance = 2.0,    // Don't need exact heading for the approach
    });

    // Final precision drive into the parking zone
    robot::chassis.move_forward(18, 2000, {
        .max_speed   = 60,    // Controlled — don't overshoot
        .min_speed   = 15,    // Keep enough power to climb the platform
        .tolerance   = 0.5,
        .settle_time = 250,
    });

    // Lock the drivetrain in place so the robot doesn't roll off the platform
    robot::chassis.get_drivetrain().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    robot::chassis.get_drivetrain().stop();

    parrot::Logger::info("Auton", "Showcase routine complete — parked");

// ═════════════════════════════════════════════════════════════════════════════
//  FUNCTION-BY-FUNCTION QUICK REFERENCE
// ═════════════════════════════════════════════════════════════════════════════
//
//  Motion Commands (all blocking by default, all take timeout in ms):
//  ────────────────────────────────────────────────────────────────────
//  move_forward(inches, timeout, {params})     — straight line, heading-corrected
//  move_backward(inches, timeout, {params})    — reverse, heading-corrected
//  turn_to(degrees, timeout, {params})         — point turn to absolute heading
//  swing_to(degrees, side, timeout, {params})  — arc turn, one side locked
//  move_to(x, y, timeout, {params})            — drive to field coordinate
//  move_to(Pose, timeout, {params})            — same, takes a Pose struct
//
//  Settle behavior (applies to all motion commands):
//  ────────────────────────────────────────────────────────────────────
//  Every motion blocks until BOTH conditions are met simultaneously:
//    1. |error| < tolerance (settle error)     — robot is close enough
//    2. Condition 1 has held for settle_time ms — robot is stable there
//  OR the timeout expires, whichever comes first.
//
//  This means you NEVER need delays between motions. When the function
//  returns, the robot has been confirmed stable for the settle duration.
//  If it returns due to timeout, the robot may still be moving — that's
//  the tradeoff of timeout protection.
//
//  Pose & Odometry:
//  ────────────────────────────────────────────────────────────────────
//  set_pose({x, y, heading})   — set the robot's position (call at auton start)
//  get_pose()                  — read current position (returns Pose)
//  reset_pose()                — zero everything to (0, 0, 0°)
//  reset_heading(degrees)      — overwrite heading only, keep XY
//
//  Pose helpers:
//    pose.distance_to(other)   — euclidean distance in inches
//    pose.angle_to(other)      — heading toward other pose in degrees
//    pose.to_string()          — "(x, y, heading°)" for debug logging
//
//  Odometry helpers:
//    get_odometry().distance_to(x, y)  — distance from current position
//    get_odometry().angle_to(x, y)     — heading from current position
//
//  Motion State:
//  ────────────────────────────────────────────────────────────────────
//  is_settled()           — true if no motion is running
//  wait_until_settled()   — block until async motion finishes
//  cancel_motion()        — kill current motion, stop motors
//
//  Settle Tuning (per-auton, persistent until changed):
//  ────────────────────────────────────────────────────────────────────
//  set_drive_settle(tolerance_inches, settle_time_ms)
//  set_turn_settle(tolerance_degrees, settle_time_ms)
//  set_swing_settle(tolerance_degrees, settle_time_ms)
//  set_move_to_settle(tolerance_inches, settle_time_ms)
//  reset_pid_defaults()   — restore everything from config.cpp
//
//  Per-Motion Overrides (temporary, auto-restores after the call):
//  ────────────────────────────────────────────────────────────────────
//  MoveParams:   .max_speed, .min_speed, .slew_rate, .tolerance,
//                .settle_time, .gains{kP,kI,kD}, .heading_gains, .async
//  TurnParams:   .max_speed, .min_speed, .tolerance, .settle_time,
//                .gains, .async
//  SwingParams:  .max_speed, .min_speed, .tolerance, .settle_time,
//                .gains, .async
//  MoveToParams: .max_speed, .min_speed, .lead_distance, .settle_radius,
//                .drive_tolerance, .settle_time, .drive_gains, .turn_gains,
//                .forward, .async
//
//  PID Tuning (full replacement, persistent):
//  ────────────────────────────────────────────────────────────────────
//  set_drive_pid(PIDConfig)
//  set_turn_pid(PIDConfig)
//  set_swing_pid(PIDConfig)
//  set_heading_pid(PIDConfig)
//  set_move_to_drive_pid(PIDConfig)
//  set_move_to_turn_pid(PIDConfig)
//
// ═════════════════════════════════════════════════════════════════════════════
//  DEBUGGING & TUNING NOTES
// ═════════════════════════════════════════════════════════════════════════════
//
//  If the robot doesn't reach the target:
//    → kP is too low, or tolerance is too tight for your hardware.
//    → Check for mechanical binding, dead motors, or low battery.
//
//  If the robot oscillates around the target:
//    → kD is too low (not enough damping) or kP is too high.
//    → Try increasing kD by 50% or reducing kP by 20%.
//    → Increase settle_time so it waits through the oscillation.
//
//  If the robot overshoots then slowly creeps back:
//    → kI is too high or windup_range is too large.
//    → Reduce kI or tighten the windup_range in config.cpp.
//
//  If straight-line drives curve left or right:
//    → Heading PID might be too weak. Increase heading kP.
//    → Check that motors are wired correctly (not reversed).
//    → Verify tracking wheel offsets are measured accurately.
//
//  If move_to() drives to the wrong spot:
//    → set_pose() at the start is wrong. Re-measure your starting position.
//    → Odom has drifted. Use reset_heading() after wall-squares to correct.
//    → Tracking wheel diameter or offset is slightly off. Recalibrate.
//
//  If turns are inconsistent:
//    → Battery voltage affects turn speed. Full battery turns faster.
//    → Tile surface varies — practice tiles may differ from competition.
//    → Try setting max_speed lower for consistency (80–100 instead of 127).
//
//  If the robot exits a motion too early (before fully stopped):
//    → Tolerance is too loose. Tighten it.
//    → Settle time is too short. The robot passes through the tolerance
//      zone while oscillating and the timer expires on a peak.
//      Increase settle_time to 200–400ms.
//
//  If the robot exits a motion too late (wastes time sitting still):
//    → Settle time is too long. Reduce it.
//    → Tolerance is too tight. Loosen it.
//    → The robot is settled but the timer hasn't expired yet.
//      For fast autons, 100–150ms settle time is usually enough.
//
//  Use `pros terminal` to watch Logger output in real time. Add logging
//  at critical points in your routine so you can see where things go wrong
//  without re-running the whole auton.
//
// ═════════════════════════════════════════════════════════════════════════════
}

// ─── ADD YOUR COMPETITION ROUTINES BELOW ─────────────────────────────────────
//
// Copy the structure from showcase(): reset_pid_defaults, set_pose, set
// your settle params, then chain your motion commands with timeouts.
// Delete the showcase once you have real routes.

// void left_side_awp() {
//     robot::chassis.reset_pid_defaults();
//     robot::chassis.set_pose({-48, -60, 90});
//     robot::chassis.set_drive_settle(0.5, 200);
//     robot::chassis.set_turn_settle(1.0, 150);
//
//     // ... your route ...
// }

// void skills() {
//     robot::chassis.reset_pid_defaults();
//     robot::chassis.set_pose({-60, -36, 0});
//     robot::chassis.set_drive_settle(0.3, 300);
//     robot::chassis.set_turn_settle(0.5, 200);
//     robot::chassis.set_move_to_settle(0.5, 250);
//
//     // ... your route (60 seconds, tighter tolerances) ...
// }

} // namespace auton
} // namespace robot
