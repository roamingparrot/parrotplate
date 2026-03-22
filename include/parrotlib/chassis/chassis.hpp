/**
 * @file chassis.hpp
 * @brief Main Chassis class — the primary user-facing API for drivetrain control.
 *
 * This is the "one object to rule them all" for your drivetrain. It owns
 * the motors (Drivetrain), position tracking (Odometry), and PID controllers,
 * providing a clean high-level interface for both autonomous and driver control.
 *
 * Typical lifecycle:
 *   1. Construct with ChassisConfig in initialize()
 *   2. Call calibrate() (blocks ~2s for IMU)
 *   3. In autonomous: use move_forward(), turn_to(), move_to(), etc.
 *   4. In opcontrol: call arcade() or tank() in the control loop
 *
 * Motion commands are blocking by default — they don't return until the
 * robot has settled or the timeout expires. Set async=true in the params
 * struct to return immediately and let the motion run in the background.
 *
 * @code
 *   // In initialize():
 *   parrot::Chassis chassis(config);
 *   chassis.calibrate();
 *
 *   // In autonomous:
 *   chassis.set_pose({0, 0, 0});
 *   chassis.move_forward(24);               // drive 24 inches
 *   chassis.turn_to(90);                    // face 90 degrees
 *   chassis.move_to({24, 24}, 3000);        // go to (24,24), 3s timeout
 *   chassis.swing_to(180, parrot::Side::LEFT);  // swing turn
 *
 *   // In opcontrol:
 *   chassis.arcade(controller.get_analog(ANALOG_LEFT_Y),
 *                  controller.get_analog(ANALOG_RIGHT_X));
 * @endcode
 */

#ifndef PARROTLIB_CHASSIS_CHASSIS_HPP
#define PARROTLIB_CHASSIS_CHASSIS_HPP

#include "parrotlib/chassis/chassis_config.hpp"
#include "parrotlib/chassis/drivetrain.hpp"
#include "parrotlib/odom/odometry.hpp"
#include "parrotlib/core/pid.hpp"
#include "parrotlib/core/timer.hpp"
#include <functional>

namespace parrot {

class Chassis {
public:
    explicit Chassis(ChassisConfig config);

    // ─── Initialization ──────────────────────────────────────────────────

    /**
     * Calibrate the IMU and start odometry tracking.
     * Blocks for ~2 seconds. Call in initialize().
     * The robot MUST be stationary.
     */
    void calibrate();

    // ─── Driver Control ──────────────────────────────────────────────────

    /**
     * Arcade drive using the configured curve and deadzone.
     * Call this in your opcontrol loop.
     */
    void arcade(double forward, double turn);

    /** Tank drive using the configured curve and deadzone. */
    void tank(double left, double right);

    // ─── Autonomous Movement ─────────────────────────────────────────────

    /**
     * Drive forward a given distance in inches.
     * Heading correction keeps the robot driving straight.
     *
     * @param distance  Distance in inches (positive = forward)
     * @param timeout   Max time in ms (0 = no timeout). Always set a timeout
     *                  in competition code — you don't want a stuck PID loop
     *                  burning your remaining autonomous time.
     * @param params    Optional per-motion overrides (speed, slew, async)
     */
    void move_forward(double distance, int timeout = 0,
                      MoveParams params = {});

    /**
     * Drive backward. Convenience wrapper — equivalent to
     * move_forward(-distance, ...) but reads better in auton routines.
     */
    void move_backward(double distance, int timeout = 0,
                       MoveParams params = {});

    /**
     * Turn in place to face an absolute heading.
     * Uses the shortest rotation direction automatically.
     *
     * @param heading  Target heading in degrees (field-absolute).
     *                 0 = starting forward direction. CW positive.
     * @param timeout  Max time in ms (0 = no timeout)
     * @param params   Optional per-motion overrides
     */
    void turn_to(double heading, int timeout = 0,
                 TurnParams params = {});

    /**
     * Swing turn: one side drives, the other holds still.
     * Creates a wider, smoother turn arc than a point turn.
     * Often faster for large heading changes when you have room.
     *
     * @param heading  Target heading (degrees, absolute)
     * @param side     Which side stays stationary (LEFT or RIGHT)
     * @param timeout  Max time in ms
     * @param params   Optional per-motion overrides
     */
    void swing_to(double heading, Side side, int timeout = 0,
                  SwingParams params = {});

    /**
     * Drive to an (x, y) coordinate on the field.
     * Combines heading and distance PID to follow a smooth path.
     *
     * The lead_distance parameter in MoveToParams controls the "carrot"
     * distance — how far ahead the robot aims. Smaller values track
     * tighter but oscillate more. 6-12 inches is a good range.
     *
     * @param x, y     Target coordinates in inches
     * @param timeout  Max time in ms
     * @param params   Optional per-motion overrides
     */
    void move_to(double x, double y, int timeout = 0,
                 MoveToParams params = {});

    /** Overload: pass a Pose (heading component is ignored). */
    void move_to(Pose target, int timeout = 0,
                 MoveToParams params = {});

    // ─── Motion State ────────────────────────────────────────────────────

    /**
     * Has the current motion finished?
     * True if no motion is running, or if the PID has settled / timed out.
     */
    bool is_settled() const;

    /** Block until the current async motion completes. */
    void wait_until_settled();

    /**
     * Immediately cancel any running motion and stop the motors.
     * Useful for "oh no" moments in autonomous, or for teleop takeover.
     */
    void cancel_motion();

    // ─── Pose Access ─────────────────────────────────────────────────────

    /** Get current position estimate from odometry. */
    Pose get_pose() const;

    /** Set the current pose (typically at autonomous start). */
    void set_pose(Pose pose);

    /** Reset everything to (0, 0, 0°). */
    void reset_pose();

    /** Reset heading only. Position is preserved. */
    void reset_heading(double heading = 0.0);

    // ─── PID Tuning (live) ───────────────────────────────────────────────
    // Full PID config replacement — use for complete re-tuning at runtime.

    void set_drive_pid(PIDConfig config);
    void set_turn_pid(PIDConfig config);
    void set_swing_pid(PIDConfig config);
    void set_heading_pid(PIDConfig config);
    void set_move_to_drive_pid(PIDConfig config);
    void set_move_to_turn_pid(PIDConfig config);

    // ─── Settle Tuning (per-auton convenience) ──────────────────────────
    // These let you change settle behavior at the top of an autonomous
    // routine without rebuilding a full PIDConfig. Handy when one auton
    // needs tighter tolerances than another.
    //
    //   void auton::left_awp() {
    //       chassis.set_drive_settle(0.3, 250);   // tighter for this route
    //       chassis.set_turn_settle(0.8, 150);
    //       // ... motions use these settle params until changed again ...
    //   }

    /** Override just the settle params for the drive PID. */
    void set_drive_settle(double tolerance, int settle_time);

    /** Override just the settle params for the turn PID. */
    void set_turn_settle(double tolerance, int settle_time);

    /** Override just the settle params for the swing PID. */
    void set_swing_settle(double tolerance, int settle_time);

    /** Override settle params for the move_to drive PID. */
    void set_move_to_settle(double tolerance, int settle_time);

    /**
     * Reset all PID configs back to the values from the original ChassisConfig.
     * Useful at the top of an auton to ensure a known starting state
     * after a previous auton may have tweaked things.
     */
    void reset_pid_defaults();

    // ─── Internal Access (for GUI / debug) ───────────────────────────────

    /** Direct access to the underlying drivetrain (motor control). */
    Drivetrain& get_drivetrain();

    /** Direct access to the odometry system. */
    Odometry& get_odometry();

    /** Access to the drive PID for monitoring (e.g. GUI plot). */
    const PID& get_drive_pid() const;
    const PID& get_turn_pid() const;

private:
    ChassisConfig config_;
    Drivetrain drivetrain_;
    Odometry odom_;

    // PID controllers — one per motion axis
    PID drive_pid_;
    PID turn_pid_;
    PID swing_pid_;
    PID heading_pid_;
    PID move_to_drive_pid_;
    PID move_to_turn_pid_;

    Timer motion_timer_;
    bool motion_running_ = false;

    // Motion task for async movements
    pros::Task* motion_task_ = nullptr;

    /** Run a blocking motion loop. Handles timeout and settle detection. */
    void run_motion(std::function<bool()> update_fn, int timeout);

    /** Stop motors and clean up motion state. */
    void end_motion();

    /** Create motor-encoder-based tracking wheels for odom fallback. */
    TrackingWheel* left_motor_tracker_  = nullptr;
    TrackingWheel* right_motor_tracker_ = nullptr;
};

} // namespace parrot

#endif // PARROTLIB_CHASSIS_CHASSIS_HPP
