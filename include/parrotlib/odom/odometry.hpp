/**
 * @file odometry.hpp
 * @brief Field-relative position tracking engine.
 *
 * Fuses tracking wheel encoders and an IMU to maintain a global (x, y, heading)
 * estimate of the robot's position on the field. This is the foundation for
 * point-to-point autonomous movement.
 *
 * Supported configurations (in order of accuracy):
 *   1. Two perpendicular tracking wheels + IMU (recommended)
 *   2. Three tracking wheels (two parallel + one perpendicular, no IMU)
 *   3. Integrated motor encoders + IMU (least accurate, but works in a pinch)
 *
 * The odometry runs in its own PROS task at ~10ms intervals. Call start()
 * during initialize() and the tracking happens automatically.
 *
 * Important: the IMU takes ~2 seconds to calibrate on startup. The odometry
 * task handles this internally — heading data will be garbage until
 * calibration finishes. is_calibrated() lets you check.
 *
 * @code
 *   // Two tracking wheels + IMU setup
 *   parrot::TrackingWheel left(1, 2.75, -2.5);   // 2.5" left of center
 *   parrot::TrackingWheel back(2, 2.75, -4.0);    // 4" behind center
 *   parrot::Odometry odom(&left, nullptr, &back, 10);  // IMU on port 10
 *
 *   odom.calibrate();  // call in initialize()
 *   odom.start();
 *
 *   // Later...
 *   parrot::Pose pos = odom.get_pose();
 *   printf("At (%.1f, %.1f) heading %.1f°\n", pos.x, pos.y, pos.heading);
 * @endcode
 */

#ifndef PARROTLIB_ODOM_ODOMETRY_HPP
#define PARROTLIB_ODOM_ODOMETRY_HPP

#include "parrotlib/odom/pose.hpp"
#include "parrotlib/odom/tracking_wheel.hpp"
#include "api.h"

namespace parrot {

class Odometry {
public:
    /**
     * Construct odometry with tracking wheels and optional IMU.
     *
     * Pass nullptr for any tracking wheel you don't have. At minimum you
     * need either (left + right) or (left/right + back) + IMU.
     *
     * @param left      Left vertical tracking wheel (or nullptr)
     * @param right     Right vertical tracking wheel (or nullptr)
     * @param back      Horizontal (rear) tracking wheel (or nullptr)
     * @param imu_port  V5 port for IMU (0 = no IMU, use 3-wheel odom only)
     */
    Odometry(TrackingWheel* left, TrackingWheel* right,
             TrackingWheel* back, int imu_port = 0);

    /**
     * Calibrate the IMU. This blocks for ~2 seconds while the gyro
     * initializes. Call this in initialize(), NOT in autonomous().
     * The robot must be stationary during calibration.
     *
     * Returns false if no IMU is configured or calibration fails.
     */
    bool calibrate();

    /** Is the IMU done calibrating? Safe to start tracking. */
    bool is_calibrated() const;

    /**
     * Start the background tracking task. Call after calibrate().
     * The task runs at ~100Hz and updates the internal pose.
     * Calling start() again is harmless (it's idempotent).
     */
    void start();

    /** Stop the background task. Rarely needed. */
    void stop();

    /** Is the tracking task currently running? */
    bool is_running() const;

    // ─── Pose access ─────────────────────────────────────────────────────

    /** Get current estimated pose. Thread-safe. */
    Pose get_pose() const;

    /**
     * Force-set the current pose. Useful at the start of autonomous
     * to set the robot's known starting position on the field.
     *
     * Also resets all encoder baselines so tracking continues
     * smoothly from the new position.
     */
    void set_pose(Pose pose);

    /** Shorthand: zero everything. Same as set_pose({0, 0, 0}). */
    void reset();

    /** Reset just the heading. Position (x, y) is preserved. */
    void reset_heading(double heading = 0.0);

    /** Reset just position. Heading is preserved. */
    void reset_position(double x = 0.0, double y = 0.0);

    // ─── Convenience helpers ─────────────────────────────────────────────

    /** Distance from current position to a target point (inches). */
    double distance_to(double x, double y) const;
    double distance_to(const Pose& target) const;

    /** Heading from current position to a target point (degrees). */
    double angle_to(double x, double y) const;
    double angle_to(const Pose& target) const;

    /** Get the raw IMU heading in degrees (bypasses odom, useful for debug). */
    double get_imu_heading() const;

private:
    // Tracking hardware
    TrackingWheel* left_tracker_;
    TrackingWheel* right_tracker_;
    TrackingWheel* back_tracker_;
    int imu_port_;

    // State
    Pose pose_;
    double prev_left_dist_   = 0.0;
    double prev_right_dist_  = 0.0;
    double prev_back_dist_   = 0.0;
    double prev_heading_rad_ = 0.0;

    // Task management
    pros::Task* task_ = nullptr;
    bool running_     = false;
    bool calibrated_  = false;

    // Mutex for thread-safe pose access (odom task vs. user code)
    mutable pros::Mutex mutex_;

    /** The actual tracking math, called every ~10ms by the background task. */
    void update();

    /** Static task trampoline — PROS tasks need a C-style function pointer. */
    static void task_fn(void* param);
};

} // namespace parrot

#endif // PARROTLIB_ODOM_ODOMETRY_HPP
