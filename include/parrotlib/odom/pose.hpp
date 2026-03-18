/**
 * @file pose.hpp
 * @brief 2D pose representation for field-relative position tracking.
 *
 * A Pose represents the robot's position and heading on the field.
 * This is the fundamental data type for odometry and motion planning.
 *
 * Coordinate convention (matches field-standard):
 *   - x: lateral position (inches, +right)
 *   - y: forward position (inches, +forward)
 *   - heading: orientation (degrees, 0° = +Y, clockwise positive)
 *
 * The origin (0, 0) is wherever the odometry was last reset — typically
 * the robot's starting position for the autonomous routine.
 */

#ifndef PARROTLIB_ODOM_POSE_HPP
#define PARROTLIB_ODOM_POSE_HPP

#include <string>

namespace parrot {

struct Pose {
    double x       = 0.0;  // inches, +right
    double y       = 0.0;  // inches, +forward
    double heading = 0.0;  // degrees, 0=forward, CW+

    Pose() = default;
    Pose(double x, double y, double heading = 0.0);

    /** Euclidean distance to another pose (ignores heading). */
    double distance_to(const Pose& other) const;

    /**
     * Absolute heading from this pose to another, in degrees [0, 360).
     * Useful for "what direction should I face to drive toward that point?"
     */
    double angle_to(const Pose& other) const;

    /** Format as "(x, y, heading°)" for debug printing. */
    std::string to_string() const;

    // Arithmetic for pose math (used internally by odom and motion)
    Pose operator+(const Pose& other) const;
    Pose operator-(const Pose& other) const;
    Pose operator*(double scalar) const;
};

} // namespace parrot

#endif // PARROTLIB_ODOM_POSE_HPP
