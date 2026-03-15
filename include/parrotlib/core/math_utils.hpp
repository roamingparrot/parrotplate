/**
 * @file math_utils.hpp
 * @brief Math helpers for robotics geometry and angle manipulation.
 *
 * These exist because standard library trig works in radians and doesn't
 * know about heading wrapping, and we deal in degrees and field coordinates
 * constantly. Every function here has been needed at least once during
 * a real competition season.
 *
 * Coordinate system (field-standard):
 *   - X axis: positive to the RIGHT
 *   - Y axis: positive FORWARD
 *   - Heading 0°: facing +Y (forward)
 *   - Heading increases CLOCKWISE (compass-style)
 *   - All angles in DEGREES unless a function name says "rad"
 *   - All distances in INCHES
 */

#ifndef PARROTLIB_CORE_MATH_UTILS_HPP
#define PARROTLIB_CORE_MATH_UTILS_HPP

namespace parrot {
namespace math {

// ─── Constants ───────────────────────────────────────────────────────────────

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double HALF_PI = PI / 2.0;

// ─── Angle Conversion ────────────────────────────────────────────────────────

/** Degrees to radians. */
double deg_to_rad(double deg);

/** Radians to degrees. */
double rad_to_deg(double rad);

// ─── Angle Wrapping ──────────────────────────────────────────────────────────

/**
 * Wrap an angle to [-180, 180). This is the one you want for heading error
 * calculations — always gives the shortest turn direction.
 */
double wrap_angle_180(double angle);

/**
 * Wrap an angle to [0, 360). Useful for display and logging where you
 * don't want negative headings.
 */
double wrap_angle_360(double angle);

/**
 * Find the shortest signed angular difference from `current` to `target`.
 * Result is in [-180, 180). Positive = turn clockwise.
 *
 * This is the function you want for "how far do I need to turn?"
 */
double angle_error(double target, double current);

// ─── Distance & Geometry ─────────────────────────────────────────────────────

/**
 * Euclidean distance between two points. Simple but you'd be surprised
 * how often you fat-finger the sqrt when it's 2AM at worlds.
 */
double distance(double x1, double y1, double x2, double y2);

/**
 * Absolute angle from (x1, y1) to (x2, y2) in degrees [0, 360).
 * Returns the heading you'd need to face to point at the target.
 * Uses our coordinate convention (0° = +Y, CW positive).
 *
 * Note: returns 0 if the points are identical (no division by zero).
 */
double angle_to_point(double x1, double y1, double x2, double y2);

// ─── Clamping & Utility ──────────────────────────────────────────────────────

/** Clamp value to [-limit, limit]. */
double clamp(double value, double limit);

/** Clamp value to [low, high]. */
double clamp(double value, double low, double high);

/** Sign function: returns -1, 0, or 1. */
double sign(double value);

/**
 * Linear interpolation. t=0 returns a, t=1 returns b.
 * Not clamped — extrapolation is sometimes intentional.
 */
double lerp(double a, double b, double t);

} // namespace math
} // namespace parrot

#endif // PARROTLIB_CORE_MATH_UTILS_HPP
