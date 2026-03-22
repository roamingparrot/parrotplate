/**
 * @file units.hpp
 * @brief Unit conversion helpers for common robotics measurements.
 *
 * The library internally uses inches and degrees everywhere, but sometimes
 * you need to convert from/to other units (tiles, feet, radians, RPM, etc.)
 *
 * VEX field tile = 24 inches (2 feet).
 */

#ifndef PARROTLIB_UTILS_UNITS_HPP
#define PARROTLIB_UTILS_UNITS_HPP

namespace parrot {
namespace units {

// ─── Distance ────────────────────────────────────────────────────────────────

/** Convert VEX field tiles to inches. 1 tile = 24 inches. */
constexpr double tiles_to_inches(double tiles) { return tiles * 24.0; }

/** Convert inches to tiles. */
constexpr double inches_to_tiles(double inches) { return inches / 24.0; }

/** Convert feet to inches. */
constexpr double feet_to_inches(double feet) { return feet * 12.0; }

/** Convert centimeters to inches. */
constexpr double cm_to_inches(double cm) { return cm / 2.54; }

/** Convert millimeters to inches. */
constexpr double mm_to_inches(double mm) { return mm / 25.4; }

// ─── Velocity ────────────────────────────────────────────────────────────────

/**
 * Convert motor RPM to inches per second for a given wheel.
 * Useful for feedforward velocity estimation.
 *
 * @param rpm             Motor RPM (after internal gearing)
 * @param wheel_diameter  Wheel diameter in inches
 * @param gear_ratio      External gear ratio (driven/driving)
 */
constexpr double rpm_to_ips(double rpm, double wheel_diameter,
                            double gear_ratio = 1.0) {
    // RPM → wheel RPM → circumference/min → circumference/sec
    return (rpm * gear_ratio * wheel_diameter * 3.14159265358979) / 60.0;
}

} // namespace units
} // namespace parrot

#endif // PARROTLIB_UTILS_UNITS_HPP
