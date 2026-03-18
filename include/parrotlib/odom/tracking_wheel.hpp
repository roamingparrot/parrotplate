/**
 * @file tracking_wheel.hpp
 * @brief Abstraction over different encoder types for odometry tracking.
 *
 * VEX teams use a variety of encoder setups for odometry:
 *   - Dedicated tracking wheels with V5 rotation sensors
 *   - Integrated motor encoders (less accurate but fewer parts)
 *   - ADI (legacy) encoders on older tracking wheel designs
 *
 * This class wraps all of them behind a common interface so the odometry
 * engine doesn't care what hardware you're running. You configure the
 * wheel diameter and gear ratio once, and get_distance() returns inches.
 *
 * @code
 *   // Tracking wheel with V5 rotation sensor on port 1, 2.75" wheel
 *   parrot::TrackingWheel left_tracker(1, 2.75);
 *
 *   // Using integrated motor encoders (port 3, 3.25" wheel, 36:60 gear ratio)
 *   parrot::TrackingWheel drive_tracker(3, 3.25, 36.0/60.0, true);
 * @endcode
 */

#ifndef PARROTLIB_ODOM_TRACKING_WHEEL_HPP
#define PARROTLIB_ODOM_TRACKING_WHEEL_HPP

#include "api.h"
#include <memory>

namespace parrot {

class TrackingWheel {
public:
    /**
     * Create a tracking wheel using a V5 Rotation Sensor.
     *
     * @param port           V5 smart port (1-21). Negative = reversed.
     * @param wheel_diameter Diameter in inches. Common sizes: 2.75", 3.25", 4".
     * @param offset         Distance from tracking center to this wheel, in inches.
     *                       Positive = right of center (for horizontal wheels: forward of center).
     *                       This offset is critical for accurate turning — measure carefully.
     * @param gear_ratio     External gear ratio (driven/driving). 1.0 if direct drive.
     */
    TrackingWheel(int port, double wheel_diameter, double offset,
                  double gear_ratio = 1.0);

    /**
     * Create a tracking wheel using integrated motor encoders.
     *
     * Less accurate than dedicated trackers because of backlash and gear slop,
     * but workable for teams that can't fit tracking wheels.
     *
     * @param port           V5 smart port (1-21). Negative = reversed.
     * @param wheel_diameter Diameter in inches.
     * @param offset         Distance from center.
     * @param gear_ratio     Motor cartridge * external ratio. For a 200rpm motor
     *                       with 36:60 gears: (200.0/360.0) * (36.0/60.0).
     * @param is_motor       Must be true — disambiguates from rotation sensor constructor.
     */
    TrackingWheel(int port, double wheel_diameter, double offset,
                  double gear_ratio, bool is_motor);

    /**
     * Get total distance traveled in inches since last reset.
     * Sign follows the direction of rotation (positive = forward/right
     * depending on mounting orientation).
     */
    double get_distance() const;

    /** Get the raw encoder position in degrees. */
    double get_raw_degrees() const;

    /** Zero the encoder. Call during odometry reset. */
    void reset();

    /** Offset from tracking center, in inches. */
    double get_offset() const;

    /** Check if the sensor is actually connected and returning data. */
    bool is_connected() const;

private:
    // We use raw PROS objects rather than smart pointers because the
    // V5 sensor lifetime is tied to the port, not our object lifetime.
    int port_;
    double wheel_diameter_;
    double offset_;
    double gear_ratio_;
    bool is_motor_;

    // Precomputed: inches per degree of encoder rotation
    double inches_per_degree_;
};

} // namespace parrot

#endif // PARROTLIB_ODOM_TRACKING_WHEEL_HPP
