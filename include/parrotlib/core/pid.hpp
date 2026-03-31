/**
 * @file pid.hpp
 * @brief General-purpose PID controller with competition-ready features.
 *
 * This PID implementation is designed around the realities of VEX V5
 * competitive robotics. It includes slew rate limiting, integral anti-windup,
 * derivative filtering, and a settle detection system — all features that
 * matter when you're tuning under pressure at a tournament.
 *
 * Typical usage pattern:
 * @code
 *   parrot::PID drive_pid({
 *       .gains = {4.5, 0.0, 35.0},
 *       .tolerance = 0.5,       // inches
 *       .settle_time = 200,     // ms — must stay within tolerance this long
 *       .output_max = 127,
 *   });
 *
 *   while (!drive_pid.is_settled()) {
 *       double error = target - current_position;
 *       double output = drive_pid.update(error);
 *       motors.move(output);
 *       parrot::delay(10);
 *   }
 * @endcode
 *
 * Coordinate convention: positive error → positive output.
 * The caller is responsible for sign conventions on their end.
 */

#ifndef PARROTLIB_CORE_PID_HPP
#define PARROTLIB_CORE_PID_HPP

#include <string>

namespace parrot {

/**
 * PID gain constants. Keep these in a struct so they're easy to swap
 * between presets (e.g. drive vs. turn) and log during tuning.
 */
struct PIDGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
};

/**
 * Full PID configuration. Designed so you can use C++17 designated
 * initializers and only specify the fields you care about.
 */
struct PIDConfig {
    PIDGains gains = {};

    /**
     * Maximum output change per update cycle. Set to 0 to disable.
     * Prevents voltage spikes that jerk the robot — especially useful
     * on heavy drivetrains that skip wheels under aggressive accel.
     */
    double slew_rate = 0.0;

    /**
     * Integral only accumulates when |error| < windup_range.
     * Set to 0 to always accumulate (not recommended for drive PID).
     * A good starting value is ~80% of your typical steady-state error.
     */
    double windup_range = 0.0;

    /** Clamp output magnitude. Usually 127 for motor voltage. */
    double output_max = 127.0;

    /**
     * The controller considers itself "settled" when |error| stays
     * below this value for at least settle_time ms.
     * Units depend on what you're controlling — inches for drive,
     * degrees for turns.
     */
    double tolerance = 0.0;

    /**
     * How long (ms) the error must remain within tolerance before
     * is_settled() returns true. 200–400ms is typical.
     * Too short and you'll exit early on oscillation peaks.
     * Too long and autonomous wastes time waiting.
     */
    int settle_time = 0;

    /**
     * Low-pass filter on the derivative term [0.0, 1.0).
     * 0 = no filtering, 0.5 = moderate smoothing.
     * Helps when sensor noise causes derivative spikes — common with
     * tracking wheels on rough field tiles.
     */
    double derivative_filter = 0.0;
};

/**
 * @class PID
 * @brief Discrete PID controller for competition robotics.
 *
 * Call update() once per control loop iteration (~10ms is standard on V5).
 * The controller tracks its own settle state internally.
 */
class PID {
public:
    /**
     * @param config  Full PID configuration (gains, limits, settle params)
     * @param name    Human-readable name for debug output ("Drive", "Turn", etc.)
     */
    explicit PID(PIDConfig config, std::string name = "PID");

    /**
     * Compute one PID iteration.
     *
     * @param error  Current error value (target - actual). Positive error
     *               produces positive output by convention.
     * @return Clamped, slew-limited PID output in [-output_max, output_max].
     */
    double update(double error);

    /**
     * Has the controller settled? True when |error| has been within
     * tolerance for at least settle_time milliseconds continuously.
     * Resets if error exceeds tolerance again.
     */
    bool is_settled() const;

    /** Zero all internal state — call this between motions. */
    void reset();

    /** Hot-swap gains without resetting state. Useful for live tuning. */
    void set_gains(PIDGains gains);

    /** Replace the entire config. Also resets internal state. */
    void set_config(PIDConfig config);

    PIDGains get_gains() const;
    PIDConfig get_config() const;
    double get_output() const;
    double get_error() const;

    /** Name for identification in debug logs. */
    std::string name;

private:
    PIDConfig config_;
    double error_        = 0.0;
    double prev_error_   = 0.0;
    double integral_     = 0.0;
    double derivative_   = 0.0;
    double output_       = 0.0;
    double prev_output_  = 0.0;

    // Settle tracking
    uint32_t settle_start_ = 0;
    bool within_tolerance_  = false;
    bool settled_           = false;
};

} // namespace parrot

#endif // PARROTLIB_CORE_PID_HPP
