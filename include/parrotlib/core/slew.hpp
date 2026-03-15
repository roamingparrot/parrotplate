/**
 * @file slew.hpp
 * @brief Slew rate limiter for smooth voltage ramps.
 *
 * Limits how fast a value can change per update cycle. In practice this
 * prevents the drivetrain from slamming to full power instantly, which
 * causes wheel slip on slick tiles and burns out motors on heavy bots.
 *
 * The PID class has slew built in, but this standalone version is useful
 * for things like flywheel spinup, lift control, or intake ramping where
 * you're not running a full PID loop.
 *
 * @code
 *   parrot::Slew intake_slew(15.0);  // max 15 units/cycle
 *   double ramped = intake_slew.update(127);  // ramps toward 127
 * @endcode
 */

#ifndef PARROTLIB_CORE_SLEW_HPP
#define PARROTLIB_CORE_SLEW_HPP

namespace parrot {

class Slew {
public:
    /**
     * @param rate  Max change per update. 0 = disabled (passthrough).
     *              Good starting values: 10-20 for drivetrain, 5-10 for lifts.
     */
    explicit Slew(double rate = 0.0);

    /**
     * Apply slew rate limiting to a target value.
     * @param target  Desired output value.
     * @return Slew-limited output.
     */
    double update(double target);

    /** Reset internal state to 0. Call between motions. */
    void reset();

    void set_rate(double rate);
    double get_rate() const;
    double get_output() const;

private:
    double rate_;
    double output_ = 0.0;
};

} // namespace parrot

#endif // PARROTLIB_CORE_SLEW_HPP
