/**
 * @file timer.hpp
 * @brief Lightweight timer utility for timeouts and profiling.
 *
 * Wraps pros::millis() into a convenient timer that tracks elapsed time
 * and supports placemark-based duration checks. Essential for timeout
 * protection in autonomous motions — you never want a PID loop running
 * forever because a robot got stuck on a field element.
 *
 * @code
 *   parrot::Timer timer;
 *   timer.start();
 *
 *   while (!pid.is_settled()) {
 *       if (timer.get_elapsed() > 2000) {
 *           // Bail — we've been trying for 2 seconds
 *           break;
 *       }
 *       // ... PID loop body ...
 *   }
 * @endcode
 */

#ifndef PARROTLIB_CORE_TIMER_HPP
#define PARROTLIB_CORE_TIMER_HPP

#include <cstdint>

namespace parrot {

class Timer {
public:
    Timer();

    /** Start or restart the timer from now. */
    void start();

    /** Reset to zero without starting. */
    void reset();

    /** Milliseconds elapsed since last start(). Returns 0 if never started. */
    uint32_t get_elapsed() const;

    /** Convenience: has more than `ms` milliseconds elapsed? */
    bool expired(uint32_t ms) const;

    /** Set a placemark — get_since_mark() measures from this point. */
    void mark();

    /** Milliseconds since last mark(). */
    uint32_t get_since_mark() const;

private:
    uint32_t start_time_ = 0;
    uint32_t mark_time_  = 0;
    bool running_        = false;
};

} // namespace parrot

#endif // PARROTLIB_CORE_TIMER_HPP
