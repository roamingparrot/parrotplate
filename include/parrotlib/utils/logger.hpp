/**
 * @file logger.hpp
 * @brief Lightweight debug logger for terminal output.
 *
 * Outputs to the PROS terminal (visible via `pros terminal` over USB).
 * This is your primary debugging tool at competitions since you can't
 * step through code on the V5 brain.
 *
 * Uses printf-style formatting because that's what most VEX students
 * are familiar with and it's lightweight on the V5 platform.
 *
 * @code
 *   parrot::Logger::info("Drive", "Moving forward %.1f inches", 24.0);
 *   parrot::Logger::warn("IMU", "Heading jumped %.1f degrees", delta);
 *   parrot::Logger::error("Odom", "Tracking wheel disconnected on port %d", 7);
 * @endcode
 */

#ifndef PARROTLIB_UTILS_LOGGER_HPP
#define PARROTLIB_UTILS_LOGGER_HPP

#include <cstdio>

namespace parrot {

class Logger {
public:
    /** Log levels control what gets printed. Set higher to reduce noise. */
    enum class Level {
        DEBUG = 0,
        INFO  = 1,
        WARN  = 2,
        ERROR = 3,
        NONE  = 4,  // Suppress all output
    };

    /** Set the minimum log level. Messages below this level are dropped. */
    static void set_level(Level level);
    static Level get_level();

    /**
     * Printf-style logging functions. Tag is a short label like "Drive"
     * or "PID" that helps you filter output when multiple systems are
     * logging simultaneously.
     */
    static void debug(const char* tag, const char* fmt, ...);
    static void info(const char* tag, const char* fmt, ...);
    static void warn(const char* tag, const char* fmt, ...);
    static void error(const char* tag, const char* fmt, ...);

private:
    static Level current_level_;
    static void log(Level level, const char* prefix, const char* tag,
                    const char* fmt, __builtin_va_list args);
};

} // namespace parrot

#endif // PARROTLIB_UTILS_LOGGER_HPP
