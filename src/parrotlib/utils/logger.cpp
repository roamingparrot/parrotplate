/**
 * @file logger.cpp
 * @brief Logger implementation.
 */

#include "parrotlib/utils/logger.hpp"
#include <cstdarg>
#include <cstdio>

namespace parrot {

Logger::Level Logger::current_level_ = Logger::Level::INFO;

void Logger::set_level(Level level) { current_level_ = level; }
Logger::Level Logger::get_level() { return current_level_; }

void Logger::log(Level level, const char* prefix, const char* tag,
                 const char* fmt, __builtin_va_list args) {
    if (level < current_level_) return;

    printf("[%s][%s] ", prefix, tag);
    vprintf(fmt, args);
    printf("\n");
}

void Logger::debug(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log(Level::DEBUG, "DBG", tag, fmt, args);
    va_end(args);
}

void Logger::info(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log(Level::INFO, "INF", tag, fmt, args);
    va_end(args);
}

void Logger::warn(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log(Level::WARN, "WRN", tag, fmt, args);
    va_end(args);
}

void Logger::error(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log(Level::ERROR, "ERR", tag, fmt, args);
    va_end(args);
}

} // namespace parrot
