/**
 * @file timer.cpp
 * @brief Timer utility implementation.
 */

#include "parrotlib/core/timer.hpp"
#include "api.h"

namespace parrot {

void delay(uint32_t ms) {
    pros::delay(ms);
}

Timer::Timer() {}

void Timer::start() {
    start_time_ = pros::millis();
    mark_time_  = start_time_;
    running_    = true;
}

void Timer::reset() {
    start_time_ = 0;
    mark_time_  = 0;
    running_    = false;
}

uint32_t Timer::get_elapsed() const {
    if (!running_) return 0;
    return pros::millis() - start_time_;
}

bool Timer::expired(uint32_t ms) const {
    return running_ && (get_elapsed() >= ms);
}

void Timer::mark() {
    mark_time_ = pros::millis();
}

uint32_t Timer::get_since_mark() const {
    return pros::millis() - mark_time_;
}

} // namespace parrot
