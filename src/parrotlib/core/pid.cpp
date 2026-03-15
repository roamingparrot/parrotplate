/**
 * @file pid.cpp
 * @brief PID controller implementation.
 */

#include "parrotlib/core/pid.hpp"
#include "api.h"
#include <cmath>
#include <algorithm>

namespace parrot {

PID::PID(PIDConfig config, std::string name)
    : name(std::move(name)), config_(config) {}

double PID::update(double error) {
    error_ = error;

    // ── Proportional ─────────────────────────────────────────────────────
    double p_term = config_.gains.kP * error_;

    // ── Integral with anti-windup ────────────────────────────────────────
    // Only accumulate integral when we're close to the target. Without this,
    // the integral term builds up massive values during the approach phase
    // and causes huge overshoot. Ask me how I know.
    if (config_.windup_range == 0.0 ||
        std::fabs(error_) < config_.windup_range) {
        integral_ += error_;
    } else {
        // Outside windup range — decay the integral instead of zeroing it.
        // Hard reset causes a discontinuity that shows up as a jerk.
        integral_ *= 0.5;
    }
    double i_term = config_.gains.kI * integral_;

    // ── Derivative with optional filtering ───────────────────────────────
    double raw_derivative = error_ - prev_error_;

    if (config_.derivative_filter > 0.0) {
        // Exponential moving average. The filter coefficient controls how
        // much of the old derivative we keep. Higher = smoother but laggier.
        derivative_ = config_.derivative_filter * derivative_ +
                      (1.0 - config_.derivative_filter) * raw_derivative;
    } else {
        derivative_ = raw_derivative;
    }
    double d_term = config_.gains.kD * derivative_;

    // ── Sum and clamp ────────────────────────────────────────────────────
    double raw_output = p_term + i_term + d_term;

    // Clamp to max output range
    if (config_.output_max > 0) {
        raw_output = std::clamp(raw_output, -config_.output_max,
                                config_.output_max);
    }

    // ── Slew rate limiting ───────────────────────────────────────────────
    if (config_.slew_rate > 0.0) {
        double delta = raw_output - prev_output_;
        if (std::fabs(delta) > config_.slew_rate) {
            raw_output = prev_output_ +
                         std::copysign(config_.slew_rate, delta);
        }
    }

    prev_error_  = error_;
    prev_output_ = raw_output;
    output_      = raw_output;

    // ── Settle detection ─────────────────────────────────────────────────
    // The robot is "settled" when error stays within tolerance for the
    // required duration. We track this with a simple start-time approach
    // rather than a countdown, so it auto-resets if we leave tolerance.
    if (config_.tolerance > 0.0 && std::fabs(error_) < config_.tolerance) {
        if (!within_tolerance_) {
            within_tolerance_ = true;
            settle_start_ = pros::millis();
        }
        if (config_.settle_time > 0 &&
            (pros::millis() - settle_start_) >=
                static_cast<uint32_t>(config_.settle_time)) {
            settled_ = true;
        } else if (config_.settle_time == 0) {
            settled_ = true;
        }
    } else {
        within_tolerance_ = false;
        settled_ = false;
    }

    return output_;
}

bool PID::is_settled() const {
    return settled_;
}

void PID::reset() {
    error_        = 0.0;
    prev_error_   = 0.0;
    integral_     = 0.0;
    derivative_   = 0.0;
    output_       = 0.0;
    prev_output_  = 0.0;
    settle_start_ = 0;
    within_tolerance_ = false;
    settled_      = false;
}

void PID::set_gains(PIDGains gains) {
    config_.gains = gains;
}

void PID::set_config(PIDConfig config) {
    config_ = config;
    reset();
}

PIDGains PID::get_gains() const { return config_.gains; }
PIDConfig PID::get_config() const { return config_; }
double PID::get_output() const { return output_; }
double PID::get_error() const { return error_; }

} // namespace parrot
