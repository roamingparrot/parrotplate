/**
 * @file slew.cpp
 * @brief Slew rate limiter implementation.
 */

#include "parrotlib/core/slew.hpp"
#include <cmath>

namespace parrot {

Slew::Slew(double rate) : rate_(rate) {}

double Slew::update(double target) {
    if (rate_ <= 0.0) {
        output_ = target;
        return output_;
    }

    double delta = target - output_;
    if (std::fabs(delta) > rate_) {
        output_ += std::copysign(rate_, delta);
    } else {
        output_ = target;
    }

    return output_;
}

void Slew::reset() {
    output_ = 0.0;
}

void Slew::set_rate(double rate) { rate_ = rate; }
double Slew::get_rate() const { return rate_; }
double Slew::get_output() const { return output_; }

} // namespace parrot
