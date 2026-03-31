/**
 * @file odometry.cpp
 * @brief Odometry engine implementation.
 *
 * The core tracking math uses the arc-based odometry approach:
 * - Compute the local displacement (forward, strafe) since the last update
 * - Rotate that displacement into the global frame using the average heading
 * - Accumulate into the global pose
 *
 * For heading: we prefer the IMU when available because it doesn't drift
 * from wheel slip. When no IMU is present, heading is derived from the
 * difference between left and right tracker distances divided by their
 * separation (track width). This works but accumulates error on turns
 * where wheels scrub.
 *
 * The math is well-documented in the Pilons odometry paper and various
 * VEX community resources. This implementation prioritizes correctness
 * and readability over micro-optimization — it runs at 100Hz on a
 * Cortex-A9, so there's headroom.
 */

#include "parrotlib/odom/odometry.hpp"
#include "parrotlib/core/math_utils.hpp"
#include "parrotlib/core/timer.hpp"
#include <cmath>

namespace parrot {

Odometry::Odometry(TrackingWheel* left, TrackingWheel* right,
                   TrackingWheel* back, int imu_port)
    : left_tracker_(left),
      right_tracker_(right),
      back_tracker_(back),
      imu_port_(imu_port) {}

bool Odometry::calibrate() {
    if (imu_port_ == 0) return false;

    pros::Imu imu(imu_port_);
    imu.reset();

    // Wait for IMU calibration with a timeout. The V5 IMU typically takes
    // about 2 seconds. We give it 3 to be safe. If it takes longer than
    // that, something is wrong (unplugged, broken, port conflict).
    uint32_t start = pros::millis();
    while (imu.is_calibrating()) {
        if (pros::millis() - start > 3000) {
            calibrated_ = false;
            return false;
        }
        parrot::delay(10);
    }

    calibrated_ = true;
    return true;
}

bool Odometry::is_calibrated() const {
    if (imu_port_ == 0) return true;  // No IMU = nothing to calibrate
    return calibrated_;
}

void Odometry::task_fn(void* param) {
    auto* self = static_cast<Odometry*>(param);
    while (self->running_) {
        self->update();
        parrot::delay(10);  // ~100Hz update rate
    }
}

void Odometry::start() {
    if (running_) return;
    running_ = true;

    // Reset encoder baselines
    if (left_tracker_) prev_left_dist_ = left_tracker_->get_distance();
    if (right_tracker_) prev_right_dist_ = right_tracker_->get_distance();
    if (back_tracker_) prev_back_dist_ = back_tracker_->get_distance();

    if (imu_port_ > 0) {
        pros::Imu imu(imu_port_);
        prev_heading_rad_ = math::deg_to_rad(imu.get_heading());
    }

    task_ = new pros::Task(task_fn, this, "parrot_odom");
}

void Odometry::stop() {
    if (!running_) return;
    running_ = false;
    // Give the task a cycle to exit cleanly
    parrot::delay(20);
    if (task_) {
        delete task_;
        task_ = nullptr;
    }
}

bool Odometry::is_running() const {
    return running_;
}

Pose Odometry::get_pose() const {
    mutex_.take(TIMEOUT_MAX);
    Pose p = pose_;
    mutex_.give();
    return p;
}

void Odometry::set_pose(Pose pose) {
    mutex_.take(TIMEOUT_MAX);
    pose_ = pose;
    mutex_.give();

    // Reset encoder baselines so delta calculations start fresh
    if (left_tracker_) {
        left_tracker_->reset();
        prev_left_dist_ = 0.0;
    }
    if (right_tracker_) {
        right_tracker_->reset();
        prev_right_dist_ = 0.0;
    }
    if (back_tracker_) {
        back_tracker_->reset();
        prev_back_dist_ = 0.0;
    }
    if (imu_port_ > 0) {
        pros::Imu imu(imu_port_);
        imu.set_heading(pose.heading);
        prev_heading_rad_ = math::deg_to_rad(pose.heading);
    }
}

void Odometry::reset() {
    set_pose({0, 0, 0});
}

void Odometry::reset_heading(double heading) {
    Pose current = get_pose();
    current.heading = heading;
    set_pose(current);
}

void Odometry::reset_position(double x, double y) {
    mutex_.take(TIMEOUT_MAX);
    pose_.x = x;
    pose_.y = y;
    mutex_.give();
}

double Odometry::distance_to(double x, double y) const {
    Pose p = get_pose();
    return math::distance(p.x, p.y, x, y);
}

double Odometry::distance_to(const Pose& target) const {
    return distance_to(target.x, target.y);
}

double Odometry::angle_to(double x, double y) const {
    Pose p = get_pose();
    return math::angle_to_point(p.x, p.y, x, y);
}

double Odometry::angle_to(const Pose& target) const {
    return angle_to(target.x, target.y);
}

double Odometry::get_imu_heading() const {
    if (imu_port_ == 0) return 0.0;
    pros::Imu imu(imu_port_);
    return imu.get_heading();
}

void Odometry::update() {
    // ── Read current encoder distances ───────────────────────────────────
    double left_dist  = left_tracker_  ? left_tracker_->get_distance()  : 0.0;
    double right_dist = right_tracker_ ? right_tracker_->get_distance() : 0.0;
    double back_dist  = back_tracker_  ? back_tracker_->get_distance()  : 0.0;

    // ── Compute deltas since last update ─────────────────────────────────
    double d_left  = left_dist  - prev_left_dist_;
    double d_right = right_dist - prev_right_dist_;
    double d_back  = back_dist  - prev_back_dist_;

    prev_left_dist_  = left_dist;
    prev_right_dist_ = right_dist;
    prev_back_dist_  = back_dist;

    // ── Determine heading change ─────────────────────────────────────────
    double heading_rad;
    double d_heading;

    if (imu_port_ > 0 && calibrated_) {
        // IMU heading — much more reliable than encoder-derived heading
        // because it doesn't drift with wheel slip.
        pros::Imu imu(imu_port_);
        double raw_heading = imu.get_heading();

        // The V5 IMU occasionally returns 0 or jumps when it glitches.
        // Sanity check: if the heading jumped more than 30° in one cycle
        // (physically impossible at 100Hz unless the robot teleported),
        // skip this update and use the previous heading.
        heading_rad = math::deg_to_rad(raw_heading);
        d_heading = heading_rad - prev_heading_rad_;

        // Wrap to [-π, π]
        while (d_heading > math::PI) d_heading -= math::TWO_PI;
        while (d_heading < -math::PI) d_heading += math::TWO_PI;

        if (std::fabs(d_heading) > math::deg_to_rad(30.0)) {
            // IMU glitch detected — fall back to encoder heading for this cycle
            if (left_tracker_ && right_tracker_) {
                double track_width = std::fabs(left_tracker_->get_offset()) +
                                     std::fabs(right_tracker_->get_offset());
                if (track_width > 0) {
                    d_heading = (d_right - d_left) / track_width;
                } else {
                    d_heading = 0.0;
                }
            } else {
                d_heading = 0.0;
            }
            heading_rad = prev_heading_rad_ + d_heading;
        }
    } else if (left_tracker_ && right_tracker_) {
        // No IMU — derive heading from left/right tracker difference.
        double track_width = std::fabs(left_tracker_->get_offset()) +
                             std::fabs(right_tracker_->get_offset());
        if (track_width > 0) {
            d_heading = (d_right - d_left) / track_width;
        } else {
            d_heading = 0.0;
        }
        heading_rad = prev_heading_rad_ + d_heading;
    } else {
        // Not enough sensors for heading. Should never happen in a real
        // setup, but we don't want to crash.
        d_heading = 0.0;
        heading_rad = prev_heading_rad_;
    }

    prev_heading_rad_ = heading_rad;

    // ── Compute local displacement ───────────────────────────────────────
    // Average heading during this interval (midpoint approximation for arcs)
    double avg_heading = prev_heading_rad_ - d_heading / 2.0;

    // Forward displacement: average of left and right, or whichever is available
    double d_forward;
    if (left_tracker_ && right_tracker_) {
        d_forward = (d_left + d_right) / 2.0;
    } else if (left_tracker_) {
        // Single vertical tracker — forward distance is the tracker delta
        // minus the arc component from turning around the offset.
        d_forward = d_left - d_heading * left_tracker_->get_offset();
    } else if (right_tracker_) {
        d_forward = d_right - d_heading * right_tracker_->get_offset();
    } else {
        d_forward = 0.0;
    }

    // Lateral (strafe) displacement from the back tracker
    double d_strafe = 0.0;
    if (back_tracker_) {
        // Subtract the arc component from the back wheel turning around its offset
        d_strafe = d_back - d_heading * back_tracker_->get_offset();
    }

    // ── Rotate local displacement into global frame ──────────────────────
    double sin_avg = std::sin(avg_heading);
    double cos_avg = std::cos(avg_heading);

    // Global coordinate update. Our convention:
    //   global X += forward * sin(heading) + strafe * cos(heading)
    //   global Y += forward * cos(heading) - strafe * sin(heading)
    // This matches X=right, Y=forward, heading=CW from +Y.
    double dx = d_forward * sin_avg + d_strafe * cos_avg;
    double dy = d_forward * cos_avg - d_strafe * sin_avg;

    // ── Update global pose ───────────────────────────────────────────────
    mutex_.take(TIMEOUT_MAX);
    pose_.x += dx;
    pose_.y += dy;
    pose_.heading = math::wrap_angle_360(math::rad_to_deg(heading_rad));
    mutex_.give();
}

} // namespace parrot
