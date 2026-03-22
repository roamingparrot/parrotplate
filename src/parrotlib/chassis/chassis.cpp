/**
 * @file chassis.cpp
 * @brief Chassis class implementation — high-level motion control.
 *
 * Each motion command follows the same pattern:
 *   1. Snapshot the active PID config
 *   2. Apply any per-motion overrides from the params struct
 *   3. Reset and run the control loop until settled or timed out
 *   4. Restore the PID config so the next motion starts clean
 *
 * Per-motion overrides use -1 as a sentinel meaning "use current default."
 * This way callers only specify the fields they care about:
 *
 *   chassis.move_forward(24, 2000, {.tolerance = 0.25, .max_speed = 80});
 *
 * For per-auton tuning, use set_drive_settle() / set_turn_settle() etc.
 * at the top of each routine, or reset_pid_defaults() to go back to the
 * original ChassisConfig values.
 */

#include "parrotlib/chassis/chassis.hpp"
#include "parrotlib/core/math_utils.hpp"
#include <cmath>

namespace parrot {

// ─── Helpers for per-motion PID overrides ────────────────────────────────────

/**
 * Apply per-motion overrides to a PID config. Fields with value -1
 * are left untouched (use the existing default). Returns the config
 * that was active before overrides, so the caller can restore it.
 */
static PIDConfig apply_overrides(PID& pid, PIDGains gains,
                                 double tolerance, int settle_time,
                                 double slew_rate = -1) {
    PIDConfig original = pid.get_config();
    PIDConfig tweaked = original;

    if (gains.kP >= 0) tweaked.gains.kP = gains.kP;
    if (gains.kI >= 0) tweaked.gains.kI = gains.kI;
    if (gains.kD >= 0) tweaked.gains.kD = gains.kD;
    if (tolerance >= 0) tweaked.tolerance = tolerance;
    if (settle_time >= 0) tweaked.settle_time = settle_time;
    if (slew_rate >= 0) tweaked.slew_rate = slew_rate;

    pid.set_config(tweaked);
    return original;
}

static void restore_config(PID& pid, const PIDConfig& original) {
    pid.set_config(original);
}

// ─── Constructor ─────────────────────────────────────────────────────────────

Chassis::Chassis(ChassisConfig config)
    : config_(config),
      drivetrain_(config.left_motors, config.right_motors,
                  config.motor_rpm, config.gear_ratio, config.wheel_diameter),
      odom_(config.left_tracker, config.right_tracker,
            config.back_tracker, config.imu_port),
      drive_pid_(config.drive_pid, "Drive"),
      turn_pid_(config.turn_pid, "Turn"),
      swing_pid_(config.swing_pid, "Swing"),
      heading_pid_(config.heading_pid, "Heading"),
      move_to_drive_pid_(config.move_to_drive_pid, "MoveTo Drive"),
      move_to_turn_pid_(config.move_to_turn_pid, "MoveTo Turn") {

    // If no tracking wheels provided, create motor-encoder-based trackers
    // as a fallback. Less accurate but works for teams without tracking
    // wheel hardware.
    if (!config.left_tracker && !config.right_tracker) {
        double half_track = 5.5;  // rough default — tune via track_width in config
        left_motor_tracker_ = new TrackingWheel(
            std::abs(config.left_motors.front()),
            config.wheel_diameter, -half_track,
            config.gear_ratio, true);
        right_motor_tracker_ = new TrackingWheel(
            std::abs(config.right_motors.front()),
            config.wheel_diameter, half_track,
            config.gear_ratio, true);

        odom_ = Odometry(left_motor_tracker_, right_motor_tracker_,
                         config.back_tracker, config.imu_port);
    }
}

void Chassis::calibrate() {
    odom_.calibrate();
    odom_.start();
    pros::delay(50);
}

// ─── Driver Control ──────────────────────────────────────────────────────────

void Chassis::arcade(double forward, double turn) {
    if (motion_running_) cancel_motion();
    drivetrain_.arcade(forward, turn, config_.drive_curve,
                       config_.joystick_deadzone);
}

void Chassis::tank(double left, double right) {
    if (motion_running_) cancel_motion();
    drivetrain_.tank(left, right, config_.drive_curve,
                     config_.joystick_deadzone);
}

// ─── Motion Infrastructure ───────────────────────────────────────────────────

void Chassis::run_motion(std::function<bool()> update_fn, int timeout) {
    motion_running_ = true;
    motion_timer_.start();

    while (motion_running_) {
        if (timeout > 0 && motion_timer_.expired(timeout)) {
            break;
        }
        if (update_fn()) {
            break;
        }
        pros::delay(10);
    }

    end_motion();
}

void Chassis::end_motion() {
    drivetrain_.stop();
    motion_running_ = false;
}

bool Chassis::is_settled() const {
    return !motion_running_;
}

void Chassis::wait_until_settled() {
    while (motion_running_) {
        pros::delay(10);
    }
}

void Chassis::cancel_motion() {
    motion_running_ = false;
    pros::delay(15);
    drivetrain_.stop();
    if (motion_task_) {
        motion_task_->remove();
        delete motion_task_;
        motion_task_ = nullptr;
    }
}

// ─── Forward / Backward ─────────────────────────────────────────────────────

void Chassis::move_forward(double distance, int timeout, MoveParams params) {
    // Apply per-motion overrides, save originals for restore
    PIDConfig orig_drive = apply_overrides(
        drive_pid_, params.gains, params.tolerance, params.settle_time,
        params.slew_rate);
    PIDConfig orig_heading = apply_overrides(
        heading_pid_, params.heading_gains, -1, -1);

    drive_pid_.reset();
    heading_pid_.reset();

    Pose start = odom_.get_pose();
    double start_heading = start.heading;

    auto update = [&]() -> bool {
        Pose current = odom_.get_pose();

        // Distance traveled along the starting heading direction
        double dx = current.x - start.x;
        double dy = current.y - start.y;
        double heading_rad = math::deg_to_rad(start_heading);
        double traveled = dx * std::sin(heading_rad) + dy * std::cos(heading_rad);

        double drive_error = distance - traveled;
        double drive_output = drive_pid_.update(drive_error);

        // Heading correction
        double heading_error = math::angle_error(start_heading, current.heading);
        double heading_output = heading_pid_.update(heading_error);

        drive_output = math::clamp(drive_output, params.max_speed);

        if (std::fabs(drive_output) < params.min_speed &&
            std::fabs(drive_error) > drive_pid_.get_config().tolerance) {
            drive_output = params.min_speed * math::sign(drive_output);
        }

        double left  = drive_output + heading_output;
        double right = drive_output - heading_output;
        drivetrain_.move(left, right);

        return drive_pid_.is_settled();
    };

    if (params.async) {
        cancel_motion();
        motion_running_ = true;
        motion_task_ = new pros::Task([this, update, timeout,
                                       orig_drive, orig_heading]() {
            run_motion(update, timeout);
            restore_config(drive_pid_, orig_drive);
            restore_config(heading_pid_, orig_heading);
        });
    } else {
        run_motion(update, timeout);
        restore_config(drive_pid_, orig_drive);
        restore_config(heading_pid_, orig_heading);
    }
}

void Chassis::move_backward(double distance, int timeout, MoveParams params) {
    move_forward(-distance, timeout, params);
}

// ─── Turn To ─────────────────────────────────────────────────────────────────

void Chassis::turn_to(double heading, int timeout, TurnParams params) {
    PIDConfig orig = apply_overrides(
        turn_pid_, params.gains, params.tolerance, params.settle_time);

    turn_pid_.reset();

    auto update = [&]() -> bool {
        Pose current = odom_.get_pose();
        double error = math::angle_error(heading, current.heading);
        double output = turn_pid_.update(error);

        output = math::clamp(output, params.max_speed);

        if (std::fabs(output) < params.min_speed &&
            std::fabs(error) > turn_pid_.get_config().tolerance) {
            output = params.min_speed * math::sign(output);
        }

        drivetrain_.move(output, -output);
        return turn_pid_.is_settled();
    };

    if (params.async) {
        cancel_motion();
        motion_running_ = true;
        motion_task_ = new pros::Task([this, update, timeout, orig]() {
            run_motion(update, timeout);
            restore_config(turn_pid_, orig);
        });
    } else {
        run_motion(update, timeout);
        restore_config(turn_pid_, orig);
    }
}

// ─── Swing Turn ──────────────────────────────────────────────────────────────

void Chassis::swing_to(double heading, Side side, int timeout,
                       SwingParams params) {
    PIDConfig orig = apply_overrides(
        swing_pid_, params.gains, params.tolerance, params.settle_time);

    swing_pid_.reset();

    auto update = [&]() -> bool {
        Pose current = odom_.get_pose();
        double error = math::angle_error(heading, current.heading);
        double output = swing_pid_.update(error);

        output = math::clamp(output, params.max_speed);

        if (std::fabs(output) < params.min_speed &&
            std::fabs(error) > swing_pid_.get_config().tolerance) {
            output = params.min_speed * math::sign(output);
        }

        if (side == Side::LEFT) {
            drivetrain_.move(0, -output);
        } else {
            drivetrain_.move(output, 0);
        }

        return swing_pid_.is_settled();
    };

    if (params.async) {
        cancel_motion();
        motion_running_ = true;
        motion_task_ = new pros::Task([this, update, timeout, orig]() {
            run_motion(update, timeout);
            restore_config(swing_pid_, orig);
        });
    } else {
        run_motion(update, timeout);
        restore_config(swing_pid_, orig);
    }
}

// ─── Move To Point ───────────────────────────────────────────────────────────

void Chassis::move_to(double x, double y, int timeout, MoveToParams params) {
    PIDConfig orig_drive = apply_overrides(
        move_to_drive_pid_, params.drive_gains,
        params.drive_tolerance, params.settle_time);
    PIDConfig orig_turn = apply_overrides(
        move_to_turn_pid_, params.turn_gains, -1, -1);

    move_to_drive_pid_.reset();
    move_to_turn_pid_.reset();

    // Resolve settle_radius: -1 means use the configured tolerance,
    // otherwise use the explicit value
    double effective_radius = params.settle_radius >= 0
        ? params.settle_radius
        : move_to_drive_pid_.get_config().tolerance;

    auto update = [&]() -> bool {
        Pose current = odom_.get_pose();

        double dist = math::distance(current.x, current.y, x, y);
        double target_heading = math::angle_to_point(current.x, current.y, x, y);

        if (!params.forward) {
            target_heading = math::wrap_angle_360(target_heading + 180.0);
        }

        double heading_error = math::angle_error(target_heading, current.heading);
        double heading_output = move_to_turn_pid_.update(heading_error);

        double cos_factor = std::cos(math::deg_to_rad(heading_error));
        if (cos_factor < 0) cos_factor = 0;

        double drive_output = move_to_drive_pid_.update(dist) * cos_factor;
        if (!params.forward) drive_output = -drive_output;

        drive_output = math::clamp(drive_output, params.max_speed);
        heading_output = math::clamp(heading_output, params.max_speed);

        double left  = drive_output + heading_output;
        double right = drive_output - heading_output;
        drivetrain_.move(left, right);

        return dist < effective_radius;
    };

    if (params.async) {
        cancel_motion();
        motion_running_ = true;
        motion_task_ = new pros::Task([this, update, timeout,
                                       orig_drive, orig_turn]() {
            run_motion(update, timeout);
            restore_config(move_to_drive_pid_, orig_drive);
            restore_config(move_to_turn_pid_, orig_turn);
        });
    } else {
        run_motion(update, timeout);
        restore_config(move_to_drive_pid_, orig_drive);
        restore_config(move_to_turn_pid_, orig_turn);
    }
}

void Chassis::move_to(Pose target, int timeout, MoveToParams params) {
    move_to(target.x, target.y, timeout, params);
}

// ─── Pose Access ─────────────────────────────────────────────────────────────

Pose Chassis::get_pose() const { return odom_.get_pose(); }
void Chassis::set_pose(Pose pose) { odom_.set_pose(pose); }
void Chassis::reset_pose() { odom_.reset(); }
void Chassis::reset_heading(double heading) { odom_.reset_heading(heading); }

// ─── Full PID Config Replacement ─────────────────────────────────────────────

void Chassis::set_drive_pid(PIDConfig config) { drive_pid_.set_config(config); }
void Chassis::set_turn_pid(PIDConfig config) { turn_pid_.set_config(config); }
void Chassis::set_swing_pid(PIDConfig config) { swing_pid_.set_config(config); }
void Chassis::set_heading_pid(PIDConfig config) { heading_pid_.set_config(config); }
void Chassis::set_move_to_drive_pid(PIDConfig config) { move_to_drive_pid_.set_config(config); }
void Chassis::set_move_to_turn_pid(PIDConfig config) { move_to_turn_pid_.set_config(config); }

// ─── Settle Convenience Methods ──────────────────────────────────────────────

void Chassis::set_drive_settle(double tolerance, int settle_time) {
    PIDConfig cfg = drive_pid_.get_config();
    cfg.tolerance = tolerance;
    cfg.settle_time = settle_time;
    drive_pid_.set_config(cfg);
}

void Chassis::set_turn_settle(double tolerance, int settle_time) {
    PIDConfig cfg = turn_pid_.get_config();
    cfg.tolerance = tolerance;
    cfg.settle_time = settle_time;
    turn_pid_.set_config(cfg);
}

void Chassis::set_swing_settle(double tolerance, int settle_time) {
    PIDConfig cfg = swing_pid_.get_config();
    cfg.tolerance = tolerance;
    cfg.settle_time = settle_time;
    swing_pid_.set_config(cfg);
}

void Chassis::set_move_to_settle(double tolerance, int settle_time) {
    PIDConfig cfg = move_to_drive_pid_.get_config();
    cfg.tolerance = tolerance;
    cfg.settle_time = settle_time;
    move_to_drive_pid_.set_config(cfg);
}

void Chassis::reset_pid_defaults() {
    drive_pid_.set_config(config_.drive_pid);
    turn_pid_.set_config(config_.turn_pid);
    swing_pid_.set_config(config_.swing_pid);
    heading_pid_.set_config(config_.heading_pid);
    move_to_drive_pid_.set_config(config_.move_to_drive_pid);
    move_to_turn_pid_.set_config(config_.move_to_turn_pid);
}

// ─── Internal Access ─────────────────────────────────────────────────────────

Drivetrain& Chassis::get_drivetrain() { return drivetrain_; }
Odometry& Chassis::get_odometry() { return odom_; }
const PID& Chassis::get_drive_pid() const { return drive_pid_; }
const PID& Chassis::get_turn_pid() const { return turn_pid_; }

} // namespace parrot
