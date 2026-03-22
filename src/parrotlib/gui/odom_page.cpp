/**
 * @file odom_page.cpp
 * @brief Odometry display page implementation.
 */

#include "parrotlib/gui/odom_page.hpp"
#include "parrotlib/core/math_utils.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>

namespace parrot {

OdomPage::OdomPage(Odometry& odom, double field_size)
    : Page("Odom"), odom_(odom), field_size_(field_size) {}

void OdomPage::render() {
    clear();

    Pose pose = odom_.get_pose();
    draw_field_view(pose);
    draw_coordinates(pose);
}

int OdomPage::field_to_screen_x(double x) const {
    // Map field x (-field_size/2 .. field_size/2) to screen pixels
    // within the mini-field view. Center of field = center of view.
    double ratio = (x + field_size_ / 2.0) / field_size_;
    return FIELD_X + static_cast<int>(ratio * FIELD_SIZE_PX);
}

int OdomPage::field_to_screen_y(double y) const {
    // Y is inverted: field +Y = up, screen +Y = down
    double ratio = 1.0 - (y + field_size_ / 2.0) / field_size_;
    return FIELD_Y + static_cast<int>(ratio * FIELD_SIZE_PX);
}

void OdomPage::draw_field_view(const Pose& pose) {
    // Draw field outline
    draw_rect(FIELD_X, FIELD_Y, FIELD_SIZE_PX, FIELD_SIZE_PX, 0x111111);
    draw_rect_outline(FIELD_X, FIELD_Y, FIELD_SIZE_PX, FIELD_SIZE_PX, 0x444444);

    // Draw center crosshairs
    int cx = FIELD_X + FIELD_SIZE_PX / 2;
    int cy = FIELD_Y + FIELD_SIZE_PX / 2;
    pros::screen::set_pen(0x222222);
    pros::screen::draw_line(cx, FIELD_Y, cx, FIELD_Y + FIELD_SIZE_PX);
    pros::screen::draw_line(FIELD_X, cy, FIELD_X + FIELD_SIZE_PX, cy);

    // Draw robot position
    int rx = field_to_screen_x(pose.x);
    int ry = field_to_screen_y(pose.y);

    // Clamp to field view bounds
    rx = std::max(FIELD_X + 3, std::min(FIELD_X + FIELD_SIZE_PX - 3, rx));
    ry = std::max(FIELD_Y + 3, std::min(FIELD_Y + FIELD_SIZE_PX - 3, ry));

    // Robot dot
    pros::screen::set_pen(0x00FF00);
    pros::screen::fill_circle(rx, ry, 5);

    // Heading direction line
    double heading_rad = math::deg_to_rad(pose.heading);
    int hx = rx + static_cast<int>(15.0 * std::sin(heading_rad));
    int hy = ry - static_cast<int>(15.0 * std::cos(heading_rad));
    pros::screen::set_pen(0x00AAFF);
    pros::screen::draw_line(rx, ry, hx, hy);
}

void OdomPage::draw_coordinates(const Pose& pose) {
    int text_x = FIELD_X + FIELD_SIZE_PX + 20;
    int y = 15;

    draw_text(text_x, y, "Position", 0xFFFFFF, TextSize::MEDIUM);
    y += 25;

    // Format with 2 decimal places
    auto fmt = [](const char* label, double val) -> std::string {
        std::ostringstream ss;
        ss << label << std::fixed << std::setprecision(2) << val;
        return ss.str();
    };

    draw_text(text_x, y, fmt("X:  ", pose.x) + " in", 0x00FF00);
    y += 22;
    draw_text(text_x, y, fmt("Y:  ", pose.y) + " in", 0x00FF00);
    y += 22;
    draw_text(text_x, y, fmt("H:  ", pose.heading) + " deg", 0x00AAFF);
    y += 35;

    // IMU heading (raw, for comparison during tuning)
    double imu_heading = odom_.get_imu_heading();
    draw_text(text_x, y, "IMU Raw", 0x888888, TextSize::SMALL);
    y += 18;
    draw_text(text_x, y, fmt("", imu_heading) + " deg", 0x888888);
    y += 30;

    // Tracking status
    bool running = odom_.is_running();
    uint32_t status_color = running ? 0x00FF00 : 0xFF4444;
    std::string status = running ? "TRACKING" : "STOPPED";
    draw_text(text_x, y, status, status_color, TextSize::SMALL);
}

} // namespace parrot
