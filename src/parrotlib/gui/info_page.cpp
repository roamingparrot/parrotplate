/**
 * @file info_page.cpp
 * @brief System info page implementation.
 */

#include "parrotlib/gui/info_page.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace parrot {

InfoPage::InfoPage(std::vector<MotorInfo> motors, int imu_port)
    : Page("Info"),
      motors_(std::move(motors)),
      imu_port_(imu_port) {}

void InfoPage::render() {
    clear();

    int y = 5;
    draw_battery(y);
    y += 50;
    draw_hline(y);
    y += 5;
    draw_motor_temps(y);
}

uint32_t InfoPage::temp_color(double temp_c) {
    if (temp_c < 45.0) return 0x00CC00;      // Green — cool
    if (temp_c < 55.0) return 0xFFAA00;      // Yellow — warm
    return 0xFF2222;                           // Red — hot, motors will throttle
}

void InfoPage::draw_battery(int y) {
    double level = pros::battery::get_capacity();
    double voltage = pros::battery::get_voltage() / 1000.0;  // mV → V
    double current = pros::battery::get_current() / 1000.0;  // mA → A

    draw_text(10, y, "Battery", 0xFFFFFF, TextSize::MEDIUM);

    y += 20;

    // Color-coded battery level
    uint32_t color;
    if (level > 60) color = 0x00CC00;
    else if (level > 30) color = 0xFFAA00;
    else color = 0xFF2222;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(0) << level << "%  "
       << std::setprecision(1) << voltage << "V  "
       << std::setprecision(1) << current << "A";

    draw_bar(10, y, 200, 16, level, 100.0, color);
    draw_text(220, y, ss.str(), color, TextSize::SMALL);
}

void InfoPage::draw_motor_temps(int y) {
    draw_text(10, y, "Motor Temps", 0xFFFFFF, TextSize::MEDIUM);
    y += 20;

    for (const auto& info : motors_) {
        // Average temperature across all ports in this group
        double total_temp = 0;
        int count = 0;
        for (int port : info.ports) {
            pros::Motor motor(std::abs(port));
            double temp = motor.get_temperature();
            if (temp != PROS_ERR_F) {
                total_temp += temp;
                count++;
            }
        }

        double avg_temp = count > 0 ? total_temp / count : 0;
        uint32_t color = temp_color(avg_temp);

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << avg_temp << " C";

        // Name and temp bar
        draw_text(10, y, info.name, 0xBBBBBB, TextSize::SMALL);
        draw_bar(130, y, 120, 12, avg_temp, 70.0, color);
        draw_text(260, y, ss.str(), color, TextSize::SMALL);

        y += 18;

        // Don't draw past the content area
        if (y > CONTENT_HEIGHT - 20) break;
    }
}

void InfoPage::draw_sensors(int y) {
    draw_text(10, y, "Sensors", 0xFFFFFF, TextSize::MEDIUM);
    y += 20;

    if (imu_port_ > 0) {
        pros::Imu imu(imu_port_);
        bool connected = !imu.is_calibrating() && imu.get_heading() != PROS_ERR_F;
        uint32_t color = connected ? 0x00CC00 : 0xFF2222;
        std::string status = connected ? "OK" : "ERR";

        std::ostringstream ss;
        ss << "IMU (port " << imu_port_ << "): " << status;
        draw_text(10, y, ss.str(), color, TextSize::SMALL);
    }
}

} // namespace parrot
