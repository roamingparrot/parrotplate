/**
 * @file info_page.hpp
 * @brief System info page — battery, motor temps, sensor status.
 *
 * The "at a glance" diagnostics page that every team needs in the pits.
 * Shows battery level, motor temperatures (with color-coded warnings),
 * and connected sensor status. Helps catch problems before they cost
 * you a match.
 *
 * Temperature color coding:
 *   - Green:  < 45°C (safe)
 *   - Yellow: 45-55°C (warm, keep an eye on it)
 *   - Red:    > 55°C (hot — motor will throttle performance)
 */

#ifndef PARROTLIB_GUI_INFO_PAGE_HPP
#define PARROTLIB_GUI_INFO_PAGE_HPP

#include "parrotlib/gui/page.hpp"
#include "api.h"
#include <vector>
#include <string>

namespace parrot {

/**
 * A named motor or motor group to display on the info page.
 * Used for showing temperature readings with labels.
 */
struct MotorInfo {
    std::string name;      // Display name ("Left Drive", "Intake", etc.)
    std::vector<int> ports; // Motor ports to monitor
};

class InfoPage : public Page {
public:
    /**
     * @param motors    List of motors/groups to monitor temperatures for.
     * @param imu_port  IMU port for heading display (0 = no IMU).
     */
    InfoPage(std::vector<MotorInfo> motors, int imu_port = 0);

    void render() override;

private:
    std::vector<MotorInfo> motors_;
    int imu_port_;

    /** Get color for a temperature value (green → yellow → red). */
    static uint32_t temp_color(double temp_c);

    /** Draw the battery section. */
    void draw_battery(int y);

    /** Draw the motor temperature section. */
    void draw_motor_temps(int y);

    /** Draw sensor status section. */
    void draw_sensors(int y);
};

} // namespace parrot

#endif // PARROTLIB_GUI_INFO_PAGE_HPP
