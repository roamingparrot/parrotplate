/**
 * @file odom_page.hpp
 * @brief Live odometry display page for the brain screen.
 *
 * Shows the robot's current (x, y, heading) on screen with a simple
 * top-down field visualization. Invaluable during odometry tuning —
 * you can push the robot around the field and watch the position update
 * in real time.
 *
 * The field view draws a dot representing the robot's position and a
 * line showing its heading direction. The coordinate display shows
 * exact numeric values.
 */

#ifndef PARROTLIB_GUI_ODOM_PAGE_HPP
#define PARROTLIB_GUI_ODOM_PAGE_HPP

#include "parrotlib/gui/page.hpp"
#include "parrotlib/odom/odometry.hpp"

namespace parrot {

class OdomPage : public Page {
public:
    /**
     * @param odom  Reference to the odometry instance. Must outlive this page.
     * @param field_size  Field size in inches for the visualization (144 = full VEX field).
     */
    explicit OdomPage(Odometry& odom, double field_size = 144.0);

    void render() override;

private:
    Odometry& odom_;
    double field_size_;

    // Mini-field view area on screen
    static constexpr int FIELD_X      = 10;
    static constexpr int FIELD_Y      = 10;
    static constexpr int FIELD_SIZE_PX = 180;  // square, pixels

    /** Convert field coordinates (inches) to screen pixels within the mini-field. */
    int field_to_screen_x(double x) const;
    int field_to_screen_y(double y) const;

    /** Draw the mini-field visualization. */
    void draw_field_view(const Pose& pose);

    /** Draw the numeric coordinate readout. */
    void draw_coordinates(const Pose& pose);
};

} // namespace parrot

#endif // PARROTLIB_GUI_ODOM_PAGE_HPP
