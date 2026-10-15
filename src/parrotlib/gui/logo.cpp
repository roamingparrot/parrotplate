/**
 * @file logo.cpp
 * @brief Draws the ParrotPlate logo with brain screen primitives.
 */

#include "parrotlib/gui/logo.hpp"
#include "parrotlib/gui/page.hpp"
#include "api.h"
#include <algorithm>
#include <cmath>

namespace parrot {

namespace {

constexpr double PI = 3.14159265358979323846;

struct Point {
    double x;
    double y;
};

// Maps the 100x100 logo grid onto the screen box being drawn into
struct Box {
    double x;
    double y;
    double scale;

    Point map(Point p) const { return {x + p.x * scale, y + p.y * scale}; }
};

// The drivetrain is turned 18 degrees anticlockwise about (50, 46)
Point turn(Point p) {
    constexpr double angle = -18.0 * PI / 180.0;
    const double dx = p.x - 50.0;
    const double dy = p.y - 46.0;
    return {50.0 + dx * std::cos(angle) - dy * std::sin(angle),
            46.0 + dx * std::sin(angle) + dy * std::cos(angle)};
}

/**
 * The screen only draws 1px lines, so a thicker stroke is a few parallel
 * lines side by side, spread across the stroke's width.
 */
void stroke(Point a, Point b, double width) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length < 0.01) return;

    // Unit vector across the line
    const double nx = -dy / length;
    const double ny = dx / length;

    const int passes = std::max(1, static_cast<int>(std::lround(width)));
    for (int i = 0; i < passes; i++) {
        const double offset = (passes == 1) ? 0.0 : (i - (passes - 1) / 2.0) * (width / passes);
        pros::screen::draw_line(
            static_cast<int16_t>(std::lround(a.x + nx * offset)),
            static_cast<int16_t>(std::lround(a.y + ny * offset)),
            static_cast<int16_t>(std::lround(b.x + nx * offset)),
            static_cast<int16_t>(std::lround(b.y + ny * offset)));
    }
}

// A rectangle on the logo grid, turned with the drivetrain
void turned_rect(const Box& box, double x, double y, double w, double h, double width) {
    const Point corners[4] = {
        box.map(turn({x, y})),
        box.map(turn({x + w, y})),
        box.map(turn({x + w, y + h})),
        box.map(turn({x, y + h})),
    };
    for (int i = 0; i < 4; i++) {
        stroke(corners[i], corners[(i + 1) % 4], width);
    }
}

void turned_line(const Box& box, Point a, Point b, double width) {
    stroke(box.map(turn(a)), box.map(turn(b)), width);
}

// The path behind the robot: a cubic curve, drawn 3 units on and 5 off
void dashed_path(const Box& box, double width) {
    const Point p0{57, 72}, p1{60, 84}, p2{44, 86}, p3{41, 98};
    auto at = [&](double t) {
        const double u = 1.0 - t;
        return Point{
            u * u * u * p0.x + 3 * u * u * t * p1.x + 3 * u * t * t * p2.x + t * t * t * p3.x,
            u * u * u * p0.y + 3 * u * u * t * p1.y + 3 * u * t * t * p2.y + t * t * t * p3.y};
    };

    constexpr int STEPS = 60;
    constexpr double DASH = 3.0;
    constexpr double GAP = 5.0;
    double travelled = 0.0;
    Point previous = at(0.0);
    for (int i = 1; i <= STEPS; i++) {
        const Point next = at(static_cast<double>(i) / STEPS);
        const double phase = std::fmod(travelled, DASH + GAP);
        if (phase < DASH) {
            stroke(box.map(previous), box.map(next), width);
        }
        travelled += std::hypot(next.x - previous.x, next.y - previous.y);
        previous = next;
    }
}

} // namespace

void draw_logo(int x, int y, int size, uint32_t color) {
    // The logo's strokes are 3.5 grid units wide; never thinner than a pixel
    const Box box{static_cast<double>(x), static_cast<double>(y), size / 100.0};
    const double body = std::max(1.0, 3.5 * box.scale);
    const double fine = std::max(1.0, 2.5 * box.scale);

    pros::screen::set_pen(color);

    // Chassis and the four wheels
    turned_rect(box, 30, 20, 40, 50, body);
    turned_rect(box, 19, 23, 9, 16, body);
    turned_rect(box, 72, 23, 9, 16, body);
    turned_rect(box, 19, 51, 9, 16, body);
    turned_rect(box, 72, 51, 9, 16, body);

    // Heading arrow on the chassis, and the tick out in front
    turned_line(box, {50, 54}, {50, 32}, body);
    turned_line(box, {43, 39}, {50, 32}, body);
    turned_line(box, {57, 39}, {50, 32}, body);
    turned_line(box, {50, 14}, {50, 4}, fine);

    dashed_path(box, fine);
}

void draw_splash() {
    pros::screen::set_pen(0x000000);
    pros::screen::fill_rect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

    // Logo on the left, name on the right. The text is left-aligned so the
    // layout doesn't depend on how wide the brain's fonts render.
    constexpr int LOGO_SIZE = 120;
    constexpr int LOGO_X = 70;
    constexpr int LOGO_Y = (SCREEN_HEIGHT - LOGO_SIZE) / 2;
    draw_logo(LOGO_X, LOGO_Y, LOGO_SIZE);

    constexpr int TEXT_X = LOGO_X + LOGO_SIZE + 24;
    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_LARGE, TEXT_X, 92, "ParrotPlate");
    pros::screen::set_pen(0x888888);
    pros::screen::print(pros::E_TEXT_SMALL, TEXT_X + 2, 134, "PROS template for VEX V5");
}

} // namespace parrot
