/**
 * @file page.cpp
 * @brief Page base class implementation with drawing helpers.
 */

#include "parrotlib/gui/page.hpp"
#include "api.h"

namespace parrot {

Page::Page(std::string title) : title_(std::move(title)) {}

std::string Page::get_title() const { return title_; }

void Page::draw_text(int x, int y, const std::string& text,
                     uint32_t color, TextSize size) {
    pros::screen::set_pen(color);

    pros::text_format_e_t fmt;
    switch (size) {
        case TextSize::SMALL:  fmt = pros::E_TEXT_SMALL; break;
        case TextSize::MEDIUM: fmt = pros::E_TEXT_MEDIUM; break;
        case TextSize::LARGE:  fmt = pros::E_TEXT_LARGE; break;
        default:               fmt = pros::E_TEXT_MEDIUM; break;
    }

    pros::screen::print(fmt, x, y, "%s", text.c_str());
}

void Page::draw_rect(int x, int y, int w, int h, uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::fill_rect(x, y, x + w, y + h);
}

void Page::draw_rect_outline(int x, int y, int w, int h, uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::draw_rect(x, y, x + w, y + h);
}

void Page::draw_hline(int y, uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::draw_line(0, y, SCREEN_WIDTH, y);
}

void Page::draw_bar(int x, int y, int w, int h,
                    double value, double max_value,
                    uint32_t fill_color, uint32_t bg_color) {
    // Background
    draw_rect(x, y, w, h, bg_color);

    // Fill proportional to value
    if (max_value > 0 && value > 0) {
        double ratio = value / max_value;
        if (ratio > 1.0) ratio = 1.0;
        int fill_w = static_cast<int>(w * ratio);
        draw_rect(x, y, fill_w, h, fill_color);
    }

    // Border
    draw_rect_outline(x, y, w, h, 0x888888);
}

void Page::clear() {
    draw_rect(0, 0, SCREEN_WIDTH, CONTENT_HEIGHT, 0x000000);
}

} // namespace parrot
