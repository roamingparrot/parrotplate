/**
 * @file auton_selector.cpp
 * @brief Autonomous selector page implementation.
 */

#include "parrotlib/gui/auton_selector.hpp"
#include <algorithm>
#include <cmath>

namespace parrot {

AutonSelector::AutonSelector(std::vector<AutonRoutine> routines)
    : Page("Auton"),
      routines_(std::move(routines)) {
    // Calculate button layout. We want buttons that are big enough
    // to tap reliably in the pits with sweaty hands.
    if (routines_.empty()) return;

    // Two columns if we have more than 5 routines, single column otherwise
    buttons_per_column_ = routines_.size() <= 5 ? routines_.size() :
                          static_cast<int>(std::ceil(routines_.size() / 2.0));
    button_height_ = std::min(38, (CONTENT_HEIGHT - 30) / buttons_per_column_);
}

void AutonSelector::render() {
    clear();

    if (routines_.empty()) {
        draw_text(10, 40, "No autonomous routines configured", 0xAAAAAA);
        return;
    }

    draw_text(10, 5, "Select Autonomous", 0xFFFFFF, TextSize::MEDIUM);
    draw_hline(22);

    int cols = routines_.size() <= 5 ? 1 : 2;
    int col_width = (SCREEN_WIDTH - 20) / cols;
    int start_y = 28;

    for (int i = 0; i < static_cast<int>(routines_.size()); i++) {
        int col = (cols == 1) ? 0 : (i / buttons_per_column_);
        int row = (cols == 1) ? i : (i % buttons_per_column_);

        int x = 10 + col * col_width;
        int y = start_y + row * (button_height_ + 4);

        bool is_selected = (i == selected_);

        // Button background
        uint32_t bg = is_selected ? 0x1A7A1A : 0x2A2A2A;
        uint32_t fg = is_selected ? 0xFFFFFF : 0xBBBBBB;
        uint32_t border = is_selected ? 0x22BB22 : 0x444444;

        draw_rect(x, y, col_width - 8, button_height_, bg);
        draw_rect_outline(x, y, col_width - 8, button_height_, border);

        // Button label
        std::string label = routines_[i].name;
        if (is_selected) label = "> " + label;
        draw_text(x + 8, y + (button_height_ - 14) / 2, label, fg,
                  TextSize::MEDIUM);
    }
}

void AutonSelector::on_touch(int x, int y) {
    if (routines_.empty()) return;

    int cols = routines_.size() <= 5 ? 1 : 2;
    int col_width = (SCREEN_WIDTH - 20) / cols;
    int start_y = 28;

    for (int i = 0; i < static_cast<int>(routines_.size()); i++) {
        int col = (cols == 1) ? 0 : (i / buttons_per_column_);
        int row = (cols == 1) ? i : (i % buttons_per_column_);

        int bx = 10 + col * col_width;
        int by = start_y + row * (button_height_ + 4);

        if (x >= bx && x <= bx + col_width - 8 &&
            y >= by && y <= by + button_height_) {
            selected_ = i;
            return;
        }
    }
}

std::function<void()> AutonSelector::get_selected() const {
    if (selected_ < 0 || selected_ >= static_cast<int>(routines_.size())) {
        return nullptr;
    }
    return routines_[selected_].function;
}

std::string AutonSelector::get_selected_name() const {
    if (selected_ < 0 || selected_ >= static_cast<int>(routines_.size())) {
        return "None";
    }
    return routines_[selected_].name;
}

int AutonSelector::get_selected_index() const { return selected_; }

void AutonSelector::set_selected(int index) {
    if (index >= 0 && index < static_cast<int>(routines_.size())) {
        selected_ = index;
    }
}

} // namespace parrot
