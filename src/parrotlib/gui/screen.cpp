/**
 * @file screen.cpp
 * @brief Screen manager implementation.
 */

#include "parrotlib/gui/screen.hpp"
#include "parrotlib/core/timer.hpp"
#include <algorithm>

namespace parrot {

Screen::Screen() {}

Screen::~Screen() {
    stop();
    for (Page* page : pages_) {
        delete page;
    }
}

void Screen::add_page(Page* page) {
    pages_.push_back(page);
}

void Screen::task_fn(void* param) {
    auto* self = static_cast<Screen*>(param);
    self->render_loop();
}

void Screen::start() {
    if (running_ || pages_.empty()) return;
    running_ = true;

    // Notify the first page it's now active
    if (!pages_.empty()) {
        pages_[0]->on_enter();
    }

    task_ = new pros::Task(task_fn, this, "parrot_screen");
}

void Screen::stop() {
    if (!running_) return;
    running_ = false;
    parrot::delay(60);  // Let the task finish its current cycle
    if (task_) {
        delete task_;
        task_ = nullptr;
    }
}

void Screen::set_page(int index) {
    if (index < 0 || index >= static_cast<int>(pages_.size())) return;
    if (index == current_page_) return;

    pages_[current_page_]->on_exit();
    current_page_ = index;
    pages_[current_page_]->on_enter();
}

int Screen::get_current_page() const { return current_page_; }
int Screen::page_count() const { return pages_.size(); }

void Screen::render_loop() {
    while (running_) {
        handle_touch();

        if (current_page_ >= 0 &&
            current_page_ < static_cast<int>(pages_.size())) {
            pages_[current_page_]->render();
        }

        draw_nav_bar();
        parrot::delay(50);  // ~20Hz refresh rate — fast enough for diagnostics
    }
}

void Screen::draw_nav_bar() {
    if (pages_.empty()) return;

    int bar_y = CONTENT_HEIGHT;
    int tab_width = SCREEN_WIDTH / pages_.size();

    for (int i = 0; i < static_cast<int>(pages_.size()); i++) {
        int x = i * tab_width;

        // Active tab gets a highlight color, inactive tabs are dark
        uint32_t bg = (i == current_page_) ? 0x1A5276 : 0x1C1C1C;
        uint32_t fg = (i == current_page_) ? 0xFFFFFF : 0x888888;

        // Draw tab background
        pros::screen::set_pen(bg);
        pros::screen::fill_rect(x, bar_y, x + tab_width, SCREEN_HEIGHT);

        // Draw tab border
        pros::screen::set_pen(0x333333);
        pros::screen::draw_rect(x, bar_y, x + tab_width, SCREEN_HEIGHT);

        // Draw tab label centered
        std::string label = pages_[i]->get_title();
        // Rough centering: assume ~7px per character for medium text
        int text_x = x + (tab_width - label.length() * 7) / 2;
        int text_y = bar_y + 8;
        pros::screen::set_pen(fg);
        pros::screen::print(pros::E_TEXT_SMALL, text_x, text_y,
                            "%s", label.c_str());
    }
}

void Screen::handle_touch() {
    pros::screen_touch_status_s_t touch = pros::screen::touch_status();

    if (touch.touch_status == pros::E_TOUCH_PRESSED ||
        touch.touch_status == pros::E_TOUCH_HELD) {
        if (!touch_was_pressed_) {
            touch_was_pressed_ = true;

            int tx = touch.x;
            int ty = touch.y;

            // Check if the touch is in the nav bar area
            if (ty >= CONTENT_HEIGHT) {
                int tab_width = SCREEN_WIDTH / pages_.size();
                int tapped_tab = tx / tab_width;
                if (tapped_tab >= 0 &&
                    tapped_tab < static_cast<int>(pages_.size())) {
                    set_page(tapped_tab);
                }
            } else {
                // Forward the touch to the active page
                if (current_page_ >= 0 &&
                    current_page_ < static_cast<int>(pages_.size())) {
                    pages_[current_page_]->on_touch(tx, ty);
                }
            }
        }
    } else {
        touch_was_pressed_ = false;
    }
}

} // namespace parrot
