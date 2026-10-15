/**
 * @file screen.hpp
 * @brief Brain screen manager with tab-based page navigation.
 *
 * The Screen class manages a collection of pages and provides
 * tab-based navigation along the bottom of the V5 brain screen.
 * It runs its own render task so the GUI doesn't block your
 * main control loops.
 *
 * Pages are added before calling start(). The first page added
 * is shown by default.
 *
 * @code
 *   parrot::Screen screen;
 *   screen.add_page(new parrot::AutonSelector(autons));
 *   screen.add_page(new parrot::OdomPage(odom));
 *   screen.add_page(new parrot::InfoPage(chassis));
 *   screen.start();
 * @endcode
 */

#ifndef PARROTLIB_GUI_SCREEN_HPP
#define PARROTLIB_GUI_SCREEN_HPP

#include "parrotlib/gui/page.hpp"
#include "api.h"
#include <vector>

namespace parrot {

class Screen {
public:
    Screen();
    ~Screen();

    /**
     * Add a page to the screen. The Screen takes ownership of the pointer
     * and will delete it on destruction. Add all pages before calling start().
     */
    void add_page(Page* page);

    /**
     * Start the GUI render task. Call this in initialize() after
     * adding all pages.
     */
    void start();

    /** Stop the render task. */
    void stop();

    /** Switch to a specific page by index. */
    void set_page(int index);

    /** Get the currently active page index. */
    int get_current_page() const;

    /** Get number of registered pages. */
    int page_count() const;

    /**
     * How long the startup splash (the ParrotPlate logo) shows before the
     * first page, in milliseconds. Set to 0 to skip it. Call before start().
     */
    void set_splash_time(int ms);

private:
    std::vector<Page*> pages_;
    int current_page_ = 0;
    bool running_     = false;
    int splash_ms_    = 1500;
    pros::Task* task_ = nullptr;

    /** Render loop — runs at ~20Hz in a background task. */
    void render_loop();

    /** Draw the navigation bar at the bottom of the screen. */
    void draw_nav_bar();

    /** Handle touch input — check nav bar buttons and forward to page. */
    void handle_touch();

    static void task_fn(void* param);

    // Touch debounce — the V5 touchscreen is noisy and sometimes
    // registers multiple taps from a single press
    bool touch_was_pressed_ = false;
};

} // namespace parrot

#endif // PARROTLIB_GUI_SCREEN_HPP
