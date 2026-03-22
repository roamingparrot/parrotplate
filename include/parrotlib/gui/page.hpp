/**
 * @file page.hpp
 * @brief Base class for brain screen pages.
 *
 * The GUI system uses a page-based model. Each page is a full-screen
 * view that draws on the V5 brain's 480x240 LCD. The Screen manager
 * handles page switching (via touch buttons) and calls render() on
 * the active page at ~20Hz.
 *
 * To create a custom page, inherit from Page and override render().
 * Optionally override on_touch() to handle touch input.
 *
 * V5 screen coordinates:
 *   - Top-left is (0, 0)
 *   - Width: 480 pixels
 *   - Height: 240 pixels
 *   - PROS uses lv_obj / direct screen write functions
 */

#ifndef PARROTLIB_GUI_PAGE_HPP
#define PARROTLIB_GUI_PAGE_HPP

#include <string>

namespace parrot {

// Screen dimensions — these are hardware constants
constexpr int SCREEN_WIDTH  = 480;
constexpr int SCREEN_HEIGHT = 240;

// Reserve the bottom strip for page navigation buttons
constexpr int NAV_BAR_HEIGHT = 30;
constexpr int CONTENT_HEIGHT = SCREEN_HEIGHT - NAV_BAR_HEIGHT;

class Page {
public:
    /**
     * @param title  Short name shown in the navigation bar tab.
     *               Keep it under ~8 chars so tabs fit on screen.
     */
    explicit Page(std::string title);
    virtual ~Page() = default;

    /**
     * Draw this page's content. Called ~20 times/sec when this page
     * is active. Use pros::screen functions for drawing.
     *
     * The content area is 480 x 210 (bottom 30px is the nav bar).
     * Don't draw below y=210 or you'll overwrite the tabs.
     */
    virtual void render() = 0;

    /**
     * Handle a touch event on this page. Called when the user taps
     * within the content area (not the nav bar).
     *
     * @param x  Touch X coordinate (0–479)
     * @param y  Touch Y coordinate (0–209)
     */
    virtual void on_touch(int x, int y) {}

    /** Called once when this page becomes the active page. */
    virtual void on_enter() {}

    /** Called once when switching away from this page. */
    virtual void on_exit() {}

    std::string get_title() const;

protected:
    std::string title_;

    // ─── Drawing helpers ─────────────────────────────────────────────────
    // These wrap pros::screen functions with sane defaults so pages
    // don't have to repeat color setup and coordinate math.

    /** Draw text at (x, y) with a given color. Small/medium/large size. */
    enum class TextSize { SMALL, MEDIUM, LARGE };
    void draw_text(int x, int y, const std::string& text,
                   uint32_t color = 0xFFFFFF,
                   TextSize size = TextSize::MEDIUM);

    /** Draw a filled rectangle. */
    void draw_rect(int x, int y, int w, int h, uint32_t color);

    /** Draw a rectangle outline. */
    void draw_rect_outline(int x, int y, int w, int h, uint32_t color);

    /** Draw a horizontal line. Handy for section dividers. */
    void draw_hline(int y, uint32_t color = 0x444444);

    /** Draw a progress/temperature bar. */
    void draw_bar(int x, int y, int w, int h,
                  double value, double max_value,
                  uint32_t fill_color, uint32_t bg_color = 0x222222);

    /** Clear the content area to black. Call at the start of render(). */
    void clear();
};

} // namespace parrot

#endif // PARROTLIB_GUI_PAGE_HPP
