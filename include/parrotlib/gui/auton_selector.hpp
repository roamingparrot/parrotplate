/**
 * @file auton_selector.hpp
 * @brief Touchscreen autonomous routine selector.
 *
 * Displays a list of autonomous routines on the brain screen and lets
 * the driver select one by tapping. The selected routine's function
 * pointer is returned by get_selected(), which you call at the start
 * of autonomous().
 *
 * This is the page most teams will interact with before every match.
 * Keep the labels short and obvious so the drive team can select the
 * right auton under time pressure.
 *
 * @code
 *   parrot::AutonSelector selector({
 *       {"Left AWP",   auton::left_awp},
 *       {"Right Elim", auton::right_elim},
 *       {"Skills",     auton::skills},
 *       {"Do Nothing", nullptr},
 *   });
 *
 *   // In autonomous():
 *   auto fn = selector.get_selected();
 *   if (fn) fn();
 * @endcode
 */

#ifndef PARROTLIB_GUI_AUTON_SELECTOR_HPP
#define PARROTLIB_GUI_AUTON_SELECTOR_HPP

#include "parrotlib/gui/page.hpp"
#include <functional>
#include <string>
#include <vector>

namespace parrot {

/**
 * An autonomous routine entry: a name and a function to run.
 * The function should contain the complete autonomous routine.
 */
struct AutonRoutine {
    std::string name;
    std::function<void()> function;
};

class AutonSelector : public Page {
public:
    /**
     * @param routines  List of autonomous routines to display.
     *                  The first one is selected by default.
     */
    explicit AutonSelector(std::vector<AutonRoutine> routines);

    void render() override;
    void on_touch(int x, int y) override;

    /**
     * Get the function pointer for the currently selected autonomous.
     * Call this at the top of your autonomous() function.
     * Returns nullptr if the selected routine has no function (e.g. "Do Nothing").
     */
    std::function<void()> get_selected() const;

    /** Get the name of the selected autonomous (for logging). */
    std::string get_selected_name() const;

    /** Get the index of the selected autonomous. */
    int get_selected_index() const;

    /** Programmatically select an auton by index. */
    void set_selected(int index);

private:
    std::vector<AutonRoutine> routines_;
    int selected_ = 0;

    // Layout constants — calculated from number of routines
    int button_height_ = 0;
    int buttons_per_column_ = 0;
};

} // namespace parrot

#endif // PARROTLIB_GUI_AUTON_SELECTOR_HPP
