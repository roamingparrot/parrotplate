/**
 * @file logo.hpp
 * @brief The ParrotPlate logo, drawn with brain screen primitives.
 *
 * The logo is a drivetrain seen from above, turned mid-drive, with a
 * dashed path trailing behind it. It is drawn from lines rather than an
 * image, so it scales to any size and costs no flash space.
 *
 * The shapes match assets/logo.svg, which is laid out on a 100x100 grid.
 */

#ifndef PARROTLIB_GUI_LOGO_HPP
#define PARROTLIB_GUI_LOGO_HPP

#include <cstdint>

namespace parrot {

// The logo's orange
constexpr uint32_t LOGO_COLOR = 0xFB923C;

/**
 * Draw the logo inside a size x size box whose top-left corner is (x, y).
 * Below about 60px the wheels and arrow start to run together.
 */
void draw_logo(int x, int y, int size, uint32_t color = LOGO_COLOR);

/**
 * Fill the whole screen with the startup splash: the logo above the
 * ParrotPlate name. The Screen shows this briefly before the first page.
 */
void draw_splash();

} // namespace parrot

#endif // PARROTLIB_GUI_LOGO_HPP
