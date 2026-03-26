/**
 * @file config.hpp
 * @brief Robot hardware configuration — THIS IS WHERE YOU DEFINE YOUR ROBOT.
 *
 * ============================================================================
 *  USER FILE: Edit this file to match your robot's hardware.
 * ============================================================================
 *
 * All motor ports, sensor ports, PID tuning values, and hardware-specific
 * constants go here. Having everything in one place makes it easy to
 * update when you rewire, swap motors, or change gear ratios.
 *
 * This file declares the global chassis, controller, and screen objects.
 * They're defined (constructed) in src/robot/config.cpp.
 */

#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "parrotlib/parrotlib.hpp"

namespace robot {

// ─── Global Objects ──────────────────────────────────────────────────────────
// These are declared here and defined in config.cpp.
// Access them from anywhere via `robot::chassis`, `robot::controller`, etc.

extern parrot::Chassis chassis;
extern pros::Controller controller;
extern parrot::Screen screen;
extern parrot::AutonSelector auton_selector;

} // namespace robot

#endif // ROBOT_CONFIG_HPP
