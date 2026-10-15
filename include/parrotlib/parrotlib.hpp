/**
 * @file parrotlib.hpp
 * @brief Master include for the ParrotPlate robotics framework.
 *
 * Include this single header to pull in the entire parrotlib API.
 * Individual headers can also be included directly if you want to
 * keep compile times down during development.
 */

#ifndef PARROTLIB_HPP
#define PARROTLIB_HPP

// ─── Core ────────────────────────────────────────────────────────────────────
#include "parrotlib/core/pid.hpp"
#include "parrotlib/core/slew.hpp"
#include "parrotlib/core/timer.hpp"
#include "parrotlib/core/math_utils.hpp"

// ─── Odometry ────────────────────────────────────────────────────────────────
#include "parrotlib/odom/pose.hpp"
#include "parrotlib/odom/tracking_wheel.hpp"
#include "parrotlib/odom/odometry.hpp"

// ─── Chassis ─────────────────────────────────────────────────────────────────
#include "parrotlib/chassis/chassis_config.hpp"
#include "parrotlib/chassis/drivetrain.hpp"
#include "parrotlib/chassis/chassis.hpp"

// ─── GUI ─────────────────────────────────────────────────────────────────────
#include "parrotlib/gui/page.hpp"
#include "parrotlib/gui/logo.hpp"
#include "parrotlib/gui/screen.hpp"
#include "parrotlib/gui/auton_selector.hpp"
#include "parrotlib/gui/odom_page.hpp"
#include "parrotlib/gui/info_page.hpp"

// ─── Utilities ───────────────────────────────────────────────────────────────
#include "parrotlib/utils/logger.hpp"
#include "parrotlib/utils/units.hpp"

#endif // PARROTLIB_HPP
