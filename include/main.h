/**
 * @file main.h
 * @brief PROS main header — includes the PROS API and project headers.
 *
 * This file is included automatically by the PROS build system in every
 * translation unit. Add any project-wide includes here.
 *
 * IMPORTANT: Do not remove the PROS API includes below.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

// ─── PROS API ────────────────────────────────────────────────────────────────
#include "api.h"

// ─── ParrotLib Framework ─────────────────────────────────────────────────────
#include "parrotlib/parrotlib.hpp"

// ─── Robot Configuration & User Code ─────────────────────────────────────────
#include "robot/config.hpp"
#include "robot/subsystems.hpp"
#include "robot/autons.hpp"

// ─── C/C++ Standard Libraries ────────────────────────────────────────────────
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <memory>

/**
 * PROS function prototypes.
 *
 * These functions are called by the PROS kernel at specific points during
 * program execution. Users implement them in src/main.cpp.
 *
 * See the PROS documentation for details:
 * https://pros.cs.purdue.edu/v5/tutorials/topical/multitasking.html
 */

#ifdef __cplusplus
extern "C" {
#endif

void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);

#ifdef __cplusplus
}
#endif

#endif // _PROS_MAIN_H_
