# Contributing to ParrotPlate

Thanks for your interest in contributing. This document covers the conventions
we follow so the codebase stays consistent and easy to work with.

## Getting started

Install the [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/) and the
`arm-none-eabi-g++` toolchain. Then:

```sh
pros make        # compile
pros mu          # compile + upload to V5 Brain
pros terminal    # view debug output over USB
```

You'll need a VEX V5 Brain and controller connected via USB or a configured
wireless link.

## Project layout

```
include/
  main.h                 PROS entry header
  parrotlib/             Library headers (core framework)
    parrotlib.hpp        Master include
    core/                PID controller, slew limiter, timer, math utils
    chassis/             Drivetrain, motion commands, per-motion config
    odom/                Pose, tracking wheels, position tracker
    gui/                 Page system, auton selector, screen pages
    sensors/             Sensor abstractions (placeholder)
    utils/               Logger, unit conversions
  robot/                 User-facing robot configuration headers
src/
  main.cpp               PROS entry point (init, opcontrol, autonomous)
  parrotlib/             Library implementations (mirrors include/)
  robot/                 User-facing code (config, subsystems, autons)
```

The data flow is: **input** (joystick / autonomous routine) produces a command
→ **chassis** executes motion using **odom** for position feedback and **core**
(PID, slew) for control → **gui** reads state and renders to the brain screen.

## Commit conventions

We use short, lowercase commit messages. No period at the end.

```
<type>: <what changed>
```

### Types

| Type       | When                                           |
|------------|------------------------------------------------|
| `feat`     | New user-facing feature                        |
| `fix`      | Bug fix                                        |
| `refactor` | Internal restructuring, no behavior change     |
| `core`     | PID, slew limiter, timer, math utility changes |
| `chassis`  | Drivetrain, motion commands, config changes    |
| `odom`     | Odometry, pose, tracking wheel changes         |
| `gui`      | Brain screen, page system, selector changes    |
| `sensors`  | Sensor abstraction changes                     |
| `docs`     | README, CONTRIBUTING, comments                 |
| `deps`     | Dependency or PROS kernel version bumps        |
| `chore`    | CI, tooling, gitignore, formatting             |

### Examples

```
feat: swing turn motion command
fix: heading drift after IMU recalibration
refactor: extract settle detection into timer
core: add integral windup guard to PID
chassis: per-motion slew rate override
odom: reset pose without blocking on IMU
gui: highlight selected auton on page switch
docs: add PID tuning walkthrough
```

### Body

Add a body when the "why" isn't obvious from the subject. Leave a blank
line between subject and body. Wrap at ~72 chars.

```
fix: progress overshoot on short drives

The settle detector was comparing raw error against tolerance before
applying the absolute value, so negative error never triggered
settlement. Now uses std::abs(error) consistently.
```

### What not to do

- Don't use `Update file.cpp` or `misc changes`
- Don't capitalize the subject or end with a period
- Don't squash unrelated changes into one commit
- Don't prefix with ticket numbers unless we start using an issue tracker

## Branches

- `main` is the release branch. It should always build.
- Feature branches: `feat/swing-turns`, `fix/imu-drift`
- Keep branches short-lived. Rebase on main before merging.

## Pull requests

### Title

Same format as commit subjects:

```
feat: move-to-point with boomerang controller
```

### Description template

```markdown
## Summary

Brief description of what this changes and why.

## Changes

- Bullet list of what was modified
- One line per logical change

## Testing

How you verified this works. "Ran the showcase auton and checked odom
output" is fine for motion changes. Mention edge cases you checked.
```

### Commit hygiene

- Each commit should be a single logical change following the `type: description` convention.
- Don't squash unrelated changes into one commit.
- Don't interweave concerns — a commit that touches odom should not also tweak GUI styling.
- Prefer multiple clean commits over one large one. Reviewers read commit-by-commit.

### Review norms

- PRs should be reviewable in one sitting. If it's big, split it.
- Reviewer approves or requests changes — no "looks good" without reading.
- Author resolves conversations after addressing feedback.

## Code style

### C++ conventions

- C++17 compiled with `arm-none-eabi-g++`. No exceptions, no RTTI.
- Use traditional header guards, not `#pragma once`:
  ```cpp
  #ifndef PARROTLIB_CORE_PID_HPP
  #define PARROTLIB_CORE_PID_HPP
  // ...
  #endif
  ```
- Guard names follow `PARROTLIB_<CATEGORY>_<FILENAME>_HPP`.
- Use designated initializers for config structs:
  ```cpp
  PIDConfig config = {
      .gains = {5.0, 0, 35.0},
      .tolerance = 0.5,
      .settle_time = 200,
  };
  ```
- Prefer `std::clamp` over manual min/max chains.

### Naming

- Namespaces: `snake_case` (`parrot`, `robot`, `units`)
- Classes / structs: `PascalCase` (`PID`, `Chassis`, `TrackingWheel`)
- Functions: `snake_case`, verb-first for actions (`move_forward`, `set_pose`, `get_elapsed`)
- Private members: trailing underscore (`kP_`, `integral_`, `current_level_`)
- Constants: `SCREAMING_SNAKE_CASE` (`SCREEN_WIDTH`, `NAV_BAR_HEIGHT`)
- Modules: lowercase, short (`core`, `odom`, `gui`, not `odometry_tracking_system`)
- No Hungarian notation, no prefixes like `m_` or `s_`

### Comments

- Don't comment obvious code. `// increment counter` above `i += 1` helps nobody.
- Do comment non-obvious decisions, workarounds, or PROS API quirks.
- Use `// TODO:` for known gaps. Include enough context to act on it later.
- Use `// ───` section dividers to separate logical blocks in longer files.

### Error handling

- No exceptions — the PROS toolchain compiles with `-fno-exceptions`.
- Use `Logger::warn()` / `Logger::error()` for internal diagnostics.
- Use timeouts on blocking operations (`timeout_ms` parameter on motion functions).
- Don't silently discard errors. If you must ignore a return value, leave a comment.

### Module boundaries

- `chassis` owns all drivetrain motion. Nothing else writes motor voltages directly.
- `odom` owns the tracking loop. Chassis reads pose but doesn't modify tracking internals.
- `gui` reads from robot state but never mutates it.
- `core` (PID, slew, timer) is pure logic — no hardware dependencies.
- `robot/` is user-space. It configures and calls into `parrotlib/` but doesn't modify library code.

### Adding dependencies

- PROS provides the kernel and device API. Prefer it over rolling your own.
- Check if the standard library or an existing parrotlib utility already covers it.
- Pin the PROS kernel version in `project.pros`. Don't bump it without testing on hardware.

## Testing

We don't have a test suite yet. When we add one:

- Unit tests for `core/` (PID, math utils) can run on the host with a PROS-free build.
- Integration tests require a V5 Brain or a simulator.
- Test motion changes with the showcase auton and verify odom output via the brain screen.

## Releases

Not formalized yet. When we get there:

- Tag with `v0.x.y`
- Changelog entries follow the commit type format
- Ensure `pros make` produces a clean build with no warnings
