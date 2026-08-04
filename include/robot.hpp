#pragma once

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.hpp"

// Hardware and subsystem objects defined in main.cpp, shared with other
// source files (e.g. autons.cpp) via extern declarations.

extern pros::Controller controller;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup liftMotors;
extern pros::MotorGroup intake;
extern pros::MotorGroup roller;
extern pros::MotorGroup arm;

extern pros::Imu imu;

extern lemlib::Chassis chassis;
