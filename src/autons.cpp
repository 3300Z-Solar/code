#include "autons.hpp"
#include "robot.hpp"

using pros::delay;

// Which autonomous this build runs. Set this to 1-8, build, and upload to a
// program slot with the matching name below so the brain's program list and
// this file always agree, e.g.:
//   pros upload --slot 1 --name "Auton 1"
//   pros upload --slot 2 --name "Auton 2"
//   ...
// Change the number and re-upload to a different slot to give that slot a
// different auton — auton1()-auton8() below all stay in the code either way.
#define ACTIVE_AUTON 2

// Rename these to whatever you pass to `pros upload --name`. Index 0 is
// auton1, index 7 is auton8.
static const char* const AUTON_NAMES[8] = {
    "Right Side", "Close Side", "Auton 3", "Auton 4",
    "Auton 5", "Auton 6", "Auton 7", "Auton 8",
};

const char* activeAutonName() {
    return AUTON_NAMES[ACTIVE_AUTON - 1];
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(path_txt); // '.' replaced with "_" to make c++ happy

void auton1() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    liftMotors.move_voltage(5650);
    chassis.moveToPoint(0, 1.75, 1000, {.maxSpeed = 100});
    delay(720);
    liftMotors.brake();
    arm.move_voltage(-2900);
    roller.move_voltage(-1000);
    delay(300);
    arm.brake();
    chassis.moveToPose(16.77, -14, 270, 2200, {.forwards = false, .maxSpeed = 90}); // move to pose x:14.77, y:-7.74, facing west (270)
    liftMotors.move_voltage(-2000);
    delay(1800);
    liftMotors.brake();
    chassis.moveToPoint(11.2, -14, 3000, {.maxSpeed = 80});
    delay(700);
    arm.move_voltage(2800);
    delay(600);
    arm.brake();
    roller.move_voltage(2000);
    arm.move_voltage(-1400);
    delay(1100);
    arm.brake();
    roller.move_voltage(3000);
    chassis.moveToPose(1, -23, 0, 600, {.maxSpeed = 70});
    chassis.moveToPose(21.66, -36, -49.4, 1800, {.forwards = false, .maxSpeed = 85});
    arm.move_voltage(-3000);
    delay(2000);
    arm.move_voltage(7000);
    delay(800);
    chassis.turnToHeading(-141, 1400, {.maxSpeed = 70});
    arm.move_voltage(-2000);
    delay(1240);
    arm.brake();
    delay(400);
    chassis.moveToPose(21.2, -11.3, 180, 1200, {.forwards = false, .maxSpeed = 50});
    delay(700);
    roller.move_voltage(-2000);
    liftMotors.move_voltage(6000);
    delay(1000);
    liftMotors.brake();
}

void auton2() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 200, 5000, {.maxSpeed = 100});
    // TODO: fill in auton2
}

void auton3() {
    chassis.setPose(0, 0, 0);
    // TODO: fill in auton3
}

void auton4() {
    chassis.setPose(0, 0, 0);
    // TODO: fill in auton4
}

void auton5() {
    chassis.setPose(0, 0, 0);
    // TODO: fill in auton5
}

void auton6() {
    chassis.setPose(0, 0, 0);
    // TODO: fill in auton6
}

void auton7() {
    chassis.setPose(0, 0, 0);
    // TODO: fill in auton7
}

void auton8() {
    chassis.setPose(0, 0, 0);
    // TODO: fill in auton8
}

void runAutonomous() {
#if ACTIVE_AUTON == 1
    auton1();
#elif ACTIVE_AUTON == 2
    auton2();
#elif ACTIVE_AUTON == 3
    auton3();
#elif ACTIVE_AUTON == 4
    auton4();
#elif ACTIVE_AUTON == 5
    auton5();
#elif ACTIVE_AUTON == 6
    auton6();
#elif ACTIVE_AUTON == 7
    auton7();
#elif ACTIVE_AUTON == 8
    auton8();
#endif
}
