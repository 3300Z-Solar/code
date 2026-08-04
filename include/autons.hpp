#pragma once

void auton1();
void auton2();
void auton3();
void auton4();
void auton5();
void auton6();
void auton7();
void auton8();

// Runs whichever auton ACTIVE_AUTON (top of autons.cpp) is set to. Called
// from main.cpp's autonomous() — set ACTIVE_AUTON in autons.cpp to pick the
// program slot's auton, no need to touch main.cpp.
void runAutonomous();

// Name of the currently ACTIVE_AUTON, e.g. "Right Side". Used to print the
// active auton to the brain screen so you can confirm what's loaded without
// checking the code. Update AUTON_NAMES in autons.cpp to rename.
const char* activeAutonName();
