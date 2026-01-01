#include "vex.h"

// --- Standard VEX Library Includes ---
using namespace vex;

// --- Drive Motors and Groups (The original error culprits) ---

// Motor Group Declarations (These resolve the main errors in autonomous.cpp)
extern motor_group LeftMotorGroup;
extern motor_group RightMotorGroup;

// Individual Motor Declarations (Needed if used directly in other files)
extern motor LeftMotorA;
extern motor LeftMotorB;
extern motor LeftMotorC;
extern motor RightMotorA;
extern motor RightMotorB;
extern motor RightMotorC;

extern vex::distance Distance_sensor;

// --- Sensors ---
extern inertial Inertial;
extern gps GPS;

// --- Mechanisms ---
extern motor intake;
extern motor roller;

// --- Pneumatics (Digital Outputs) ---
// Note: digital_out is a common alias or the correct class for a VEX solenoid/piston.
extern digital_out Piston;
extern digital_out PistonB;
extern digital_out PistonC;

// --- Controller ---
extern controller Controller;

extern distance Distance_sensor;

// --- Initialization Function (Standard VEX function) ---
/**
 * Used to initialize code/tasks settings
 */
void vexcodeInit(void);