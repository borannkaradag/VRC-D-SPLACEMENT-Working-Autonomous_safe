/* Module:       main.cpp                                                  */
/* Author:       boran                                                     */
/* Created:      10/7/2025, 7:34:58 AM                                     */
/* Description:  V5 project with 3 Pistons & Configurable Buttons          */
/* */

#include "vex.h"
#include <cmath>

using namespace vex;

// --- Competition & Brain ---
competition Competition;
brain Brain;

// --- PID Constants ---
// ADDED to fix the 'turnTo' function in autonomous
const double kP_Turn = 0.5;
const double turn_tolerance = 3.0;
const int turn_max_speed = 60;
// -----------------------

// --- DEVICE DEFINITIONS ---
// Drive Motors
motor LeftMotorA = motor(PORT4, ratio6_1, true);
motor LeftMotorB = motor(PORT3, ratio6_1, true);
motor LeftMotorC = motor(PORT1, ratio6_1, true);
motor_group LeftMotorGroup(LeftMotorA, LeftMotorB, LeftMotorC);

motor RightMotorA = motor(PORT11, ratio6_1, false);
motor RightMotorB = motor(PORT16, ratio6_1, false);
motor RightMotorC = motor(PORT17, ratio6_1, false);
motor_group RightMotorGroup(RightMotorA, RightMotorB, RightMotorC);

// Sensors
vex::inertial Inertial = inertial(PORT5);

// Mechanisms
motor intake = motor(PORT7, ratio18_1, true);
motor roller = motor(PORT18, ratio6_1, true);

// Pneumatics
digital_out Piston = digital_out(Brain.ThreeWirePort.A);
digital_out PistonB = digital_out(Brain.ThreeWirePort.E);
digital_out PistonC = digital_out(Brain.ThreeWirePort.B);

// Controller
controller Controller = controller();

distance Distance_sensor = distance(PORT12);
distance Distance_sensor_front = distance(PORT15);

// ============================================================================
// --- CONTROL CONFIGURATION (CHANGE BUTTONS HERE) ---
// ============================================================================
const vex::controller::button &btn_PistonA = Controller.ButtonY;
const vex::controller::button &btn_PistonB = Controller.ButtonX;
const vex::controller::button &btn_PistonC = Controller.ButtonA;

const vex::controller::button &btn_IntakeIn = Controller.ButtonL1;
const vex::controller::button &btn_IntakeOut = Controller.ButtonL2;

const vex::controller::button &btn_RollerFwd = Controller.ButtonR2;
const vex::controller::button &btn_RollerRev = Controller.ButtonR1;
// ============================================================================

// --- Global Variables ---
volatile bool isPairA_Healthy = true;
volatile bool isPairB_Healthy = true;
volatile bool isPairC_Healthy = true;
vex::task peripheral_task_auton;

// --- Piston State Variables ---
bool isPistonExtended = false;
bool isButtonAPressed = false;

bool isPistonBExtended = false;
bool isButtonBPressed = false;

bool isPistonCExtended = false;
bool isButtonCPressed = false;

// --- Functions ---
void togglePistonA()
{
  if (isPistonExtended)
  {
    Piston.set(false);
    isPistonExtended = false;
  }
  else
  {
    Piston.set(true);
    isPistonExtended = true;
  }
}

void togglePistonB()
{
  if (isPistonBExtended)
  {
    PistonB.set(false);
    isPistonBExtended = false;
  }
  else
  {
    PistonB.set(true);
    isPistonBExtended = true;
  }
}

void togglePistonC()
{
  if (isPistonCExtended)
  {
    PistonC.set(false);
    isPistonCExtended = false;
  }
  else
  {
    PistonC.set(true);
    isPistonCExtended = true;
  }
}

// --- NEW FUNCTION: Turn Robot to Absolute Heading (Fixes Compile Error) ---
void turnTo(double targetHeading)
{
  // Normalize the target heading to be between 0 and 360
  while (targetHeading >= 360)
    targetHeading -= 360;
  while (targetHeading < 0)
    targetHeading += 360;

  double error = 0;
  int motorPower = 0;

  // Use a P-Loop to turn until the robot is within the tolerance
  while (true)
  {
    double currentHeading = Inertial.heading();
    error = targetHeading - currentHeading;

    // Calculate the shortest path
    if (error > 180)
    {
      error -= 360;
    }
    else if (error < -180)
    {
      error += 360;
    }

    // Exit condition
    if (std::abs(error) <= turn_tolerance)
    {
      break;
    }

    // Calculate motor power
    motorPower = (int)(error * kP_Turn);

    // Clamp motor power
    if (motorPower > turn_max_speed)
      motorPower = turn_max_speed;
    if (motorPower < -turn_max_speed)
      motorPower = -turn_max_speed;

    // Apply power (Left positive, Right negative = turn right; Left negative, Right positive = turn left)
    LeftMotorGroup.setVelocity(motorPower, percent);
    RightMotorGroup.setVelocity(-motorPower, percent);

    LeftMotorGroup.spin(vex::forward);
    RightMotorGroup.spin(vex::forward);

    vex::wait(20, msec);
  }

  // Stop the motors
  LeftMotorGroup.stop(brakeType::brake);
  RightMotorGroup.stop(brakeType::brake);
}
// -------------------------------------------------------------------------

/* Pre-Autonomous Functions */
void pre_auton(void)
{
  Inertial.calibrate();
  while (Inertial.isCalibrating())
  {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Inertial is calibrating");
    wait(100, msec);
  }
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Inertial is calibrated.");
}
void drivePID(double target_distance_mm, double motor_power, double threshold) // Use the parameter passed in
{
  double kP_drive = 0.5;                                        // Proportional constant for driving
  double current_distance = Distance_sensor.objectDistance(mm); // Use the passed sensor
  double error = current_distance - target_distance_mm;

  // Loop until the error is small enough
  while (fabs(error) > threshold)
  {
    // 1. RE-READ the sensor inside the loop
    double current_distance = Distance_sensor.objectDistance(mm); // Use the passed sensor
    // 2. Update the error
    error = current_distance - target_distance_mm;

    // 3. Calculate power (Proportional control)
    double motorPower = error * kP_drive;

    // 4. Cap the voltage (VEX motors max at 12.0V)
    if (motorPower > 11.0)
      motorPower = -motor_power;
    if (motorPower < -11.0)
      motorPower = motor_power;

    // 5. Apply to motors
    LeftMotorGroup.spin(forward, motorPower, volt);
    RightMotorGroup.spin(forward, motorPower, volt);

    wait(10, msec);
  }

  LeftMotorGroup.stop(brake);
  RightMotorGroup.stop(brake);
}
void drivePID_FRONT(double target_distance_mm, double motor_power, double threshold)
{
  // We use a negative kP because to INCREASE distance (backward),
  // we need NEGATIVE motor power.
  double kP_drive = -0.7;

  // Safety: Stop the loop after 3 seconds even if target isn't reached
  uint32_t startTime = Brain.Timer.system();

  while (Brain.Timer.system() - startTime < 1100)
  {
    double current_distance = Distance_sensor_front.objectDistance(mm);

    // If the sensor sees "nothing" or an error, it often returns a very high value.
    // This check prevents the robot from haywire movements.
    if (current_distance > 2000)
      break;

    double error = target_distance_mm - current_distance;

    if (std::abs(error) <= threshold)
    {
      break;
    }

    double motorPower = error * kP_drive;

    // Clamp the voltage
    if (motorPower > motor_power)
      motorPower = motor_power;
    if (motorPower < -motor_power)
      motorPower = -motor_power;

    // Apply to motors
    LeftMotorGroup.spin(forward, motorPower, volt);
    RightMotorGroup.spin(forward, motorPower, volt);

    wait(20, msec);
  }

  LeftMotorGroup.stop(brake);
  RightMotorGroup.stop(brake);
}

void PID_straight(vex::distance PID_distance_sensor, double target_distance_mm, double max_motor_power, double error_margin, double targeted_heading, double kP_DriveHeading, double kP, double kI, double kD, double delta_t, double sign)
{

  float current_distance = PID_distance_sensor.objectDistance(mm);
  float actual_error = target_distance_mm - current_distance;
  double total_error = 0;
  double last_error = 0;

  while (std::abs(actual_error) > error_margin)
  {
    current_distance = PID_distance_sensor.objectDistance(mm);
    actual_error = target_distance_mm - current_distance;
    double dt = delta_t / 1000;

    double heading_error = targeted_heading - Inertial.heading();
    double heading_correction = heading_error * kP_DriveHeading;

    float P = actual_error * kP;
    if (std::abs(actual_error) < 50)
    {
      total_error += actual_error;
    }
    else
    {
      total_error = 0;
    }
    double I = total_error * kI;

    float derivative = (actual_error - last_error) / dt;
    float D = derivative * kD;
    double motor_output = P + I + D;

    if (motor_output > max_motor_power)
      motor_output = max_motor_power;
    if (motor_output < -max_motor_power)
      motor_output = -max_motor_power;

    double left_power = motor_output + heading_correction;
    double right_power = motor_output - heading_correction;

    LeftMotorGroup.spin(forward, motor_output * sign, volt);
    RightMotorGroup.spin(forward, motor_output * sign, volt);

    last_error = actual_error;
    wait(delta_t, msec);
  }
  LeftMotorGroup.stop(brake);
  RightMotorGroup.stop(brake);
}

void autonomous_left_four_score(void)
{
  PistonC.set(true);
  roller.setVelocity(100, percent);
  intake.setVelocity(-100, percent);

  roller.spin(forward);
  intake.spin(forward);

  drivePID(400.0, 8.0, 20.0);
  turnTo(90);
  drivePID(410, 4.75, 7.0);
  turnTo(180);
  wait(1, msec);
  roller.setVelocity(600, rpm);
  roller.spin(forward);
  drivePID_FRONT(450, 5.4, 10);

  for (int i = 0; i < 5; i++)
  {
    LeftMotorGroup.spin(reverse, 5.0, volt);
    RightMotorGroup.spin(reverse, 5.0, volt);
    wait(100, msec);
    LeftMotorGroup.spin(forward, 7.0, volt);
    RightMotorGroup.spin(forward, 7.0, volt);
    wait(100, msec);
  }
  wait(700, msec);
  LeftMotorGroup.stop(brake);
  RightMotorGroup.stop(brake);
  // ---------------------------------
  // --- REPLACE THE OLD BACKWARD CODE WITH THIS ---
  drivePID_FRONT(710, 9, 20);
  wait(300, msec);
  turnTo(360); // This will only run AFTER the 800 degrees are finished
  PistonC.set(false);
  drivePID(825, 6, 20);

  PistonB.set(true);
}

int firePistonC()
{
  wait(1200, msec); // Optional: small delay before firing
  PistonC.set(true);
  return 0;
}

void seven_goal_auton()
{
  roller.spin(forward, 11, volt);
  intake.spin(reverse, 11, volt);
  turnTo(350);
  vex::task pistonTask(firePistonC);
  PID_straight(Distance_sensor, 830.0, 10, 10.0, 350, 0.15, 0.1, 0.01, 0.005, 10, 1);
  PID_straight(Distance_sensor, 930.0, 10, 10.0, 350, 0.15, 0.1, 0.01, 0.005, 10, 1);
  turnTo(250);
  PID_straight(Distance_sensor_front, 680.0, 10, 10.0, 250, 0.15, 0.2, 0.01, 0.005, 10, -1);
  turnTo(270);
  PID_straight(Distance_sensor_front, 720.0, 10, 10.0, 270, 0.15, 0.2, 0.01, 0.005, 10, -1);
  turnTo(180);
  PistonC.set(true);
  PID_straight(Distance_sensor_front, 470.0, 10, 10.0, 180, 0.15, 0.1, 0.01, 0.005, 10, -1);
  for (int i = 0; i < 5; i++)
  {
    LeftMotorGroup.spin(reverse, 5.0, volt);
    RightMotorGroup.spin(reverse, 5.0, volt);
    wait(100, msec);
    LeftMotorGroup.spin(forward, 7.0, volt);
    RightMotorGroup.spin(forward, 7.0, volt);
    wait(100, msec);
  }
  wait(700, msec);
  PID_straight(Distance_sensor_front, 710.0, 10, 10.0, 350, 0.15, 0.1, 0.01, 0.005, 10, -1);
  turnTo(0); // This will only run AFTER the 800 degrees are finished
  PistonC.set(false);
  PID_straight(Distance_sensor, 825.0, 10, 10.0, 350, 0.15, 0.3, 0.01, 0.005, 10, 1);
  PistonB.set(true);
}

void autonomous(void)
{
  seven_goal_auton();
}

/* User Control Task - UPDATED WITH VARIABLE BUTTONS */
int motors_pneumatics(void)
{
  while (true)
  {

    // --- PISTON A CONTROL ---
    if (btn_PistonA.pressing())
    {
      if (!isButtonAPressed)
      {
        togglePistonA();
        isButtonAPressed = true;
      }
    }
    else
    {
      isButtonAPressed = false;
    }

    // --- PISTON B CONTROL ---
    if (btn_PistonB.pressing())
    {
      if (!isButtonBPressed)
      {
        togglePistonB();
        isButtonBPressed = true;
      }
    }
    else
    {
      isButtonBPressed = false;
    }

    // --- PISTON C CONTROL (NEW) ---
    if (btn_PistonC.pressing())
    {
      if (!isButtonCPressed)
      {
        togglePistonC();
        isButtonCPressed = true;
      }
    }
    else
    {
      isButtonCPressed = false;
    }

    // --- ROLLER (FIXED VELOCITY/DIRECTION) ---
    if (btn_RollerFwd.pressing())
    {
      roller.setVelocity(600, rpm);
      roller.spin(vex::forward);
    }
    else if (btn_RollerRev.pressing())
    {
      roller.setVelocity(600, rpm); // Set positive velocity
      roller.spin(vex::reverse);    // Spin in reverse
    }
    else
    {
      roller.stop(brakeType::coast);
    }

    // --- INTAKE (FIXED VELOCITY/DIRECTION) ---
    if (btn_IntakeIn.pressing())
    {
      intake.setVelocity(200, rpm);
      intake.spin(vex::forward);
    }
    else if (btn_IntakeOut.pressing())
    {
      intake.setVelocity(200, rpm); // Set positive velocity
      intake.spin(vex::reverse);    // Spin in reverse
    }
    else
    {
      intake.stop(brakeType::coast);
    }

    wait(20, msec);
  }

  return (0);
}

void usercontrol(void)
{
  peripheral_task_auton.stop();
  vex::task peripheral_task(motors_pneumatics);

  while (1)
  {
    double value = Distance_sensor.objectDistance(distanceUnits::mm);
    double value_front = Distance_sensor_front.objectDistance(distanceUnits::mm);
    // Lambda for curved input
    auto curveInput = [](int input)
    {
      double scaled = input / 100.0;
      return scaled * std::abs(scaled) * 100;
    };

    int forward = curveInput(Controller.Axis3.position());
    int turn = curveInput(Controller.Axis1.position());

    double turnScale = 1.0 - (std::abs(forward) / 100.0) * 0.2;
    int leftSpeed = forward + turn * turnScale;
    int rightSpeed = forward - turn * turnScale;

    // Clamp
    if (leftSpeed > 100)
      leftSpeed = 100;
    if (leftSpeed < -100)
      leftSpeed = -100;
    if (rightSpeed > 100)
      rightSpeed = 100;
    if (rightSpeed < -100)
      rightSpeed = -100;

    LeftMotorGroup.setVelocity(leftSpeed, percent);
    RightMotorGroup.setVelocity(rightSpeed, percent);
    LeftMotorGroup.spin(vex::forward);
    RightMotorGroup.spin(vex::forward);

    wait(5, msec);
  }
}

int main()
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true)
  {
    wait(100, msec);
  }
}
