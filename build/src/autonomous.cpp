#include "vex.h"
#include "autonomous.h"

using namespace vex; // <--- ADD THIS LINE
// You can use all devices defined in main.cpp
// because vex.h links them across files

double kP = 0.25;
double kI = 0.02;
double kD = 0.02;

// Turn Function
void turnTo(double targetHeading)
{
    double error = 0, prevError = 0, integral = 0, derivative = 0, motorPower = 0;
    bool settled = false;
    double settleTime = 0, timeOut = 0, maxTime = 3000;

    while (!settled)
    {
        double currentPosition = Inertial.rotation(degrees);
        error = targetHeading - currentPosition;

        if (fabs(error) < 15)
            integral = integral + error;
        else
            integral = 0;

        derivative = error - prevError;
        prevError = error;

        motorPower = (error * kP) + (integral * kI) + (derivative * kD);

        if (motorPower > 12.0)
            motorPower = 12.0;
        if (motorPower < -12.0)
            motorPower = -12.0;

        LeftMotorGroup.spin(forward, motorPower, voltageUnits::volt);
        RightMotorGroup.spin(reverse, motorPower, voltageUnits::volt);

        if (fabs(error) < 1.0 && fabs(derivative) < 0.5)
            settleTime += 20;
        else
            settleTime = 0;

        if (settleTime > 200)
            settled = true;

        timeOut += 20;
        if (timeOut > maxTime)
            settled = true;

        wait(20, msec);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}
