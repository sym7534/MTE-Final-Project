#include "vex.h"
using namespace vex;

brain Brain;
inertial BrainInertial = inertial();
motor MotorLeft      = motor(PORT3,  false);
motor MotorRight     = motor(PORT6,  true);
motor MotorDispense  = motor(PORT1,  false);
optical OpticalSensor = optical(PORT4);  

// ---------------------- pid rotation functions
double clamp(double power, double minPower, double maxPower)
{
  double temp = fabs(power);
  if (temp < minPower) temp = minPower; // corrects if the current power is
  if (temp > maxPower) temp = maxPower; // outside of our limits
  return copysign(temp, power);
}

double convertAngle(double angle)
{
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0; // makes a within -180 to 180 
  return angle;
}

bool rotateToHeadingPID(double target,
                        double kp=1.2, double ki=0.0, double kd=0.15,
                        int timeout=2000, double tolerance =2.0)
{
  const double maxPower = 70.0;        // max motor power
  const double minPower = 7.0;         // min motor power
  const double integralLimit = 20.0;   // Threshold to prevent integral windup when far from target
  const int loopTime = 15;             // update frequency (in ms)

  MotorLeft.setStopping(brake);
  MotorRight.setStopping(brake);
  MotorLeft.spin(forward);
  MotorRight.spin(forward);

  timer t; // initializes timer t
  
  double error = convertAngle(target - BrainInertial.heading(degrees));
  // converts target angle to a lowest angle to target
  // IN DEGREES
  // IN DEGREES

  double prevError  = error;  // for derivative calc
  double integral = 0.0;      // accumulated error over time (integral)

  // ---------------------- Main Control Loop ----------------------
  // Continue until within tolerance or timeout is reached
  while (fabs(error) > tolerance && t.time(msec) < timeout)
  {
    // INTEGRAL: RIEMANN'S SUM OF ERROR * TIME ELAPSED
    if (fabs(error) < integralLimit)
    // only start calculating integral as it approaches the limit
    {
        integral += error * (loopTime/1000.0);
    }
    else
    {
        integral = 0.0; // Reset integral when far from target
    }

    // DERIVATIVE: RATE OF CHANGE OF ERROR
    double derivative = (error - prevError) / (loopTime/1000.0);

    // PID CALCULATION
    // pos u = cw rotation, neg u = ccw rotation
    double u = kp*error + ki*integral + kd*derivative;

    // uses clamp() to make sure u is within min max values for power
    double leftPower  = clamp( u, minPower, maxPower);
    double rightPower = clamp(-u, minPower, maxPower);

    MotorLeft.setVelocity(leftPower,  percent);
    MotorRight.setVelocity(rightPower, percent);

    wait(loopTime, msec); // waits until next loop of while loop, based on
    //                     set value for loopDt

    prevError = error; // stores current error for next derivative calculation
    error = convertAngle(target - BrainInertial.heading(degrees)); 
    // get new error
  }

  MotorLeft.stop();
  MotorRight.stop();

  // returns false if it failed to reach the target within timeout time
  return fabs(error) <= tolerance;
}