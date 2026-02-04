/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       om31d                                                     */
/*    Created:      2/2/2026, 10:07:12 PM                                     */
/*    Description:  EXP project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "DriverControl.h"
#include "OdometryTracker.h"
#include "PIDController.h"
#include "vex.h"

using namespace	vex;

// A global instance of vex::brain used for printing to the EXP brain screen
vex::brain Brain;
vex::controller Controller1;
vex::inertial IMU = inertial();

vex::competition Competition;

DriverControl	driverControl(ControlType::ARCADE, 10, 2.0, 60, 40);

PIDController	drivePID(/*P, I, D*/);
PIDController	turnPID(/*P, I, D*/);

OdometryTracker	odometry;

// define your global instances of motors and other devices here
double			currentAngle = IMU.heading();

void	preAuton(void)
{
	IMU.calibrate();
}

void	autonomous(void)
{
	Brain.Screen.clearScreen();
	Brain.Screen.print("autonomous");
}

void	driver(void)
{
}

int	main(void)
{
}