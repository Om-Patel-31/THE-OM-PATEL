/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       om31d                                                     */
/*    Created:      2/2/2026, 10:07:12 PM                                     */
/*    Description:  VEX EXP Robot Starting Template                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "DriverControl.h"
#include "OdometryTracker.h"
#include "PIDController.h"
#include "vex.h"

using namespace vex;

/*---------------------------------------------------------------------------*/
/*                          Device Configuration                             */
/*---------------------------------------------------------------------------*/
// Brain and Controller
vex::brain Brain;
vex::controller Controller1;

// Sensors - VEX EXP
vex::inertial IMU = inertial();
// Add more EXP sensors here:
// distance distSensor = distance(PORT5);
// optical colorSensor = optical(PORT6);
// bumper bumperSwitch = bumper(PORT7);
// rotation rotationSensor = rotation(PORT8);

// Motors - VEX EXP uses PORT1-PORT8
// Example:
// motor leftMotor = motor(PORT1, false);   // false = not reversed
// motor rightMotor = motor(PORT2, true);   // true = reversed
// motor armMotor = motor(PORT3, false);
// motor clawMotor = motor(PORT4, false);

// Note: VEX EXP motors don't have gear ratios like V5
// Speed is controlled through voltage percentage

/*---------------------------------------------------------------------------*/
/*                          Control Systems                                  */
/*---------------------------------------------------------------------------*/
vex::competition Competition;

// Driver Control: ARCADE, TANK, SPLIT_ARCADE, CURVATURE, SINGLE_STICK
DriverControl driverControl(ControlType::ARCADE, 10, 2.0, 60, 40);

// PID Controllers (tune P, I, D values for your robot)
PIDController drivePID(0.5, 0.0, 0.1);  // Forward/Backward movement
PIDController turnPID(1.0, 0.0, 0.15);  // Turning movement
PIDController armPID(0.8, 0.0, 0.1);    // Example: arm/lift control

// Odometry Tracker
OdometryTracker odometry;

/*---------------------------------------------------------------------------*/
/*                          Global Variables                                 */
/*---------------------------------------------------------------------------*/
double currentHeading = 0.0;
bool autonomousFinished = false;

/*---------------------------------------------------------------------------*/
/*                          Helper Functions                                 */
/*---------------------------------------------------------------------------*/

// Wait for IMU calibration to complete
void waitForIMU()
{
	Brain.Screen.clearScreen();
	Brain.Screen.setCursor(1, 1);
	Brain.Screen.print("IMU Calibrating...");
	
	while (IMU.isCalibrating())
	{
		wait(50, msec);
	}
	
	Brain.Screen.clearScreen();
	Brain.Screen.setCursor(1, 1);
	Brain.Screen.print("IMU Ready!");
	wait(500, msec);
}

// Drive straight for a distance using PID
void driveDistance(double inches, double maxSpeed = 50.0)
{
	// Implementation example - customize for your motors
	// VEX EXP motors track position in degrees by default
	/*
	drivePID.reset();
	
	// For EXP: use motor.position(degrees) or rotation sensor
	double startPosition = (leftMotor.position(degrees) + rightMotor.position(degrees)) / 2.0;
	double degreesPerInch = 28.65;  // Adjust based on wheel diameter
	double targetDegrees = startPosition + (inches * degreesPerInch);
	
	while (!drivePID.atTarget(targetDegrees - getCurrentPosition()))
	{
		double currentPos = (leftMotor.position(degrees) + rightMotor.position(degrees)) / 2.0;
		double error = targetDegrees - currentPos;
		double output = drivePID.calculate(error, 0.02);
		
		// Clamp output to maxSpeed
		output = std::fmax(-maxSpeed, std::fmin(maxSpeed, output));
		
		leftMotor.spin(forward, output, percent);
		rightMotor.spin(forward, output, percent);
		
		wait(20, msec);
	}
	
	leftMotor.stop(brake);
	rightMotor.stop(brake);
	*/
}

// Turn to an absolute heading using PID
void turnToHeading(double targetHeading, double maxSpeed = 40.0)
{
	// Implementation example - customize for your robot
	/*
	turnPID.reset();
	
	while (!turnPID.atTarget(targetHeading - IMU.heading()))
	{
		double error = targetHeading - IMU.heading();
		
		// Normalize error to -180 to 180
		while (error > 180) error -= 360;
		while (error < -180) error += 360;
		
		double output = turnPID.calculate(error, 0.02);
		output = std::fmax(-maxSpeed, std::fmin(maxSpeed, output));
		
		leftMotor.spin(forward, output, percent);
		rightMotor.spin(forward, -output, percent);
		
		wait(20, msec);
	}
	
	leftMotor.stop(brake);
	rightMotor.stop(brake);
	*/
}

// Update odometry in a background task
int odometryTask()
{
	while (true)
	{
		// Update odometry with motor position or rotation sensors and IMU
		/*
		// VEX EXP: Convert motor degrees to distance
		double wheelDiameter = 2.75;  // inches - adjust for your wheels
		double degreesToInches = (wheelDiameter * 3.14159) / 360.0;
		
		double leftDist = leftMotor.position(degrees) * degreesToInches;
		double rightDist = rightMotor.position(degrees) * degreesToInches;
		double heading = IMU.heading();
		
		odometry.update(leftDist, rightDist, heading);
		*/
		
		wait(10, msec);
	}
	return 0;
}

// Display robot information on brain screen
void displayInfo()
{
	Brain.Screen.clearScreen();
	Brain.Screen.setCursor(1, 1);
	Brain.Screen.print("Battery: %.1f%%", Brain.Battery.capacity());
	Brain.Screen.setCursor(2, 1);
	Brain.Screen.print("Heading: %.1f deg", IMU.heading());
	// Add more information as needed
	/*
	Brain.Screen.setCursor(3, 1);
	Brain.Screen.print("X: %.1f  Y: %.1f", odometry.x, odometry.y);
	*/
}

/*---------------------------------------------------------------------------*/
/*                          Competition Functions                            */
/*---------------------------------------------------------------------------*/

// Pre-Autonomous: Runs ONCE when the program starts
void preAuton(void)
{
	// Calibrate IMU
	IMU.calibrate();
	waitForIMU();
	
	// Reset sensors
	odometry.reset();
	
	// Reset PID controllers
	drivePID.reset();
	turnPID.reset();
	
	// Display initial info
	displayInfo();
	
	// Start odometry tracking task (optional)
	// task odometryUpdate(odometryTask);
}

// Autonomous: Runs for 60 seconds in VEX EXP competition
void autonomous(void)
{
	Brain.Screen.clearScreen();
	Brain.Screen.setCursor(1, 1);
	Brain.Screen.print("Autonomous Running...");
	
	// Add your autonomous routine here
	// Example:
	// driveDistance(24);      // Drive forward 24 inches
	// turnToHeading(90);      // Turn to 90 degrees
	// driveDistance(-12);     // Drive backward 12 inches
	
	autonomousFinished = true;
}

// Driver Control: Runs during driver control period
void driver(void)
{
	Brain.Screen.clearScreen();
	Brain.Screen.setCursor(1, 1);
	Brain.Screen.print("Driver Control");
	
	while (true)
	{
		// Get motor values from driver control
		auto [leftSpeed, rightSpeed] = driverControl.getMotorValues(Controller1);
		
		// Apply to motors - customize for your robot
		/*
		leftMotor.spin(forward, leftSpeed, percent);
		rightMotor.spin(forward, rightSpeed, percent);
		*/
		
		// Additional button controls - VEX EXP Controller
		// Available buttons: ButtonEUp, ButtonEDown, ButtonFUp, ButtonFDown
		//                   ButtonL1, ButtonL2, ButtonR1, ButtonR2
		// Example: Arm/Claw control
		/*
		if (Controller1.ButtonL1.pressing())
		{
			armMotor.spin(forward, 50, percent);  // Arm up
		}
		else if (Controller1.ButtonL2.pressing())
		{
			armMotor.spin(reverse, 50, percent);  // Arm down
		}
		else
		{
			armMotor.stop(hold);  // Hold position when released
		}
		
		if (Controller1.ButtonR1.pressing())
		{
			clawMotor.spin(forward, 100, percent);  // Close claw
		}
		else if (Controller1.ButtonR2.pressing())
		{
			clawMotor.spin(reverse, 100, percent);  // Open claw
		}
		else
		{
			clawMotor.stop(hold);
		}
		*/
		
		// Update display periodically
		if (Brain.Timer.value() > 1.0)  // Every second
		{
			displayInfo();
			Brain.Timer.reset();
		}
		
		wait(20, msec);  // Prevent CPU overload
	}
}

/*---------------------------------------------------------------------------*/
/*                          Main Program                                     */
/*---------------------------------------------------------------------------*/
int main(void)
{
	// Set up competition callbacks
	Competition.autonomous(autonomous);
	Competition.drivercontrol(driver);
	
	// Run pre-autonomous setup
	preAuton();
	
	// Prevent main from exiting with an infinite loop
	while (true)
	{
		wait(100, msec);
	}
}