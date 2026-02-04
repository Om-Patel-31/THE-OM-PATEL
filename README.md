# THE-OM-PATEL

A VEX EXP robotics project featuring advanced control systems, odometry tracking, and PID controllers.

## Project Overview

This project provides a comprehensive control framework for VEX EXP robots, including:
- Multiple driver control schemes (Arcade, Tank, Split Arcade, Curvature, Single Stick)
- Advanced joystick input shaping with exponential curves and deadzone handling
- PID (Proportional-Integral-Derivative) controllers for precise autonomous movement
- Odometry tracking system for position and heading estimation

## Features

### DriverControl System
The `DriverControl` class offers flexible driver control with multiple control schemes:
- **Arcade**: Left stick Y for drive, Right stick X for turn
- **Tank**: Left stick Y for left side, Right stick Y for right side
- **Split Arcade**: Left stick Y for drive, Left stick X for turn
- **Curvature**: Arcade with velocity-based turn scaling
- **Single Stick**: Single stick for both drive and turn

Features:
- Configurable deadzone to eliminate controller drift
- Exponential curve shaping for precise low-speed control
- Adjustable turn sensitivity for normal and slow-turn modes
- Input clamping and smoothing

### PID Controller
A robust PID controller implementation for autonomous movements:
- Configurable P, I, and D gains
- Integral windup protection
- Error tolerance checking
- Individual term tracking for debugging
- Reset functionality

### Odometry Tracker
Position tracking system for autonomous navigation:
- Real-time position estimation (x, y coordinates)
- Heading tracking using IMU sensor
- Distance and angle calculations to target points
- Manual position override capability
- Reset functionality

## Project Structure

```
THE-OM-PATEL/
├── src/
│   └── main.cpp              # Main program entry point
├── include/
│   ├── DriverControl.h       # Driver control system
│   ├── PIDController.h       # PID controller implementation
│   ├── OdometryTracker.h     # Odometry tracking system
│   └── vex.h                 # VEX library header
├── build/                    # Compiled output
├── docs/                     # Documentation
├── vex/                      # VEX build system files
└── makefile                  # Build configuration
```

## Getting Started

### Prerequisites
- VEX VS Code Extension
- VEX EXP Brain
- VEX V5 Controller

### Building the Project
1. Open the project in VS Code
2. Press `Ctrl+Shift+P` and select "Build Project"
3. Or use the makefile: `make`

### Uploading to Robot
1. Connect your VEX EXP Brain via USB or wirelessly
2. Press `Ctrl+Shift+P` and select "Upload Project"

## Configuration

### DriverControl Setup
```cpp
DriverControl driverControl(
    ControlType::ARCADE,  // Control scheme
    10,                   // Deadzone (percent)
    2.0,                  // Exponential curve
    60,                   // Turn sensitivity (percent)
    40                    // Slow turn sensitivity (percent)
);
```

### PID Controller Setup
```cpp
PIDController drivePID(
    0.5,  // kP - Proportional gain
    0.0,  // kI - Integral gain
    0.1   // kD - Derivative gain
);
```

## Usage Examples

### Driver Control
```cpp
void driver() {
    // Get motor values based on control scheme
    auto [left, right] = driverControl.getMotorValues(Controller1);
    
    // Apply to motors
    leftMotor.spin(forward, left, percent);
    rightMotor.spin(forward, right, percent);
}
```

### Autonomous Movement with PID
```cpp
void autonomous() {
    // Move forward 24 inches
    double targetDistance = 24.0;
    double currentDistance = 0.0;
    
    while (!drivePID.atTarget(targetDistance - currentDistance)) {
        double error = targetDistance - currentDistance;
        double output = drivePID.calculate(error, 0.02);
        
        // Apply output to motors
        leftMotor.spin(forward, output, percent);
        rightMotor.spin(forward, output, percent);
        
        wait(20, msec);
    }
}
```

### Odometry Tracking
```cpp
// Update odometry in a loop
void updatePosition() {
    double leftDist = leftEncoder.position(inches);
    double rightDist = rightEncoder.position(inches);
    double heading = IMU.heading();
    
    odometry.update(leftDist, rightDist, heading);
}

// Navigate to a point
void driveToPoint(double targetX, double targetY) {
    double distance = odometry.distanceTo(targetX, targetY);
    double heading = odometry.headingTo(targetX, targetY);
    
    // Use PID controllers to reach target
    // ... implementation
}
```

## Author

**om31d**

Created: February 2, 2026

## License

This project is intended for educational and competitive robotics purposes.

## Contributing

Feel free to fork this project and submit pull requests with improvements or bug fixes.

## Acknowledgments

- VEX Robotics for the VEX EXP platform
- VEXcode VS Code Extension team
