# Autonomous Position Tracking Robot

Embedded robotics project implementing real-time position estimation using wheel encoders and inertial sensing.

The system uses a **dual-microcontroller architecture (ESP32 + Arduino Uno)** to separate sensor processing, state estimation, and motor control. Wheel encoder interrupts and IMU data are used to estimate the robot’s pose, while PID control and a navigation controller generate motor commands.

This project was developed for **ELEC6101 – Autonomous Vehicles I** at the University of New Haven.

---

# System Overview

Mobile robots operating indoors cannot rely on GPS for localization. Instead, they must estimate position through **dead reckoning**, integrating sensor measurements over time.

This project investigates a practical implementation of dead-reckoning navigation using:

- wheel encoder odometry for distance estimation
- inertial sensing for heading estimation
- closed-loop motor control
- wireless telemetry for monitoring and debugging

Experimental testing demonstrated that the odometry system could estimate position with approximately **5–10 cm error over 1 meter of travel** under controlled conditions.

---

# Hardware Architecture

Main components:

- **ESP32**
  - encoder interrupt processing
  - IMU acquisition
  - odometry computation
  - navigation algorithms
  - Bluetooth communication

- **Arduino Uno**
  - dedicated motor controller
  - PWM motor driver signals
  - receives wheel speed commands via I2C

- **DFRobot FIT0450 DC motors with quadrature encoders**
  - 960 ticks per revolution

- **MPU6050 IMU**
  - accelerometer and gyroscope
  - digital low-pass filtering

The robot is built on a **Keyestudio 4WD V2 chassis**.

---

# Software Architecture

The control software follows a layered architecture:

1. **Sensor Acquisition**
   - encoder signals captured via interrupt service routines
   - IMU data read over I2C

2. **State Estimation**
   - encoder counts converted to wheel velocities
   - differential-drive kinematic model updates position
   - complementary filter used for heading estimation

3. **Control**
   - PID controllers regulate wheel speeds
   - high-level controller generates navigation commands

4. **Communication**
   - Bluetooth interface allows real-time commands and telemetry

The ESP32 performs all estimation and navigation logic, while the Arduino Uno acts as a dedicated motor driver controller.

---

# Control Modes

The robot supports several operating modes:

### DRIVE
Maintains a commanded wheel speed using independent PID controllers.

Command example: GO X

---

### TURN
Rotates the robot by a specified angle using heading feedback.

Command example: TURN X

---

### NAVIGATION
Drives the robot toward a target coordinate using an input-output state feedback linearization controller.

Command example: TO X Y


---

### AUXILIARY COMMANDS

STOP, ZERO, CAL, SET PID Kx X, SET IOSFL Kx X


These commands allow calibration, tuning, and debugging during development.

---

# Experimental Results

Several experiments were conducted to evaluate the sensing and estimation pipeline.

### Straight Line Test (1 meter)

| Trial | X (m) | Y (m) | θ (rad) |
|------|------|------|------|
| 1 | 0.931 | 0.008 | 0.007 |
| 2 | 0.895 | 0.033 | 0.015 |
| 3 | 0.953 | 0.040 | 0.038 |

These tests show consistent encoder-based distance estimation with small positional error under controlled motion conditions.

---

### Arc Motion Test

Target pose:

x = 1.00 m
y = 0.50 m
theta = 1.571 rad


Measured results closely matched expected position and orientation, demonstrating successful integration of encoder and IMU data.

---

# Limitations

While the sensing and estimation pipeline performed well, overall navigation performance was limited by drivetrain characteristics.

Major issues included:

- large motor dead zone
- static friction
- wheel slip during turning
- voltage drops under high motor load

These factors reduced the resolution available to the control system and prevented smooth low-speed corrections during autonomous navigation.

---

# Future Improvements

Potential improvements include:

- higher-quality motors with reduced deadband
- improved drivetrain torque characteristics
- extended Kalman filter sensor fusion
- obstacle detection using vision or range sensors
- full autonomous parking behavior

---

# Repository Structure

autonomous-position-tracking-robot/  

Documentation/  
    - Report.pdf  
    - BlockDiagram.png  

  Firmware/  
    - esp32/  
    - arduino/  

  Media/  
    - BreakoutBoard_no_ESP32.jpg  
    - BreakoutBoard_with_ESP32.jpg  
    - DFRobot_FIT0450_w_Encoder.jpg  
    - ModifiedLowerChassis.jpg  
    - Robot.jpg  
    - TestSetup.jpg  
    - UI.png  

README.md  

---

# Key Technical Concepts Demonstrated

- embedded systems design
- interrupt-driven sensor processing
- multi-microcontroller architecture
- encoder odometry
- inertial sensing
- PID motor control
- mobile robot kinematics
- wireless telemetry

---

# Author

John Renner  
M.S. Electrical Engineering  
University of New Haven
