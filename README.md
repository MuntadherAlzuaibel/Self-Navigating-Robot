# Self-Navigating Robot Using ESP32

## Overview
An autonomous 4WD robot built with an ESP32 microcontroller, capable of real-time obstacle avoidance and path planning. Uses a servo-mounted ultrasonic sensor for scanning ahead, and IR sensors for blind spot detection. No user control required.

## Features
- Automatic obstacle detection and avoidance using ultrasonic and IR sensors
- Scans left/right to select the safest path
- Autonomous path planning logic
- Real-time system status via onboard LCD
- All code and wiring diagrams included

## Hardware Used
- ESP32 microcontroller
- 4 DC motors with motor driver (L298N or similar)
- Ultrasonic distance sensor (HC-SR04 or equivalent)
- Servo motor (SG90 or equivalent)
- 2 IR sensors (for blind spots)
- 1602 I2C LCD display (optional)
- 6xAA battery pack
- Custom 3D-printed sensor mount

## Getting Started

### Hardware Setup
- Follow the included wiring diagram (see `Wiring/Wiring Diagram.png` or `Wiring/Wiring Diagram.fzz`) to assemble the circuit.
- Mount the ultrasonic sensor on the servo, and attach both to the robot chassis.
- Install all sensors and connect to the ESP32 as shown.

### Software Setup 
1. Clone this repository or download the code files.
2. Open the project in PlatformIO or Arduino IDE.
3. Install the required libraries (Wire, LiquidCrystal_I2C, etc.).
4. Uncomment all the LCD-related lines if you want to use a 16x2 LCD display.
5. **Note:** Your hardware may differ. Edit the code as needed for your components and wiring.
6. Upload the code to your ESP32.

## Usage
- Power on the robot using the battery pack.
- The robot will begin scanning for obstacles and navigating autonomously.
- Monitor real-time status and sensor readings on the onboard LCD display.

## Author
Muntadher Alzuaibel

