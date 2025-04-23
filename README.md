# Hexapod Robot

A simple hexapod robot based on ESP-IDF (ESP32 and FreeRTOS).

## Project Overview

This project implements a hexapod robot using ESP-IDF framework for ESP32 micro-controllers. The robot features six legs, each with three servos, allowing for various walking gaits and movements. The system uses PCA9685 PWM controllers to drive the servos and implements a gait control system to coordinate leg movements.

## System Architecture

The hexapod system is composed of several key components:

1. **Core Components**:
   - ESP32 micro-controller running FreeRTOS
   - PCA9685 PWM controllers for servo control
   - Multiple servos for leg articulation

2. **Software Components**:
   - Hexapod control system for leg kinematics
   - Servo control system for precise positioning
   - Gait control system for coordinated movement patterns
   - Task-based architecture using FreeRTOS

## Key Components

### Hexapod Control System

The hexapod control system handles the kinematics and movement of the robot's legs. It includes:

- **Coordinate to Servo Conversion**: Converts 3D coordinates to servo angles using inverse kinematics
- **Leg Position Control**: Controls the position of each leg in 3D space
- **Servo Calibration**: Adjusts servo angles to account for physical differences

```c
typedef struct {
    double x;
    double y;
    double z;
} Coordination;

typedef struct {
    uint8_t a;
    uint8_t b;
    uint8_t c;
} HexapodLegServoDegree;

HexapodLegServoDegree hexapod_leg_position_to_servo_degrees(Coordination coord);
void set_leg_angle(uint8_t leg_id, HexapodLegServoDegree degrees);
```

### Servo Control System

The servo control system manages the PCA9685 PWM controllers and servo motors:

- **I2C Communication**: Uses I2C protocol for communication with PCA9685 controllers
- **PWM Generation**: Generates PWM signals for precise servo control
- **Servo Angle Control**: Converts desired angles to PWM duty cycles

```c
void pca9685_init(uint8_t addr);
void set_servo_angle(uint8_t addr, uint8_t channel, uint16_t angle);
```

## Gait Control

The hexapod implements a gait control system with four basic gaits:

1. **Gait A**:
   - Leg group 0: Lowered and not moving forward (stance phase)
   - Leg group 1: Lowered and moving forward (swing phase)

2. **Gait B**:
   - Leg group 0: Raised and moving forward (swing phase)
   - Leg group 1: Lowered and not moving forward (stance phase)

3. **Gait C**:
   - Leg group 0: Lowered and moving forward (swing phase)
   - Leg group 1: Lowered and not moving forward (stance phase)

4. **Gait D**:
   - Leg group 0: Lowered and not moving forward (stance phase)
   - Leg group 1: Raised and moving forward (swing phase)

The gait control system cycles through these patterns to create a walking motion.

## Configuration

The hexapod system uses several configuration parameters:

- **Physical Parameters**:
  - Leg segment lengths (L1, L2, L3)
  - Core height
  - Leg rise height
  - Wingspan
  - Step size

- **Servo Parameters**:
  - Servo angle range (0-180 degrees)
  - Servo calibration factor

- **I2C Configuration**:
  - I2C pins (SCL, SDA)
  - I2C frequency (100kHz)
  - I2C port (I2C_NUM_0, I2C_NUM_1)

## Usage

The hexapod system is implemented using a task-based architecture:

1. **Initialization**:
   - Initializes I2C communication
   - Initializes PCA9685 controllers
   - Creates message queues for communication between tasks

2. **Gait Control Task**:
   - Cycles through different gait patterns
   - Sends leg position commands to servo control task

3. **Servo Control Task**:
   - Receives leg position commands
   - Converts coordinates to servo angles
   - Controls servos to achieve desired positions

## Building and Flashing

To build and flash the hexapod firmware:

1. Install ESP-IDF following the official documentation
2. Navigate to the project directory
3. Run `idf.py build` to compile the firmware
4. Run `idf.py flash` to flash the firmware to the ESP32
5. Run `idf.py monitor` to view the serial output

## License

This project is licensed under the MIT License - see the LICENSE file for details.
