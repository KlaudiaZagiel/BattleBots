# BattleBots Project - Line Following

## Project Overview

This repository contains the weekly assignments, which had as the purpose:
- Create familiarity with hardware
- Understand the different kinds of connections and access such connection allows us to have to control the robot behavior
- Prepare us develop the functions we need for the RaceDay itself

Besides the weekly assignments, we too present in this repository the final relay race code corresponding to our part of the race, which completes the goal of the project itself, which is to build an autonomous robot capable of:
- Detecting the raise of the racing flag, which signals the beginning of the race
- Go forward to take hold of the object with the use of the grippers
- Turn to find the line
- Follow a line using 8 line sensors
- Detecting and avoiding obstacles using an ultrasonic sensor
- Indicating movement direction using NeoPixel LEDs
- Detecting the drop-off zone
- Stopping automatically
- Drop off the object
- And go backwards to create some space for the next robot to be able to grab a hold of the object and continue the race

The repository is organized to show the full development process from weekly assignments to the final race implementation.

## Hardware Used

The robot uses the following hardware components:

- Arduino-compatible microcontroller
- 2 DC motors with motor driver
- 2 wheel rotation sensors / encoders
- 8 analog line sensors
- 1 ultrasonic distance sensor
- 3 push button
- 4 NeoPixel LEDs
- 1 servo motor for gripper control

## Pin Mapping (final version)

|         Component         | Pin |
|         ---------         | --- |
|    Left Motor Forward     | D10 |
|    Left Motor Backward    | D11 |
|    Right Motor Forward    | D9  |
|    Right Motor Backward   | D6  |
|   Left Encoder(rotation)  | D2  |
|  Right Encoder(rotation)  | D3  |
|    Ultrasonic Trigger     | D4  |
|      Ultrasonic Echo      | D13 |
|       Start Button        | D7  |
|   Line Sensor Most Left   | A7  |
|  Line Sensor Center Left  | A6  |
|  Line Sensor Right Left   | A5  |
|  Line Sensor Left Center  | A4  |
|  Line Sensor Right Center | A3  |
|  Line Sensor Left Right   | A2  |
|  Line Sensor Center Right | A1  |
|   Line Sensor Most Right  | A0  |
|       Gripper Servo       | D12 |

## Main Features

### 1. Line Following
The robot follows a black line on a white track using 8 analog sensors and a weighted position calculation to facilitate turns.
A proportional-derivative (PD) control method is used to adjust motor speeds.

### 2. Dynamic Speed Adjustment
The base speed changes depending on how sharp the curve is:
- High speed on straight sections
- Lower speed on medium curves
- Lowest speed on sharp turns

### 3. Obstacle Detection
The ultrasonic sensor is continuously measuring the distance in front of the robot-
If an obstacle is detected within the configured threshold, the robot interrupts line following and starts the obstacle avoidance sequence.

### 4. Obstacle Avoidance
To save us a little bit more time, we opted to avoid object in a triangle shape way instead of the previously built box shaped avoidance we used in week 2 assignment.
So, upon object detection the robot:
- Drives slightly backwards for space safety from the object
- Turns left around 45 degree
- Goes forward to avoid obstacle
- Turns right around 90 degrees
- Goes forward while seeking line again
- Upon line detection, obstacle avoidance stops and line following code continues

### 5. LED Indicators
NeoPixel LEDs are used to indicate turns and behaviors just like a car:
- Steering left turns LED's on the left side
- Steering right turns LED's on the right side
- Driving backwards turns LED's on the back in red color
- Driving forward the LED's no longer need to be turned on

### 6. Finish Line Detection
Once all line sensors detect black, the robot:
- Stops, getting out of Line Following state
- Opens grippers to drop object
- Goes backwards to create safe space for next robot to pick object and continue race

To save false positives of all sensors detecting the drop-off zone, we have given the sensors a configurable timer, so unless the robot detects black on all sensors after that time will it stop, saving us from having undesired behavior on situations where line crosses.

## Final Race Code

The final code used for the race is located in:
`RaceDayCode/RaceDayCode.ino`
This is the main file that should be uploaded to the robot for the final evaluation.

## Software / Library Requirements

The following arduino library is required:
- Adafruit NeoPixel
This library should be installed from the Arduino Library Manager before uploading the final race code.

## Calibration / Tuning Notes

Some values in the final code code may need some tuning depending on track conditions and/or robot behavior:
- `threshold` for line sensor detection
- `kp` and `kd` for line following turn behavior
- Obstacle distance threshold
- Wheel pulse counts for turns
- Motors speed for forward, reverse and search phases

These values were adjusted experimentally during testing and are all set at the top for easy access.

## Weekly Development Process

The weekly assignments folders documents the step-by-step development of the robot:
- Lights control
- Basic motor control
- Ultrasonic obstacle detection
- Obstacle avoidance performance
- Line sensor reading
- Line following behavior
- Gripper behavior testing

This structure shows the progression from simple subsystems tests, to the final autonomous robot, all supported by the final understanding of the importance of the control of robot-stage behaviors.

## How To Run

1. Open Arduino
2. Copy and paste the race day code:
   `RaceDayCode.ino`
3. Install required libraries:
   `Adafruit NeoPixel`
4. Connect the Arduino board via USB
5. Select the correct:
   ```
   - Board type
   - COM / Serial port
   ```
6. Upload the sketch
7. Place the robot on the track and press the start button

## Notes / Known Limitations

- Turn pulses values may need small adjustments depending on battery level and surface friction
- Line sensor threshold may need recalibration for different lighting conditions
- Obstacle avoidance performance depends on the exact geometry of the obstacle and track layout

## Authors

1º Year Students (2025-2026) of Group A - Subgroup 2:
 - Klaudia Zagiel
 - Jessica Camacho

Course project for:
 `IT - BattleBot (2025-2026)`

Institution:
 `NHL Stenden University Of Applied Sciences`

## Media

Place here a short video of the robot performing the whole final track?

## License

This repository is submitted for educational purposes as part of a course project.
