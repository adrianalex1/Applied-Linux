# Lab 3: Robot Telemanipulation and Workpiece Registration

**ME 4272 — Applied Robotics | Fall 2026**

## Overview
This lab implements robot telemanipulation and workpiece registration using ROS 2 and MoveIt 2 Servo. A game controller is used to manually position a robot-mounted laser pointer over target locations on a workpiece.

The recorded target positions are used to estimate the position and orientation of the workpiece relative to the robot's base frame using Arun's point-set registration method.

## Implementation
- Implemented game controller-based robot telemanipulation using MoveIt 2 Servo.
- Controlled end-effector translation in the XY plane while maintaining a downward-facing orientation.
- Used a laser pointer mounted to the robot's end-effector to identify target locations on a workpiece.
- Recorded five target positions in the robot's base coordinate frame.
- Accounted for the offset between the robot's end-effector and the workpiece surface.
- Implemented Arun's method to estimate the rigid transformation between the workpiece and robot base frames.
- Used four target points to calculate the registration transformation.
- Used the fifth target point to evaluate Target Registration Error (TRE).
- Calculated Fiducial Registration Error (FRE) to evaluate registration accuracy.
- Output the estimated workpiece position, orientation, FRE, and TRE.

## Technologies
- ROS 2
- C++
- MoveIt 2
- MoveIt 2 Servo
- Eigen
- Game Controller Teleoperation
- Rigid Point-Set Registration (Arun's Method)

## Running the Experiment
The experiment requires a configured ROS 2 robot environment, a connected game controller, and a robot-mounted laser pointer.

Start the game controller node:

```bash
ros2 run joy joy_node
```

For DENSO robots, launch the telemanipulation system:

```bash
ros2 launch workpiece_reg servo.launch.py model:=<YOUR_MODEL>
```

Then launch the registration node:

```bash
ros2 launch workpiece_reg workpiece_reg.launch.py model:=<YOUR_MODEL>
```

For other supported robots, launch the registration node using:

```bash
ros2 run workpiece_reg workpiece_reg
```

After starting MoveIt 2 Servo, use the game controller to align the laser pointer with each target and press the A button to record its position.

Once all five points have been recorded, the program estimates the workpiece transformation and reports the registration errors.

## Applications
This experiment demonstrates fundamental concepts used in robotic calibration, workpiece localization, and robot-assisted manufacturing, where accurate transformations between coordinate frames are necessary for positioning and manipulation tasks.
