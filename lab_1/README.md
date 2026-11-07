# Lab 1: Robot Kinematics and Circular Trajectory Simulation

**ME 4272 — Applied Robotics | Fall 2026**

## Overview
This lab focuses on modeling and simulating a 7-degree-of-freedom (7-DoF) robotic manipulator using ROS 2. The robot is modeled using the Universal Robot Description Format (URDF), and inverse kinematics is used to command the end-effector along a circular trajectory.

## Implementation
- Created a URDF/Xacro model of a 7-DoF serial manipulator with revolute and prismatic joints.
- Defined the robot's kinematic structure, joint limits, and coordinate frames.
- Implemented inverse kinematics using the Kinematics and Dynamics Library (KDL).
- Generated a circular end-effector trajectory in the YZ plane with a radius of 0.1 m.
- Discretized the trajectory into 360 waypoints at 1-degree intervals.
- Solved inverse kinematics at each waypoint to determine the required joint positions.
- Published joint states at 100 Hz to simulate the robot's motion.
- Visualized the robot and verified its circular end-effector trajectory using RViz.

## Technologies
- ROS 2
- C++
- URDF / Xacro
- Kinematics and Dynamics Library (KDL)
- RViz

## Running the Simulation
```bash
colcon build
source install/setup.bash
ros2 launch example_robot example_robot.launch.py
```

The simulation demonstrates how inverse kinematics can be used to generate coordinated joint motion while maintaining a desired end-effector orientation.
