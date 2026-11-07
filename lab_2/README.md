# Lab 2: Robot Kinematic Validation Using Optical Tracking

**ME 4272 — Applied Robotics | Fall 2026**

## Overview
This lab evaluates the accuracy of an industrial robot's kinematic model using an NDI Polaris optical tracking system. The robot follows a predefined circular trajectory while its measured end-effector pose is compared against the pose predicted by its kinematic model.

The experiment is based on a simplified application of ISO 9283, which defines methods for evaluating industrial robot performance.

## Implementation
- Integrated the NDI Polaris optical tracking system with ROS 2.
- Configured optical tracking markers to measure the robot base and end-effector poses.
- Established coordinate transformations between the optical tracker, robot base, and end-effector using TF2.
- Generated a circular end-effector trajectory in the YZ plane.
- Used KDL to solve inverse kinematics at each trajectory waypoint.
- Used MoveIt 2 to plan and execute joint-space motions.
- Compared the predicted end-effector pose against measurements from the optical tracker.
- Calculated the average and maximum translational and rotational errors over the trajectory.

## Technologies
- ROS 2
- C++
- MoveIt 2
- Kinematics and Dynamics Library (KDL)
- TF2
- NDI Polaris Optical Tracking System

## Running the Experiment
The experiment requires a configured ROS 2 robot environment and a connected NDI Polaris optical tracker.

Launch the optical tracking system:

```bash
ros2 launch polaris_config polaris.launch.yaml
```

For DENSO robots, launch the kinematic validation node:

```bash
ros2 launch robot_kin_validation robot_kin_validation.launch.py model:=<YOUR_MODEL>
```

For other supported robots:

```bash
ros2 run robot_kin_validation robot_kin_validation
```

The experiment measures discrepancies between the robot's predicted and measured end-effector poses to evaluate the accuracy of its kinematic model.
