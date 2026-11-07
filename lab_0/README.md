# Lab 0: Mass-Spring-Damper Simulation

**ME 4272 — Applied Robotics | Fall 2026**

## Overview
This lab implements a mass-spring-damper simulation using ROS 2 and C++. The system models the dynamic response of a mass subjected to a sinusoidal external force using a state-space representation and Euler numerical integration.

## Implementation
- Developed two ROS 2 nodes for force generation and dynamic simulation.
- Implemented a custom ROS 2 message type (`FloatStamped`) for publishing timestamped data.
- Generated a sinusoidal force input with configurable amplitude and frequency.
- Modeled the mass-spring-damper system using state-space equations and the Eigen linear algebra library.
- Implemented Euler integration to calculate the mass position over time.
- Configured a YAML launch file to initialize both nodes with adjustable system parameters.
- Visualized the applied force and resulting mass displacement using RQT Plot.

## Technologies
- ROS 2
- C++
- Eigen
- RQT Plot
- YAML

## Running the Simulation
```bash
colcon build
source install/setup.bash
ros2 launch mass_spring_damper mass_spring_damper.launch.yaml
```

The simulation publishes the applied force to `/force` and the calculated mass position to `/mass_position`.
