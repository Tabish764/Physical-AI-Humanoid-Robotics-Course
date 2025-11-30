---
id: urdf-sdf-formats
title: URDF and SDF Formats
sidebar_position: 2
---

## Objectives
- Differentiate between URDF and SDF robot description formats.
- Understand the structure and components of URDF files.
- Learn when to use URDF versus SDF for robot modeling.

## Introduction to URDF
Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. It represents the kinematic and dynamic properties of a robot, including its visual and collision properties. URDF is primarily used for defining a single robot and its hierarchical link-joint structure.

### URDF Structure
A URDF file typically consists of `<link>` and `<joint>` elements.
- **`<link>`**: Describes a rigid body with its inertial, visual, and collision properties.
- **`<joint>`**: Describes the kinematic and dynamic properties of the connection between two links.

## Introduction to SDF
Simulation Description Format (SDF) is a more comprehensive XML format designed for describing robots, environments, and even entire worlds for simulation in Gazebo. Unlike URDF, SDF can describe multiple robots, static objects, environmental properties (like lighting and terrain), and more complex physics.

### SDF Structure
SDF also uses `<model>`, `<link>`, and `<joint>` elements, but extends them with more features relevant to simulation, such as `<gravity>`, `<wind>`, and more detailed physics properties.

## URDF vs. SDF
| Feature             | URDF                               | SDF                                         |
|---------------------|------------------------------------|---------------------------------------------|
| **Purpose**         | Robot description (kinematics/dynamics) | Full simulation environment (robot, world, sensors, physics) |
| **Scope**           | Single robot                       | Multiple robots, environments, static objects |
| **ROS Integration** | Primary ROS robot description      | Used by Gazebo, converted to URDF for ROS   |
| **Physics**         | Limited                            | Extensive (friction, gravity, sensors)      |

## When to Use Which
- **Use URDF** when you need to describe a robot for ROS applications, such as motion planning, visualization in RViz, and basic control.
- **Use SDF** when you need to simulate a robot within a complex environment in Gazebo, requiring detailed physics and environmental interactions. Often, a URDF model is converted to SDF for Gazebo simulation.

## Key Takeaways
- URDF is for robot description in ROS, focusing on kinematics and dynamics.
- SDF is for comprehensive world and robot simulation in Gazebo, including environmental physics.
- Choose the format based on whether you are primarily working with ROS tools or Gazebo simulation.
