---
id: bipedal-locomotion
title: Bipedal Locomotion for Humanoid Robots
sidebar_position: 2
---

## Objectives
- Understand the challenges and principles of bipedal locomotion.
- Explore different control strategies for achieving stable walking.
- Learn about common gait generation techniques for humanoid robots.

## Challenges of Bipedal Locomotion
Bipedal locomotion, or walking on two legs, is inherently unstable compared to wheeled or quadrupedal locomotion. Humanoid robots face several significant challenges:
- **Balance and Stability**: Maintaining balance while moving, especially during dynamic phases like single-support.
- **Underactuation**: The robot's center of mass often cannot be directly controlled, leading to complex control problems.
- **Ground Contact**: Managing transitions between different contact states (e.g., heel strike, toe-off) and dealing with uneven terrain.
- **Energy Efficiency**: Designing gaits that minimize energy consumption.

## Principles of Bipedal Locomotion

### Zero Moment Point (ZMP)
The Zero Moment Point (ZMP) is a fundamental concept in bipedal locomotion. It represents the point on the ground where the net moment of all forces (gravity, inertial, contact) acting on the robot is zero. For stable walking, the ZMP must remain within the robot's support polygon (the area on the ground enclosed by the contact points of the feet).

### Center of Mass (CoM) Control
Controlling the robot's Center of Mass (CoM) trajectory is critical for maintaining balance. By carefully planning the CoM movement, the ZMP can be kept within the support polygon, ensuring stability.

## Control Strategies

### Model Predictive Control (MPC)
Model Predictive Control (MPC) is a popular control strategy for bipedal locomotion. It uses a dynamic model of the robot to predict its future behavior and optimize control inputs (e.g., joint torques) over a short time horizon to achieve desired CoM and ZMP trajectories while satisfying constraints.

### Whole-Body Control
Whole-body control approaches consider the entire robot's kinematics and dynamics to coordinate all joints simultaneously, enabling complex and agile movements while maintaining balance and interacting with the environment.

## Gait Generation Techniques

### Trajectory Optimization
This involves optimizing a sequence of joint angles and forces over time to generate a stable and efficient walking gait, often using numerical optimization methods.

### Pattern Generators
Central Pattern Generators (CPGs) are biologically inspired models that can generate rhythmic movements, which can be adapted for walking gaits.

### Reinforcement Learning
Reinforcement learning techniques can be used to train humanoid robots to walk by having them learn optimal control policies through trial and error in simulated environments.

## Key Takeaways
- Bipedal locomotion is challenging due to inherent instability and complex dynamics.
- Concepts like ZMP and CoM control are crucial for maintaining balance.
- Control strategies like MPC and whole-body control, along with gait generation techniques, enable stable and efficient walking for humanoid robots.
