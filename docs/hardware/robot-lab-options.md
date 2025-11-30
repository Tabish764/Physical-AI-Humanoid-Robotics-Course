---
id: robot-lab-options
title: Robot Lab Options - Physical Platforms
sidebar_position: 3
---

## Objectives
- Explore various physical robot platforms suitable for humanoid robotics research and development.
- Understand the considerations for choosing between commercial humanoid robots and custom builds.
- Analyze the benefits and drawbacks of cloud-based vs. on-premise robot lab setups.

## Introduction
Setting up a robot lab for physical AI and humanoid robotics involves critical decisions regarding hardware platforms and infrastructure. This section provides an overview of available robot options, focusing on commercial humanoid platforms like Unitree Go2 and G1, and discusses the architectural choices between running experiments in the cloud or on-premise.

## Physical Robot Platforms

### Commercial Humanoid Robots
Commercial humanoid robots offer advanced hardware, integrated software, and often comprehensive SDKs, accelerating research and development.

-   **Unitree Go2 / Go2 Pro**: A quadruped robot (often used as a base for humanoid research) known for its agility, advanced locomotion capabilities, and open SDK. Excellent for learning dynamic movement and advanced control.
    -   **Pros**: Highly stable, dynamic locomotion, widely adopted for research, good community support.
    -   **Cons**: Not a true bipedal humanoid, limited manipulation capabilities without add-ons.
-   **Unitree G1 / G1 Pro**: A compact bipedal humanoid robot designed for research and education. Features advanced joint control and a focus on bipedal locomotion and basic manipulation.
    -   **Pros**: True bipedal platform, designed for education/research, active development.
    -   **Cons**: Newer platform, potentially less mature ecosystem compared to quadruped robots.
-   **Other Humanoids (e.g., Agility Robotics Digit, Boston Dynamics Atlas/Stretch)**: High-end research platforms with superior capabilities but significantly higher cost and limited accessibility.

### Custom Builds
For specific research goals or educational purposes, custom-built humanoid robots offer flexibility in hardware and software design.

-   **Pros**: Full control over hardware components, cost-effective for specific tasks, deep understanding of robot mechanics.
-   **Cons**: Requires significant engineering effort, complex integration, limited pre-built software.

## Robot Lab Infrastructure

### Cloud-Based Robot Labs
Leveraging cloud infrastructure for robotics development involves running simulations and sometimes even controlling physical robots remotely.

-   **Benefits**: Scalability (spin up many simulation instances), access to high-performance computing (GPUs), remote accessibility, reduced local hardware burden.
-   **Use Cases**: Large-scale synthetic data generation, distributed AI model training, CI/CD for robotics software, remote collaboration.
-   **Considerations**: Data transfer costs, latency for real-time control, security implications.

### On-Premise Robot Labs
Traditional robot labs involve local physical hardware and infrastructure.

-   **Benefits**: Minimal latency for real-time control, direct physical interaction with robots, full control over network and hardware, reduced cloud costs for ongoing operations.
-   **Use Cases**: Low-latency control experiments, hardware-in-the-loop (HIL) testing, experiments requiring direct human-robot interaction or unique physical setups.
-   **Considerations**: High initial hardware investment, maintenance overhead, space requirements, power consumption.

## Key Takeaways
- Unitree Go2/G1 offer accessible platforms for dynamic robotics research.
- Choose between commercial robots for integrated solutions or custom builds for flexibility.
- Cloud labs provide scalability and HPC for simulations, while on-premise labs offer low-latency control and direct interaction.