---
id: edge-computing
title: Edge Computing Requirements for Robotics
sidebar_position: 2
---

## Objectives
- Understand the role of edge computing in robotics and AI applications.
- Learn about recommended hardware for edge deployments, specifically NVIDIA Jetson Orin.
- Identify key software considerations for developing on edge platforms.

## Introduction
Edge computing is critical for robotics, allowing real-time processing of sensor data and immediate decision-making closer to the source, reducing latency and reliance on cloud resources. This section details the hardware and software specifications for edge devices suitable for AI and humanoid robotics development, focusing on NVIDIA Jetson Orin platforms and Intel RealSense cameras.

## Hardware Specifications

### Edge AI Platform (NVIDIA Jetson Orin)
NVIDIA Jetson Orin modules provide powerful AI computing at the edge, ideal for deploying trained models for perception, navigation, and control in robots.

-   **Recommended**: NVIDIA Jetson Orin Nano (8GB/16GB) or Jetson Orin NX (8GB/16GB) Developer Kit
-   **Minimum**: NVIDIA Jetson Xavier NX or Jetson TX2 (for basic projects)

### Sensors
Robots require robust sensor suites for environmental perception and interaction.

-   **Depth Camera (Recommended)**: Intel RealSense D435i or D455 for high-quality RGB-D data (essential for SLAM, object detection, and manipulation).
-   **Lidar (Optional)**: RPLidar S1/S2 or similar for 360-degree environmental mapping and navigation.
-   **IMU (Integrated)**: Inertial Measurement Units are typically integrated into robot platforms or advanced IMU sensors can be added for precise pose estimation.

### Power Supply
Stable and sufficient power is crucial for edge devices, especially in mobile robotic platforms.

-   **Recommended**: Dedicated power supply with adequate wattage for the Jetson module and all connected peripherals.
-   **Portable**: High-capacity power bank (e.g., 20,000mAh+) for mobile deployments.

## Software Specifications

### Operating System
NVIDIA JetPack SDK provides the necessary OS (Ubuntu), CUDA, cuDNN, and other libraries optimized for Jetson platforms.

-   **Recommended**: JetPack 5.x or newer (Ubuntu 20.04 LTS or 22.04 LTS based)

### Development Tools
-   **ROS 2**: Humble Hawksbill or Iron Irwini (compatible with JetPack and textbook content).
-   **NVIDIA Container Toolkit**: For running GPU-accelerated Docker containers on Jetson.
-   **AI Frameworks**: TensorFlow, PyTorch, or NVIDIA TensorRT for deploying optimized AI models.
-   **OpenCV**: For computer vision tasks.
-   **Python**: Python 3.8+ (included with JetPack).

## Key Takeaways
- Edge computing with platforms like NVIDIA Jetson Orin is vital for real-time AI and robotics.
- Integrate robust depth cameras (e.g., Intel RealSense) for environmental perception.
- Ensure proper software setup with NVIDIA JetPack and ROS 2 for optimal performance and development.