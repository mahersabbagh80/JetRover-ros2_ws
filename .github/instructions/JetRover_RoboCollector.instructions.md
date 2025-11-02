---
applyTo: "**"
---

# JetRover RoboCollector Project Instructions

## Project Overview

**Project Name:** JetRover RoboCollector

The **JetRover RoboCollector** is a robotics project designed to demonstrate autonomous navigation, object recognition, and robotic manipulation using the HiWonder JetRover platform. The goal is for the JetRover to navigate within a defined indoor space (such as a room), locate colored blocks, pick them up using its robotic arm, and return them to a designated box or collection area.

This project serves as a hands-on introduction to the integration of **ROS 2**, **computer vision**, **path planning**, and **manipulator control** on a real robotic platform.

---

## Objectives

1. **Autonomous Navigation**

   - Enable the JetRover to map and navigate your room using SLAM (Simultaneous Localization and Mapping).
   - Plan and follow paths while avoiding obstacles.

2. **Object Detection and Color Recognition**

   - Detect and identify colored blocks using onboard camera sensors.
   - Classify blocks by color using OpenCV or a deep learning model.

3. **Arm Manipulation and Object Pickup**

   - Use inverse kinematics or position presets to control the JetRover arm.
   - Safely grip and lift blocks from the floor.

4. **Block Delivery and Placement**
   - Navigate back to a known location (the “home” box area).
   - Place the blocks into the box precisely.

---

## Required Components

- HiWonder **JetRover** platform (with Jetson Orin Nano or equivalent)
- **Camera module** (included with JetRover)
- **Bus servos** for robotic arm control
- **Colored blocks** for vision detection tasks
- **ROS 2 workspace** (configured under `~/JetRover/ros2_ws`)
- **Python 3.10+** and relevant ROS 2 packages
- **Copilot_Robotics_Guide.md** (for Copilot setup and workflow tips)

---

## Suggested Project Structure

```
JetRover_Project/
├── src/
│   ├── robocollector_navigation/      # Navigation nodes (SLAM, path planning)
│   ├── robocollector_vision/          # Color detection and block localization
│   ├── robocollector_manipulator/     # Arm control and pickup logic
│   ├── robocollector_controller/      # Central node to coordinate all modules
│   └── robocollector_msgs/            # Custom ROS2 message definitions
├── launch/
│   ├── robocollector_bringup.launch.py
│   └── simulation.launch.py
├── config/
│   ├── nav_params.yaml
│   └── vision_params.yaml
├── docs/
│   ├── Copilot_Robotics_Guide.md
│   └── JetRover_robocollector_Instructions.md
└── README.md
```

---

## Development Stages

1. **Stage 1 – Environment Setup**

   - Prepare ROS 2 workspace and ensure JetRover drivers are operational.
   - Verify that the robot arm and wheels respond to control commands.

2. **Stage 2 – Mapping and Navigation**

   - Implement SLAM to map your room environment.
   - Use `nav2` or a custom path planner for autonomous driving.

3. **Stage 3 – Vision and Object Detection**

   - Develop a color detection node using OpenCV.
   - Publish the position of detected blocks relative to the robot.

4. **Stage 4 – Arm Control and Pickup**

   - Program pickup sequences for each detected block position.
   - Add error recovery (e.g., retry pickup if the block is missed).

5. **Stage 5 – Return and Placement**

   - Command the robot to return to the box location.
   - Drop the block into the box accurately.

6. **Stage 6 – Integration and Testing**
   - Combine navigation, vision, and manipulation nodes.
   - Perform complete test runs in your room environment.

---

## Future Extensions

- Multi-block sorting by color or size.
- Speech command interface.
- Integration with a local web dashboard for live telemetry.
- Simulation and reinforcement learning for navigation optimization.

---

**Author:** Maher Alshirazi Alsabbagh  
**Date:** October 2025  
**Complementary File:** `Copilot_Robotics_Guide.md`

---

> Note: For a repository‑wide folder reference, see the “Workspace Overview (Repo‑level)” section in `Copilot_Robotics_Guide.instructions.md`.

