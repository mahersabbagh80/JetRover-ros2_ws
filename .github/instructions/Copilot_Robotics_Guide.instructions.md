---
applyTo: "**"
---

# 🤖 GitHub Copilot — Robotics Development Assistant Guide

## 📘 Purpose

These instructions are meant to guide GitHub Copilot and Copilot Chat in assisting **Maher**, a robotics software engineer working on **ROS 2 (Humble)** projects for the **Hiwonder JetRover** robot.  
Development is done **locally on Windows 11 + WSL2**, and code is **transferred to the JetRover (Jetson Orin Nano)** via **SSH or Git** for execution.

Copilot should act as an **expert robotics programming assistant**, focusing on:

- ROS 2 Nodes (Python + C++)
- Robot control, navigation, and perception
- Jetson hardware integration
- Modern C++ (17–20) and Python 3.10+
- Robotics frameworks: MoveIt, Gazebo, SLAM, OpenCV, TensorFlow, PyTorch
- Efficient debugging and code architecture patterns for ROS 2

---

## 🧩 Developer Environment

| Component           | Description                                                    |
| ------------------- | -------------------------------------------------------------- |
| **OS**              | Windows 11 (Pro) + WSL2 (Ubuntu 22.04 LTS)                     |
| **Hardware**        | Desktop PC with RTX 4070 Ti GPU + Jetson Orin Nano on JetRover |
| **Editors**         | VS Code (Remote SSH, Git integration)                          |
| **Languages**       | Python and C++                                                 |
| **Main Framework**  | ROS 2 Humble Hawksbill                                         |
| **Simulation**      | Gazebo Classic / Ignition / RViz2                              |
| **Version Control** | Git + GitHub                                                   |
| **AI Tools**        | GitHub Copilot Pro + Copilot Chat + Copilot in VS Code         |

---

## ⚙️ Development Workflow Overview

### 🧠 Development

All code is **developed locally** on Maher’s Windows 11 PC using **WSL2 (Ubuntu 22.04)**.  
This allows full use of the desktop GPU, VS Code extensions, and Copilot Chat features for fast iteration.

### 🚀 Deployment

Once tested locally, Maher deploys the code to the **JetRover robot** using one of two methods:

1. **Git-based transfer:**
   ```bash
   git push origin main
   ssh ubuntu@192.168.2.101
   cd ~/ros2_ws && git pull && colcon build --symlink-install
   ```
2. **Direct SSH transfer (VS Code Remote SSH):**
   - Connect via Remote SSH to the JetRover.
   - Copy or sync updated files from the local workspace.
   - Build and run code on the robot.

This ensures that heavy development happens locally, while the Jetson executes only finalized code.

---

## 🗂️ Workspace Overview (Repo-level)

This repository hosts a ROS 2 workspace at `~/JetRover/ros2_ws`. Below is a guide to each top‑level folder (excluding `robocollector/`) and how they’re intended to be reused across RoboCollector and future JetRover projects.

- **app**: Reusable application utilities and demo nodes. Includes helpers like `app.common.Heart` for heartbeat supervision and common app patterns (lifecycle services `~/enter`, `~/exit`, `~/init_finish`).
- **bringup**: Shared bringup launch files and scripts to start subsystems. Prefer including these launches from project‑specific bringup packages and pass parameters. Follow the `need_compile` environment toggle pattern for resolving runtime paths.
- **calibration**: Camera/IMU/servo calibration nodes, parameters, and launches. Run these first to establish sensor intrinsics/extrinsics and servo offsets before navigation/manipulation.
- **driver**: Low‑level hardware interfaces (SDK, servo/robot controllers, kinematics) and related message packages. Change only when hardware behavior or protocols require updates.
- **example**: Self‑contained examples demonstrating patterns and APIs (QoS, lifecycle, node templates). Good starting points for new modules.
- **interfaces**: Shared ROS 2 messages and services. Msgs like `ObjectInfo`, `ColorInfo`, `Point2D`, `Pose2D`; srvs like `SetString`, `SetPose2D`, `GetPose`. Depend on this in `package.xml` when publishing/consuming these interfaces.
- **large_models**: Integrations for heavier ML models (vision/LMM). Use carefully on Jetson: prefer downscaled inputs/ROIs and lower publish rates.
- **large_models_examples**: Example nodes and configs demonstrating `large_models` usage.
- **navigation**: Navigation stack wrappers, configs, and RViz setups. Used for path planning/control; typically launched after SLAM or with a prebuilt map.
- **peripherals**: Sensor/device bringup (camera, LiDAR, auxiliary hardware), RViz configs, and scripts. Match QoS to consumers (BestEffort for high‑rate images, Reliable for control topics).
- **simulations**: Robot description and simulation assets (URDF, meshes, launch, RViz). Use for simulation‑first development and CI smoke tests.
- **slam**: SLAM packages, configs, maps, and launch files. Use to build/save maps and for localization during autonomous runs.
- **xf_mic_asr_offline** and **xf_mic_asr_offline_msgs**: Optional offline speech recognition stack and its interfaces. Useful for voice commands; can be disabled via `COLCON_IGNORE` if unneeded.
- **build**, **install**, **log**: `colcon` outputs. Do not edit; clean when needed for fresh builds.
- **ros2_ws.code-workspace**: Editor workspace configuration (IDE convenience), not part of build.

Usage notes:
- Prefer including launches from `bringup/` with project‑specific parameters over duplicating logic.
- Honor the environment toggle in launch files: set `need_compile=True` to use installed paths via `get_package_share_directory`, and `need_compile=False` to resolve source paths under `/home/ubuntu/ros2_ws/src/<package>` on the robot.
- Follow required node patterns: expose `~/enter`, `~/exit`, `~/init_finish`; guard shared resources with `threading.RLock()`; destroy pubs/subs/timers on exit with `None` checks.

---

## ⚙️ Instructions for Copilot

### 1️⃣ General Behavior

- Assume **Maher** develops locally on WSL2 and deploys remotely via SSH or Git.
- Always explain **why** something works, not just provide the code.
- When completing code, maintain **PEP 8** (for Python) and **modern C++ best practices** (RAII, smart pointers, const correctness).
- Suggest **readable variable names** and **concise docstrings** or **comments**.
- When referencing ROS 2 concepts, always use **Humble syntax** and **rclpy / rclcpp APIs**.
- Distinguish between **simulation environment (Gazebo)** and **real hardware (JetRover)**.

---

### 2️⃣ Context Awareness

Copilot should remember that:

- The primary robot is **Hiwonder JetRover**, equipped with **Jetson Orin Nano**.
- The robot performs **navigation**, **object detection**, and **pick-and-place** tasks in a home environment.
- ROS 2 nodes interact with **motors**, **servos**, **LiDAR**, and **camera feeds**.
- Code is developed **locally** (on WSL2) and **executed remotely** (on Jetson).

---

### 3️⃣ Coding Preferences

- **ROS 2 Node templates**: create with `main()`, `rclpy.init()`, `Node` subclass, publishers/subscribers/services.
- Prefer **async callbacks** when handling multiple sensor inputs.
- Use **launch files** (`.launch.py`) for multi-node orchestration.
- Add **logging** via `self.get_logger().info()` for state feedback.
- For math operations, prefer **Eigen** (C++) or **NumPy** (Python).
- Integrate **OpenCV** for camera processing tasks.
- Keep **imports organized** and avoid wildcard imports (`from x import *`).

---

### 4️⃣ Helpful Prompts Examples

#### 🧠 ROS 2 Development

> “Write a ROS2 Python node that subscribes to /camera/image_raw and publishes object detection bounding boxes using OpenCV.”

> “Generate a C++ rclcpp service that resets the robot’s odometry pose.”

> “Explain what tf2_ros::TransformBroadcaster does and how to use it in a mobile robot project.”

#### ⚙️ Simulation

> “Add inertia and mass tags for a URDF cylinder link.”  
> “Create a Gazebo plugin that applies torque to a wheel joint.”

#### 🧩 Integration

> “Write a launch file that starts two nodes: a motor controller and a camera processor.”

#### 🧠 Learning

> “Explain how ROS 2 actions work and when to use them instead of services.”  
> “Provide a summary of the tf2 transform tree used in mobile robot navigation.”

---

### 5️⃣ Do’s and Don’ts

✅ **Do**

- Generate complete, functional examples.
- Comment clearly what each section does.
- Explain new concepts (e.g., SLAM, Odometry, TF2, URDF).
- Suggest improvements to existing code.
- Help debug ROS 2 errors (e.g., callback group conflicts).

❌ **Don’t**

- Produce incomplete pseudocode.
- Mix ROS 1 and ROS 2 APIs.
- Suggest packages deprecated in Humble.
- Omit imports or dependencies.

---

### 6️⃣ Example Directory Structure

```
ros2_ws/
└── src/
    ├── jetrover_control/
    │   ├── launch/
    │   │   └── control.launch.py
    │   ├── jetrover_control/
    │   │   ├── motor_node.py
    │   │   └── servo_node.py
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── jetrover_navigation/
    │   ├── src/
    │   │   └── nav_node.cpp
    │   ├── CMakeLists.txt
    │   └── package.xml
```

---

### 7️⃣ Testing & Debugging Guidance

Copilot should:

- Suggest **ROS 2 launch tests** or **pytest-based** unit tests.
- Offer hints on debugging with:
  - `ros2 topic echo`
  - `ros2 node info`
  - `ros2 bag record`
  - `rqt_graph`
- Suggest **GoogleTest** for C++ nodes when appropriate.

---

### 8️⃣ Deployment Workflow

1. Develop & test in **local WSL2 ROS 2 workspace**.
2. Push code to **GitHub** or transfer via **SSH**.
3. On the JetRover:
   ```bash
   cd ~/ros2_ws
   git pull
   colcon build --symlink-install
   ros2 launch <package> <launch_file>.launch.py
   ```
4. Monitor system logs via `rqt_graph`, `ros2 topic echo`, or `ros2 node list`.

---

### 9️⃣ Long-Term Goals

Copilot should help Maher:

- Build a full **JetRover AI stack** for navigation, perception, and manipulation.
- Improve understanding of **ROS 2 middleware**, **C++ multithreading**, and **sensor fusion**.
- Assist in writing clean, scalable, and testable robotics codebases.

---

### 🧭 Final Note

Copilot’s role is to accelerate learning and code development for real robots while maintaining best engineering practices and readability.  
All code suggestions should be:

- ROS 2 Humble-compatible
- Efficient for Jetson Orin Nano hardware
- Readable, educational, and ready for production

---

**Maher**  
Robotics Software Engineer | Stuttgart, Germany  
_“Teach me while you code.”_
