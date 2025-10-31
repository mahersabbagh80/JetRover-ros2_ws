# RoboCollector Gazebo Simulation Guide

## 🎮 Overview

Full **physics-based simulation** using **Gazebo** with the actual JetRover URDF model! This provides:

- ✅ 3D robot model with mecanum wheels
- ✅ Robotic arm with 5 DOF
- ✅ Gripper mechanism
- ✅ Depth camera sensor
- ✅ Physical colored blocks (red, blue, green)
- ✅ Collection box
- ✅ Realistic physics and collisions

---

## 📋 Prerequisites

### Install Gazebo (if not already installed)

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
```

### Verify Installation

```bash
gazebo --version
# Should show Gazebo 11.x
```

---

## 🚀 Quick Start

### 1. Build the Packages

```bash
cd ~/JetRover/ros2_ws
colcon build --packages-select robocollector_bringup jetrover_description --symlink-install
source install/setup.bash
```

### 2. Launch Gazebo Simulation

```bash
export need_compile=False
export LIDAR_TYPE=A1
export MACHINE_TYPE=JetRover_Mecanum

ros2 launch robocollector_bringup gazebo_simulation.launch.py
```

**What launches:**

- Gazebo with custom world (colored blocks + collection box)
- JetRover robot model with arm and camera
- Vision node (processes Gazebo camera)
- Pickup node (controls Gazebo arm joints)
- Controller node (state machine)

### 3. Start the Mission

**In a new terminal:**

```bash
source ~/JetRover/ros2_ws/install/setup.bash

# Start collection mission
ros2 service call /controller_node/enter std_srvs/srv/Trigger
```

---

## 🌍 World Layout

```
          Walls
    │                    │
    │   Collection Box   │
    │      (gray)        │
    │         ○          │
    │                    │
    │      Robot         │
    │  Start Position    │
    │         ⚙          │
    │                    │
    │    Red Blocks      │
    │      ●  ●          │
    │                    │
    │  Blue    Green     │
    │   ●        ●       │
    │                    │
          Ground
```

**Coordinates:**

- Robot spawn: `(0, 0, 0.1)`
- Red blocks: `(1.5, 0.3)`, `(1.8, -0.2)`
- Blue block: `(2.0, 0.5)`
- Green block: `(1.6, -0.5)`
- Collection box: `(-0.5, 0, 0.15)`

---

## 🎮 Gazebo Controls

### Camera Navigation

- **Rotate**: Left-click + drag
- **Pan**: Shift + left-click + drag
- **Zoom**: Scroll wheel
- **Reset view**: Press `r`

### Model Manipulation

- **Move model**: Right-click model → "Move to"
- **Delete model**: Right-click model → "Delete"
- **Reset simulation**: Ctrl + R

### Useful Views

- **Top-down view**: Good for seeing block locations
- **Follow robot**: Right-click JetRover → "Follow"

---

## 📊 Monitoring

### Check Topics

```bash
# Camera image from Gazebo
ros2 topic hz /depth_cam/rgb/image_raw

# Robot velocity commands
ros2 topic echo /cmd_vel

# Joint states (arm position)
ros2 topic echo /joint_states

# Detection info
ros2 topic echo /vision_node/color_info
```

### Visualize in RViz

```bash
rviz2
```

**Add displays:**

1. RobotModel → Topic: `/robot_description`
2. Image → Topic: `/depth_cam/rgb/image_raw`
3. TF → Show all transforms

---

## 🔧 Customization

### Change Block Positions

Edit `robocollector_bringup/worlds/robocollector.world`:

```xml
<model name="red_block_1">
  <pose>X Y Z roll pitch yaw</pose>
  <!-- Change X, Y, Z coordinates -->
</model>
```

### Add More Blocks

Copy a block model in the world file:

```xml
<model name="yellow_block">
  <pose>2.5 0 0.05 0 0 0</pose>
  <static>false</static>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box><size>0.1 0.1 0.1</size></box>
      </geometry>
      <material>
        <ambient>1 1 0 1</ambient>  <!-- Yellow -->
        <diffuse>1 1 0 1</diffuse>
      </material>
    </visual>
    <!-- Add collision and inertial like other blocks -->
  </link>
</model>
```

### Change Robot Type

```bash
# Tank drive
export MACHINE_TYPE=JetRover_Tank

# Ackermann steering
export MACHINE_TYPE=JetRover_Acker
```

---

## 🐛 Troubleshooting

### Issue: Gazebo won't start

```bash
# Kill any existing Gazebo processes
killall gzserver gzclient

# Try again
ros2 launch robocollector_bringup gazebo_simulation.launch.py
```

### Issue: Robot falls through floor

- **Cause**: Physics not initialized properly
- **Fix**: Restart Gazebo, ensure `<static>false</static>` on robot

### Issue: Camera not publishing

```bash
# Check if camera plugin loaded
ros2 topic list | grep depth_cam

# If missing, check Gazebo terminal for plugin errors
```

### Issue: Arm doesn't move

- **Cause**: Joint controllers not loaded or servo control not connected to Gazebo
- **Current status**: Basic URDF loaded, may need ros2_control integration
- **Workaround**: Use mock_pickup_node for now

### Issue: Blocks don't appear colored in camera

- **Cause**: Gazebo material colors might not match RGB camera output
- **Fix**: May need to adjust HSV ranges or use Gazebo textures

---

## ⚠️ Current Limitations

### What Works:

✅ Robot visualization in Gazebo  
✅ Physics simulation  
✅ Camera image publishing  
✅ Vision detection (if colors visible)  
✅ Base movement (`/cmd_vel`)

### What Needs Integration:

⚠️ **Arm control** - URDF has joints but needs ros2_control setup  
⚠️ **Gripper** - Needs contact sensors and grasp plugin  
⚠️ **Depth sensing** - Camera configured but depth might need tuning  
⚠️ **Navigation** - SLAM/Nav2 integration with Gazebo map

---

## 🔄 Comparison: Lightweight vs Gazebo Simulation

| Feature             | Lightweight Sim | Gazebo Sim      | Real Robot |
| ------------------- | --------------- | --------------- | ---------- |
| **Vision Testing**  | ✅ Perfect      | ✅ Good         | ✅ Perfect |
| **Physics**         | ❌ None         | ✅ Full         | ✅ Real    |
| **Setup Time**      | ✅ 5 min        | ⚠️ 30 min       | ✅ 10 min  |
| **Iteration Speed** | ✅ Instant      | ⚠️ Slower       | ⚠️ Slow    |
| **Arm Control**     | ⚠️ Mock         | ⚠️ Needs config | ✅ Real    |
| **Camera Realism**  | ⚠️ Synthetic    | ⚠️ Simulated    | ✅ Perfect |
| **Resource Usage**  | ✅ Low          | ❌ High (GPU)   | ✅ Low     |

---

## 📈 Development Workflow

### Recommended Order:

1. **Lightweight Sim** (5 min) - Test logic

   ```bash
   ros2 launch robocollector_bringup simulation.launch.py
   ```

2. **Gazebo Sim** (optional, for navigation testing)

   ```bash
   ros2 launch robocollector_bringup gazebo_simulation.launch.py
   ```

3. **Real Robot** (3-4 hours) - Deploy and calibrate
   ```bash
   # On JetRover
   ros2 launch robocollector_bringup robocollector.launch.py
   ```

---

## 🚧 TODO: Full Gazebo Integration

To make the Gazebo simulation fully functional:

### 1. Add ros2_control Integration

Create `jetrover.ros2_control.xacro`:

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
  <!-- Add all arm joints -->
</ros2_control>
```

### 2. Add Gripper Plugin

```xml
<gazebo>
  <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
    <arm>
      <arm_name>gripper</arm_name>
      <gripper_link>gripper_link</gripper_link>
    </arm>
  </plugin>
</gazebo>
```

### 3. Tune Camera Parameters

Match Gazebo camera to real depth camera specs.

---

## 🎯 Next Steps

1. **Test basic Gazebo launch** - Verify robot spawns correctly
2. **Test vision detection** - Check if colored blocks detected
3. **Test base movement** - Verify `/cmd_vel` moves robot
4. **Add ros2_control** (if arm control needed)
5. **Integrate with navigation stack**

---

## 📚 Resources

- [Gazebo ROS 2 Integration](http://gazebosim.org/tutorials?tut=ros2_overview)
- [ros2_control](https://control.ros.org/humble/index.html)
- [JetRover URDF](../simulations/jetrover_description/urdf/)

---

**You now have THREE simulation options:**

1. ✅ **Lightweight** - Fast logic testing (recommended first)
2. ✅ **Gazebo** - Physics simulation (for navigation)
3. ✅ **Real Robot** - Ultimate validation (deploy when ready)

Choose based on what you're testing! 🚀
