# RoboCollector Simulation Testing Guide

## 🎮 Overview

Test the RoboCollector system **WITHOUT real hardware** using simulated camera feeds and mock arm movements. This allows you to validate the vision detection, state machine logic, and system integration before deploying to the robot.

---

## 🏗️ What Gets Simulated

| Component            | Simulation Status  | How It Works                                                              |
| -------------------- | ------------------ | ------------------------------------------------------------------------- |
| **Camera Feed**      | ✅ Fully Simulated | `simulation_test_node` generates synthetic RGB images with colored blocks |
| **Vision Detection** | ✅ Real Code       | Actual `vision_node` processes simulated images                           |
| **State Machine**    | ✅ Real Code       | Actual `controller_node` runs full logic                                  |
| **Arm Control**      | ⚠️ Mock            | `mock_pickup_node` simulates timing without hardware                      |
| **Robot Movement**   | ⚠️ Velocity Only   | `/cmd_vel` published but no physics simulation                            |

---

## 🚀 Quick Start

### 1. Build Simulation Packages

```bash
cd ~/JetRover/ros2_ws
colcon build --packages-select robocollector_vision robocollector_manipulation robocollector_controller robocollector_bringup --symlink-install
source install/setup.bash
```

### 2. Launch Full Simulation

```bash
export need_compile=False
ros2 launch robocollector_bringup simulation.launch.py
```

**What you'll see:**

- 4 nodes starting: `simulation_camera`, `vision_node`, `pickup_node` (mock), `controller_node`
- OpenCV window showing simulated camera view with detection overlay
- Colored log output showing state transitions

### 3. Start the Mission

**In a new terminal:**

```bash
source ~/JetRover/ros2_ws/install/setup.bash

# Start the collection mission
ros2 service call /controller_node/enter std_srvs/srv/Trigger
```

**Expected behavior:**

1. Controller enters **SEARCHING** state
2. Simulated camera shows red block moving
3. Vision node detects block and publishes `ColorInfo`
4. Controller transitions through states:
   - SEARCHING → APPROACHING → ALIGNING → PICKING
5. Mock pickup node logs simulated arm movements
6. Controller transitions: RETURNING → PLACING → COMPLETE

---

## 🧪 Testing Scenarios

### Scenario 1: Basic Detection Test

**Test vision node alone:**

```bash
# Terminal 1: Start simulated camera
ros2 run robocollector_vision simulation_test_node

# Terminal 2: Start vision node with debug
ros2 run robocollector_vision vision_node --ros-args -p debug:=true -p target_color:=red

# Terminal 3: Activate vision
ros2 service call /vision_node/enter std_srvs/srv/Trigger

# Terminal 4: Monitor detections
ros2 topic echo /vision_node/color_info
```

**Expected output:**

```
color: red
x: 320
y: 240
width: 640
height: 480
```

### Scenario 2: State Machine Test

**Test controller logic:**

```bash
# Launch full system
ros2 launch robocollector_bringup simulation.launch.py

# In new terminal: Monitor state
ros2 topic echo /cmd_vel

# Start mission
ros2 service call /controller_node/enter std_srvs/srv/Trigger
```

**Watch for state transitions in logs:**

```
[controller_node] SEARCHING - rotating to find block
[controller_node] Block detected! Approaching...
[controller_node] Aligned! Moving to pickup position
[mock_pickup_node] [SIM] === PICKUP SEQUENCE ===
[controller_node] Pickup successful! Returning to base
[controller_node] Mission complete!
```

### Scenario 3: Multi-Color Test

**Change target color during runtime:**

```bash
# While system is running
ros2 service call /controller_node/set_target interfaces/srv/SetString "{data: 'blue'}"

# Stop and restart
ros2 service call /controller_node/exit std_srvs/srv/Trigger
ros2 service call /controller_node/enter std_srvs/srv/Trigger
```

### Scenario 4: Mock Arm Test

**Test manipulation sequences:**

```bash
# Terminal 1: Start mock pickup node
ros2 run robocollector_manipulation mock_pickup_node

# Terminal 2: Test sequences
ros2 service call /pickup_node/enter std_srvs/srv/Trigger
ros2 service call /pickup_node/home std_srvs/srv/Trigger
ros2 service call /pickup_node/pickup std_srvs/srv/Trigger
ros2 service call /pickup_node/place std_srvs/srv/Trigger
```

---

## 📊 Monitoring Tools

### RQT Graph - Visualize Node Connections

```bash
rqt_graph
```

### RQT Topic Monitor

```bash
rqt
# Plugins → Topics → Topic Monitor
```

### Command Velocity Monitor

```bash
ros2 topic echo /cmd_vel
```

### Detection Rate

```bash
ros2 topic hz /vision_node/color_info
```

---

## 🐛 Debugging

### Enable Debug Visualization

Edit `simulation.launch.py` or pass parameter:

```bash
ros2 run robocollector_vision vision_node --ros-args -p debug:=true
```

### Slow Down Simulation

Edit `simulation_test_node.py`:

```python
# Line ~40: Change timer frequency
self.timer = self.create_timer(0.5, self.publish_test_image)  # Was 0.1
```

### Check Service Availability

```bash
ros2 service list | grep -E "(vision|pickup|controller)"
ros2 service type /controller_node/enter
```

### Verify Node Status

```bash
ros2 node list
ros2 node info /controller_node
```

---

## 🎨 Customizing Simulation

### Change Block Color

Edit `simulation_test_node.py`:

```python
# Line ~58: Change color
color = (0, 255, 0)  # Green in RGB
color = (0, 0, 255)  # Blue in RGB
color = (255, 255, 0)  # Yellow in RGB
```

### Add Multiple Blocks

Edit `simulation_test_node.py`:

```python
# Draw multiple blocks
for i, col in enumerate([(255,0,0), (0,255,0), (0,0,255)]):
    x_offset = 150 + i * 200
    cv2.rectangle(image, (x_offset-50, 200), (x_offset+50, 300), col, -1)
```

### Change Detection Threshold

```bash
ros2 run robocollector_vision vision_node --ros-args -p min_area:=300
```

---

## ✅ Validation Checklist

Before deploying to real robot, verify:

- [ ] Vision node detects red/blue/green/yellow blocks correctly
- [ ] Controller transitions through all 7 states
- [ ] Mock pickup sequences complete without errors
- [ ] Color change service works dynamically
- [ ] System recovers when detection is lost
- [ ] Enter/exit services work for all nodes
- [ ] No memory leaks (run for 5+ minutes)
- [ ] `/cmd_vel` publishes expected velocities

---

## 🚫 Limitations

**What CANNOT be simulated without Gazebo:**

1. ❌ **Physics** - No collision detection or object interaction
2. ❌ **Real depth** - Cannot test distance-based approach
3. ❌ **Navigation** - No SLAM or obstacle avoidance
4. ❌ **Servo feedback** - No actual arm position verification
5. ❌ **Lighting** - Real-world color detection may differ

**To test these, you need:**

- Full Gazebo simulation (requires URDF model)
- Or deploy to real JetRover hardware

---

## 📈 Next Steps After Simulation

Once simulation tests pass:

1. **Deploy to JetRover**:

   ```bash
   git add robocollector/
   git commit -m "Add simulation testing capabilities"
   git push origin main

   # On robot
   ssh ubuntu@192.168.2.101
   cd ~/ros2_ws && git pull
   colcon build --packages-select robocollector_vision robocollector_manipulation robocollector_controller robocollector_bringup
   ```

2. **Test with real camera** (no movement):

   ```bash
   # On robot - test vision only
   ros2 launch robocollector_vision vision_node.launch.py
   ros2 service call /vision_node/enter std_srvs/srv/Trigger
   ```

3. **Calibrate servos** (no autonomous control):

   ```bash
   # Test individual servo positions
   ros2 service call /ros_robot_controller/bus_servo/set_position ...
   ```

4. **Full integration test** with real hardware

---

## 🎯 Success Criteria

Simulation is successful when:

✅ Vision detects blocks reliably (>90% detection rate)  
✅ State machine completes full cycle without errors  
✅ All service calls respond correctly  
✅ System runs for 10+ minutes without crashes  
✅ Color changes work dynamically  
✅ Mock timing matches expected real-world duration

---

**You're now ready to test the full system logic before touching real hardware!** 🚀
