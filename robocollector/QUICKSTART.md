# RoboCollector Quick Start Guide

## ✅ Package Structure Created

```
robocollector/
├── robocollector_vision/          # Color detection node
│   ├── config/vision_params.yaml
│   ├── launch/vision_node.launch.py
│   └── robocollector_vision/vision_node.py
│
├── robocollector_manipulation/    # Arm control node
│   ├── config/pickup_params.yaml
│   ├── launch/pickup_node.launch.py
│   └── robocollector_manipulation/pickup_node.py
│
├── robocollector_controller/      # State machine coordinator
│   ├── launch/controller_node.launch.py
│   └── robocollector_controller/controller_node.py
│
├── robocollector_navigation/      # Navigation (existing)
│
└── robocollector_bringup/         # System orchestration
    └── launch/robocollector.launch.py
```

## 🚀 Quick Start

### 1. Build (Already Done ✅)

```bash
cd ~/JetRover/ros2_ws
colcon build --packages-select robocollector_vision robocollector_manipulation robocollector_controller robocollector_bringup --symlink-install
source install/setup.bash
```

### 2. Launch the System

```bash
# Set development environment
export need_compile=False

# Start all nodes
ros2 launch robocollector_bringup robocollector.launch.py
```

### 3. Control the Mission

**In a new terminal:**

```bash
# Source workspace
source ~/JetRover/ros2_ws/install/setup.bash

# Start the mission
ros2 service call /controller_node/enter std_srvs/srv/Trigger

# Monitor detection
ros2 topic echo /vision_node/color_info

# Stop the mission
ros2 service call /controller_node/exit std_srvs/srv/Trigger
```

## 🧪 Testing Individual Nodes

### Test Vision Node

```bash
# Terminal 1: Launch vision node with debug
export need_compile=False
ros2 launch robocollector_vision vision_node.launch.py

# Terminal 2: Start vision processing
ros2 service call /vision_node/enter std_srvs/srv/Trigger

# Terminal 3: Monitor detections
ros2 topic echo /vision_node/color_info
```

### Test Manipulation Node

```bash
# Terminal 1: Launch pickup node
ros2 launch robocollector_manipulation pickup_node.launch.py

# Terminal 2: Test arm movements
ros2 service call /pickup_node/enter std_srvs/srv/Trigger
ros2 service call /pickup_node/home std_srvs/srv/Trigger
ros2 service call /pickup_node/pickup std_srvs/srv/Trigger
ros2 service call /pickup_node/place std_srvs/srv/Trigger
```

### Test Controller (Full Integration)

```bash
# Terminal 1: Launch full system
ros2 launch robocollector_bringup robocollector.launch.py

# Terminal 2: Start mission
ros2 service call /controller_node/enter std_srvs/srv/Trigger

# Change target color
ros2 service call /controller_node/set_target interfaces/srv/SetString "{data: 'blue'}"
```

## 📊 Monitoring and Debugging

### Check Running Nodes

```bash
ros2 node list
# Expected output:
# /vision_node
# /pickup_node
# /controller_node
```

### Check Available Services

```bash
ros2 service list | grep -E "(vision|pickup|controller)"
```

### Monitor Robot Movement

```bash
ros2 topic echo /cmd_vel
```

### View Topics

```bash
ros2 topic list
ros2 topic hz /vision_node/color_info
ros2 topic hz /depth_cam/rgb/image_raw
```

## 🔧 Configuration

### Adjust Vision Detection

Edit: `robocollector_vision/config/vision_params.yaml`

```yaml
vision_node:
  ros__parameters:
    target_color: "red" # Change to 'blue', 'green', 'yellow'
    min_area: 500 # Adjust for object size
    debug: true # Enable OpenCV visualization
```

### Calibrate Arm Positions

Edit: `robocollector_manipulation/config/pickup_params.yaml`

Test positions manually:

```bash
# Use JetRover servo control to find correct positions
ros2 service call /ros_robot_controller/bus_servo/set_position ...
```

## 🐛 Common Issues

### Issue: No camera image

```bash
# Check camera node is running
ros2 topic list | grep depth_cam
ros2 topic hz /depth_cam/rgb/image_raw
```

### Issue: Services not responding

```bash
# Verify nodes are running
ros2 node list

# Check service exists
ros2 service list | grep vision_node

# Test service manually
ros2 service call /vision_node/init_finish std_srvs/srv/Trigger
```

### Issue: No detections

- Enable debug mode in config
- Check lighting conditions
- Adjust HSV color ranges
- Verify camera is pointing at colored objects

### Issue: Arm not moving

- Verify robot controller is running
- Check servo connections
- Calibrate servo positions in config

## 📝 Next Steps

1. **Test on real hardware** - Deploy to JetRover and test with camera
2. **Calibrate colors** - Tune HSV ranges for your environment
3. **Calibrate arm** - Find correct servo positions for pickup
4. **Integrate navigation** - Add SLAM and path planning
5. **Add depth sensing** - Use depth camera for distance estimation

## 🚢 Deploy to JetRover

```bash
# From local WSL2
git add robocollector/
git commit -m "Add RoboCollector package structure"
git push origin main

# On JetRover (SSH)
ssh ubuntu@192.168.2.101
cd ~/ros2_ws
git pull
colcon build --packages-select robocollector_vision robocollector_manipulation robocollector_controller robocollector_bringup
source ~/.robotrc
export need_compile=True
ros2 launch robocollector_bringup robocollector.launch.py
```

## 📚 Architecture Reference

- **Service Pattern**: All nodes follow JetRover's `enter/exit/init_finish` service pattern
- **Dual Environment**: Launch files support both development (`need_compile=False`) and production (`need_compile=True`)
- **Thread Safety**: All nodes use `threading.RLock()` for concurrent access
- **Color Logging**: Success messages in green, errors in red

## 🎯 Mission Flow

1. **IDLE** → System waiting
2. **SEARCHING** → Robot rotates to find block
3. **APPROACHING** → Moves toward detected block
4. **ALIGNING** → Fine-tunes position
5. **PICKING** → Arm grabs block
6. **RETURNING** → Navigates to collection area
7. **PLACING** → Deposits block
8. **COMPLETE** → Ready for next cycle

---

**Status**: ✅ All packages built and ready for testing!
